#pragma once

#include "ik/bone.hpp"
#include "ik/ecs.hpp"
#include "ik/vec.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace ik
{

namespace detail
{

template <unsigned D>
inline void apply_angle_constraints(const std::vector<bone<D>*>&, ecs<D>&) {}

template <>
inline void apply_angle_constraints<2>(const std::vector<bone<2>*>& bones, ecs<2>& state) {
    for (auto* b : bones) {
        if (!b->parent || !b->constrained) continue;

        vec2 parent_dir{1.0f, 0.0f};
        if (b->parent->parent) {
            parent_dir = b->parent->position(state) - b->parent->parent->position(state);
            if (length_squared(parent_dir) <= 1e-12f) {
                parent_dir = vec2{1.0f, 0.0f};
            }
        }
        float parent_angle = angle(parent_dir);
        vec2 dir = b->position(state) - b->parent->position(state);
        float world = angle(dir);
        float local = wrap_angle(world - parent_angle);
        float clamped = clamp_angle(local, b->min_rotation[0], b->max_rotation[0]);
        if (clamped != local) {
            float len = b->length > 0.0f ? b->length : length(dir);
            b->position(state) =
                b->parent->position(state) + rotate(vec2{len, 0.0f}, parent_angle + clamped);
        }
    }
}

} // namespace detail

// Iterative FABRIK solver with optional force blending, joint limits, and traces.
template <unsigned D>
class solver
{
public:
    struct target {
        unsigned bone_id = 0;
        vec<float, D> position;
        float weight = 1.0f;
    };

    solver() = default;
    ~solver() { clearTrace(); }

    solver(const solver&) = delete;
    solver& operator=(const solver&) = delete;

    solver& setKeepTrace(bool keep) { keepTrace = keep; return *this; }
    solver& setMaxIterations(unsigned iterations) { maxIterations = iterations; return *this; }
    solver& setTolerance(float tol) { tolerance = tol; return *this; }
    // 0 = hard IK (FABRIK teleports effector); (0,1] blends effector toward target each iter.
    solver& setForce(float f) { force = f; return *this; }

    solver& clearTargets() { targets_.clear(); return *this; }
    solver& addTarget(unsigned bone_id, vec<float, D> position, float weight = 1.0f) {
        targets_.push_back(target{bone_id, position, weight});
        return *this;
    }
    solver& addTarget(const bone<D>* b, vec<float, D> position, float weight = 1.0f) {
        return addTarget(b->id, position, weight);
    }

    const std::vector<target>& targets() const { return targets_; }

    ecs<D>* getTrace(unsigned time) {
        if (time >= trace.size()) return nullptr;
        return trace[time];
    }
    unsigned traceSize() const { return static_cast<unsigned>(trace.size()); }

    void clearTrace() {
        for (auto* t : trace) {
            delete t;
        }
        trace.clear();
    }

    // Returns true if every target is within tolerance after solving.
    bool solve(skeleton<D>& sk, ecs<D>& state) {
        if (sk.bones().empty()) {
            sk.topologicalSort();
        }
        const auto& bones = sk.bones();
        if (bones.empty() || targets_.empty()) {
            return true;
        }

        clearTrace();
        if (keepTrace) {
            recordTrace(state);
        }

        bool reached = false;
        unsigned iterations = maxIterations == 0 ? 1u : maxIterations;
        for (unsigned iter = 0; iter < iterations; ++iter) {
            for (const auto& t : targets_) {
                if (t.bone_id >= bones.size()) continue;
                solve_chain(bones, state, bones[t.bone_id], t);
            }

            apply_constraints(bones, state);
            sk.update_rotations(state);

            if (keepTrace) {
                recordTrace(state);
            }

            reached = true;
            for (const auto& t : targets_) {
                if (t.bone_id >= bones.size()) continue;
                if (distance(bones[t.bone_id]->position(state), t.position) > tolerance) {
                    reached = false;
                    break;
                }
            }
            if (reached) break;
        }
        return reached;
    }

    // Single force-based integration step toward current targets (soft IK).
    void step(skeleton<D>& sk, ecs<D>& state, float dt = 1.0f) {
        float previous_force = force;
        if (force <= 0.0f) {
            force = std::min(1.0f, std::max(0.0f, dt));
        } else {
            force = std::min(1.0f, force * std::max(0.0f, dt));
        }
        unsigned previous_iters = maxIterations;
        maxIterations = 1;
        solve(sk, state);
        maxIterations = previous_iters;
        force = previous_force;
    }

protected:
    void recordTrace(const ecs<D>& state) {
        trace.push_back(state.clone());
    }

    static void place_on_line(vec<float, D>& point,
                              const vec<float, D>& anchor,
                              float bone_length) {
        vec<float, D> dir = point - anchor;
        float len = length(dir);
        if (len <= 1e-8f) {
            dir = vec<float, D>{};
            dir[0] = 1.0f;
            len = 1.0f;
        }
        point = anchor + dir * (bone_length / len);
    }

    void solve_chain(const std::vector<bone<D>*>& bones,
                     ecs<D>& state,
                     bone<D>* end,
                     const target& t) {
        std::vector<bone<D>*> chain;
        for (bone<D>* b = end; b != nullptr; b = b->parent) {
            chain.push_back(b);
        }
        if (chain.size() < 2) {
            return;
        }
        std::reverse(chain.begin(), chain.end());
        bone<D>* root = chain.front();
        vec<float, D> root_pos = root->position(state);

        float chain_length = 0.0f;
        for (size_t i = 1; i < chain.size(); ++i) {
            chain_length += chain[i]->length;
        }

        vec<float, D> desired = t.position;
        if (force > 0.0f && force < 1.0f) {
            vec<float, D> cur = end->position(state);
            desired = cur + (t.position - cur) * (force * t.weight);
        }

        if (distance(root_pos, desired) >= chain_length) {
            // Target unreachable: stretch chain toward target.
            for (size_t i = 1; i < chain.size(); ++i) {
                vec<float, D> dir = desired - chain[i - 1]->position(state);
                float len = length(dir);
                if (len > 1e-8f) {
                    chain[i]->position(state) =
                        chain[i - 1]->position(state) + dir * (chain[i]->length / len);
                } else {
                    place_on_line(chain[i]->position(state),
                                  chain[i - 1]->position(state),
                                  chain[i]->length);
                }
            }
            return;
        }

        // Backward reaching.
        end->position(state) = desired;
        for (size_t i = chain.size() - 1; i > 0; --i) {
            place_on_line(chain[i - 1]->position(state),
                          chain[i]->position(state),
                          chain[i]->length);
        }

        // Forward reaching: pin root.
        root->position(state) = root_pos;
        for (size_t i = 1; i < chain.size(); ++i) {
            place_on_line(chain[i]->position(state),
                          chain[i - 1]->position(state),
                          chain[i]->length);
        }
    }

    void apply_constraints(const std::vector<bone<D>*>& bones, ecs<D>& state) {
        detail::apply_angle_constraints<D>(bones, state);
    }

    bool keepTrace = false;
    unsigned maxIterations = 16;
    float tolerance = 1e-3f;
    float force = 0.0f;

    std::vector<target> targets_;
    std::vector<ecs<D>*> trace;
};

typedef solver<2> solver2d;
typedef solver<3> solver3d;

} // namespace ik
