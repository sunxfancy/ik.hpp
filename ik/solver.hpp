#pragma once

#include "ik/bone.hpp"
#include "ik/ecs.hpp"
#include "ik/joint.hpp"
#include "ik/vec.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace ik
{

namespace detail
{

template <unsigned D>
inline void apply_angle_constraints(const std::vector<bone<D>*>& bones, ecs<D>& state) {
    for (auto* b : bones) {
        if (!b->parent || !b->constrained) continue;

        const vec<float, D>* gp = nullptr;
        vec<float, D> gp_pos;
        if (b->parent->parent) {
            gp_pos = b->parent->parent->position(state);
            gp = &gp_pos;
        }
        vec<float, D> parent_fwd = parent_forward_of<D>(
            b->parent->orientation(state), b->parent->position(state), gp);

        vec<float, D> child_dir =
            safe_normalize(b->position(state) - b->parent->position(state));
        auto local = local_angles_from_direction<D>(parent_fwd, child_dir);
        auto clamped = clamp_local_angles(local, b->min_rotation, b->max_rotation);

        bool changed = false;
        for (unsigned i = 0; i < D - 1; ++i) {
            if (clamped[i] != local[i]) {
                changed = true;
                break;
            }
        }
        if (!changed) continue;

        vec<float, D> dir = direction_from_local_angles<D>(parent_fwd, clamped);
        float len = b->length > 0.0f ? b->length : length(b->position(state) - b->parent->position(state));
        b->orientation(state) = dir;
        b->rotation(state) = clamped;
        b->position(state) = b->parent->position(state) + dir * len;
    }
}

template <unsigned D>
inline void place_on_line(vec<float, D>& point, const vec<float, D>& anchor, float bone_length) {
    vec<float, D> dir = point - anchor;
    float len = length(dir);
    if (len <= 1e-8f) {
        dir = default_forward<D>();
        len = 1.0f;
    }
    point = anchor + dir * (bone_length / len);
}

template <unsigned D>
inline std::vector<bone<D>*> build_chain(skeleton<D>& sk, unsigned root_id, unsigned tip_id) {
    if (sk.bones().empty()) {
        sk.topologicalSort();
    }
    const auto& bones = sk.bones();
    if (root_id >= bones.size() || tip_id >= bones.size()) {
        return {};
    }

    std::vector<bone<D>*> chain;
    for (bone<D>* b = bones[tip_id]; b != nullptr; b = b->parent) {
        chain.push_back(b);
        if (b->id == root_id) {
            break;
        }
    }
    if (chain.empty() || chain.back()->id != root_id || chain.size() < 2) {
        return {};
    }
    std::reverse(chain.begin(), chain.end());
    return chain;
}

/** One FABRIK reach on chain[0..tip_index], pinning chain[0]. */
template <unsigned D>
inline void reach_target(ecs<D>& state, const std::vector<bone<D>*>& chain, unsigned tip_index,
                         const vec<float, D>& desired) {
    const vec<float, D> root_pos = chain[0]->position(state);

    float chain_length = 0.f;
    for (unsigned i = 1; i <= tip_index; ++i) {
        chain_length += chain[i]->length;
    }

    if (distance(root_pos, desired) >= chain_length) {
        const vec<float, D> dir = safe_normalize(desired - root_pos);
        for (unsigned i = 1; i <= tip_index; ++i) {
            chain[i]->position(state) = chain[i - 1]->position(state) + dir * chain[i]->length;
        }
        return;
    }

    chain[tip_index]->position(state) = desired;
    for (int i = static_cast<int>(tip_index); i >= 1; --i) {
        place_on_line(chain[static_cast<size_t>(i) - 1]->position(state),
                      chain[static_cast<size_t>(i)]->position(state),
                      chain[static_cast<size_t>(i)]->length);
    }

    chain[0]->position(state) = root_pos;
    for (unsigned i = 1; i <= tip_index; ++i) {
        place_on_line(chain[i]->position(state), chain[i - 1]->position(state), chain[i]->length);
    }
}

/** Pull mid-chain joints toward a pole (3D only). Preserves distance to the root→goal axis. */
template <unsigned D>
inline void apply_pole(ecs<D>& state, const std::vector<bone<D>*>& chain, const vec<float, D>& goal,
                       const vec3& pole, float weight) {
    if constexpr (D == 3) {
        if (weight <= 0.f || chain.size() < 2) {
            return;
        }
        const vec3 root_pos = chain[0]->position(state);
        const vec3 to_goal = goal - root_pos;
        const float goal_len = length(to_goal);
        if (goal_len <= 1e-8f) {
            return;
        }
        const vec3 axis = to_goal / goal_len;

        const vec3 to_pole = pole - root_pos;
        const vec3 proj = to_pole - axis * dot(to_pole, axis);
        if (length_squared(proj) <= 1e-8f) {
            return;
        }
        const vec3 pole_dir = safe_normalize(proj);

        for (size_t i = 1; i + 1 < chain.size(); ++i) {
            const vec3 v = chain[i]->position(state) - root_pos;
            const float along = dot(v, axis);
            vec3 radial = v - axis * along;
            const float radial_len = length(radial);
            if (radial_len <= 1e-8f) {
                continue;
            }
            const vec3 dir = radial / radial_len;
            const vec3 new_dir = safe_normalize(dir + (pole_dir - dir) * weight);
            chain[i]->position(state) = root_pos + axis * along + new_dir * radial_len;
        }
    } else {
        (void)state;
        (void)chain;
        (void)goal;
        (void)pole;
        (void)weight;
    }
}

} // namespace detail

/**
 * Iterative FABRIK solver with force blending, joint limits, chain-scoped solves,
 * optional 3D pole vector, tip orientation override, and influence blending.
 */
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
    // 0 = keep input pose, 1 = full solve (default). Applied after solve / solveChain.
    solver& setInfluence(float inf) {
        influence = std::min(1.0f, std::max(0.0f, inf));
        return *this;
    }
    float getInfluence() const { return influence; }

    // 3D magnet / hint: pulls mid-chain joints toward pole during solveChain / stepChain.
    solver& setPole(vec3 pole_pos, float weight = 1.0f) {
        usePole = true;
        pole = pole_pos;
        poleWeight = std::min(1.0f, std::max(0.0f, weight));
        return *this;
    }
    solver& clearPole() {
        usePole = false;
        poleWeight = 1.0f;
        return *this;
    }
    bool hasPole() const { return usePole; }
    float getPoleWeight() const { return usePole ? poleWeight : 0.0f; }

    // After solving, blend the tip bone's local angles toward the given values.
    solver& setTipRotation(unsigned bone_id, vec<float, D - 1> angles, float weight = 1.0f) {
        tipBoneId = static_cast<int>(bone_id);
        tipAngles = angles;
        tipRotWeight = std::min(1.0f, std::max(0.0f, weight));
        return *this;
    }
    solver& clearTipRotation() {
        tipBoneId = -1;
        tipAngles = {};
        tipRotWeight = 0.0f;
        return *this;
    }
    float getTipRotationWeight() const { return tipRotWeight; }

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

    // Full-skeleton FABRIK (pins the skeleton root). Returns true if every target is within tolerance.
    bool solve(skeleton<D>& sk, ecs<D>& state) {
        if (sk.bones().empty()) {
            sk.topologicalSort();
        }
        const auto& bones = sk.bones();
        if (bones.empty() || targets_.empty()) {
            apply_tip_rotation(sk, state);
            return true;
        }

        std::vector<vec<float, D>> original;
        if (influence < 1.0f) {
            original = state.position_;
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
                solve_from_effector(state, bones[t.bone_id], t);
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

        blend_influence(sk, state, original);
        apply_tip_rotation(sk, state);
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

    /**
     * FABRIK restricted to root_id..tip_id. The chain root stays pinned; bones outside
     * the chain are untouched. Only targets on the chain participate. Pole applies here.
     * Returns false if the chain is invalid; true when every chain target is within tolerance
     * (or there are no chain targets).
     */
    bool solveChain(skeleton<D>& sk, ecs<D>& state, unsigned root_id, unsigned tip_id) {
        std::vector<bone<D>*> chain = detail::build_chain(sk, root_id, tip_id);
        if (chain.size() < 2) {
            return false;
        }

        struct ChainTarget {
            unsigned index = 0;
            const target* tgt = nullptr;
        };
        std::vector<ChainTarget> chain_targets;
        for (const auto& t : targets_) {
            for (size_t i = 0; i < chain.size(); ++i) {
                if (chain[i]->id == t.bone_id) {
                    chain_targets.push_back({static_cast<unsigned>(i), &t});
                    break;
                }
            }
        }
        if (chain_targets.empty()) {
            apply_tip_rotation(sk, state);
            return true;
        }

        const ChainTarget* goal = &chain_targets.front();
        for (const auto& ct : chain_targets) {
            if (ct.index > goal->index) {
                goal = &ct;
            }
        }

        std::vector<vec<float, D>> original_pos(chain.size());
        for (size_t i = 0; i < chain.size(); ++i) {
            original_pos[i] = chain[i]->position(state);
        }

        const float pole_w = std::min(1.f, std::max(0.f, poleWeight));
        const unsigned iterations = maxIterations == 0 ? 1u : maxIterations;
        bool reached = false;
        for (unsigned iter = 0; iter < iterations; ++iter) {
            for (const auto& ct : chain_targets) {
                vec<float, D> desired = ct.tgt->position;
                if (force > 0.f && force < 1.f) {
                    const vec<float, D> cur = chain[ct.index]->position(state);
                    desired = cur + (desired - cur) * (force * ct.tgt->weight);
                }
                detail::reach_target(state, chain, ct.index, desired);
            }

            if (usePole && pole_w > 0.f) {
                vec<float, D> goal_pos = goal->tgt->position;
                if (force > 0.f && force < 1.f) {
                    const vec<float, D> cur = chain[goal->index]->position(state);
                    goal_pos = cur + (goal_pos - cur) * (force * goal->tgt->weight);
                }
                detail::apply_pole(state, chain, goal_pos, pole, pole_w);
            }

            if (chain.size() >= 2) {
                std::vector<bone<D>*> rest(chain.begin() + 1, chain.end());
                detail::apply_angle_constraints<D>(rest, state);
            }
            sk.update_rotations(state);

            reached = true;
            for (const auto& ct : chain_targets) {
                if (distance(chain[ct.index]->position(state), ct.tgt->position) > tolerance) {
                    reached = false;
                    break;
                }
            }
            if (reached) {
                break;
            }
        }

        if (influence < 1.f) {
            const float t = std::min(1.f, std::max(0.f, influence));
            for (size_t i = 0; i < chain.size(); ++i) {
                vec<float, D>& pos = chain[i]->position(state);
                pos = original_pos[i] + (pos - original_pos[i]) * t;
            }
            sk.update_rotations(state);
        }

        apply_tip_rotation(sk, state);
        return reached;
    }

    // Time-scaled chain-scoped step (same force/dt semantics as step()).
    void stepChain(skeleton<D>& sk, ecs<D>& state, unsigned root_id, unsigned tip_id, float dt = 1.0f) {
        float previous_force = force;
        if (force <= 0.0f) {
            force = std::min(1.0f, std::max(0.0f, dt));
        } else {
            force = std::min(1.0f, force * std::max(0.0f, dt));
        }
        unsigned previous_iters = maxIterations;
        maxIterations = 1;
        solveChain(sk, state, root_id, tip_id);
        maxIterations = previous_iters;
        force = previous_force;
    }

protected:
    void recordTrace(const ecs<D>& state) {
        trace.push_back(state.clone());
    }

    void blend_influence(skeleton<D>& sk, ecs<D>& state, const std::vector<vec<float, D>>& original) {
        if (influence >= 1.0f || original.empty()) {
            return;
        }
        const float t = influence;
        const size_t n = std::min(original.size(), state.position_.size());
        for (size_t i = 0; i < n; ++i) {
            state.position_[i] = original[i] + (state.position_[i] - original[i]) * t;
        }
        sk.update_rotations(state);
    }

    void apply_tip_rotation(skeleton<D>& sk, ecs<D>& state) {
        if (tipRotWeight <= 0.0f || tipBoneId < 0) {
            return;
        }
        if (sk.bones().empty()) {
            sk.topologicalSort();
        }
        const auto& bones = sk.bones();
        if (static_cast<size_t>(tipBoneId) >= bones.size()) {
            return;
        }
        bone<D>* b = bones[static_cast<size_t>(tipBoneId)];
        if (!b->parent) {
            return;
        }
        const float w = tipRotWeight;
        vec<float, D - 1> rot = b->rotation(state);
        for (unsigned i = 0; i < D - 1; ++i) {
            rot[i] = rot[i] + wrap_angle(tipAngles[i] - rot[i]) * w;
        }
        b->rotation(state) = rot;

        const vec<float, D>* gp = nullptr;
        vec<float, D> gp_pos;
        if (b->parent->parent) {
            gp_pos = b->parent->parent->position(state);
            gp = &gp_pos;
        }
        const vec<float, D> parent_fwd =
            detail::parent_forward_of<D>(b->parent->orientation(state), b->parent->position(state), gp);
        b->orientation(state) = detail::direction_from_local_angles<D>(parent_fwd, rot);
    }

    void solve_from_effector(ecs<D>& state, bone<D>* end, const target& t) {
        std::vector<bone<D>*> chain;
        for (bone<D>* b = end; b != nullptr; b = b->parent) {
            chain.push_back(b);
        }
        if (chain.size() < 2) {
            return;
        }
        std::reverse(chain.begin(), chain.end());

        vec<float, D> desired = t.position;
        if (force > 0.0f && force < 1.0f) {
            vec<float, D> cur = end->position(state);
            desired = cur + (t.position - cur) * (force * t.weight);
        }
        detail::reach_target(state, chain, static_cast<unsigned>(chain.size() - 1), desired);
    }

    void apply_constraints(const std::vector<bone<D>*>& bones, ecs<D>& state) {
        detail::apply_angle_constraints<D>(bones, state);
    }

    bool keepTrace = false;
    unsigned maxIterations = 16;
    float tolerance = 1e-3f;
    float force = 0.0f;
    float influence = 1.0f;

    bool usePole = false;
    vec3 pole{};
    float poleWeight = 1.0f;

    int tipBoneId = -1;
    vec<float, D - 1> tipAngles{};
    float tipRotWeight = 0.0f;

    std::vector<target> targets_;
    std::vector<ecs<D>*> trace;
};

typedef solver<2> solver2d;
typedef solver<3> solver3d;

} // namespace ik
