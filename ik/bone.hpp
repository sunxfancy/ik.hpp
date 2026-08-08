#pragma once

#include "ik/vec.hpp"
#include "ik/ecs.hpp"
#include <queue>
#include <vector>

namespace ik
{

// 骨骼的数据分为两部分，一部分是固定的，不随时间变化的，另一部分是随时间变化的，应分开存储

template <unsigned D>
class bone {
public:
    vec<float, D> position(const ecs<D>& ecs) const { return ecs.position_[id]; }
    vec<float, D> orientation(const ecs<D>& ecs) const { return ecs.orientation_[id]; }
    vec<float, D> scale(const ecs<D>& ecs) const { return ecs.scale_[id]; }
    vec<float, D-1> rotation(const ecs<D>& ecs) const { return ecs.rotation_[id]; }

    vec<float, D>& position(ecs<D>& ecs) { return ecs.position_[id]; }
    vec<float, D>& orientation(ecs<D>& ecs) { return ecs.orientation_[id]; }
    vec<float, D>& scale(ecs<D>& ecs) { return ecs.scale_[id]; }
    vec<float, D-1>& rotation(ecs<D>& ecs) { return ecs.rotation_[id]; }

    bone* createBone(float bone_length = 1.0f) {
        bone* child = new bone();
        child->parent = this;
        child->length = bone_length;
        const float pi = 3.14159265358979323846f;
        for (unsigned i = 0; i < D - 1; ++i) {
            child->min_rotation[i] = -pi;
            child->max_rotation[i] = pi;
        }
        children.push_back(child);
        return child;
    }

    bone& setLength(float bone_length) {
        length = bone_length;
        return *this;
    }

    bone& setConstraints(vec<float, D-1> min_rot, vec<float, D-1> max_rot) {
        min_rotation = min_rot;
        max_rotation = max_rot;
        constrained = true;
        return *this;
    }

    bone& clearConstraints() {
        constrained = false;
        return *this;
    }

// protected:
    unsigned id = 0;
    float length = 0.0f;
    bool constrained = false;
    vec<float, D-1> min_rotation;
    vec<float, D-1> max_rotation;
    unsigned child_begin = 0;
    unsigned child_count = 0;
    bone* parent = nullptr;
    std::vector<bone*> children;
};

template <unsigned D>
class skeleton {
public:
    skeleton() {
        root = new bone<D>();
        const float pi = 3.14159265358979323846f;
        for (unsigned i = 0; i < D - 1; ++i) {
            root->min_rotation[i] = -pi;
            root->max_rotation[i] = pi;
        }
    }
    ~skeleton() {
        for (auto* b : topologicalSort()) {
            delete b;
        }
        root = nullptr;
    }
    bone<D>* root = nullptr;

    // 拓扑排序，要注意将每个节点的孩子连续排放，并记录孩子的开始位置和数量，方便后面查找
    std::vector<bone<D>*> topologicalSort() {
        std::vector<bone<D>*> sorted;
        if (!root) return sorted;

        std::queue<bone<D>*> queue;
        unsigned id = 0;
        queue.push(root);
        while (!queue.empty()) {
            auto b = queue.front();
            queue.pop();
            b->id = id++;
            sorted.push_back(b);
            for (auto child : b->children) {
                queue.push(child);
            }
        }

        for (auto* b : sorted) {
            b->child_count = static_cast<unsigned>(b->children.size());
            b->child_begin = b->child_count ? b->children.front()->id : 0;
            if (b->parent == nullptr) {
                b->length = 0.0f;
            }
        }
        bones_ = sorted;
        return sorted;
    }

    const std::vector<bone<D>*>& bones() const { return bones_; }

    // Capture bone lengths from current world joint positions in ecs.
    void bind(ecs<D>& state) {
        if (bones_.empty()) {
            topologicalSort();
        }
        for (auto* b : bones_) {
            if (!b->parent) {
                b->length = 0.0f;
                continue;
            }
            b->length = distance(b->position(state), b->parent->position(state));
        }
        update_rotations(state);
    }

    // Place the skeleton in a straight chain starting at root_position along +X (2D/3D).
    void init_straight_pose(ecs<D>& state, vec<float, D> root_position = {}) {
        if (bones_.empty()) {
            topologicalSort();
        }
        if (state.size() < bones_.size()) {
            state = ecs<D>(static_cast<unsigned>(bones_.size()));
        }
        for (auto* b : bones_) {
            if (!b->parent) {
                b->position(state) = root_position;
                continue;
            }
            vec<float, D> offset{};
            offset[0] = b->length > 0.0f ? b->length : 1.0f;
            if (b->length <= 0.0f) {
                b->length = offset[0];
            }
            b->position(state) = b->parent->position(state) + offset;
        }
        update_rotations(state);
    }

    // Build world joint positions from local joint angles in rotation_ (2D).
    // root.position is the fixed world root; each child uses its length and local angle.
    void forward_kinematics(ecs<D>& state) const {
        if (bones_.empty()) return;
        for (auto* b : bones_) {
            if (!b->parent) {
                continue;
            }
            if constexpr (D == 2) {
                float parent_world = 0.0f;
                if (b->parent->parent) {
                    vec2 parent_dir =
                        b->parent->position(state) - b->parent->parent->position(state);
                    parent_world = angle(parent_dir);
                } else {
                    parent_world = b->parent->rotation(state)[0];
                }
                float local_angle = b->rotation(state)[0];
                float world_angle = parent_world + local_angle;
                float len = b->length > 0.0f ? b->length : 1.0f;
                b->position(state) =
                    b->parent->position(state) + rotate(vec2{len, 0.0f}, world_angle);
            } else {
                vec<float, D> dir{};
                dir[0] = 1.0f;
                b->position(state) = b->parent->position(state) + dir * b->length;
            }
        }
        update_rotations(state);
    }

    // Derive joint rotations from world positions (2D: absolute world angle of bone segment).
    void update_rotations(ecs<D>& state) const {
        if constexpr (D == 2) {
            for (auto* b : bones_) {
                if (!b->parent) {
                    continue;
                }
                vec2 dir = b->position(state) - b->parent->position(state);
                b->rotation(state)[0] = angle(dir);
            }
        }
    }

    float total_length_to(const bone<D>* end) const {
        float total = 0.0f;
        for (const bone<D>* b = end; b && b->parent; b = b->parent) {
            total += b->length;
        }
        return total;
    }

// protected:
    ecs<D>* state = nullptr;

private:
    std::vector<bone<D>*> bones_;
};



typedef bone<2> bone2d;
typedef bone<3> bone3d;

typedef skeleton<2> skeleton2d;
typedef skeleton<3> skeleton3d;


} // namespace ik
