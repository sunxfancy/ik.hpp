#pragma once

#include "ik/vec.hpp"
#include "ik/ecs.hpp"
#include <vector>
#include <queue>

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


    bone* createBone() {
        bone* child = new bone();
        child->parent = this;
        children.push_back(child);
        return child;
    }

// protected:
    unsigned id;
    bone* parent = nullptr;
    std::vector<bone*> children;
};

template <unsigned D>
class skeleton {
public:
    skeleton() { root = new bone<D>(); }
    ~skeleton() {
        for (auto bone : topologicalSort()) {
            delete bone;
        }
    }
    bone<D>* root = nullptr;

// protected:
    // 拓扑排序，要注意将每个节点的孩子连续排放，并记录孩子的开始位置和数量，方便后面查找
    std::vector<bone<D>*> topologicalSort() {
        std::vector<bone<D>*> sorted;
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
        return sorted;
    }
    ecs<D>* state = nullptr;
};



typedef bone<2> bone2d;
typedef bone<3> bone3d;

typedef skeleton<2> skeleton2d;
typedef skeleton<3> skeleton3d;


} // namespace ik