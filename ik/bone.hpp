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
    vec<float, D> position(ecs<D>* ecs) const { return ecs->position_[id]; }
    vec<float, D> orientation(ecs<D>* ecs) const { return ecs->orientation_[id]; }
    vec<float, D> scale(ecs<D>* ecs) const { return ecs->scale_[id]; }
    vec<float, D-1> rotation(ecs<D>* ecs) const { return ecs->rotation_[id]; }

    bone* createBone() {
        bone* child = new bone();
        child->parent = this;
        children.push_back(child);
        return child;
    }

protected:
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

protected:
    // 拓扑排序，要注意将每个节点的孩子连续排放，并记录孩子的开始位置和数量，方便后面查找
    std::vector<bone<D>*> topologicalSort() {
        std::vector<bone<D>*> sorted;
        std::queue<bone<D>*> queue;
        unsigned id = 0;
        queue.push_back(root);
        while (!stack.empty()) {
            bone<D>* current = stack.back();
            stack.pop_back();
            sorted.push_back(current);
            current->id = id++;
            for (auto child : current->children) {
                stack.push_back(child);
            }
        }
        return sorted;
    }
    ecs<D>* ecs = nullptr;
    bone<D>* root = nullptr;
};



typedef bone<2> bone2d;
typedef bone<3> bone3d;

typedef skeleton<2> skeleton2d;
typedef skeleton<3> skeleton3d;


} // namespace ik