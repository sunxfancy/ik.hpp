#pragma once
#include <vector>
#include "ik/vec.hpp"

namespace ik
{

template <unsigned D>
struct ecs
{
    std::vector<vec<float, D>> position_;
    std::vector<vec<float, D>> orientation_;
    std::vector<vec<float, D>> scale_;
    std::vector<vec<float, D-1>> rotation_;

    ecs() = default;
    explicit ecs(unsigned size)
        : position_(size), orientation_(size), scale_(size), rotation_(size) {}

    ecs(const ecs& other) = default;
    ecs& operator=(const ecs& other) = default;

    unsigned size() const { return static_cast<unsigned>(position_.size()); }

    ecs* clone() const { return new ecs(*this); }
};

typedef ecs<2> ecs2d;
typedef ecs<3> ecs3d;

} // namespace ik
