#include "ik.hpp"
#include <math.h>
#define ZEROERR_IMPLEMENTATION
#include "zeroerr.hpp"

using namespace zeroerr;

TEST_CASE("1. vector") {
    ik::vec3 v1 = {1, 2, 3};
    ik::vec3 v2 = {4, 5, 6};
    
    CHECK_EQ(v1 + v2, (ik::vec3{5, 7, 9}));
    CHECK_EQ(v1 - v2, (ik::vec3{-3, -3, -3}));

    CHECK_EQ(v1 * 2.0f, (ik::vec3{2, 4, 6}));
    CHECK_EQ(2.0f * v1, (ik::vec3{2, 4, 6}));

    CHECK_EQ(v1 / 2.0f, (ik::vec3{0.5, 1, 1.5}));
    CHECK_EQ(2.0f / v1, (ik::vec3{2, 1, 2.0/3}));

    CHECK_LT(abs(dot(v1, v2) - 32.0f), 0.0001f);
    CHECK_EQ(cross(v1, v2), (ik::vec3{-3, 6, -3}));
}

TEST_CASE("2. matrix") {
    ik::vec2 v1 = {1, 2};
    ik::mat3x2 m1 = {{1, 2, 3}, {4, 5, 6}};
    ik::mat2x3 m2 = {{1, 2}, {3, 4}, {5, 6}};
    CHECK_EQ(v1 * m1, (ik::vec3{9, 12, 15}));
    CHECK_EQ(m1 * m2, (ik::mat2{{22, 28}, {49, 64}}));
}

TEST_CASE("3. bone") {
    ik::skeleton2d skeleton;
    auto* r = skeleton.root;
    auto* b1 = r->createBone();
    auto* b2 = b1->createBone();
    skeleton.topologicalSort();
    CHECK_EQ(b1->id, 1);
    CHECK_EQ(b2->id, 2);

    ik::ecs2d ecs(3);
    r->position(ecs) = {0, 0};
    r->rotation(ecs) = {0, 1};
    b1->position(ecs) = {1, 0};
    b1->rotation(ecs) = {1, 0};
    b2->position(ecs) = {1, 0};        
    b2->rotation(ecs) = {1, 0};

}
