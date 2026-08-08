#include "ik.hpp"
#include <cmath>
#define ZEROERR_IMPLEMENTATION
#include "zeroerr.hpp"

using namespace zeroerr;

TEST_CASE("1. vector arithmetic") {
    ik::vec3 v1 = {1, 2, 3};
    ik::vec3 v2 = {4, 5, 6};

    CHECK_EQ(v1 + v2, (ik::vec3{5, 7, 9}));
    CHECK_EQ(v1 - v2, (ik::vec3{-3, -3, -3}));
    CHECK_EQ(-v1, (ik::vec3{-1, -2, -3}));

    CHECK_EQ(v1 * 2.0f, (ik::vec3{2, 4, 6}));
    CHECK_EQ(2.0f * v1, (ik::vec3{2, 4, 6}));

    CHECK_EQ(v1 / 2.0f, (ik::vec3{0.5f, 1.0f, 1.5f}));
    CHECK_EQ(2.0f / v1, (ik::vec3{2.0f, 1.0f, 2.0f / 3.0f}));

    CHECK_EQ(v1 + 1.0f, (ik::vec3{2, 3, 4}));
    CHECK_EQ(1.0f + v1, (ik::vec3{2, 3, 4}));
    CHECK_EQ(v1 - 1.0f, (ik::vec3{0, 1, 2}));
}

TEST_CASE("2. vector products") {
    ik::vec3 v1 = {1, 2, 3};
    ik::vec3 v2 = {4, 5, 6};

    CHECK_LT(std::abs(dot(v1, v2) - 32.0f), 0.0001f);
    CHECK_EQ(cross(v1, v2), (ik::vec3{-3, 6, -3}));
    CHECK_EQ(sum(v1), 6.0f);

    ik::vec2 a = {1, 0};
    ik::vec2 b = {0, 1};
    CHECK_EQ(cross(a, b), (ik::vec2{1, 0}));
}

TEST_CASE("3. vector equality and indexing") {
    ik::vec3 v = {1, 2, 3};
    CHECK_EQ(v[0], 1.0f);
    CHECK_EQ(v[1], 2.0f);
    CHECK_EQ(v[2], 3.0f);
    CHECK_EQ(v, (ik::vec3{1, 2, 3}));

    ik::vec3 zero;
    CHECK_EQ(zero, (ik::vec3{0, 0, 0}));
}

TEST_CASE("4. matrix multiply") {
    ik::vec2 v1 = {1, 2};
    ik::mat3x2 m1 = {{1, 2, 3}, {4, 5, 6}};
    ik::mat2x3 m2 = {{1, 2}, {3, 4}, {5, 6}};
    CHECK_EQ(v1 * m1, (ik::vec3{9, 12, 15}));
    CHECK_EQ(m1 * m2, (ik::mat2{{22, 28}, {49, 64}}));
}

TEST_CASE("5. matrix arithmetic") {
    ik::mat2 a = {{1, 2}, {3, 4}};
    ik::mat2 b = {{5, 6}, {7, 8}};

    CHECK_EQ(a + b, (ik::mat2{{6, 8}, {10, 12}}));
    CHECK_EQ(b - a, (ik::mat2{{4, 4}, {4, 4}}));
    CHECK_EQ(a * 2.0f, (ik::mat2{{2, 4}, {6, 8}}));
    CHECK_EQ(2.0f * a, (ik::mat2{{2, 4}, {6, 8}}));
    CHECK_EQ(-a, (ik::mat2{{-1, -2}, {-3, -4}}));
    CHECK_EQ(a + 1.0f, (ik::mat2{{2, 3}, {4, 5}}));
}

TEST_CASE("6. matrix times vector") {
    ik::mat2 m = {{1, 2}, {3, 4}};
    ik::vec2 v = {1, 1};
    CHECK_EQ(m * v, (ik::vec2{3, 7}));
}

TEST_CASE("7. bone topology") {
    ik::skeleton2d skeleton;
    auto* r = skeleton.root;
    auto* b1 = r->createBone();
    auto* b2 = b1->createBone();
    auto* b3 = r->createBone();
    auto sorted = skeleton.topologicalSort();

    CHECK_EQ(sorted.size(), 4u);
    CHECK_EQ(r->id, 0u);
    CHECK_EQ(b1->id, 1u);
    CHECK_EQ(b3->id, 2u);
    CHECK_EQ(b2->id, 3u);
    CHECK_EQ(b1->parent, r);
    CHECK_EQ(b2->parent, b1);
    CHECK_EQ(r->children.size(), 2u);
}

TEST_CASE("8. ecs state access") {
    ik::skeleton2d skeleton;
    auto* r = skeleton.root;
    auto* b1 = r->createBone();
    auto* b2 = b1->createBone();
    skeleton.topologicalSort();

    ik::ecs2d ecs(3);
    r->position(ecs) = {0, 0};
    r->rotation(ecs) = {0};
    b1->position(ecs) = {1, 0};
    b1->rotation(ecs) = {1};
    b2->position(ecs) = {1, 0};
    b2->rotation(ecs) = {1};

    CHECK_EQ(r->position(ecs), (ik::vec2{0, 0}));
    CHECK_EQ(b1->position(ecs), (ik::vec2{1, 0}));
    CHECK_EQ(b2->position(ecs), (ik::vec2{1, 0}));
    CHECK_EQ(b1->rotation(ecs)[0], 1.0f);
    CHECK_EQ(ecs.position_.size(), 3u);
    CHECK_EQ(ecs.rotation_.size(), 3u);
}

TEST_CASE("9. solver configuration") {
    ik::solver<2> s;
    auto& chained = s.setKeepTrace(true).setMaxIterations(32).setTolerance(1e-4f);
    CHECK_EQ(&chained, &s);
}
