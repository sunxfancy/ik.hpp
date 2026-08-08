#include "ik.hpp"
#include <cmath>
#define ZEROERR_IMPLEMENTATION
#include "zeroerr.hpp"

using namespace zeroerr;

static bool near(float a, float b, float eps = 1e-3f) {
    return std::abs(a - b) <= eps;
}

static bool near_vec(ik::vec2 a, ik::vec2 b, float eps = 1e-3f) {
    return near(a[0], b[0], eps) && near(a[1], b[1], eps);
}

static bool near_vec(ik::vec3 a, ik::vec3 b, float eps = 1e-3f) {
    return near(a[0], b[0], eps) && near(a[1], b[1], eps) && near(a[2], b[2], eps);
}

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
    CHECK_EQ(r->child_count, 2u);
    CHECK_EQ(r->child_begin, 1u);
    CHECK_EQ(b1->child_count, 1u);
    CHECK_EQ(b1->child_begin, 3u);
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
    auto& chained = s.setKeepTrace(true).setMaxIterations(32).setTolerance(1e-4f).setForce(0.5f);
    CHECK_EQ(&chained, &s);
}

TEST_CASE("10. vector length helpers") {
    ik::vec2 v = {3, 4};
    CHECK_LT(std::abs(length(v) - 5.0f), 1e-5f);
    CHECK_LT(std::abs(length_squared(v) - 25.0f), 1e-5f);
    CHECK(near_vec(normalize(v), ik::vec2{0.6f, 0.8f}));
    CHECK(near_vec(rotate(ik::vec2{1, 0}, 3.14159265f / 2.0f), ik::vec2{0, 1}, 1e-4f));
}

TEST_CASE("11. bind and straight pose") {
    ik::skeleton2d skeleton;
    auto* r = skeleton.root;
    auto* b1 = r->createBone(1.0f);
    auto* b2 = b1->createBone(1.0f);
    skeleton.topologicalSort();

    ik::ecs2d ecs(3);
    skeleton.init_straight_pose(ecs, {0, 0});

    CHECK(near_vec(r->position(ecs), {0, 0}));
    CHECK(near_vec(b1->position(ecs), {1, 0}));
    CHECK(near_vec(b2->position(ecs), {2, 0}));
    CHECK_LT(std::abs(b1->length - 1.0f), 1e-5f);
    CHECK_LT(std::abs(b2->length - 1.0f), 1e-5f);
}

TEST_CASE("12. forward kinematics from local angles") {
    ik::skeleton2d skeleton;
    auto* r = skeleton.root;
    auto* b1 = r->createBone(1.0f);
    auto* b2 = b1->createBone(1.0f);
    skeleton.topologicalSort();

    ik::ecs2d ecs(3);
    r->position(ecs) = {0, 0};
    r->rotation(ecs) = {0};
    b1->rotation(ecs) = {0};                 // local
    b2->rotation(ecs) = {3.14159265f / 2.0f}; // local +90 deg

    skeleton.forward_kinematics(ecs);

    CHECK(near_vec(b1->position(ecs), {1, 0}, 1e-3f));
    CHECK(near_vec(b2->position(ecs), {1, 1}, 1e-3f));
}

TEST_CASE("13. ik reaches reachable target") {
    ik::skeleton2d skeleton;
    auto* r = skeleton.root;
    auto* b1 = r->createBone(1.0f);
    auto* b2 = b1->createBone(1.0f);
    skeleton.topologicalSort();

    ik::ecs2d ecs(3);
    skeleton.init_straight_pose(ecs, {0, 0});

    ik::vec2 target{1.0f, 1.0f};
    ik::solver2d s;
    s.setMaxIterations(32).setTolerance(1e-3f);
    s.addTarget(b2, target);
    CHECK(s.solve(skeleton, ecs));
    CHECK(near_vec(b2->position(ecs), target, 1e-2f));

    // Bone lengths preserved.
    CHECK(near(distance(b1->position(ecs), r->position(ecs)), 1.0f, 1e-2f));
    CHECK(near(distance(b2->position(ecs), b1->position(ecs)), 1.0f, 1e-2f));
    CHECK(near_vec(r->position(ecs), {0, 0}, 1e-3f));
}

TEST_CASE("14. ik unreachable target stretches toward it") {
    ik::skeleton2d skeleton;
    auto* r = skeleton.root;
    auto* b1 = r->createBone(1.0f);
    auto* b2 = b1->createBone(1.0f);
    skeleton.topologicalSort();

    ik::ecs2d ecs(3);
    skeleton.init_straight_pose(ecs, {0, 0});

    ik::vec2 target{10.0f, 0.0f};
    ik::solver2d s;
    s.setMaxIterations(8).setTolerance(1e-3f);
    s.addTarget(b2, target);
    CHECK(!s.solve(skeleton, ecs));
    CHECK(near_vec(b2->position(ecs), {2, 0}, 1e-2f));
}

TEST_CASE("15. joint constraints") {
    ik::skeleton2d skeleton;
    auto* r = skeleton.root;
    auto* b1 = r->createBone(1.0f);
    auto* b2 = b1->createBone(1.0f);
    const float pi = 3.14159265f;
    b2->setConstraints({-0.1f}, {0.1f}); // nearly straight relative to parent
    skeleton.topologicalSort();

    ik::ecs2d ecs(3);
    skeleton.init_straight_pose(ecs, {0, 0});

    ik::solver2d s;
    s.setMaxIterations(32).setTolerance(1e-3f);
    s.addTarget(b2, {1.0f, 1.0f});
    s.solve(skeleton, ecs);

    ik::vec2 parent_dir = b1->position(ecs) - r->position(ecs);
    ik::vec2 dir = b2->position(ecs) - b1->position(ecs);
    float local = ik::wrap_angle(ik::angle(dir) - ik::angle(parent_dir));
    CHECK(local >= -0.15f);
    CHECK(local <= 0.15f);
    (void)pi;
}

TEST_CASE("16. force-based step moves gradually") {
    ik::skeleton2d skeleton;
    auto* r = skeleton.root;
    auto* b1 = r->createBone(1.0f);
    auto* b2 = b1->createBone(1.0f);
    skeleton.topologicalSort();

    ik::ecs2d ecs(3);
    skeleton.init_straight_pose(ecs, {0, 0});
    ik::vec2 start = b2->position(ecs);
    ik::vec2 target{1.0f, 1.0f};

    ik::solver2d s;
    s.setForce(0.25f).addTarget(b2, target);
    s.step(skeleton, ecs, 1.0f);

    float d0 = distance(start, target);
    float d1 = distance(b2->position(ecs), target);
    CHECK(d1 < d0);
    CHECK(d1 > 1e-3f);
}

TEST_CASE("17. solver trace recording") {
    ik::skeleton2d skeleton;
    auto* r = skeleton.root;
    auto* b1 = r->createBone(1.0f);
    auto* b2 = b1->createBone(1.0f);
    skeleton.topologicalSort();

    ik::ecs2d ecs(3);
    skeleton.init_straight_pose(ecs, {0, 0});

    ik::solver2d s;
    s.setKeepTrace(true).setMaxIterations(3).addTarget(b2, {1.0f, 1.0f});
    s.solve(skeleton, ecs);

    CHECK(s.traceSize() >= 1u);
    auto* first = s.getTrace(0);
    CHECK(first);
    CHECK_EQ(first->size(), 3u);
}

TEST_CASE("18. 3d ik reaches target") {
    ik::skeleton3d skeleton;
    auto* r = skeleton.root;
    auto* b1 = r->createBone(1.0f);
    auto* b2 = b1->createBone(1.0f);
    skeleton.topologicalSort();

    ik::ecs3d ecs(3);
    skeleton.init_straight_pose(ecs, {0, 0, 0});

    ik::vec3 target{0.5f, 0.5f, 0.5f};
    ik::solver3d s;
    s.setMaxIterations(32).setTolerance(1e-3f).addTarget(b2, target);
    CHECK(s.solve(skeleton, ecs));
    CHECK(near_vec(b2->position(ecs), target, 1e-2f));
    CHECK(near(distance(b1->position(ecs), r->position(ecs)), 1.0f, 1e-2f));
    CHECK(near(distance(b2->position(ecs), b1->position(ecs)), 1.0f, 1e-2f));
}

TEST_CASE("19. 3d forward kinematics yaw pitch") {
    ik::skeleton3d skeleton;
    auto* r = skeleton.root;
    auto* b1 = r->createBone(1.0f);
    auto* b2 = b1->createBone(1.0f);
    skeleton.topologicalSort();

    ik::ecs3d ecs(3);
    r->position(ecs) = {0, 0, 0};
    r->rotation(ecs) = {0, 0};
    b1->rotation(ecs) = {0, 0};                    // along +X
    b2->rotation(ecs) = {0, 3.14159265f / 2.0f};   // pitch +90 deg -> +Z in parent frame

    skeleton.forward_kinematics(ecs);

    CHECK(near_vec(b1->position(ecs), {1, 0, 0}, 1e-3f));
    CHECK(near_vec(b2->position(ecs), {1, 0, 1}, 1e-3f));
    CHECK(near_vec(b1->orientation(ecs), {1, 0, 0}, 1e-3f));
    CHECK(near_vec(b2->orientation(ecs), {0, 0, 1}, 1e-3f));
}

TEST_CASE("20. 3d joint yaw pitch constraints") {
    ik::skeleton3d skeleton;
    auto* r = skeleton.root;
    auto* b1 = r->createBone(1.0f);
    auto* b2 = b1->createBone(1.0f);
    // Almost straight: tiny yaw/pitch allowed
    b2->setConstraints({-0.1f, -0.1f}, {0.1f, 0.1f});
    skeleton.topologicalSort();

    ik::ecs3d ecs(3);
    skeleton.init_straight_pose(ecs, {0, 0, 0});

    ik::solver3d s;
    s.setMaxIterations(48).setTolerance(1e-3f);
    s.addTarget(b2, {0.0f, 1.0f, 1.0f});
    s.solve(skeleton, ecs);

    auto local = b2->rotation(ecs);
    CHECK(local[0] >= -0.15f);
    CHECK(local[0] <= 0.15f);
    CHECK(local[1] >= -0.15f);
    CHECK(local[1] <= 0.15f);
    CHECK(near(distance(b2->position(ecs), b1->position(ecs)), 1.0f, 1e-2f));
}

TEST_CASE("21. 3d unreachable and force step") {
    ik::skeleton3d skeleton;
    auto* r = skeleton.root;
    auto* b1 = r->createBone(1.0f);
    auto* b2 = b1->createBone(1.0f);
    skeleton.topologicalSort();

    ik::ecs3d ecs(3);
    skeleton.init_straight_pose(ecs, {0, 0, 0});

    ik::solver3d hard;
    hard.setMaxIterations(8).addTarget(b2, {10, 0, 0});
    CHECK(!hard.solve(skeleton, ecs));
    CHECK(near_vec(b2->position(ecs), {2, 0, 0}, 1e-2f));

    skeleton.init_straight_pose(ecs, {0, 0, 0});
    ik::vec3 start = b2->position(ecs);
    ik::vec3 target{0.5f, 0.5f, 0.5f};
    ik::solver3d soft;
    soft.setForce(0.2f).addTarget(b2, target);
    soft.step(skeleton, ecs, 1.0f);
    CHECK(distance(b2->position(ecs), target) < distance(start, target));
    CHECK(distance(b2->position(ecs), target) > 1e-3f);
}

TEST_CASE("22. 3d rotation extraction roundtrip") {
    ik::skeleton3d skeleton;
    auto* r = skeleton.root;
    auto* b1 = r->createBone(1.0f);
    skeleton.topologicalSort();

    ik::ecs3d ecs(2);
    r->position(ecs) = {0, 0, 0};
    r->orientation(ecs) = {1, 0, 0};
    b1->rotation(ecs) = {0.4f, -0.25f};
    skeleton.forward_kinematics(ecs);

    auto pos = b1->position(ecs);
    skeleton.update_rotations(ecs);
    CHECK(near(b1->rotation(ecs)[0], 0.4f, 1e-3f));
    CHECK(near(b1->rotation(ecs)[1], -0.25f, 1e-3f));
    CHECK(near_vec(b1->position(ecs), pos, 1e-4f));
}
