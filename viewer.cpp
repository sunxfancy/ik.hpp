#include <SFML/Graphics.hpp>
#include "ik.hpp"
#include <cmath>
#include <vector>

using namespace ik;
using namespace std;

static skeleton2d* sk;
static bone2d* end_effector = nullptr;
static solver2d ik_solver;
static vec2 target_pos{600.f, 200.f};
static bool has_target = true;

static vector<bone2d*> createSkeleton() {
    sk = new skeleton2d();
    auto* r = sk->root;
    auto* b1 = r->createBone(120.f);
    auto* b2 = b1->createBone(100.f);
    auto* b3 = b2->createBone(80.f);
    end_effector = b3;
    return sk->topologicalSort();
}

static void createInitState(ecs2d& ecs) {
    sk->init_straight_pose(ecs, {300.f, 300.f});
    ik_solver.setMaxIterations(16).setTolerance(0.5f).setForce(0.35f);
    ik_solver.clearTargets().addTarget(end_effector, target_pos);
}

static void updateState(ecs2d& ecs) {
    if (!has_target || !end_effector) return;
    ik_solver.clearTargets().addTarget(end_effector, target_pos);
    ik_solver.step(*sk, ecs, 1.0f);
}

static void renderBone(vector<bone2d*>& bones, ecs2d& ecs, sf::RenderWindow& window) {
    for (auto* bone : bones) {
        if (!bone->parent) continue;
        auto p0 = bone->parent->position(ecs);
        auto p1 = bone->position(ecs);
        vec2 delta = p1 - p0;
        float len = length(delta);
        if (len <= 1e-3f) continue;

        sf::RectangleShape line(sf::Vector2f(len, 8.f));
        line.setOrigin(0.f, 4.f);
        line.setPosition(p0[0], p0[1]);
        line.setRotation(angle(delta) * 180.f / 3.14159265f);
        line.setFillColor(sf::Color(220, 220, 220));
        window.draw(line);

        sf::CircleShape joint(5.f);
        joint.setOrigin(5.f, 5.f);
        joint.setPosition(p1[0], p1[1]);
        joint.setFillColor(sf::Color(100, 180, 255));
        window.draw(joint);
    }

    if (has_target) {
        sf::CircleShape marker(7.f);
        marker.setOrigin(7.f, 7.f);
        marker.setPosition(target_pos[0], target_pos[1]);
        marker.setFillColor(sf::Color(255, 120, 80));
        window.draw(marker);
    }
}

static bool lock_click = false;

int main()
{
    auto bones = createSkeleton();
    ecs2d ecs(static_cast<unsigned>(bones.size()));
    createInitState(ecs);

    sf::RenderWindow window(sf::VideoMode(800, 600), "ik.hpp viewer");
    window.setFramerateLimit(60);

    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();

            if (event.type == sf::Event::MouseButtonPressed)
            {
                if (event.mouseButton.button == sf::Mouse::Left && !lock_click)
                {
                    target_pos = {static_cast<float>(event.mouseButton.x),
                                  static_cast<float>(event.mouseButton.y)};
                    has_target = true;
                    lock_click = true;
                }
            }

            if (event.type == sf::Event::MouseButtonReleased)
            {
                if (event.mouseButton.button == sf::Mouse::Left)
                {
                    lock_click = false;
                }
            }
        }

        if (sf::Mouse::isButtonPressed(sf::Mouse::Left)) {
            auto mp = sf::Mouse::getPosition(window);
            target_pos = {static_cast<float>(mp.x), static_cast<float>(mp.y)};
            has_target = true;
        }

        updateState(ecs);

        window.clear(sf::Color(30, 30, 36));
        renderBone(bones, ecs, window);
        window.display();
    }
    return EXIT_SUCCESS;
}
