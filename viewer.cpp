#include <SFML/Graphics.hpp>
#include "ik.hpp"
#include <vector>

using namespace ik;
using namespace std;

static skeleton2d* sk;
static vector<bone2d*> createSkeleton() {
    sk = new skeleton2d();
    auto* r = sk->root;
    auto* b1 = r->createBone();
    auto* b2 = b1->createBone();
    return sk->topologicalSort();
}

static void createInitState(ecs2d& ecs) {
    
}

static void updateState(ecs2d& ecs) {
    
}
 
static void renderBone(vector<bone2d*>& bones, ecs2d& ecs, sf::RenderWindow& window) {
    for (auto bone : bones) {
        auto p = bone->position(ecs);
        auto r = bone->rotation(ecs);
        sf::RectangleShape line(sf::Vector2f(150.f, 10.f));
        line.rotate(r[0]);
        line.setPosition(p[0], p[1]);
        window.draw(line);
    }
}

static bool lock_click; // Create a bool variable for locking the click.

int main()
{
    auto bones = createSkeleton();
    ecs2d ecs(2);

    // Create the main window
    sf::RenderWindow window(sf::VideoMode(800, 600), "SFML window");
    // Create a graphical text to display
    // Start the game loop
    while (window.isOpen())
    {
        // Process events
        sf::Event event;
        while (window.pollEvent(event))
        {
            // Close window: exit
            if (event.type == sf::Event::Closed)
                window.close();

            if (event.type == sf::Event::MouseButtonPressed) //Mouse button Pressed
            {
                if (event.mouseButton.button == sf::Mouse::Left && lock_click != true) //specifies
                {
                    /* your code here*/
                    lock_click = true; //And now, after all your code, this will lock the loop and not print "lmb" in a x held time. 
                    /* or here idk */
                }   
            }

            if (event.type == sf::Event::MouseButtonReleased) //Mouse button Released now.
            {
                if (event.mouseButton.button == sf::Mouse::Left) //specifies the held button       again. (You can make a variable for that if you want)
                {
                    lock_click = false; //unlock when the button has been released.
                }
            } //Released Scope
        }
        // Clear screen
        window.clear();

        renderBone(bones, ecs, window);

        // Update the window
        window.display();
    }
    return EXIT_SUCCESS;
}