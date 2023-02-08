#include <SFML/Graphics.hpp>
int main()
{
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
        }
        // Clear screen
        window.clear();

        sf::RectangleShape line(sf::Vector2f(150.f, 10.f));
        line.rotate(45.f);
        line.setPosition(100.f, 100.f);
        window.draw(line);

        // Update the window
        window.display();
    }
    return EXIT_SUCCESS;
}