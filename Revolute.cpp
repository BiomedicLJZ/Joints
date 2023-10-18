#include <SFML/Graphics.hpp>
#include <Box2D/Box2D.h>
#include <cstdio>

// Conversion factor for converting from meters to pixels and vice versa.
const float M2P = 20.0f;
const float P2M = 1.0f / M2P;


int main() {
    // Create SFML window
    sf::RenderWindow window(sf::VideoMode(800, 600), "SFML & Box2D Simulation");

    // Create SFML shapes for pivot and bob
    sf::CircleShape pivotShape(5.0f);
    pivotShape.setOrigin(5.0f, 5.0f);
    pivotShape.setPosition(400.0f, 300.0f);

    sf::CircleShape bobShape(20.0f);
    bobShape.setOrigin(20.0f, 20.0f);
    bobShape.setFillColor(sf::Color::Red);

    // Step 1: Create a b2World
    b2Vec2 gravity(0.0f, -10.0f);
    b2World world(gravity);

    // Step 2: Create a static body (the pivot point of the pendulum)
    b2BodyDef pivotBodyDef;
    pivotBodyDef.position.Set(400.0f * P2M, (600.0f - 300.0f) * P2M); // Centered pivot
    b2Body* pivotBody = world.CreateBody(&pivotBodyDef);

    // Step 3: Create a dynamic body (the pendulum bob)
    b2BodyDef bobBodyDef;
    bobBodyDef.type = b2_dynamicBody;
    bobBodyDef.position.Set(pivotBodyDef.position.x, pivotBodyDef.position.y - 2.0f); // Slightly below the pivot
    b2Body* bobBody = world.CreateBody(&bobBodyDef);

    // Apply shape and fixture to the dynamic body
    b2CircleShape circleShape;
    circleShape.m_radius = bobShape.getRadius() * P2M;

    b2FixtureDef bobFixtureDef;
    bobFixtureDef.shape = &circleShape;
    bobFixtureDef.density = 1.0f;
    bobBody->CreateFixture(&bobFixtureDef);

    // Step 4: Connect the bodies using a revolute joint
    b2RevoluteJointDef revoluteJointDef;
    revoluteJointDef.Initialize(pivotBody, bobBody, pivotBody->GetPosition());
    revoluteJointDef.enableLimit = true;
    revoluteJointDef.lowerAngle = -0.25f * b2_pi;
    revoluteJointDef.upperAngle = 0.25f * b2_pi;
    revoluteJointDef.enableMotor = true;
    revoluteJointDef.motorSpeed = 0.0f;
    revoluteJointDef.maxMotorTorque = 10.0f;


    world.CreateJoint(&revoluteJointDef);


    bobBody->ApplyLinearImpulse(b2Vec2(1.0f, 0.0f), bobBody->GetWorldCenter(), true);

    // Running the physics simulation and listening for close event
    while (window.isOpen())
    {
        sf::Event event{};
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        // Get position of bob
        b2Vec2 position = bobBody->GetPosition();

        // Update SFML shape position
        bobShape.setPosition(position.x * M2P, (600 - position.y * M2P));

        // Step the Box2D simulation
        world.Step(1.0f / 60.0f, 6, 2);

        // Clear the window
        window.clear(sf::Color::White);

        // Draw the pivot and bob
        window.draw(pivotShape);
        window.draw(bobShape);

        // Display the frame
        window.display();
    }

    return 0;
}
