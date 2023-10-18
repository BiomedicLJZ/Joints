#include <iostream>
#include <box2d/box2d.h>
int main() {
    b2Vec2 gravity(0.0f, 0.0f);
    b2World world(gravity);

    //Distance joint Simulation
    std::cout << "Distance joint simulation" << std::endl;

    // Create two Dynamic bodies
    b2BodyDef bodyDef;
    bodyDef.type = b2_dynamicBody;
    bodyDef.position.Set(0.0f, 10.0f);
    b2Body* body1 = world.CreateBody(&bodyDef);

    //Create the second body 4 units below the first
    bodyDef.position.Set(0.0f, 6.0f);
    b2Body* body2 = world.CreateBody(&bodyDef);

    // Create a Box shape for our bodies
    b2PolygonShape dynamicBox;
    dynamicBox.SetAsBox(1.0f, 1.0f);

    // Create fixture definition
    b2FixtureDef fixtureDef;
    fixtureDef.shape = &dynamicBox;
    fixtureDef.density = 1.0f;

    // Add the shape to the body
    body1->CreateFixture(&fixtureDef);
    body2->CreateFixture(&fixtureDef);

    b2DistanceJointDef jointDis;
    jointDis.Initialize(body1, body2, body1->GetWorldCenter(), body2->GetWorldCenter());
    float frequencyHz = 30.0f;
    float dampingRatio = 0.0f;
    b2LinearStiffness(jointDis.stiffness, jointDis.damping,
                      frequencyHz,
                      dampingRatio,
                      jointDis.bodyA, jointDis.bodyB);

    world.CreateJoint(&jointDis);

    // Set impulse to move the body1
    b2Vec2 impulse(2.0f, -1.0f);
    body1->ApplyLinearImpulse(impulse, body1->GetWorldCenter(), true);

    // Set the world step
    float timeStep = 1.0f / 60.0f;

    int velocityIterations = 6;
    int positionIterations = 2;

    for (int i = 0; i < 60; i++) {
        world.Step(timeStep, velocityIterations, positionIterations);
        b2Vec2 position1 = body1->GetPosition();
        b2Vec2 position2 = body2->GetPosition();

        std::cout << i << " " << position1.x << " " << position1.y << " " << position2.x << " " << position2.y << std::endl;
    }
return 0;
}
