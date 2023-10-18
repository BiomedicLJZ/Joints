#include <Box2D/Box2D.h>

#include <cstdio>

int main() {
    // Step 1: Create a b2World
    b2Vec2 gravity(0.0f, -10.0f);
    b2World world(gravity);

    // Step 2: Create static body
    b2BodyDef staticBodyDef;
    staticBodyDef.position.Set(0.0f, 20.0f);
    b2Body* staticBody = world.CreateBody(&staticBodyDef);

    // Step 3: Create dynamic body
    b2BodyDef dynamicBodyDef;
    dynamicBodyDef.type = b2_dynamicBody;
    dynamicBodyDef.position.Set(0.0f, 15.0f);
    b2Body* dynamicBody = world.CreateBody(&dynamicBodyDef);

    // Apply shape and fixture to the dynamic body
    b2PolygonShape polygonShape;
    polygonShape.SetAsBox(1.0f, 1.0f);

    b2FixtureDef fixtureDef;
    fixtureDef.shape = &polygonShape;
    fixtureDef.density = 1.0f;
    dynamicBody->CreateFixture(&fixtureDef);

    // Step 4: Create a rope joint
    b2RopeJointDef ropeJointDef;
    ropeJointDef.bodyA = staticBody;
    ropeJointDef.bodyB = dynamicBody;
    ropeJointDef.maxLength = 5.0f;

    world.CreateJoint(&ropeJointDef);

    // Step 5: Run the simulation
    for (int32 i = 0; i < 60.0f; ++i) {
        world.Step(1.0f / 60.0f, 6, 2);
        b2Vec2 position = dynamicBody->GetPosition();
        printf("Dynamic Body Position: %4.2f %4.2f\n", position.x, position.y);
    }

    return 0;
}
