#include <Box2D/Box2D.h>
#include <cmath>

b2WeldJoint* weldJoint = nullptr;
const float maxForce = 1000.0f;

class MyListener : public b2ContactListener {
    void PreSolve(b2Contact* contact, const b2Manifold* oldManifold) {
        b2WorldManifold worldManifold;
        contact->GetWorldManifold(&worldManifold);

        float impactMagnitude = b2Dot(worldManifold.normal, contact->GetFixtureA()->GetBody()->GetLinearVelocity());
        if (abs(impactMagnitude) > maxForce && weldJoint) {
            // If the impact magnitude exceeds the max force and the weld joint exists, destroy the joint
            weldJoint->GetBodyA()->GetWorld()->DestroyJoint(weldJoint);
            weldJoint = nullptr;
        }
    }
};

int main() {
    // Step 1: Create a b2World
    b2Vec2 gravity(0.0f, -10.0f);
    b2World world(gravity);

    // Step 2: Create the bodies
    b2BodyDef bodyDef;
    bodyDef.type = b2_dynamicBody;

    // Create the body shapes
    b2PolygonShape boxShape;
    boxShape.SetAsBox(1.0f, 1.0f);

    // Create the body fixtures
    b2FixtureDef fixtureDef;
    fixtureDef.shape = &boxShape;
    fixtureDef.density = 1.0f;

    // Create and add fixtures to bodies
    bodyDef.position.Set(0.0f, 20.0f);
    b2Body* bodyA = world.CreateBody(&bodyDef);
    bodyA->CreateFixture(&fixtureDef);

    bodyDef.position.Set(0.0f, 10.0f);
    b2Body* bodyB = world.CreateBody(&bodyDef);
    bodyB->CreateFixture(&fixtureDef);

    // Step 3: Create a weld joint
    b2WeldJointDef weldJointDef;
    weldJointDef.Initialize(bodyA, bodyB, bodyA->GetWorldCenter());
    weldJoint = (b2WeldJoint*)world.CreateJoint(&weldJointDef);

    // Step 4: Set the contact listener
    MyListener myListener;
    world.SetContactListener(&myListener);

    // Step 5: Run the physics simulation
    for (int32 i = 0; i < 60.0f; ++i) {
        world.Step(1.0f / 60.0f, 6, 2);  // 60 Hz
    }

    return 0;
}
