#include <Box2D/Box2D.h>
#include <cstdio>

int main() {
    // Step 1: Create a b2World
    b2Vec2 gravity(0.0f, -10.0f);
    b2World world(gravity);

    // Step 2: Create the bodies
    b2BodyDef bodyDef;

    // Static body
    bodyDef.position.Set(0.0f, 10.0f);
    b2Body* staticBody = world.CreateBody(&bodyDef);

    // Dynamic bodies
    bodyDef.type = b2_dynamicBody;
    bodyDef.position.Set(-5.0f, 10.0f);
    b2Body* dynamicBodyA = world.CreateBody(&bodyDef);
    bodyDef.position.Set(5.0f, 10.0f);
    b2Body* dynamicBodyB = world.CreateBody(&bodyDef);

    // Step 3: Create shapes and fixtures for dynamic bodies
    b2PolygonShape boxShape;
    boxShape.SetAsBox(2.0f, 2.0f);

    b2FixtureDef boxFixture;
    boxFixture.shape = &boxShape;
    boxFixture.density = 1.0f;

    dynamicBodyA->CreateFixture(&boxFixture);
    dynamicBodyB->CreateFixture(&boxFixture);

    // Step 4: Create revolute joints and attach them to the bodies
    b2RevoluteJointDef jointDefA;
    jointDefA.Initialize(staticBody, dynamicBodyA, dynamicBodyA->GetPosition());
    b2RevoluteJoint* jointA = (b2RevoluteJoint*)world.CreateJoint(&jointDefA);

    b2RevoluteJointDef jointDefB;
    jointDefB.Initialize(staticBody, dynamicBodyB, dynamicBodyB->GetPosition());
    b2RevoluteJoint* jointB = (b2RevoluteJoint*)world.CreateJoint(&jointDefB);

    // Step 5: Create gear joint and attach it to the revolute joints
    b2GearJointDef gearJointDef;
    gearJointDef.joint1 = jointA;
    gearJointDef.joint2 = jointB;
    gearJointDef.ratio = jointB->GetJointAngle() / jointA->GetJointAngle();
    world.CreateJoint(&gearJointDef);

    // Step 6: Run the physics simulation
    for (int32 i = 0; i < 300.0f; ++i) {
        world.Step(1.0f / 60.0f, 6, 2);  // 60 Hz

        printf("Body A Angle: %4.2f radians\n", dynamicBodyA->GetAngle());
        printf("Body B Angle: %4.2f radians\n\n", dynamicBodyB->GetAngle());
    }

    return 0;
}
