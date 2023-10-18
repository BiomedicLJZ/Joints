#include <Box2D/Box2D.h>
#include <cstdio>

int main() {
    // Step 1: Create a b2World
    b2Vec2 gravity(0.0f, -10.0f);
    b2World world(gravity);

    // Step 2: Create two dynamic bodies
    b2BodyDef bodyDef;
    bodyDef.type = b2_dynamicBody;

    b2PolygonShape boxShape;
    boxShape.SetAsBox(1.0f, 1.0f);

    b2FixtureDef fixtureDef;
    fixtureDef.shape = &boxShape;
    fixtureDef.density = 1.0f;

    // Body A
    bodyDef.position.Set(-10.0f, 10.0f);
    b2Body* bodyA = world.CreateBody(&bodyDef);
    bodyA->CreateFixture(&fixtureDef);

    // Body B
    bodyDef.position.Set(10.0f, 10.0f);
    b2Body* bodyB = world.CreateBody(&bodyDef);
    bodyB->CreateFixture(&fixtureDef);

    // Step 3: Create a pulley joint
    b2PulleyJointDef pulleyDef;
    b2Vec2 anchorA(-10.0f, 10.0f);//Punto 1
    b2Vec2 anchorB(10.0f, 10.0f);//Punto 2
    b2Vec2 groundAnchorA(-10.0f, 20.0f);//Punto 3
    b2Vec2 groundAnchorB(10.0f, 20.0f);//Punto 4
    float ratio = 1.0f;
    pulleyDef.Initialize(bodyA, bodyB, groundAnchorA, groundAnchorB, bodyA->GetWorldPoint(anchorA), bodyB->GetWorldPoint(anchorB), ratio);

    world.CreateJoint(&pulleyDef);

    // Step 4: Run the physics simulation
    for (int32 i = 0; i < 60.0f; ++i) {
        world.Step(1.0f / 60.0f, 6, 2);
        b2Vec2 positionA = bodyA->GetPosition();
        b2Vec2 positionB = bodyB->GetPosition();
        float angleA = bodyA->GetAngle();
        float angleB = bodyB->GetAngle();
        printf("BodyA: %4.2f %4.2f %4.2f\n", positionA.x, positionA.y, angleA);
        printf("BodyB: %4.2f %4.2f %4.2f\n", positionB.x, positionB.y, angleB);
    }

    return 0;
}
