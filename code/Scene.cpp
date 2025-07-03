//
//  Scene.cpp
//
#include "Scene.h"
#include "Physics/Contact.h"
#include "Physics/Intersections.h"
#include "Physics/Broadphase.h"

#include <algorithm>

/*
========================================================================================================

Scene

========================================================================================================
*/

/*
====================================================
Scene::~Scene
====================================================
*/
Scene::~Scene() {
    for ( size_t i = 0; i < m_bodies.size(); i++ ) {
        delete m_bodies[ i ].m_shape;
    }
    m_bodies.clear();
}

/*
====================================================
Scene::Reset
====================================================
*/
void Scene::Reset() {
    for ( size_t i = 0; i < m_bodies.size(); i++ ) {
        delete m_bodies[ i ].m_shape;
    }
    m_bodies.clear();

    Initialize();
}

/*
====================================================
Scene::Initialize
====================================================
*/
void Scene::Initialize() {
    // Dynamic bodies
    for (int x = 0; x < 6; ++x)
    {
        for (int y = 0; y < 6; ++y)
        {
            float radius = 0.5f;
            float xx = (x - 3.0f) * radius * 2.2f;
            float yy = (y - 3.0f) * radius * 2.2f;
            Body body;
            body.m_position = Vec3(xx, yy, 10.0f);
            body.m_orientation = Quat(0, 0, 0, 1);
            body.m_linearVelocity.Zero();
            body.m_inverseMass = 1.0f;
            body.m_elasticity = 0.5f;
            body.m_friction = 0.5f;
            body.m_shape = new ShapeSphere(radius);
            m_bodies.push_back(body);
        }
    }

    // Floor bodies
    for (int x = 0; x < 3; ++x)
    {
        for (int y = 0; y < 3; ++y)
        {
            float radius = 80.0f;
            float xx = (x - 1.5f) * radius * 0.25f;
            float yy = (y - 1.5f) * radius * 0.25f;
            Body body;
            body.m_position = Vec3(xx, yy, -radius);
            body.m_orientation = Quat(0, 0, 0, 1);
            body.m_linearVelocity.Zero();
            body.m_inverseMass = 0.0f;
            body.m_elasticity = 0.9f;
            body.m_friction = 0.5f;
            body.m_shape = new ShapeSphere(radius);
            m_bodies.push_back(body);
        }
    }

#if 0
    Body body1;
    body1.m_position = Vec3(-3, 0, 3);
    body1.m_orientation = Quat(0, 0, 0, 1);
    body1.m_linearVelocity = Vec3(10, 0, 0);
    body1.m_inverseMass = 1.0f;
    body1.m_elasticity = 0.2f;
    body1.m_friction = 0.5f;
    body1.m_shape = new ShapeSphere(0.5f);
    m_bodies.push_back(body1);

    Body body2;
    body2.m_position = Vec3(0, 0, 2.7);
    body2.m_orientation = Quat(0, 0, 0, 1);
    body2.m_linearVelocity = Vec3(0, 0, 0);
    body2.m_inverseMass = 0.0f;
    body2.m_elasticity = 0.0f;
    body2.m_friction = 0.5f;
    body2.m_shape = new ShapeSphere(0.5f);
    m_bodies.push_back(body2);

    // Ground
    Body ground;
    ground.m_position = Vec3(0, 0, -1001);
    ground.m_orientation = Quat(0, 0, 0, 1);
    ground.m_inverseMass = 0.0f;
    ground.m_elasticity = 1.0f;
    ground.m_friction = 0.5f;
    ground.m_shape = new ShapeSphere(1000.0f);
    m_bodies.push_back(ground);
#endif
}

/*
====================================================
Scene::Update
====================================================
*/
void Scene::Update(const float dt_sec)
{
    for (Body& body : m_bodies)
    {
        // Apply gravity as an impulse
        // I = dp, F = dp/dt => dp = F * dt => I = F * dt
        // F = mgs
        float mass = 1.0f / body.m_inverseMass;
        Vec3 gravityImpulse = Vec3(0, 0, -9.81f) * mass * dt_sec;
        body.ApplyLinearImpulse(gravityImpulse);
    }

    // Broad-phase collision detection
    std::vector<collisionPair_t> collisionPairs;
    BroadPhase(m_bodies.data(), m_bodies.size(), collisionPairs, dt_sec);

    // Narrow-phase collision detection and response
    int numContacts = 0;
    int maxContacts = m_bodies.size() * (m_bodies.size() - 1) / 2;
    contact_t *contacts = (contact_t *)alloca(maxContacts * sizeof(contact_t));

    for (const collisionPair_t &pair : collisionPairs)
    {
        Body& bodyA = m_bodies[pair.a];
        Body& bodyB = m_bodies[pair.b];

        if (bodyA.m_inverseMass == 0.0f && bodyB.m_inverseMass == 0.0f)
        {
            // Both bodies are static, no response needed
            continue;
        }

        contact_t contact;
        if (Intersect(&bodyA, &bodyB, dt_sec, contact))
        {
            contacts[numContacts++] = contact;
        }
    }

    if (numContacts > 1)
    {
        std::sort(contacts, contacts + numContacts);
    }

    float accumulatedTime = 0.0f;
    for (int i = 0; i < numContacts; ++i)
    {
        contact_t& contact = contacts[i];
        float dt = contact.timeOfImpact - accumulatedTime;

        for (size_t j = 0; j < m_bodies.size(); ++j)
        {
            m_bodies[j].Update(dt);
        }

        ResolveContact(contact);
        accumulatedTime += dt;
    }

    float timeRemaining = dt_sec - accumulatedTime;
    if (timeRemaining > 0.0f)
    {
        for (Body& body : m_bodies)
        {
            body.Update(timeRemaining);
        }
    }
}
