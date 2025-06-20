//
//  Scene.cpp
//
#include "Scene.h"
#include "Physics/Contact.h"
#include "Physics/Intersections.h"
#include "Physics/Broadphase.h"

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
    Body body1;
    body1.m_position = Vec3( 0, 0, 10 );
    body1.m_orientation = Quat( 0, 0, 0, 1 );
    body1.m_linearVelocity = Vec3( 1, 0, 0 );
    body1.m_inverseMass = 1.0f;
    body1.m_elasticity = 0.2f;
    body1.m_friction = 0.5f;
    body1.m_shape = new ShapeSphere( 1.0f );
    m_bodies.push_back( body1 );

    Body body2;
    body2.m_position = Vec3( 0, 0, -1001 );
    body2.m_orientation = Quat( 0, 0, 0, 1 );
    body2.m_inverseMass = 0.0f;
    body2.m_elasticity = 1.0f;
    body2.m_friction = 0.5f;
    body2.m_shape = new ShapeSphere( 1000.0f );
    m_bodies.push_back( body2 );

    // TODO: Add code
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

    // Check for collisions
    for (size_t i = 0; i < m_bodies.size(); ++i)
    {
        for (size_t j = i + 1; j < m_bodies.size(); ++j)
        {
            if (m_bodies[i].m_inverseMass == 0.0f && m_bodies[j].m_inverseMass == 0.0f)
            {
                // Both bodies are static, no response needed
                continue;
            }

            contact_t contact;
            if (Intersect(&m_bodies[i], &m_bodies[j], contact))
            {
                ResolveContact(contact);
            }
        }
    }

    for (Body& body : m_bodies)
    {
        body.Update(dt_sec);
    }
}
