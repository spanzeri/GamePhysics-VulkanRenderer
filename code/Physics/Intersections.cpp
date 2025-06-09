//
//  Intersections.cpp
//
#include "Intersections.h"
#include "GJK.h"

/*
====================================================
Intersect
====================================================
*/
bool Intersect( Body * bodyA, Body * bodyB, contact_t & contact ) {
    assert(bodyA);
    assert(bodyB);

    Vec3 ab = bodyB->m_position - bodyA->m_position;
    contact.normal = ab;
    contact.normal.Normalize();

    assert(bodyA->m_shape->GetType() == Shape::SHAPE_SPHERE);
    assert(bodyB->m_shape->GetType() == Shape::SHAPE_SPHERE);

    ShapeSphere* sphereA = (ShapeSphere*)bodyA->m_shape;
    ShapeSphere* sphereB = (ShapeSphere*)bodyB->m_shape;

    contact.bodyA = bodyA;
    contact.bodyB = bodyB;
    contact.ptOnA_WorldSpace = bodyA->m_position + contact.normal * sphereA->m_radius;
    contact.ptOnB_WorldSpace = bodyB->m_position - contact.normal * sphereB->m_radius;

    float radiusAB = sphereA->m_radius + sphereB->m_radius;
    float lenghtSqr = ab.GetLengthSqr();

    if (lenghtSqr <= (radiusAB * radiusAB)) {
        return true;
    }

    return false;
}

/*
====================================================
Intersect
====================================================
*/
bool Intersect( Body * bodyA, Body * bodyB, const float dt, contact_t & contact ) {
    // TODO: Add Code

    return false;
}

