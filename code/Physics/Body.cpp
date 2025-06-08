//
//  Body.cpp
//
#include "Body.h"

/*
====================================================
Body::Body
====================================================
*/
Body::Body()
    : m_position{ 0.0f }
    , m_orientation{ 0.0f, 0.0f, 0.0f, 1.0f }
    , m_linearVelocity{ 0.0f }
    , m_inverseMass( 1.0f )
    , m_shape( nullptr )
{
}

Vec3 Body::GetCenterOfMassWorldSpace() const
{
    Vec3 com = m_shape->GetCenterOfMass();
    Vec3 pos = m_position + m_orientation.RotatePoint(com);
    return pos;
}

Vec3 Body::GetCenterOfMassModelSpace() const
{
    return m_shape->GetCenterOfMass();
}

Vec3 Body::WorldSpaceToBodySpace(Vec3 pt) const
{
    Vec3 tmp = pt - GetCenterOfMassWorldSpace();
    Quat invOrient = m_orientation.Inverse();
    return invOrient.RotatePoint(tmp);
}

Vec3 Body::BodySpaceToWorldSpace(Vec3 pt) const
{
    return GetCenterOfMassWorldSpace() + m_orientation.RotatePoint(pt);
}

void Body::ApplyLinearImpulse(Vec3 impulse)
{
    if (m_inverseMass == 0.0f) {
        return; // No effect if the body is static
    }

    // p = mv
    // dp = m * dv = J
    // => dv = J / m
    m_linearVelocity += impulse * m_inverseMass;
}

