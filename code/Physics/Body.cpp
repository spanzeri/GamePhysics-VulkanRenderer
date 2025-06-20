//
//  Body.cpp
//
#include "Body.h"

#include "Shapes.h"

/*
====================================================
Body::Body
====================================================
*/
Body::Body()
    : m_position { 0.0f }
    , m_orientation { 0.0f, 0.0f, 0.0f, 1.0f }
    , m_linearVelocity { 0.0f }
    , m_inverseMass { 1.0f }
    , m_elasticity { 0.5f }
    , m_friction { 0.1f }
    , m_shape { nullptr }
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

Mat3 Body::GetInverseInertiaTensorBodySpace() const
{
    return m_shape->InertiaTensor().Inverse() * m_inverseMass;
}

Mat3 Body::GetInverseInertiaTensorWorldSpace() const
{
    Mat3 inverseInertiaTensor = GetInverseInertiaTensorBodySpace();
    Mat3 rotationMatrix = m_orientation.ToMat3();
    return rotationMatrix * inverseInertiaTensor * rotationMatrix.Transpose();
}

void Body::ApplyImpulse(Vec3 impulse, Vec3 position)
{
    if (m_inverseMass == 0.0f) {
        return; // No effect if the body is static
    }

    // position is the world space location where the impulse is applied.
    // impulse is the world space impulse vector.
    ApplyLinearImpulse(impulse);

    Vec3 com = GetCenterOfMassWorldSpace();
    Vec3 r = position - com;
    Vec3 dL = r.Cross(impulse);
    ApplyAngularImpulse(dL);
}

void Body::Update(float dt_sec)
{
    m_position += m_linearVelocity * dt_sec;

    Vec3 com = GetCenterOfMassWorldSpace();
    Vec3 com_to_position = m_position - com;

    Mat3 orientation = m_orientation.ToMat3();
    Mat3 inertiaTensor = orientation * m_shape->InertiaTensor() * orientation.Transpose();
    Vec3 alpha = inertiaTensor.Inverse() * m_angularVelocity.Cross(inertiaTensor * m_angularVelocity);
    m_angularVelocity += alpha * dt_sec;

    // Update orientation based on angular velocity
    Vec3 dAngle = m_angularVelocity * dt_sec;
    Quat dq = Quat(dAngle, dAngle.GetMagnitude());
    m_orientation = m_orientation * dq;
    m_orientation.Normalize();

    // Now get he new model position
    m_position = com + m_orientation.RotatePoint(com_to_position);
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

void Body::ApplyAngularImpulse(Vec3 impulse)
{
    if (m_inverseMass == 0.0f) {
        return; // No effect if the body is static
    }

    // L= I * w = r * p
    // dL = I * dw = r * J
    // => dw = I^-1 * (r x J)
    m_angularVelocity += GetInverseInertiaTensorWorldSpace() * impulse;

    const float max_angularSpeed = 30.0f; // Limit angular speed to prevent instability
    if (m_angularVelocity.GetLengthSqr() > (max_angularSpeed * max_angularSpeed)) {
        m_angularVelocity.Normalize();
        m_angularVelocity *= max_angularSpeed;
    }
}
