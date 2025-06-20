//
//  Body.h
//
#pragma once
#include "../Math/Vector.h"
#include "../Math/Quat.h"
#include "../Math/Matrix.h"

class Shape;

/*
====================================================
Body
====================================================
*/
class Body {
public:
    Body();

    Vec3    m_position;
    Quat    m_orientation;
    Vec3    m_linearVelocity;
    Vec3    m_angularVelocity;
    float   m_inverseMass;
    float   m_elasticity; // 0.0f = inelastic, 1.0f = elastic
    float   m_friction; // 0.0f = no friction, 1.0f = full friction

    Shape*  m_shape;

    Vec3    GetCenterOfMassWorldSpace() const;
    Vec3    GetCenterOfMassModelSpace() const;

    Vec3    WorldSpaceToBodySpace(Vec3 pt ) const;
    Vec3    BodySpaceToWorldSpace(Vec3 pt ) const;

    Mat3    GetInverseInertiaTensorBodySpace() const;
    Mat3    GetInverseInertiaTensorWorldSpace() const;

    void    Update(float dt_sec);

    void    ApplyImpulse(Vec3 impulse, Vec3 position);
    void    ApplyLinearImpulse(Vec3 impulse);
    void    ApplyAngularImpulse(Vec3 impulse);
};
