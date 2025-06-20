//
//  Contact.cpp
//
#include "Contact.h"

/*
====================================================
ResolveContact
====================================================
*/
void ResolveContact( contact_t & contact )
{
    // Solving conservation of momentum for linear and angular momentum gives us:
    // J = (1 + e) * (vA - vB) . n / (mA^-1 + mB^-1 + (I_A^-1 * rA x n) . rA + (I_B^-1 * rB x n) . rB)


    // Resolve such that the com of the system remains at the same position
    Body* bodyA = contact.bodyA;
    Body* bodyB = contact.bodyB;
    assert(bodyA);
    assert(bodyB);

    Vec3 ptOnA = contact.ptOnA_WorldSpace;
    Vec3 ptOnB = contact.ptOnB_WorldSpace;

    float inverseMassA = bodyA->m_inverseMass;
    float inverseMassB = bodyB->m_inverseMass;

    float elasticityA = bodyA->m_elasticity;
    float elasticityB = bodyB->m_elasticity;
    float elasticity = elasticityA * elasticityB;

    Mat3 inverseWorldInertiaTensorA = bodyA->GetInverseInertiaTensorWorldSpace();
    Mat3 inverseWorldInertiaTensorB = bodyB->GetInverseInertiaTensorWorldSpace();

    Vec3 normal = contact.normal;

    Vec3 ra = ptOnA - bodyA->GetCenterOfMassWorldSpace();
    Vec3 rb = ptOnB - bodyB->GetCenterOfMassWorldSpace();

    Vec3 angularJA = (inverseWorldInertiaTensorA * ra.Cross(normal)).Cross(ra);
    Vec3 angularJB = (inverseWorldInertiaTensorB * rb.Cross(normal)).Cross(rb);
    float angularFactor = (angularJA + angularJB).Dot(normal);

    Vec3 velA = bodyA->m_linearVelocity + bodyA->m_angularVelocity.Cross(ra);
    Vec3 velB = bodyB->m_linearVelocity + bodyB->m_angularVelocity.Cross(rb);

    // Calculate the collision impulse
    Vec3 vab = velA - velB;
    float impulseJ = (1.0f + elasticity) * vab.Dot(normal) /
                     (inverseMassA + inverseMassB + angularFactor);
    Vec3 impulse = normal * impulseJ;

    // Apply the impulse to the bodies
    bodyA->ApplyImpulse(impulse * -1.0f, ptOnA);
    bodyB->ApplyImpulse(impulse, ptOnB);

    //
    // Calculate the impulse caused by friction
    //

    float frictionA = bodyA->m_friction;
    float frictionB = bodyB->m_friction;
    float friction = frictionA * frictionB;

    Vec3 velNorm = normal * normal.Dot(vab);
    Vec3 velTangent = vab - velNorm;

    Vec3 relativeVelTangent = velTangent;
    relativeVelTangent.Normalize();

    Vec3 inertiaA = (inverseWorldInertiaTensorA * ra.Cross(relativeVelTangent)).Cross(ra);
    Vec3 inertiaB = (inverseWorldInertiaTensorB * rb.Cross(relativeVelTangent)).Cross(rb);
    float inertiaFactor = (inertiaA + inertiaB).Dot(relativeVelTangent);

    float reducedMass = 1.0f / (inverseMassA + inverseMassB + inertiaFactor);
    Vec3 frictionImpulse = relativeVelTangent * reducedMass * friction;

    bodyA->ApplyImpulse(frictionImpulse * -1.0f, ptOnA);
    bodyB->ApplyImpulse(frictionImpulse, ptOnB);

    //
    // Separate the colliding bodies
    //
    float tA = bodyA->m_inverseMass / (bodyA->m_inverseMass + bodyB->m_inverseMass);
    float tB = bodyB->m_inverseMass / (bodyA->m_inverseMass + bodyB->m_inverseMass);

    Vec3 ds = contact.ptOnB_WorldSpace - contact.ptOnA_WorldSpace;
    bodyA->m_position += ds * tA;
    bodyB->m_position -= ds * tB;
}
