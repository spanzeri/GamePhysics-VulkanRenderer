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
    // Resolve such that the com of the system remains at the same position
    Body* bodyA = contact.bodyA;
    Body* bodyB = contact.bodyB;
    assert(bodyA);
    assert(bodyB);

    float inverseMassA = bodyA->m_inverseMass;
    float inverseMassB = bodyB->m_inverseMass;

    float elasticityA = bodyA->m_elasticity;
    float elasticityB = bodyB->m_elasticity;
    float elasticity = elasticityA * elasticityB;

    Vec3 normal = contact.normal;
    Vec3 vab = bodyB->m_linearVelocity - bodyA->m_linearVelocity;

    float impulseJ = (1.0f + elasticity) * vab.Dot(normal) / (inverseMassA + inverseMassB);
    Vec3 impulse = normal * impulseJ;

    // Apply the impulse to the bodies
    bodyA->ApplyLinearImpulse(impulse);
    bodyB->ApplyLinearImpulse(impulse * -1.0f);

    // Move the colliding objects apart
    float tA = bodyA->m_inverseMass / (bodyA->m_inverseMass + bodyB->m_inverseMass);
    float tB = bodyB->m_inverseMass / (bodyA->m_inverseMass + bodyB->m_inverseMass);

    Vec3 ds = contact.ptOnB_WorldSpace - contact.ptOnA_WorldSpace;
    bodyA->m_position += ds * tA;
    bodyB->m_position -= ds * tB;
}

