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

    bodyA->m_linearVelocity.Zero();
    bodyB->m_linearVelocity.Zero();

    // Move the colliding objects apart
    float tA = bodyA->m_inverseMass / (bodyA->m_inverseMass + bodyB->m_inverseMass);
    float tB = bodyB->m_inverseMass / (bodyA->m_inverseMass + bodyB->m_inverseMass);

    Vec3 ds = contact.ptOnB_WorldSpace - contact.ptOnA_WorldSpace;
    bodyA->m_position += ds * tA;
    bodyB->m_position -= ds * tB;
}

