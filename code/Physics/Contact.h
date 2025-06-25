//
//  Contact.h
//
#pragma once
#include "Body.h"

struct contact_t {
    Vec3 ptOnA_WorldSpace;
    Vec3 ptOnB_WorldSpace;
    Vec3 ptOnA_LocalSpace;
    Vec3 ptOnB_LocalSpace;

    Vec3 normal;                // In World Space coordinates
    float separationDistance;   // positive when non-penetrating, negative when penetrating
    float timeOfImpact;

    Body * bodyA;
    Body * bodyB;
};

void ResolveContact( contact_t & contact );

inline int operator <=>( const contact_t &a, const contact_t &b)
{
    if (a.timeOfImpact < b.timeOfImpact) return -1;
    if (a.timeOfImpact > b.timeOfImpact) return 1;
    return 0;
}

