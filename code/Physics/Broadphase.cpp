//
//  Broadphase.cpp
//
#include "Broadphase.h"

#include "Shapes.h"
#include <algorithm>
#include <stdlib.h>

struct PseudoBody
{
    int     id;
    float   value;
    bool    is_min;
};

bool CompareSAP(const PseudoBody &a, const PseudoBody &b)
{
    return (a.value < b.value);
}
void SortBodiesBounds(const Body *bodies, int num, PseudoBody* sortedArray, float dt_sec)
{
    Vec3 axis = Vec3(1.0f, 1.0f, 1.0f);
    axis.Normalize();

    for (int i = 0; i < num; ++i)
    {
        const Body &body = bodies[i];
        Bounds bounds = body.m_shape->GetBounds(body.m_position, body.m_orientation);

        Vec3 eps = Vec3(1e-2f, 1e-2f, 1e-2f);
        bounds.Expand(bounds.mins + body.m_linearVelocity * dt_sec - eps);
        bounds.Expand(bounds.maxs + body.m_linearVelocity * dt_sec + eps);

        sortedArray[i * 2 + 0].id = i;
        sortedArray[i * 2 + 0].value = bounds.mins.Dot(axis);
        sortedArray[i * 2 + 0].is_min = true;

        sortedArray[i * 2 + 1].id = i;
        sortedArray[i * 2 + 1].value = bounds.maxs.Dot(axis);
        sortedArray[i * 2 + 1].is_min = false;
    }

    std::sort(sortedArray, sortedArray + (num * 2), CompareSAP);
}

void BuildPairs(std::vector<collisionPair_t> &collisionPairs, const PseudoBody *sortedBodies, int num)
{
    collisionPairs.clear();

    for (int i = 0; i < num * 2; ++i)
    {
        const PseudoBody &current = sortedBodies[i];
        if (!current.is_min)
            continue;

        collisionPair_t pair;
        pair.a = current.id;

        for (int j = i + 1; j < num * 2; ++j)
        {
            const PseudoBody &next = sortedBodies[j];
            if (current.id == next.id)
                break;

            if (!next.is_min)
                continue;

            pair.b = next.id;
            collisionPairs.push_back(pair);
        }
    }
}

void SweepAndPrune1D(const Body *bodies, int num, std::vector<collisionPair_t> &collisionPairs, float dt_sec)
{
    PseudoBody *sortedBodies = (PseudoBody *)alloca(num * 2 * sizeof(PseudoBody));
    SortBodiesBounds(bodies, num, sortedBodies, dt_sec);
    BuildPairs(collisionPairs, sortedBodies, num);
}

/*
====================================================
Broad-Phase
====================================================
*/
void BroadPhase( const Body * bodies, const int num, std::vector< collisionPair_t > & finalPairs, const float dt_sec ) {
    finalPairs.clear();
    SweepAndPrune1D(bodies, num, finalPairs, dt_sec);
}
