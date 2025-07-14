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

    assert(bodyA->m_shape->GetType() == Shape::Type::Sphere);
    assert(bodyB->m_shape->GetType() == Shape::Type::Sphere);

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

bool RaySphere(Vec3 rayStart, Vec3 rayDir, Vec3 sphereCenter, float sphereRadius, float& t1, float& t2)
{
    Vec3 m = sphereCenter - rayStart;
    float a = rayDir.Dot(rayDir);
    float b = m.Dot(rayDir);
    float c = m.Dot(m) - sphereRadius * sphereRadius;

    float delta = b * b - a * c;
    float invA = 1.0f / a;

    if (delta < 0.0f) {
        return false;
    }

    float deltaSqrt = sqrtf(delta);
    t1 = (b - deltaSqrt) * invA;
    t2 = (b + deltaSqrt) * invA;

    return true;
}

bool SphereSphereDynamic(const ShapeSphere* sphereA, Vec3 posA, Vec3 velA,
                         const ShapeSphere* sphereB, Vec3 posB, Vec3 velB,
                         float dt,
                         Vec3& pointOnA, Vec3& pointOnB, float& timeOfImpact)
{
    Vec3 relativeVelocity = velA - velB;

    Vec3 startPtA = posA;
    Vec3 endPtA = posA + relativeVelocity * dt;
    Vec3 rayDir = endPtA - startPtA;

    float t1 = 0.0f, t2 = 0.0f;
    if (rayDir.GetLengthSqr() < (1e-3f * 1e-3f)) {
        // Ray too short, check if already intersecting
        Vec3 ab = posB - posA;
        float radius = sphereA->m_radius + sphereB->m_radius + 0.001f;
        if (ab.GetLengthSqr() > (radius * radius))
        {
            return false;
        }
    }
    else if (!RaySphere(startPtA, rayDir, posB, sphereA->m_radius + sphereB->m_radius, t1, t2)) {
        return false;
    }

    t1 *= dt;
    t2 *= dt;
    if (t2 < 0.0f) { return false; }

    timeOfImpact = (t1 < 0.0f) ? 0.0f : t1;
    if (timeOfImpact > dt) {
        return false;
    }

    Vec3 newPosA = posA + velA * timeOfImpact;
    Vec3 newPosB = posB + velB * timeOfImpact;
    Vec3 ab = newPosB - newPosA;
    ab.Normalize();

    pointOnA = newPosA + ab * sphereA->m_radius;
    pointOnB = newPosB - ab * sphereB->m_radius;
    return true;
}

/*
====================================================
Intersect
====================================================
*/
bool Intersect( Body * bodyA, Body * bodyB, const float dt, contact_t & contact ) {
    contact.bodyA = bodyA;
    contact.bodyB = bodyB;

    if (!bodyA || !bodyB) { return false; }

    if (bodyA->m_shape->GetType() == Shape::Type::Sphere && bodyB->m_shape->GetType() == Shape::Type::Sphere) {
        ShapeSphere* sphereA = (ShapeSphere*)bodyA->m_shape;
        ShapeSphere* sphereB = (ShapeSphere*)bodyB->m_shape;

        Vec3 posA = bodyA->m_position;
        Vec3 posB = bodyB->m_position;

        Vec3 velA = bodyA->m_linearVelocity;
        Vec3 velB = bodyB->m_linearVelocity;

        if (SphereSphereDynamic(
                sphereA, posA, velA, sphereB, posB, velB, dt,
                contact.ptOnA_WorldSpace, contact.ptOnB_WorldSpace, contact.timeOfImpact))
        {
            bodyA->Update(contact.timeOfImpact);
            bodyB->Update(contact.timeOfImpact);

            contact.ptOnA_LocalSpace = bodyA->WorldSpaceToBodySpace(contact.ptOnA_WorldSpace);
            contact.ptOnB_LocalSpace = bodyB->WorldSpaceToBodySpace(contact.ptOnB_WorldSpace);

            contact.normal = bodyA->m_position - bodyB->m_position;
            contact.normal.Normalize();

            bodyA->Update(-contact.timeOfImpact);
            bodyB->Update(-contact.timeOfImpact);

            Vec3 ab = bodyB->m_position - bodyA->m_position;
            float r = ab.GetMagnitude() - (sphereA->m_radius + sphereB->m_radius);
            contact.separationDistance = r;

            return true;
        }
    }

    return false;
}

