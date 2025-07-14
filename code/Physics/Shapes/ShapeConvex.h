//
//  ShapeConvex.h
//
#pragma once

#include "ShapeBase.h"
#include <vector>
#include <span>

struct Triangle {
    int a;
    int b;
    int c;
};

struct Edge
{
    int a;
    int b;

    bool operator == (Edge rhs) const
    {
        return ((a == rhs.a && b == rhs.b) || (a == rhs.b && b == rhs.a));
    }
};

void BuildConvexHull(std::span<Vec3> verts, std::vector<Vec3> &hullPts, std::vector<Triangle> &hullTris );

/*
====================================================
ShapeConvex
====================================================
*/
class ShapeConvex : public Shape
{
public:
    explicit ShapeConvex(std::span<Vec3> pts)
    {
        Build(pts);
    }

    void Build(std::span<Vec3> pts);

    Vec3 Support(Vec3 dir, Vec3 pos, Quat orient, float bias) const override;

    Mat3 InertiaTensor() const override { return m_inertiaTensor; }

    Bounds GetBounds(Vec3 pos, Quat orient ) const override;
    Bounds GetBounds() const override { return m_bounds; }

    float FastestLinearSpeed(Vec3 angularVelocity, Vec3 dir) const override;

    Type GetType() const override { return Type::Convex; }

public:
    std::vector<Vec3>   m_points;
    Bounds              m_bounds;
    Mat3                m_inertiaTensor;
};
