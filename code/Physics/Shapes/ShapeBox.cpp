//
//  Shapes.cpp
//
#include "ShapeBox.h"

/*
========================================================================================================

ShapeBox

========================================================================================================
*/

/*
====================================================
ShapeBox::Build
====================================================
*/
void ShapeBox::Build(std::span<const Vec3> pts)
{
    for (Vec3 pt : pts)
        m_bounds.Expand(pt);

    m_points.clear();
    m_points.push_back(Vec3(m_bounds.mins.x, m_bounds.mins.y, m_bounds.mins.z));
    m_points.push_back(Vec3(m_bounds.maxs.x, m_bounds.mins.y, m_bounds.mins.z));
    m_points.push_back(Vec3(m_bounds.mins.x, m_bounds.maxs.y, m_bounds.mins.z));
    m_points.push_back(Vec3(m_bounds.mins.x, m_bounds.mins.y, m_bounds.maxs.z));

    m_points.push_back(Vec3(m_bounds.maxs.x, m_bounds.maxs.y, m_bounds.maxs.z));
    m_points.push_back(Vec3(m_bounds.mins.x, m_bounds.maxs.y, m_bounds.maxs.z));
    m_points.push_back(Vec3(m_bounds.maxs.x, m_bounds.mins.y, m_bounds.maxs.z));
    m_points.push_back(Vec3(m_bounds.maxs.x, m_bounds.mins.y, m_bounds.mins.z));

    m_centerOfMass = (m_bounds.mins + m_bounds.maxs) * 0.5f;
}

/*
====================================================
ShapeBox::Support
====================================================
*/
Vec3 ShapeBox::Support(Vec3 dir, Vec3 pos, Quat orient, float bias) const
{
    Vec3 maxPt = orient.RotatePoint(m_points[0]) + pos;
    float maxDist = dir.Dot(maxPt);
    for (size_t i = 1; i < m_points.size(); ++i) {
        Vec3 pt = orient.RotatePoint(m_points[i]) + pos;
        float dist = dir.Dot(pt);
        if (dist > maxDist) {
            maxDist = dist;
            maxPt = pt;
        }
    }

    Vec3 norm = dir;
    norm.Normalize();
    norm *= bias; // Apply bias to the direction
    return maxPt + norm;
}

/*
====================================================
ShapeBox::InertiaTensor
====================================================
*/
Mat3 ShapeBox::InertiaTensor() const
{
    // Tensor for a box centered at the origin is given by:
    float dx = m_bounds.maxs.x - m_bounds.mins.x;
    float dy = m_bounds.maxs.y - m_bounds.mins.y;
    float dz = m_bounds.maxs.z - m_bounds.mins.z;

    Mat3 tensor;
    tensor.Zero();
    tensor.rows[0][0] = (dy * dy + dz * dz) / 12.0f;
    tensor.rows[1][1] = (dx * dx + dz * dz) / 12.0f;
    tensor.rows[2][2] = (dx * dx + dy * dy) / 12.0f;

    // Now we need to use the parallel axis theorem to adjust the tensor for a
    // box that is not centered at the origin.
    Vec3 cm = (m_bounds.maxs + m_bounds.mins) * 0.5f;

    Vec3 R = Vec3(0, 0, 0) - cm; // Center of mass offset
    float R2 = R.GetLengthSqr();
    Mat3 patTensor;
    patTensor.rows[0] = Vec3(R2 - R.x * R.x, R.x * R.y, R.x * R.z);
    patTensor.rows[1] = Vec3(R.x * R.y, R2 - R.y * R.y, R.y * R.z);
    patTensor.rows[2] = Vec3(R.x * R.z, R.y * R.z, R2 - R.z * R.z);

    tensor += patTensor;
    return tensor;
}

/*
====================================================
ShapeBox::GetBounds
====================================================
*/
Bounds ShapeBox::GetBounds(Vec3 pos, Quat orient) const
{
    Vec3 corners[8];
    corners[0] = pos + orient.RotatePoint(m_points[0]);
    corners[1] = pos + orient.RotatePoint(m_points[1]);
    corners[2] = pos + orient.RotatePoint(m_points[2]);
    corners[3] = pos + orient.RotatePoint(m_points[3]);
    corners[4] = pos + orient.RotatePoint(m_points[4]);
    corners[5] = pos + orient.RotatePoint(m_points[5]);
    corners[6] = pos + orient.RotatePoint(m_points[6]);
    corners[7] = pos + orient.RotatePoint(m_points[7]);

    Bounds bounds;
    bounds.mins = corners[0];
    bounds.maxs = corners[0];
    for (int i = 1; i < 8; ++i) {
        bounds.Expand(corners[i]);
    }

    return bounds;
}

/*
====================================================
ShapeBox::FastestLinearSpeed
====================================================
*/
float ShapeBox::FastestLinearSpeed(Vec3 angularVelocity, Vec3 dir) const
{
    float maxSpeed = 0.0f;

    for (size_t i = 0; i < m_points.size(); ++i) {
        Vec3 r = m_points[i] - m_centerOfMass;
        Vec3 linearVelocity = angularVelocity.Cross(r);
        float speed = linearVelocity.Dot(dir);
        maxSpeed = std::max(maxSpeed, speed);
    }

    return maxSpeed;
}

