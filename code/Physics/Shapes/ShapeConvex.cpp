//
//  ShapeConvex.cpp
//
#include "ShapeConvex.h"

int FindPointFurthestInDirection(std::span<Vec3> points, Vec3 dir)
{
    int maxIndex = 0;
    float maxDot = points[0].Dot(dir);
    for (size_t i = 1; i < points.size(); ++i)
    {
        float dot = points[i].Dot(dir);
        if (dot > maxDot)
        {
            maxDot = dot;
            maxIndex = (int)i;
        }
    }
    return maxIndex;
}

float DistanceFromLine(Vec3 a, Vec3 b, Vec3 point)
{
    Vec3 ab = b - a;
    ab.Normalize();

    Vec3 ap = point - a;
    Vec3 proj = ab * ap.Dot(ab);
    Vec3 perpendicular = ap - proj;
    return perpendicular.GetMagnitude();
}

Vec3 FindPointFurthestFromLine(std::span<Vec3> points, Vec3 a, Vec3 b)
{
    float maxIndex = 0;
    float maxDistance = DistanceFromLine(a, b, points[0]);
    for (size_t i = 1; i < points.size(); ++i)
    {
        float distance = DistanceFromLine(a, b, points[i]);
        if (distance > maxDistance)
        {
            maxDistance = distance;
            maxIndex = (int)i;
        }
    }
    return points[(int)maxIndex];
}

float DistanceFromTriangleSigned(Vec3 a, Vec3 b, Vec3 c, Vec3 point)
{
    // Calculate the normal of the triangle
    Vec3 ab = b - a;
    Vec3 ac = c - a;
    Vec3 normal = ab.Cross(ac);
    normal.Normalize();

    // Project the point onto the plane of the triangle
    float distance = (point - a).Dot(normal);
    return distance;
}

Vec3 FindPointFurthestFromTriangle(std::span<Vec3> points, Vec3 a, Vec3 b, Vec3 c)
{
    float maxIndex = 0;
    float maxDistance = DistanceFromTriangleSigned(a, b, c, points[0]);
    for (size_t i = 1; i < points.size(); ++i)
    {
        float distance = DistanceFromTriangleSigned(a, b, c, points[i]);
        if (std::abs(distance) > maxDistance)
        {
            maxDistance = distance;
            maxIndex = (int)i;
        }
    }
    return points[(int)maxIndex];
}

void BuildTetrahedron(std::span<Vec3> verts, std::vector<Vec3> &hullPts, std::vector<Triangle> &hullTris)
{
    hullPts.clear();
    hullTris.clear();

    Vec3 points[4];

    int idx = FindPointFurthestInDirection(verts, Vec3(1, 0, 0));
    points[0] = verts[idx];
    idx = FindPointFurthestInDirection(verts, points[0] * -1.0f);
    points[1] = verts[idx];
    points[2] = FindPointFurthestFromLine(verts, points[0], points[1]);
    points[3] = FindPointFurthestFromTriangle(verts, points[0], points[1], points[2]);

    // This is important to ensure the points are in a consistent (CCW) order
    float dist = DistanceFromTriangleSigned(points[0], points[1], points[2], points[3]);
    if (dist > 0.0f) {
        std::swap(points[0], points[1]);
    }

    hullPts.push_back(points[0]);
    hullPts.push_back(points[1]);
    hullPts.push_back(points[2]);
    hullPts.push_back(points[3]);

    hullTris.push_back({ 0, 1, 2 });
    hullTris.push_back({ 0, 2, 3 });
    hullTris.push_back({ 2, 1, 3 });
    hullTris.push_back({ 1, 0, 3 });
}

void RemoveInternalPoints(std::vector<Vec3>& hullPoints, std::vector<Triangle>& hullTris, std::vector<Vec3>& checkPoints)
{
    for (size_t i = 0; i < checkPoints.size(); i++)
    {
        Vec3 pt = checkPoints[i];
        bool isInternal = true;
        for (const Triangle& tri : hullTris)
        {
            Vec3 a = hullPoints[tri.a];
            Vec3 b = hullPoints[tri.b];
            Vec3 c = hullPoints[tri.c];
            if (DistanceFromTriangleSigned(a, b, c, pt) > 0.0f)
            {
                isInternal = false;
                break;
            }
        }

        bool isTooClose = false;
        if (!isInternal)
        {
            for (const Vec3& hullPt : hullPoints)
            {
                if ((hullPt - pt).GetLengthSqr() < (1e-2f * 1e-2f))
                {
                    isTooClose = true;
                    break;
                }
            }
        }

        if (isInternal || isTooClose)
        {
            checkPoints.erase(checkPoints.begin() + i);
            i--;
        }
    }
}

bool IsEdgeUnique(std::span<Triangle> tris, std::span<int> facingTris, int ignoreTri, const Edge& edge)
{
    for (size_t i = 0; i < facingTris.size(); ++i)
    {
        int triIndex = facingTris[i];
        if (triIndex == ignoreTri)
            continue;

        const Triangle& tri = tris[triIndex];
        Edge edges[3] = {
            { tri.a, tri.b },
            { tri.b, tri.c },
            { tri.c, tri.a }
        };

        for (int e = 0; e < 3; ++e)
        {
            if (edges[e] == edge)
            {
                return false;
            }
        }
    }
    return true;
}

void AddPoint(std::vector<Vec3>& hullPoints, std::vector<Triangle>& hullTris, Vec3 pt)
{
    // Find all existing triangles that are facing the point.
    std::vector<int> facingTris;
    for (size_t i = 0; i < hullTris.size(); ++i)
    {
        const Triangle& tri = hullTris[i];
        Vec3 a = hullPoints[tri.a];
        Vec3 b = hullPoints[tri.b];
        Vec3 c = hullPoints[tri.c];

        if (DistanceFromTriangleSigned(a, b, c, pt) > 0.0f)
        {
            facingTris.push_back((int)i);
        }
    }

    // New find all edges that are unique to the tris. These will be the edges
    // form the new triangles.
    std::vector<Edge> uniqueEdges;
    for (size_t i = 0; i < facingTris.size(); ++i)
    {
        int triIndex = facingTris[i];
        const Triangle& tri = hullTris[triIndex];

        Edge edges[3] = {
            { tri.a, tri.b },
            { tri.b, tri.c },
            { tri.c, tri.a }
        };

        for (int e = 0; e < 3; ++e)
        {
            if (IsEdgeUnique(hullTris, facingTris, triIndex, edges[e]))
                uniqueEdges.push_back(edges[e]);
        }
    }

    // Remove the old facing triangles.
    for (int i = (int)facingTris.size() - 1; i >= 0; --i)
    {
        int triIndex = facingTris[i];
        hullTris.erase(hullTris.begin() + triIndex);
    }

    // Add the new point to the hull.
    hullPoints.push_back(pt);
    const int newPointIndex = (int)hullPoints.size() - 1;

    // Create new triangles from the unique edges and the new point.
    for (size_t i = 0; i < uniqueEdges.size(); ++i)
    {
        const Edge& edge = uniqueEdges[i];

        Triangle newTri;
        newTri.a = edge.a;
        newTri.b = edge.b;
        newTri.c = newPointIndex;
        hullTris.push_back(newTri);
    }
}

void RemoveUnreferencedVerts(std::vector<Vec3>& hullPoints, std::vector<Triangle>& hullTris)
{
    for (size_t i = 0; i < hullPoints.size(); ++i)
    {
        bool isReferenced = false;
        for (const Triangle& tri : hullTris)
        {
            if (tri.a == (int)i || tri.b == (int)i || tri.c == (int)i)
            {
                isReferenced = true;
                break;
            }
        }

        if (isReferenced)
            continue;

        for (Triangle& tri : hullTris)
        {
            if (tri.a > (int)i) tri.a--;
            if (tri.b > (int)i) tri.b--;
            if (tri.c > (int)i) tri.c--;
        }

        hullPoints.erase(hullPoints.begin() + i);
        i--;
    }
}

void ExpandConvexHull(std::vector<Vec3>& hullPoints, std::vector<Triangle>& hullTris, std::span<Vec3> verts)
{
    std::vector externalVerts = std::vector(verts.begin(), verts.end());
    while (externalVerts.size() > 0)
    {
        int ptIndex = FindPointFurthestInDirection(externalVerts, externalVerts[0]);
        Vec3 pt = externalVerts[ptIndex];
        if (ptIndex != (int)externalVerts.size() - 1)
            externalVerts[ptIndex] = externalVerts.back();
        externalVerts.pop_back();
        AddPoint(hullPoints, hullTris, pt);
        RemoveInternalPoints(hullPoints, hullTris, externalVerts);
    }
    RemoveUnreferencedVerts(hullPoints, hullTris);
}

void BuildConvexHull(std::span<Vec3> verts, std::vector<Vec3>& hullPts, std::vector<Triangle>& hullTris)
{
    if (verts.size() < 4)
        return;

    BuildTetrahedron(verts, hullPts, hullTris);
    ExpandConvexHull(hullPts, hullTris, verts);
}

bool IsExternalPoint(std::span<Vec3> hullPoints, std::span<Triangle> hullTris, Vec3 pt)
{
    for (const Triangle& tri : hullTris)
    {
        Vec3 a = hullPoints[tri.a];
        Vec3 b = hullPoints[tri.b];
        Vec3 c = hullPoints[tri.c];
        if (DistanceFromTriangleSigned(a, b, c, pt) > 0.0f)
            return true;
    }
    return false;
}

Vec3 CalculateCenterOfMass(std::span<Vec3> hullPoints, std::span<Triangle> hullTris)
{
    const int numSamples = 100;
    Bounds bounds;
    for (const Vec3& pt : hullPoints)
    {
        bounds.Expand(pt);
    }

    Vec3 centerOfMass(0.0f);
    int sampleCount = 0;
    float dx = bounds.WidthX() / numSamples;
    float dy = bounds.WidthY() / numSamples;
    float dz = bounds.WidthZ() / numSamples;

    for (int x = 0; x < numSamples; ++x)
    {
        for (int y = 0; y < numSamples; ++y)
        {
            for (int z = 0; z < numSamples; ++z)
            {
                Vec3 samplePoint = bounds.mins + Vec3(x * dx, y * dy, z * dz);
                if (IsExternalPoint(hullPoints, hullTris, samplePoint))
                    continue;
                centerOfMass += samplePoint;
                sampleCount++;
            }
        }
    }

    centerOfMass /= (float)sampleCount;
    return centerOfMass;
}

Mat3 CalculateInertiaTensor(std::span<Vec3> hullPoints, std::span<Triangle> hullTris, Vec3 centerOfMass)
{
    Bounds bounds;
    bounds.Expand(hullPoints.data(), (int)hullPoints.size());

    Mat3 tensor;
    tensor.Zero();

    const int numSamples = 100;

    float dx = bounds.WidthX() / numSamples;
    float dy = bounds.WidthY() / numSamples;
    float dz = bounds.WidthZ() / numSamples;

    int sampleCount = 0;
    for (int x = 0; x < numSamples; ++x)
    {
        for (int y = 0; y < numSamples; ++y)
        {
            for (int z = 0; z < numSamples; ++z)
            {
                Vec3 pt = bounds.mins + Vec3(x * dx, y * dy, z * dz);
                if (IsExternalPoint(hullPoints, hullTris, pt))
                    continue;

                pt -= centerOfMass;

                tensor.rows[0][0] += pt.y * pt.y + pt.z * pt.z;
                tensor.rows[1][1] += pt.x * pt.x + pt.z * pt.z;
                tensor.rows[2][2] += pt.x * pt.x + pt.y * pt.y;

                tensor.rows[0][1] -= pt.x * pt.y;
                tensor.rows[0][2] -= pt.x * pt.z;
                tensor.rows[1][2] -= pt.y * pt.z;

                tensor.rows[1][0] = tensor.rows[0][1];
                tensor.rows[2][0] = tensor.rows[0][2];
                tensor.rows[2][1] = tensor.rows[1][2];

                sampleCount++;
            }
        }
    }

    tensor *= (1.0f / (float)sampleCount);
    return tensor;
}

/*
====================================================
ShapeConvex::Build
====================================================
*/
void ShapeConvex::Build(std::span<Vec3> pts)
{
    m_points.clear();
    m_points.reserve(pts.size());
    for (const Vec3& pt : pts)
        m_points.push_back(pt);

    std::vector<Vec3> hullPts;
    std::vector<Triangle> hullTris;
    BuildConvexHull(m_points, hullPts, hullTris);
    m_points = std::move(hullPts);

    m_bounds.Clear();
    m_bounds.Expand(m_points.data(), (int)m_points.size());
    m_centerOfMass  = CalculateCenterOfMass(m_points, hullTris);
    m_inertiaTensor = CalculateInertiaTensor(m_points, hullTris, m_centerOfMass);
}

/*
====================================================
ShapeConvex::Support
====================================================
*/
Vec3 ShapeConvex::Support(Vec3 dir, Vec3 pos, Quat orient, float bias) const
{
    Vec3 supportPt;

    // TODO: Add code

    return supportPt;
}

/*
====================================================
ShapeConvex::GetBounds
====================================================
*/
Bounds ShapeConvex::GetBounds(Vec3 pos, Quat orient) const
{
    Vec3 corners[8];
    corners[0] = pos + orient.RotatePoint(Vec3(m_bounds.mins.x, m_bounds.mins.y, m_bounds.mins.z));
    corners[1] = pos + orient.RotatePoint(Vec3(m_bounds.maxs.x, m_bounds.mins.y, m_bounds.mins.z));
    corners[2] = pos + orient.RotatePoint(Vec3(m_bounds.mins.x, m_bounds.maxs.y, m_bounds.mins.z));
    corners[3] = pos + orient.RotatePoint(Vec3(m_bounds.mins.x, m_bounds.mins.y, m_bounds.maxs.z));
    corners[4] = pos + orient.RotatePoint(Vec3(m_bounds.maxs.x, m_bounds.maxs.y, m_bounds.maxs.z));
    corners[5] = pos + orient.RotatePoint(Vec3(m_bounds.mins.x, m_bounds.maxs.y, m_bounds.maxs.z));
    corners[6] = pos + orient.RotatePoint(Vec3(m_bounds.maxs.x, m_bounds.mins.y, m_bounds.maxs.z));
    corners[7] = pos + orient.RotatePoint(Vec3(m_bounds.maxs.x, m_bounds.mins.y, m_bounds.mins.z));

    Bounds bounds;
    bounds.mins = corners[0];
    bounds.maxs = corners[0];
    for (int i = 1; i < 8; ++i)
        bounds.Expand(corners[i]);
    return bounds;
}

/*
====================================================
ShapeConvex::FastestLinearSpeed
====================================================
*/
float ShapeConvex::FastestLinearSpeed(Vec3 angularVelocity, Vec3 dir) const
{
    float maxSpeed = 0.0f;
    for (const Vec3 & pt : m_points)
    {
        Vec3 r = pt - m_centerOfMass;
        Vec3 linearVelocity = angularVelocity.Cross(r);
        float speed = linearVelocity.Dot(dir);
        maxSpeed = std::max(maxSpeed, speed);
    }
    return maxSpeed;
}
