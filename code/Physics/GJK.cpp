//
//  GJK.cpp
//
#include "GJK.h"
#include <cstring>

//
// Signed Volumes
//

Vec2 SignedVolume1D(Vec3 s1, Vec3 s2)
{
    Vec3 ab = s2 - s1;
    Vec3 ap = Vec3(0.0f) - s1;
    Vec3 p0 = s1 + ab * ab.Dot(ap) / ab.GetLengthSqr();

    int idx = 0;
    float muMax = 0;
    for (int i = 0; i < 3; ++i) {
        float mu = s2[i] - s1[i];
        if (mu * mu > muMax * muMax) {
            muMax = mu;
            idx = i;
        }
    }

    float a = s1[idx];
    float b = s2[idx];
    float p = p0[idx];

    float c1 = p - a;
    float c2 = b - p;

    if ((p > a && p < b) || (p < a && p > b)) {
        return Vec2(c2 / muMax, c1 / muMax);
    }

    // If p is on the far side of A.
    if ((a <= b && p <= a) || (a >= b && p >= a)) {
        return Vec2(1.0f, 0.0f);
    }

    // P must be on the far side of B.
    return Vec2(0.0f, 1.0f);
}

int CompareSigns(float a, float b)
{
    if (a > 0.0f && b > 0.0f) return 1;
    if (a < 0.0f && b < 0.0f) return 1;
    return 0;
}

Vec3 SignedVolume2D(Vec3 s1, Vec3 s2, Vec3 s3)
{
    Vec3 normal = (s2 - s1).Cross(s3 - s1);
    Vec3 p0 = normal * s1.Dot(normal) / normal.GetLengthSqr();

    // Find the axis with the greatest projected area.
    int idx = 0;
    float area_max = 0;
    for (int i = 0; i < 3; ++i) {
        int j = (i + 1) % 3;
        int k = (i + 2) % 3;

        Vec2 a = Vec2(s1[j], s1[k]);
        Vec2 b = Vec2(s2[j], s2[k]);
        Vec2 c = Vec2(s3[j], s3[k]);
        Vec2 ab = b - a;
        Vec2 ac = c - a;

        float area = ab.x * ac.y - ab.y * ac.x;
        if (area * area > area_max * area_max) {
            idx = i;
            area_max = area;
        }
    }

    //Project onto the appropriate axis
    int x = (idx + 1) % 3;
    int y = (idx + 2) % 3;
    Vec2 s[3] = {
        Vec2(s1[x], s1[y]),
        Vec2(s2[x], s2[y]),
        Vec2(s3[x], s3[y]),
    };

    Vec2 p0_2d = Vec2(p0[x], p0[y]);

    // Get the sub-areas of the triangles formed from the projected origin and
    // the edges
    Vec3 areas;
    for (int i = 0; i < 3; ++i) {
        int j = (i + 1) % 3;
        int k = (i + 2) % 3;

        Vec2 a = p0_2d;
        Vec2 b = s[j];
        Vec2 c = s[k];
        Vec2 ab = b - a;
        Vec2 ac = c - a;

        areas[i] = ab.x * ac.y - ab.y * ac.x;
    }

    // If the projected origin is inside the triangle, then return the barycentric
    // coordinates of the origin in the triangle.
    if (CompareSigns(area_max, areas[0]) > 0 &&
        CompareSigns(area_max, areas[1]) > 0 &&
        CompareSigns(area_max, areas[2]) > 0)
    {
        Vec3 lambdas = areas / area_max;
        return lambdas;
    }

    // If we make it here, then we need to project onto the edges and determine
    // the closest point
    float dist = 1e10f;
    Vec3 lambdas = Vec3(1.0f, 0.0f, 0.0f);
    for (int i = 0; i < 3; ++i) {
        int k = (i + 1) % 3;
        int l = (i + 2) % 3;

        Vec3 edges_pts[3];
        edges_pts[0] = s1;
        edges_pts[1] = s2;
        edges_pts[2] = s3;

        Vec2 lambda_edge = SignedVolume1D(edges_pts[k], edges_pts[l]);
        Vec3 pt = edges_pts[k] * lambda_edge[0] + edges_pts[l] * lambda_edge[1];
        if (pt.GetLengthSqr() < dist) {
            dist = pt.GetLengthSqr();
            lambdas[i] = 0;
            lambdas[k] = lambda_edge[0];
            lambdas[l] = lambda_edge[1];
        }
    }

    return lambdas;
}

Vec4 SignedVolume3D(Vec3 s1, Vec3 s2, Vec3 s3, Vec3 s4)
{
    Mat4 M;
    M.rows[0] = Vec4(s1.x, s2.x, s3.x, s4.x);
    M.rows[1] = Vec4(s1.y, s2.y, s3.y, s4.y);
    M.rows[2] = Vec4(s1.z, s2.z, s3.z, s4.z);
    M.rows[3] = Vec4(1.0f, 1.0f, 1.0f, 1.0f);

    Vec4 C4;
    C4[0] = M.Cofactor(3, 0);
    C4[1] = M.Cofactor(3, 1);
    C4[2] = M.Cofactor(3, 2);
    C4[3] = M.Cofactor(3, 3);

    const float det = C4[0] + C4[1] + C4[2] + C4[3];

    // If the barycentric coordinates put the origin inside the simplex, then
    // return them

    if (CompareSigns(det, C4[0]) > 0 &&
        CompareSigns(det, C4[1]) > 0 &&
        CompareSigns(det, C4[2]) > 0 &&
        CompareSigns(det, C4[3]) > 0)
    {
        Vec4 lambdas = C4 * (1.0f / det);
        return lambdas;
    }

    // If we get here, then we need to project the origin onto the faces and
    // determine the closest one
    //
    Vec4 lambdas;
    float dist = 1e10f;
    for (int i = 0; i < 4; ++i) {
        int j = (i + 1) % 4;
        int k = (i + 2) % 4;

        Vec3 face_pts[4];
        face_pts[0] = s1;
        face_pts[1] = s2;
        face_pts[2] = s3;
        face_pts[3] = s4;

        Vec3 lambda_face = SignedVolume2D(face_pts[i], face_pts[j], face_pts[k]);
        Vec3 pt = face_pts[i] * lambda_face[0] + face_pts[j] * lambda_face[1] + face_pts[k] * lambda_face[2];
        if (pt.GetLengthSqr() < dist) {
            dist = pt.GetLengthSqr();
            lambdas.Zero();
            lambdas[i] = lambda_face[0];
            lambdas[j] = lambda_face[1];
            lambdas[k] = lambda_face[2];
        }
    }

    return lambdas;
}

//
// Point struct
//

struct Point {
    Vec3 xyz;
    Vec3 ptA;
    Vec3 ptB;

    Point() : xyz(0.0f), ptA(0.0f), ptB(0.0f) {}
    Point(const Point& other) = default;
    Point& operator=(const Point& other) = default;

    bool operator==(const Point& other) const
    {
        return (xyz == other.xyz && ptA == other.ptA && ptB == other.ptB);
    }
};

Point Support(const Body *bodyA, const Body *bodyB, Vec3 dir, float bias)
{
    dir.Normalize();
    Point point;

    point.ptA = bodyA->m_shape->Support(dir, bodyA->m_position, bodyA->m_orientation, bias);
    dir *= -1.0f; // Reverse direction for body B
    point.ptB = bodyB->m_shape->Support(dir, bodyB->m_position, bodyB->m_orientation, bias);

    point.xyz = point.ptA - point.ptB; // Minkowski difference
    return point;
}

/*
========================================
 SimplexSignedVolumes

 Projects the origin onto the simplex to acquire a new search direction.
 Also checks if the origin is "inside" the simplex.
========================================
*/
bool SimplexSignedVolumes(std::span<Point> points, Vec3& new_dir, Vec4 lambdas_out)
{
    constexpr float EPS = 1e-4f * 1e-4f;
    lambdas_out.Zero();

    bool does_intersect = false;
    switch (points.size()) {
        default:
        case 2: {
            Vec2 lambdas = SignedVolume1D(points[0].xyz, points[1].xyz);
            Vec3 v(0.0f);
            for (int i = 0; i < 2; i++) {
                v += points[i].xyz * lambdas[i];
            }
            new_dir = v * -1.0f;
            does_intersect = v.GetLengthSqr() < EPS;
            lambdas_out[0] = lambdas[0];
            lambdas_out[1] = lambdas[1];
        } break;

        case 3: {
            Vec3 lambdas = SignedVolume2D(points[0].xyz, points[1].xyz, points[2].xyz);
            Vec3 v(0.0f);
            for (int i = 0; i < 3; i++) {
                v += points[i].xyz * lambdas[i];
            }
            new_dir = v * -1.0f;
            does_intersect = v.GetLengthSqr() < EPS;
            lambdas_out[0] = lambdas[0];
            lambdas_out[1] = lambdas[1];
            lambdas_out[2] = lambdas[2];
        } break;

        case 4: {
            Vec4 lambdas = SignedVolume3D(points[0].xyz, points[1].xyz, points[2].xyz, points[3].xyz);
            Vec3 v(0.0f);
            for (int i = 0; i < 4; i++) {
                v += points[i].xyz * lambdas[i];
            }
            new_dir = v * -1.0f;
            does_intersect = v.GetLengthSqr() < EPS;
            lambdas_out[0] = lambdas[0];
            lambdas_out[1] = lambdas[1];
            lambdas_out[2] = lambdas[2];
            lambdas_out[3] = lambdas[3];
        } break;
    }

    return does_intersect;
}

/*
========================================
 HasPoint

Checks wether the simplex already contains the point.
========================================
*/

bool HasPoint(const Point simplex_points[4], const Point& new_pt) {
    constexpr float precision = 1e-6f;

    for (size_t i = 0; i < 4; i++) {
        const Point& pt = simplex_points[i];
        Vec3 delta = pt.xyz - new_pt.xyz;
        if (delta.GetLengthSqr() < precision * precision) {
            return true;
        }
    }
    return false;
}

/*
========================================
 SortValids

Sort the valid support points to the beginning of the array.
========================================
*/
void SortValids(Point simplex_points[4], Vec4& lambdas)
{
    bool valids[4] = {};
    for (size_t i = 0; i < 4; i++) {
        valids[i] = true;
        if (lambdas[i] == 0.0f) {
            valids[i] = false;
        }
    }

    Vec4 valid_lambdas(0.0f);
    int valid_count = 0;
    Point valid_points[4] = {};

    for (size_t i = 0; i < 4; i++) {
        if (valids[i]) {
            valid_points[valid_count] = simplex_points[i];
            valid_lambdas[valid_count] = lambdas[i];
            valid_count++;
        }
    }

    // Copy the valid points and lambdas back into the simplex_points and lambdas
    for (size_t i = 0; i < 4; i++) {
        simplex_points[i] = valid_points[i];
        lambdas[i] = valid_lambdas[i];
    }
}

int NumValids(const Vec4& lambdas) {
    int count = 0;
    for (int i = 0; i < 4; i++) {
        if (lambdas[i] != 0.0f) {
            count++;
        }
    }
    return count;
}

/*
================================
GJK_DoesIntersect
================================
*/
bool GJK_DoesIntersect( const Body * bodyA, const Body * bodyB ) {
    Vec3 origin(0.0f);

    int num_pts = 1;
    Point simplex_points[4];
    simplex_points[0] = Support(bodyA, bodyB, Vec3(1, 1, 1), 0.0f);

    float closest_dist = 1e10f;
    bool does_contain_origin = false;
    Vec3 new_dir = simplex_points[0].xyz * -1.0f;
    while (!does_contain_origin) {
        Point new_pt = Support(bodyA, bodyB, new_dir, 0.0f);

        // If the new point is already in the simplex, we can't expand further
        if (HasPoint(simplex_points, new_pt)) {
            break;
        }

        simplex_points[num_pts] = new_pt;
        num_pts++;

        // If this new point hasn't moved past the origin, then the origin
        // cannot be in the set. And therefore there is no collision.
        float dotdot = new_dir.Dot(new_pt.xyz - origin);
        if (dotdot < 0.0f) {
            break;
        }

        Vec4 lambdas;
        does_contain_origin = SimplexSignedVolumes(simplex_points, new_dir, lambdas);
        if (does_contain_origin)
            break;

        // Check that the new projection of the origin onto the simplex is closer
        // than the previous
        float dist = new_dir.GetLengthSqr();
        if (dist >= closest_dist)
            break;

        closest_dist = dist;

        // Use the lambdas that support the new search direction, and invalidate
        // any points that don't support it.
        SortValids(simplex_points, lambdas);
        num_pts = NumValids(lambdas);
        does_contain_origin = (4 == num_pts);
    }

    return does_contain_origin;
}

/*
================================
GJK_ClosestPoints
================================
*/
void GJK_ClosestPoints( const Body * bodyA, const Body * bodyB, Vec3 & ptOnA, Vec3 & ptOnB ) {
    // TODO: Add code
}

/*
================================
GJK_DoesIntersect
================================
*/
bool GJK_DoesIntersect( const Body * bodyA, const Body * bodyB, const float bias, Vec3 & ptOnA, Vec3 & ptOnB ) {
    // TODO: Add code

    return false;
}

//
// Test code for signed volumes
//


void TestSignedVolumeProjection() {
	const Vec3 orgPts[ 4 ] = {
		Vec3( 0, 0, 0 ),
		Vec3( 1, 0, 0 ),
		Vec3( 0, 1, 0 ),
		Vec3( 0, 0, 1 ),
	};
	Vec3 pts[ 4 ];
	Vec4 lambdas;
	Vec3 v;

	for ( int i = 0; i < 4; i++ ) {
		pts[ i ] = orgPts[ i ] + Vec3( 1, 1, 1 );
	}
	lambdas = SignedVolume3D( pts[ 0 ], pts[ 1 ], pts[ 2 ], pts[ 3 ] );
	v.Zero();
	for ( int i = 0; i < 4; i++ ) {
		v += pts[ i ] * lambdas[ i ];
	}
	printf( "lambdas: %.3f %.3f %.3f %.3f        v: %.3f %.3f %.3f\n",
		lambdas.x, lambdas.y, lambdas.z, lambdas.w,
		v.x, v.y, v.z
	);

	for ( int i = 0; i < 4; i++ ) {
		pts[ i ] = orgPts[ i ] + Vec3( -1, -1, -1 ) * 0.25f;
	}
	lambdas = SignedVolume3D( pts[ 0 ], pts[ 1 ], pts[ 2 ], pts[ 3 ] );
	v.Zero();
	for ( int i = 0; i < 4; i++ ) {
		v += pts[ i ] * lambdas[ i ];
	}
	printf( "lambdas: %.3f %.3f %.3f %.3f        v: %.3f %.3f %.3f\n",
		lambdas.x, lambdas.y, lambdas.z, lambdas.w,
		v.x, v.y, v.z
	);

	for ( int i = 0; i < 4; i++ ) {
		pts[ i ] = orgPts[ i ] + Vec3( -1, -1, -1 );
	}
	lambdas = SignedVolume3D( pts[ 0 ], pts[ 1 ], pts[ 2 ], pts[ 3 ] );
	v.Zero();
	for ( int i = 0; i < 4; i++ ) {
		v += pts[ i ] * lambdas[ i ];
	}
	printf( "lambdas: %.3f %.3f %.3f %.3f        v: %.3f %.3f %.3f\n",
		lambdas.x, lambdas.y, lambdas.z, lambdas.w,
		v.x, v.y, v.z
	);

	for ( int i = 0; i < 4; i++ ) {
		pts[ i ] = orgPts[ i ] + Vec3( 1, 1, -0.5f );
	}
	lambdas = SignedVolume3D( pts[ 0 ], pts[ 1 ], pts[ 2 ], pts[ 3 ] );
	v.Zero();
	for ( int i = 0; i < 4; i++ ) {
		v += pts[ i ] * lambdas[ i ];
	}
	printf( "lambdas: %.3f %.3f %.3f %.3f        v: %.3f %.3f %.3f\n",
		lambdas.x, lambdas.y, lambdas.z, lambdas.w,
		v.x, v.y, v.z
	);

	pts[ 0 ] = Vec3( 51.1996613f, 26.1989613f, 1.91339576f );
	pts[ 1 ] = Vec3( -51.0567360f, -26.0565681f, -0.436143428f );
	pts[ 2 ] = Vec3( 50.8978920f, -24.1035538f, -1.04042661f );
	pts[ 3 ] = Vec3( -49.1021080f, 25.8964462f, -1.04042661f );
	lambdas = SignedVolume3D( pts[ 0 ], pts[ 1 ], pts[ 2 ], pts[ 3 ] );
	v.Zero();
	for ( int i = 0; i < 4; i++ ) {
		v += pts[ i ] * lambdas[ i ];
	}
	printf( "lambdas: %.3f %.3f %.3f %.3f        v: %.3f %.3f %.3f\n",
		lambdas.x, lambdas.y, lambdas.z, lambdas.w,
		v.x, v.y, v.z
	);
}
