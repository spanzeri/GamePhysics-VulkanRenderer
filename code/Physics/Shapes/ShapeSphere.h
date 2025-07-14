//
//	ShapeSphere.h
//
#pragma once
#include "ShapeBase.h"

/*
====================================================
ShapeSphere
====================================================
*/
class ShapeSphere : public Shape {
public:
	explicit ShapeSphere( const float radius ) : m_radius( radius ) {
		m_centerOfMass.Zero();
	}

	Vec3 Support(Vec3 dir, Vec3 pos, Quat orient, float bias ) const override;

	Mat3 InertiaTensor() const override;

	Bounds GetBounds(Vec3 pos, Quat orient ) const override;
	Bounds GetBounds() const override;

	Type GetType() const override { return Type::Sphere; }

public:
	float m_radius;
};
