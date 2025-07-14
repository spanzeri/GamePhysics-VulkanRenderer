//
//	ShapeBox.h
//
#pragma once

#include "ShapeBase.h"

#include <vector>

/*
====================================================
ShapeBox
====================================================
*/
class ShapeBox : public Shape {
public:
	explicit ShapeBox( const Vec3 * pts, const int num ) {
		assert(num >= 0);
		Build(std::span(pts, (size_t)num));
	}

	void Build(std::span<const Vec3> pts);

	Vec3 Support(Vec3 dir, Vec3 pos, Quat orient, float bias) const override;

	Mat3 InertiaTensor() const override;

	Bounds GetBounds(Vec3 pos, Quat orient ) const override;
	Bounds GetBounds() const override { return m_bounds; }

	float FastestLinearSpeed(Vec3 angularVelocity, Vec3 dir) const override;

	Type GetType() const override { return Type::Box; }

public:
	std::vector<Vec3> m_points;
	Bounds m_bounds;
};
