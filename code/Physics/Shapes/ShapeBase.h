//
//	ShapeBase.h
//
#pragma once
#include "../../Math/Vector.h"
#include "../../Math/Quat.h"
#include "../../Math/Matrix.h"
#include "../../Math/Bounds.h"

#include <span>

/*
====================================================
Shape
====================================================
*/
class Shape {
public:
	virtual ~Shape() = default;

	virtual Vec3 Support(Vec3 dir, Vec3 pos, Quat orient, float bias) const = 0;

	virtual Mat3 InertiaTensor() const = 0;

	virtual Bounds GetBounds(Vec3 pos, Quat orient) const = 0;
	virtual Bounds GetBounds() const = 0;

	virtual Vec3 GetCenterOfMass() const { return m_centerOfMass; }

	enum struct Type {
		Sphere,
		Box,
		Convex,
	};

	virtual Type GetType() const = 0;

	virtual float FastestLinearSpeed(Vec3 angularVelocity, Vec3 dir) const { return 0.0f; }

protected:
	Vec3 m_centerOfMass;
};
