#include "vector3.h"
#include <stdio.h>
#include <assert.h>
#include <math.h>

const Vector3 UP_VECTOR3 = { 0.0f, 0.0f, 1.0f };
const Vector3 RIGHT_VECTOR3 = { 1.0f, 0.0f, 0.0f };
const Vector3 FORWARD_VECTOR3 = { 0.0f, 1.0f, 0.0f };
const Vector3 ZERO_VECTOR3 = { 0.0f, 0.0f, 0.0f };
const Vector3 ONE_VECTOR3 = { 1.0f, 1.0f, 1.0f };

Vector3::Vector3()
{
	_x = 0.0f;
	_y = 0.0f;
	_z = 0.0f;
}

Vector3::Vector3(float x, float y, float z)
{
	this->_x = x;
	this->_y = y;
	this->_z = z;
}

Vector3::Vector3(const Vector3& other)
{
	*this = other; //code smell is okay for POD classes.
}

void Vector3::set(float x, float y, float z)
{
	this->_x = x;
	this->_y = y;
	this->_z = z;
}

bool Vector3::operator==(const Vector3& other) const
{
	return _x == other._x && _y == other._y && _z == other._z;
}

Vector3 Vector3::operator+(const Vector3& other) const
{
	Vector3 tmp = *this;
	tmp += other;

	return tmp;
}

Vector3 Vector3::operator-(const Vector3& other) const
{
	Vector3 tmp = *this;
	tmp -= other;

	return tmp;
}

Vector3 Vector3::operator*(float s) const
{
	Vector3 tmp = *this;
	tmp *= s;

	return tmp;
}

Vector3 Vector3::operator/(float s) const
{
	Vector3 tmp = *this;
	tmp /= s;

	return tmp;
}

Vector3& Vector3::operator+=(const Vector3& other)
{
	_x += other._x;
	_y += other._y;
	_z += other._z;

	return *this;
}

Vector3& Vector3::operator-=(const Vector3& other)
{
	_x -= other._x;
	_y -= other._y;
	_z -= other._z;

	return *this;
}

Vector3& Vector3::operator*=(float s)
{
	_x *= s;
	_y *= s;
	_z *= s;

	return *this;
}

Vector3& Vector3::operator/=(float s)
{
	_x /= s;
	_y /= s;
	_z /= s;

	return *this;
}

Vector3 Vector3::lerp(Vector3 a, Vector3 b, float t)
{
	if (t < 0.0f)
		t = 0.0f;
	if (t > 1.0f)
		t = 1.0f;

	return lerpUnclamped(a, b, t);
}

Vector3 Vector3::lerpUnclamped(Vector3 a, Vector3 b, float t)
{
	return a * (1.0f - t) + b * t;
}

float Vector3::dot(Vector3 a, Vector3 b)
{
	return
		a._x * b._x +
		a._y * b._y +
		a._z * b._z;
}

Vector3 Vector3::cross(Vector3 a, Vector3 b)
{
	return Vector3(
		a._y * b._z - a._z * b._y,
		a._x * b._z - a._z * b._x,
		a._x * b._y - a._y * b._x
	);
}

Vector3 Vector3::normalized(Vector3 a)
{
	if (a.magnitudeSquared() == 1.0f)
		return a;

	return a /= a.magnitude();
}

float Vector3::magnitudeSquared() const
{
	return _x * _x + _y * _y + _z * _z;
}

float Vector3::magnitude() const
{
	return sqrtf(magnitudeSquared());
}

void Vector3::print() const
{
	printf("%f, %f, %f\n", _x, _y, _z);
}
