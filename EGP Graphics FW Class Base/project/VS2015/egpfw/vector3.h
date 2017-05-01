#ifndef __VECTOR_3_H
#define __VECTOR_3_H

class Vector3
{
private:
	float _x, _y, _z;

public:
	Vector3();
	Vector3(float x, float y, float z);
	Vector3(const Vector3& other);
	~Vector3() = default;

	void set(float x, float y, float z);

	bool operator == (const Vector3& other) const;
	bool operator != (const Vector3& other) const { return !(*this == other); }

	Vector3 operator + (const Vector3& other) const;
	Vector3 operator - (const Vector3& other) const;
	Vector3 operator * (float s) const;
	Vector3 operator / (float s) const;

	Vector3& operator += (const Vector3& other);
	Vector3& operator -= (const Vector3& other);
	Vector3& operator *= (float s);
	Vector3& operator /= (float s);

	void add(const Vector3& other) { *this += other; }
	void subtract(const Vector3& other) { *this -= other; }
	void multipliy(float s) { *this *= s; }
	void divide(float s) { *this /= s; }

	static Vector3 lerp(Vector3 a, Vector3 b, float t);
	static Vector3 lerpUnclamped(Vector3 a, Vector3 b, float t);
	static float dot(Vector3 a, Vector3 b);
	static Vector3 cross(Vector3 a, Vector3 b);
	static Vector3 normalized(Vector3 a);

	float x() const { return _x; }
	float y() const { return _y; }
	float z() const { return _z; }

	float getX() const { return x(); }
	float getY() const { return y(); }
	float getZ() const { return z(); }

	void setX(float x) { _x = x; }
	void setY(float y) { _y = y; }
	void setZ(float z) { _z = z; }
	void normalize() { *this = normalized(*this); }

	float magnitudeSquared() const;
	float magnitude() const;

	void print() const;
};

const extern Vector3 UP_VECTOR3;
const extern Vector3 RIGHT_VECTOR3;
const extern Vector3 FORWARD_VECTOR3;
const extern Vector3 ZERO_VECTOR3;
const extern Vector3 ONE_VECTOR3;

#endif
