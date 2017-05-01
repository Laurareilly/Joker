#include "Quaternion.h"

float Quaternion::getMagnitude() const
{
	return sqrt(
		vals[X] * vals[X] +
		vals[Y] * vals[Y] +
		vals[Z] * vals[Z] +
		vals[W] * vals[W]);
}

float Quaternion::getMagnitudeSqrd() const
{
	return
		vals[X] * vals[X] +
		vals[Y] * vals[Y] +
		vals[Z] * vals[Z] +
		vals[W] * vals[W];
}

bool Quaternion::isNormalized() const
{
	float magSqr = getMagnitudeSqrd();
	magSqr = abs(magSqr - 1.0f);
	return magSqr < 0.001f; //floating point error tolerance
}

void Quaternion::normalize()
{
	float mag = getMagnitude();
	vals[X] /= mag;
	vals[Y] /= mag;
	vals[Z] /= mag;
	vals[W] /= mag;
}

Quaternion Quaternion::normalized() const
{
	if (this->isNormalized())
		return Quaternion(*this);

	Quaternion result = *this;
	result.normalize();

	return result;
}

Quaternion Quaternion::operator+(const Quaternion& other) const
{
	Quaternion result = *this;
	result *= other;

	return result;
}

Quaternion Quaternion::operator-(const Quaternion& other) const
{
	Quaternion result = *this;
	result -= other;

	return result;
}

Quaternion Quaternion::operator*(float scale) const
{
	Quaternion result = *this;
	result *= scale;

	return result;
}

Quaternion Quaternion::operator/(float scale) const
{
	Quaternion result = *this;
	result /= scale;

	return result;
}

Quaternion& Quaternion::operator+=(const Quaternion& other)
{
	this->vals[X] += other[X];
	this->vals[Y] += other[Y];
	this->vals[Z] += other[Z];
	this->vals[W] += other[W];

	return *this;
}

Quaternion& Quaternion::operator-=(const Quaternion& other)
{
	this->vals[X] -= other[X];
	this->vals[Y] -= other[Y];
	this->vals[Z] -= other[Z];
	this->vals[W] -= other[W];

	return *this;
}

Quaternion& Quaternion::operator*=(float scale)
{
	this->vals[X] *= scale;
	this->vals[Y] *= scale;
	this->vals[Z] *= scale;
	this->vals[W] *= scale;

	return *this;
}

Quaternion& Quaternion::operator/=(float scale)
{
	this->vals[X] /= scale;
	this->vals[Y] /= scale;
	this->vals[Z] /= scale;
	this->vals[W] /= scale;

	return *this;
}

float Quaternion::operator[](Val v) const
{
	return vals[v];
}

Quaternion::Quaternion()
{
	vals[X] = 0.0f;
	vals[Y] = 0.0f;
	vals[Z] = 0.0f;
	vals[W] = 1.0f;
}

Quaternion::Quaternion(const Quaternion& other)
{
	vals = other.vals;
}

Quaternion::Quaternion(float amount, Vector3 axis)
{
	axis.normalize();
	axis *= sin(amount);
	
	vals[X] = axis.x();
	vals[Y] = axis.y();
	vals[Z] = axis.z();
	vals[W] = cos(amount);
}

Quaternion::Quaternion(float x, float y, float z, float w)
{
	vals[X] = x;
	vals[Y] = y;
	vals[Z] = z;
	vals[W] = w;
}

Quaternion::~Quaternion()
{
}

Quaternion Quaternion::identity()
{
	return Quaternion(); //default constructor makes an identity Quaternion
}

Quaternion Quaternion::inverse(const Quaternion& original)
{
	float magnitude = original.getMagnitude();

	return Quaternion(
		original[X] / magnitude,
		-1.0f * original[Y] / magnitude,
		-1.0f * original[Z] / magnitude,
		-1.0f * original[W] / magnitude);
}

Quaternion Quaternion::concatenate(const Quaternion& a, const Quaternion& b)
{
	if (!a.isNormalized() || !b.isNormalized())
		throw std::invalid_argument("Tried to multiply non-normalized Quaternions!");

	float x = 
		a[W] * b[X] + 
		a[X] * b[W] + 
		a[Y] * b[Z] - 
		a[Z] * b[Y];

	float y =
		a[W] * b[Y] -
		a[X] * b[Z] +
		a[Y] * b[W] +
		a[Z] * b[X];

	float z =
		a[W] * b[Z] +
		a[X] * b[Y] -
		a[Y] * b[X] +
		a[Z] * b[W];

	float w =
		a[W] * b[W] -
		a[X] * b[X] -
		a[Y] * b[Y] -
		a[Z] * b[Z];

	return Quaternion(x, y, z, w);
}

Quaternion Quaternion::lerp(const Quaternion& from, const Quaternion& to, float val)
{
	return (from * (1.0f - val) + to * val).normalized();
}

Quaternion Quaternion::slerp(const Quaternion& from, const Quaternion& to, float perc)
{
	Quaternion realTo;
	float dot = Quaternion::dot(from, to);

	if (dot < 0) //more than 90 degrees apart
	{
		dot = -1.0f * dot;
		realTo = inverse(to);
	}
	else
		realTo = to;

	float angle = acosf(dot);

	return (from * sinf(angle * (1.0f - perc)) + realTo * sinf(angle * perc)) / sinf(angle);
}

float Quaternion::dot(const Quaternion& from, const Quaternion& to)
{
	return from[X] * to[X] + from[Y] * to[Y] + from[Z] * to[Z] + from[W] * to[W];
}

TransformationMatrix Quaternion::makeTransformationMatrix(const Quaternion& from)
{
	TransformationMatrix result;

	Quaternion f = from;

	if (!f.isNormalized())
		f.normalize();

	result.vals[0][0] = 1.0f - 2.0f * f[Y] * f[Y] - 2.0f * f[Z] * f[Z];
	result.vals[0][1] = 2.0f * f[X] * f[Y] - 2.0f * f[Z] * f[W];
	result.vals[0][2] = 2.0f * f[X] * f[Z] + 2.0f * f[Y] * f[W];

	result.vals[1][0] = 2.0f * f[X] * f[Y] + 2.0f * f[Z] * f[W];
	result.vals[1][1] = 1.0f - 2.0f * f[X] * f[X] - 2.0f * f[Z] * f[Z];
	result.vals[1][2] = 2.0f * f[Y] * f[Z] - 2.0f * f[X] * f[W];

	result.vals[2][0] = 2.0f * f[X] * f[Z] - 2.0f * f[Y] * f[W];
	result.vals[2][1] = 2.0f * f[Y] * f[Z] + 2.0f * f[X] * f[W];
	result.vals[2][2] = 1.0f - 2.0f * f[X] * f[X] - 2.0f * f[Y] * f[Y];

	return result;
}

Quaternion Quaternion::makeRotationZYX(float yaw, float pitch, float roll)
{
	//https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
	float yawCos = cos(yaw / 2.0f);		//t0
	float yawSin = sin(yaw / 2.0f);		//t1
	float pitchCos = cos(pitch / 2.0f);	//t2
	float pitchSin = sin(pitch / 2.0f);	//t3
	float rollCos = cos(roll / 2.0f);	//t4
	float rollSin = sin(roll / 2.0f);	//t5

	float x = yawCos * pitchSin * rollCos - yawSin * pitchCos * rollSin;
	float y = yawCos * pitchCos * rollSin + yawSin * pitchSin * rollCos;
	float z = yawSin * pitchCos * rollCos - yawCos * pitchSin * rollSin;
	float w = yawCos * pitchCos * rollCos + yawSin * pitchSin * rollSin;

	return Quaternion(x, y, z, w);
}

Quaternion Quaternion::makeRotationXYZ(float x, float y, float z)
{
	return makeRotationX(x) * makeRotationY(y) * makeRotationZ(z);
}

Vector3 Quaternion::applyToPoint(const Vector3& point) const
{
	//I don't care about optimization hahahaha there's no stopping me now
	TransformationMatrix matrix = getTransformationMatrix();
	return matrix.applyToPoint(point);
}

Quaternion Quaternion::operator*(const Quaternion& other) const
{
	Quaternion result = *this;
	result *= other;

	return result;
}

Quaternion Quaternion::operator=(const Quaternion& other)
{
	this->vals = other.vals;
	return *this;
}

Quaternion& Quaternion::operator*=(const Quaternion& other)
{
	*this = concatenate(*this, other);
	return *this;
}
