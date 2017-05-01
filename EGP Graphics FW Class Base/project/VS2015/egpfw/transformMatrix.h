#ifndef __TRANSFORM_MATRIX_H
#define __TRANSFORM_MATRIX_H
#include "vector3.h"

class TransformationMatrix
{
	friend class Quaternion;

private:
	float vals[4][4];

public:
	TransformationMatrix();
	TransformationMatrix(const TransformationMatrix& other);
	~TransformationMatrix() = default;

	TransformationMatrix& operator = (const TransformationMatrix& other);
	bool operator == (const TransformationMatrix& other) const;
	bool operator != (const TransformationMatrix& other) const { return !(*this == other); }

	TransformationMatrix operator * (float scalar) const;
	TransformationMatrix operator * (const TransformationMatrix& matrix) const;
	TransformationMatrix& operator *= (float scalar);
	TransformationMatrix& operator *= (const TransformationMatrix& matrix);

	Vector3 operator * (const Vector3& point) const;

	void multiply(float s) { *this *= s; }
	void multiply(const TransformationMatrix& other) { *this *= other; }
	void concatenate(const TransformationMatrix& other) { multiply(other); }

	Vector3 multipliy(const Vector3& point) const { return *this * point; };
	Vector3 applyToPoint(const Vector3& point) const { return *this * point; }

	
	void makeIdentity();
	void transpose();

	TransformationMatrix getInverseUnscaled();
	void inverseUnscaled() { *this = getInverseUnscaled(); }

	bool equals(const TransformationMatrix& other) const { return *this == other; }
	void print();

	static TransformationMatrix identity() { return TransformationMatrix(); }

	static TransformationMatrix makeXDirectionRotation(float amount);
	static TransformationMatrix makeYDirectionRotation(float amount);
	static TransformationMatrix makeZDirectionRotation(float amount);

	static TransformationMatrix makeRotationXYZ(float x, float y, float z);
	static TransformationMatrix makeRotationZYX(float z, float y, float x);
	static TransformationMatrix makeScale(float x, float y, float z);
	static TransformationMatrix makeUniformScale(float s) { return makeScale(s, s, s); }

	static TransformationMatrix makeTranslation(float x, float y, float z);
};

void runTransformTestSuite();
bool gluInvertMatrix(const float m[16], float invOut[16]);


#endif
