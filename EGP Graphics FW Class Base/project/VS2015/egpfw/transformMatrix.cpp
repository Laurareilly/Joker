#include "transformMatrix.h"
#include "vector3.h"
#include <math.h>
#include <stdio.h>
#include "utils.h"

TransformationMatrix::TransformationMatrix()
{
	for (char i = 0; i < 4; i++)
	{
		for (char j = 0; j < 4; j++)
		{
			vals[i][j] = 0.0f;
		}
	}

	vals[0][0] = 1.0f;
	vals[1][1] = 1.0f;
	vals[2][2] = 1.0f;
	vals[3][3] = 1.0f;
}

TransformationMatrix::TransformationMatrix(const TransformationMatrix& other)
{
	*this = other; //This code smell is fine for a POD class.
}

TransformationMatrix& TransformationMatrix::operator=(const TransformationMatrix& other)
{
	for (char i = 0; i < 4; i++)
	{
		for (char j = 0; j < 4; j++)
		{
			vals[i][j] = other.vals[i][j];
		}
	}

	return *this;
}

bool TransformationMatrix::operator==(const TransformationMatrix& other) const
{
	for (char i = 0; i < 4; i++)
	{
		for (char j = 0; j < 4; j++)
		{
			if (vals[i][j] != other.vals[i][j])
				return false;
		}
	}

	return true;
}

TransformationMatrix TransformationMatrix::operator*(float scalar) const
{
	TransformationMatrix result = *this;
	result *= scalar;

	return result;
}

TransformationMatrix TransformationMatrix::operator*(const TransformationMatrix& matrix) const
{
	TransformationMatrix result = *this;
	result *= matrix;

	return result;
}

TransformationMatrix& TransformationMatrix::operator*=(float scalar)
{
	for (char i = 0; i < 4; i++)
	{
		for (char j = 0; j < 4; j++)
		{
			vals[i][j] *= scalar;
		}
	}

	return *this;
}

TransformationMatrix& TransformationMatrix::operator*=(const TransformationMatrix& other)
{
	//Thanks to http://ogldev.atspace.co.uk/www/tutorial11/tutorial11.html for some help understanding this.
	TransformationMatrix result;

	for (char i = 0; i < 4; i++)
	{
		for (char j = 0; j < 4; j++)
		{
			result.vals[i][j] =
				this->vals[i][0] * other.vals[0][j] +
				this->vals[i][1] * other.vals[1][j] +
				this->vals[i][2] * other.vals[2][j] +
				this->vals[i][3] * other.vals[3][j];
		}
	}

	*this = result;
	return *this;
}

Vector3 TransformationMatrix::operator*(const Vector3& point) const
{
	return Vector3 (
		vals[0][0] * point.x() + vals[0][1] * point.y() + vals[0][2] * point.z() + vals[0][3],
		vals[1][0] * point.x() + vals[1][1] * point.y() + vals[1][2] * point.z() + vals[1][3],
		vals[2][0] * point.x() + vals[2][1] * point.y() + vals[2][2] * point.z() + vals[2][3]);
}

TransformationMatrix TransformationMatrix::makeXDirectionRotation(float amount)
{
	TransformationMatrix result;

	result.vals[1][1] = cos(amount);
	result.vals[1][2] = sin(amount) * -1.0f;
	result.vals[2][1] = result.vals[1][2] * -1.0f;
	result.vals[2][2] = result.vals[1][1];

	return result;
}

TransformationMatrix TransformationMatrix::makeYDirectionRotation(float amount)
{
	TransformationMatrix result;

	result.vals[0][0] = cos(amount);
	result.vals[0][2] = sin(amount);
	result.vals[2][0] = result.vals[0][2] * -1.0f;
	result.vals[2][2] = result.vals[0][0];

	return result;
}

TransformationMatrix TransformationMatrix::makeZDirectionRotation(float amount)
{
	TransformationMatrix result;

	result.vals[0][0] = cos(amount);
	result.vals[0][1] = sin(amount) * -1.0f;
	result.vals[1][0] = result.vals[0][1] * -1.0f;
	result.vals[1][1] = result.vals[1][1];

	return result;
}

TransformationMatrix TransformationMatrix::makeRotationXYZ(float x, float y, float z)
{
	TransformationMatrix xT, yT, zT;

	xT.makeXDirectionRotation(x);
	yT.makeZDirectionRotation(y);
	zT.makeYDirectionRotation(z);

	return xT * yT * zT;
}

TransformationMatrix TransformationMatrix::makeRotationZYX(float z, float y, float x)
{
	TransformationMatrix xT, yT, zT;

	xT.makeXDirectionRotation(x);
	yT.makeZDirectionRotation(y);
	zT.makeYDirectionRotation(z);

	return zT * yT * xT;
}

TransformationMatrix TransformationMatrix::makeScale(float x, float y, float z)
{
	TransformationMatrix result;

	result.vals[0][0] = x;
	result.vals[1][1] = y;
	result.vals[2][2] = z;

	return result;
}

TransformationMatrix TransformationMatrix::makeTranslation(float x, float y, float z)
{
	TransformationMatrix result;

	result.vals[0][3] = x;
	result.vals[1][3] = y;
	result.vals[2][3] = z;

	return result;
}

void TransformationMatrix::makeIdentity()
{
	*this = TransformationMatrix();
}

void TransformationMatrix::transpose()
{
	TransformationMatrix tmp = *this;

	for (char i = 0; i < 4; i++)
	{
		for (char j = 0; j < 4; j++)
		{
			tmp.vals[i][j] = this->vals[j][i];
		}
	}

	*this = tmp;
}

TransformationMatrix TransformationMatrix::getInverseUnscaled()
{
	TransformationMatrix rotTranspose = *this;
	rotTranspose.transpose();

	TransformationMatrix negRotTranspose = rotTranspose * -1.0f;

	TransformationMatrix result;
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			result.vals[i][j] = rotTranspose.vals[i][j];
		}
	}

	Vector3 tmp = { vals[0][3], vals[1][3], vals[2][3] };
	tmp = negRotTranspose * tmp;

	result.vals[0][3] = tmp.x();
	result.vals[1][3] = tmp.y();
	result.vals[2][3] = tmp.z();

	result.vals[3][0] = 0.0f;
	result.vals[3][1] = 0.0f;
	result.vals[3][2] = 0.0f;
	result.vals[3][3] = 1.0f;

	return result;
}

void TransformationMatrix::print()
{
	for (char i = 0; i < 4; i++)
	{
		printf("[%.1f, %.1f, %.1f, %.1f]\n", vals[i][0], vals[i][1], vals[i][2], vals[i][3]);
	}
	printf("\n");
}

void runTransformTestSuite()
{
	Vector3 simplePoint;
	TransformationMatrix test;

	test.print();

	test *= 4.0f;
	test.print();

	test.makeScale(4.0f, 6.0f, 2.0f);
	test.print();

	test.makeXDirectionRotation(M_PI / 2.0f);
	test.print();

	simplePoint = UP_VECTOR3;
	simplePoint = test.applyToPoint(simplePoint);
	simplePoint.print();
	printf("\n");

	test.makeYDirectionRotation(M_PI / 2.0f);
	test.print();

	simplePoint = test.applyToPoint(UP_VECTOR3);
	simplePoint.print();
	printf("\n");

	test.makeZDirectionRotation(M_PI / 2.0f);
	test.print();

	simplePoint = test.applyToPoint(UP_VECTOR3);
	simplePoint.print();
	printf("\n");

	test.makeRotationZYX(0.0f, M_PI / 2.0f, 0.0f);
	simplePoint = test.applyToPoint(UP_VECTOR3);
	simplePoint.print();
	printf("\n");

	test.makeScale(4.0f, 2.0f, 3.0f);
	simplePoint = test.applyToPoint(UP_VECTOR3);
	simplePoint.print();
	printf("\n");

	test.makeTranslation(2.0f, 3.0f, 2.0f);
	simplePoint = test.applyToPoint(UP_VECTOR3);
	simplePoint.print();
	printf("\n");

	test.inverseUnscaled();
	simplePoint = test.applyToPoint(simplePoint);
	simplePoint.print();
	printf("\n");


	TransformationMatrix rotation, scale, translation;
	simplePoint = UP_VECTOR3;

	rotation.makeRotationZYX(0.0f, M_PI, 0.0f);
	scale.makeUniformScale(5.0f);
	translation.makeTranslation(0.0f, 1.0f, 0.0f);

	TransformationMatrix result = translation * scale * rotation;

	simplePoint = result.applyToPoint(UP_VECTOR3);
	simplePoint.print();
	printf("\n");
}

//I "borrowed" this from stack overflow :D
//http://stackoverflow.com/questions/1148309/inverting-a-4x4-matrix
bool gluInvertMatrix(const float m[16], float invOut[16])
{
	float inv[16], det;
	int i;

	inv[0] = m[5] * m[10] * m[15] -
		m[5] * m[11] * m[14] -
		m[9] * m[6] * m[15] +
		m[9] * m[7] * m[14] +
		m[13] * m[6] * m[11] -
		m[13] * m[7] * m[10];

	inv[4] = -m[4] * m[10] * m[15] +
		m[4] * m[11] * m[14] +
		m[8] * m[6] * m[15] -
		m[8] * m[7] * m[14] -
		m[12] * m[6] * m[11] +
		m[12] * m[7] * m[10];

	inv[8] = m[4] * m[9] * m[15] -
		m[4] * m[11] * m[13] -
		m[8] * m[5] * m[15] +
		m[8] * m[7] * m[13] +
		m[12] * m[5] * m[11] -
		m[12] * m[7] * m[9];

	inv[12] = -m[4] * m[9] * m[14] +
		m[4] * m[10] * m[13] +
		m[8] * m[5] * m[14] -
		m[8] * m[6] * m[13] -
		m[12] * m[5] * m[10] +
		m[12] * m[6] * m[9];

	inv[1] = -m[1] * m[10] * m[15] +
		m[1] * m[11] * m[14] +
		m[9] * m[2] * m[15] -
		m[9] * m[3] * m[14] -
		m[13] * m[2] * m[11] +
		m[13] * m[3] * m[10];

	inv[5] = m[0] * m[10] * m[15] -
		m[0] * m[11] * m[14] -
		m[8] * m[2] * m[15] +
		m[8] * m[3] * m[14] +
		m[12] * m[2] * m[11] -
		m[12] * m[3] * m[10];

	inv[9] = -m[0] * m[9] * m[15] +
		m[0] * m[11] * m[13] +
		m[8] * m[1] * m[15] -
		m[8] * m[3] * m[13] -
		m[12] * m[1] * m[11] +
		m[12] * m[3] * m[9];

	inv[13] = m[0] * m[9] * m[14] -
		m[0] * m[10] * m[13] -
		m[8] * m[1] * m[14] +
		m[8] * m[2] * m[13] +
		m[12] * m[1] * m[10] -
		m[12] * m[2] * m[9];

	inv[2] = m[1] * m[6] * m[15] -
		m[1] * m[7] * m[14] -
		m[5] * m[2] * m[15] +
		m[5] * m[3] * m[14] +
		m[13] * m[2] * m[7] -
		m[13] * m[3] * m[6];

	inv[6] = -m[0] * m[6] * m[15] +
		m[0] * m[7] * m[14] +
		m[4] * m[2] * m[15] -
		m[4] * m[3] * m[14] -
		m[12] * m[2] * m[7] +
		m[12] * m[3] * m[6];

	inv[10] = m[0] * m[5] * m[15] -
		m[0] * m[7] * m[13] -
		m[4] * m[1] * m[15] +
		m[4] * m[3] * m[13] +
		m[12] * m[1] * m[7] -
		m[12] * m[3] * m[5];

	inv[14] = -m[0] * m[5] * m[14] +
		m[0] * m[6] * m[13] +
		m[4] * m[1] * m[14] -
		m[4] * m[2] * m[13] -
		m[12] * m[1] * m[6] +
		m[12] * m[2] * m[5];

	inv[3] = -m[1] * m[6] * m[11] +
		m[1] * m[7] * m[10] +
		m[5] * m[2] * m[11] -
		m[5] * m[3] * m[10] -
		m[9] * m[2] * m[7] +
		m[9] * m[3] * m[6];

	inv[7] = m[0] * m[6] * m[11] -
		m[0] * m[7] * m[10] -
		m[4] * m[2] * m[11] +
		m[4] * m[3] * m[10] +
		m[8] * m[2] * m[7] -
		m[8] * m[3] * m[6];

	inv[11] = -m[0] * m[5] * m[11] +
		m[0] * m[7] * m[9] +
		m[4] * m[1] * m[11] -
		m[4] * m[3] * m[9] -
		m[8] * m[1] * m[7] +
		m[8] * m[3] * m[5];

	inv[15] = m[0] * m[5] * m[10] -
		m[0] * m[6] * m[9] -
		m[4] * m[1] * m[10] +
		m[4] * m[2] * m[9] +
		m[8] * m[1] * m[6] -
		m[8] * m[2] * m[5];

	det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];

	if (det == 0)
		return false;

	det = 1.0 / det;

	for (i = 0; i < 16; i++)
		invOut[i] = inv[i] * det;

	return true;
}
