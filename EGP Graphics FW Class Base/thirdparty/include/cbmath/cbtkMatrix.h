/*
Copyright (c) 2011-2016 Daniel S. Buckstein

Permission is hereby granted, free of charge, to any person obtaining 
a copy of this software and associated documentation files (the "Software"), 
to deal in the Software without restriction, including without limitation the 
rights to use, copy, modify, merge, publish, distribute, sublicense, and/or 
sell copies of the Software, and to permit persons to whom the Software is 
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in 
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL 
THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE 
SOFTWARE.
*/

/*
	Cheezburger Toolkit Matrix Classes
	By Dan Buckstein
	Standalone matrix classes.
	Contains structures for 2D, 3D and 4D *column-major matrices, with associated 
		functions and operators.
	The primary purpose for these structures is for rigid transforms, not just 
		any old matrix, but feel free to extend these if you wish!

	*Column-major: this is how the data in the array is arranged in matrix form:
		0	4	8	12
		1	5	9	13
		2	6	10	14
		3	7	11	15
*/


#ifndef	__CBTK_MATH_MATRIX_H
#define	__CBTK_MATH_MATRIX_H


#include "cbmath/cbtkVector.h"
// these structures describe column-major matrices
// they are arranged here as the transpose of the actual data
// the first number refers to the column
//	m.m[column, row]


namespace cbtk
{
namespace cbmath
{


union mat2;
union mat3;
union mat4;


//-----------------------------------------------------------------------------
// structures

union mat2
{
	// ctor (default is identity)
	mat2();
	explicit mat2(	real M00, real M01, 
					real M10, real M11);
	explicit mat2(real M);
	explicit mat2(const real *Mp);
	explicit mat2(const vec2 &C0, const vec2 &C1);
	explicit mat2(const mat3 &M3);
	explicit mat2(const mat4 &M4);

	// set and reset
	mat2 &set();
	mat2 &set(	real M00, real M01, 
				real M10, real M11);
	mat2 &set(real M);
	mat2 &set(const real *Mp);
	mat2 &set(const vec2 &C0, const vec2 &C1);
	mat2 &set(const mat3 &M3);
	mat2 &set(const mat4 &M4);

	// get row
	const vec2 r0() const;
	const vec2 r1() const;

	// data
	struct { vec2 c0, c1; };
	struct { vec2 col[2]; };
	struct { real 
		m00, m01, 
		m10, m11; };
	real mm[2][2];
	real m[4];
	
	// access values as column or row
	enum Column { col0, col1 };
	enum Row    { row0, row1 };
	const vec2 operator()(Row) const;
	const vec2 operator()(Column) const;
	vec2 &operator()(Column);
};
static const mat2 m2Identity;


union mat3
{
	// ctor (default is identity)
	mat3();
	explicit mat3(	real M00, real M01, real M02, 
					real M10, real M11, real M12, 
					real M20, real M21, real M22 );
	explicit mat3(real M);
	explicit mat3(const real *Mp);
	explicit mat3(const vec3 &C0, const vec3 &C1, const vec3 &C2);
	explicit mat3(const mat2 &M2);
	explicit mat3(const mat4 &M4);
	
	// set and reset
	mat3 &set();
	mat3 &set(	real M00, real M01, real M02, 
				real M10, real M11, real M12, 
				real M20, real M21, real M22 );
	mat3 &set(real M);
	mat3 &set(const real *Mp);
	mat3 &set(const vec3 &C0, const vec3 &C1, const vec3 &C2);
	mat3 &set(const mat2 &M2);
	mat3 &set(const mat4 &M4);

	// get row
	const vec3 r0() const;
	const vec3 r1() const;
	const vec3 r2() const;

	// data
	struct { vec3 c0, c1, c2; };
	struct { vec3 col[3]; };
	struct { real 
		m00, m01, m02, 
		m10, m11, m12, 
		m20, m21, m22; };
	real mm[3][3];
	real m[9];
	
	// access values as column or row
	enum Column { col0, col1, col2 };
	enum Row    { row0, row1, row2 };
	const vec3 operator()(Row) const;
	const vec3 operator()(Column) const;
	vec3 &operator()(Column);
};
static const mat3 m3Identity;


union mat4
{
	// ctor (default is identity)
	mat4();
	explicit mat4(	real M00, real M01, real M02, real M03, 
					real M10, real M11, real M12, real M13, 
					real M20, real M21, real M22, real M23, 
					real M30, real M31, real M32, real M33 );
	explicit mat4(real M);
	explicit mat4(const real *Mp);
	explicit mat4(const vec4 &C0, const vec4 &C1, const vec4 &C2, const vec4 &C3);
	explicit mat4(const mat2 &M2);
	explicit mat4(const mat3 &M3);
	
	// set and reset
	mat4 &set();
	mat4 &set(	real M00, real M01, real M02, real M03, 
				real M10, real M11, real M12, real M13, 
				real M20, real M21, real M22, real M23, 
				real M30, real M31, real M32, real M33 );
	mat4 &set(real M);
	mat4 &set(const real *Mp);
	mat4 &set(const vec4 &C0, const vec4 &C1, const vec4 &C2, const vec4 &C3);
	mat4 &set(const mat2 &M2);
	mat4 &set(const mat3 &M3);

	// get row
	const vec4 r0() const;
	const vec4 r1() const;
	const vec4 r2() const;
	const vec4 r3() const;

	// data
	struct { vec4 c0, c1, c2, c3; };
	struct { vec4 col[4]; };
	struct { real 
		m00, m01, m02, m03, 
		m10, m11, m12, m13, 
		m20, m21, m22, m23, 
		m30, m31, m32, m33; };
	real mm[4][4];
	real m[16];
	
	// access values as column or row
	enum Column { col0, col1, col2, col3 };
	enum Row    { row0, row1, row2, row3 };
	const vec4 operator()(Row) const;
	const vec4 operator()(Column) const;
	vec4 &operator()(Column);
};
static const mat4 m4Identity;



//-----------------------------------------------------------------------------
// operators

const mat2 operator +(const mat2 &m0, const mat2 &m1);
const mat3 operator +(const mat3 &m0, const mat3 &m1);
const mat4 operator +(const mat4 &m0, const mat4 &m1);

mat2 &operator +=(mat2 &m0, const mat2 &m1);
mat3 &operator +=(mat3 &m0, const mat3 &m1);
mat4 &operator +=(mat4 &m0, const mat4 &m1);

const mat2 operator +(const mat2 &m);
const mat3 operator +(const mat3 &m);
const mat4 operator +(const mat4 &m);

const mat2 operator -(const mat2 &m0, const mat2 &m1);
const mat3 operator -(const mat3 &m0, const mat3 &m1);
const mat4 operator -(const mat4 &m0, const mat4 &m1);

mat2 &operator -=(mat2 &m0, const mat2 &m1);
mat3 &operator -=(mat3 &m0, const mat3 &m1);
mat4 &operator -=(mat4 &m0, const mat4 &m1);

const mat2 operator -(const mat2 &m);
const mat3 operator -(const mat3 &m);
const mat4 operator -(const mat4 &m);

const mat2 operator *(const mat2 &m0, const mat2 &m1);
const mat3 operator *(const mat3 &m0, const mat3 &m1);
const mat4 operator *(const mat4 &m0, const mat4 &m1);

mat2 &operator *=(mat2 &m0, const mat2 &m1);
mat3 &operator *=(mat3 &m0, const mat3 &m1);
mat4 &operator *=(mat4 &m0, const mat4 &m1);

const vec2 operator *(const mat2 &m, const vec2 &v);
const vec3 operator *(const mat3 &m, const vec3 &v);
const vec4 operator *(const mat4 &m, const vec4 &v);

vec2 &operator *=(vec2 &v, const mat2 &m);
vec3 &operator *=(vec3 &v, const mat3 &m);
vec4 &operator *=(vec4 &v, const mat4 &m);

const mat2 operator *(const mat2 &m, const real s);
const mat3 operator *(const mat3 &m, const real s);
const mat4 operator *(const mat4 &m, const real s);

const mat2 operator *(const real s, const mat2 &m);
const mat3 operator *(const real s, const mat3 &m);
const mat4 operator *(const real s, const mat4 &m);

mat2 &operator *=(mat2 &m, const real s);
mat3 &operator *=(mat3 &m, const real s);
mat4 &operator *=(mat4 &m, const real s);

const mat2 operator /(const mat2 &m, const real s);
const mat3 operator /(const mat3 &m, const real s);
const mat4 operator /(const mat4 &m, const real s);

const mat2 operator /(const real s, const mat2 &m);
const mat3 operator /(const real s, const mat3 &m);
const mat4 operator /(const real s, const mat4 &m);

mat2 &operator /=(mat2 &m, const real s);
mat3 &operator /=(mat3 &m, const real s);
mat4 &operator /=(mat4 &m, const real s);

bool operator ==(const mat2 &m0, const mat2 &m1);
bool operator ==(const mat3 &m0, const mat3 &m1);
bool operator ==(const mat4 &m0, const mat4 &m1);

bool operator !=(const mat2 &m0, const mat2 &m1);
bool operator !=(const mat3 &m0, const mat3 &m1);
bool operator !=(const mat4 &m0, const mat4 &m1);



//-----------------------------------------------------------------------------
// special functions

// compute the determinant of a matrix
real determinant(const mat2 &m);
real determinant(const mat3 &m);

real invDeterminant(const mat2 &m);
real invDeterminant(const mat3 &m);


// compute the transpose of a matrix (flip values along diagonal)
const mat2 transpose(const mat2 &m);
const mat3 transpose(const mat3 &m);
const mat4 transpose(const mat4 &m);


// create axial rotations
const mat3 makeRotationX3(real radians);
const mat4 makeRotationX4(real radians);

const mat3 makeRotationY3(real radians);
const mat4 makeRotationY4(real radians);

const mat2 makeRotationZ2(real radians);
const mat3 makeRotationZ3(real radians);
const mat4 makeRotationZ4(real radians);

// create complex rotation from euler angles (radians)
const mat3 makeRotationEuler3XYZ(real rx, real ry, real rz);
const mat3 makeRotationEuler3XZY(real rx, real ry, real rz);
const mat3 makeRotationEuler3YXZ(real rx, real ry, real rz);
const mat3 makeRotationEuler3YZX(real rx, real ry, real rz);
const mat3 makeRotationEuler3ZXY(real rx, real ry, real rz);
const mat3 makeRotationEuler3ZYX(real rx, real ry, real rz);

const mat4 makeRotationEuler4XYZ(real rx, real ry, real rz);
const mat4 makeRotationEuler4XZY(real rx, real ry, real rz);
const mat4 makeRotationEuler4YXZ(real rx, real ry, real rz);
const mat4 makeRotationEuler4YZX(real rx, real ry, real rz);
const mat4 makeRotationEuler4ZXY(real rx, real ry, real rz);
const mat4 makeRotationEuler4ZYX(real rx, real ry, real rz);

// the euler angles are stored in order (XYZ) and the multiplication happens
//	in the order of the indices provided from right to left
const mat3 makeRotationEuler3(real rx, real ry, real rz, const unsigned int order[3]);
const mat3 makeRotationEuler3(real rx, real ry, real rz, const unsigned short order[3]);
const mat4 makeRotationEuler4(real rx, real ry, real rz, const unsigned int order[3]);
const mat4 makeRotationEuler4(real rx, real ry, real rz, const unsigned short order[3]);

// angle axis rotation
const mat3 makeRotationAngleAxis3(real radians, vec3 axis);
const mat4 makeRotationAngleAxis4(real radians, vec3 axis);

// create uniform scale
const mat2 makeScale2(real s);
const mat3 makeScale3(real s);
const mat4 makeScale4(real s);

// create non-uniform scale
const mat2 makeScale2(real sx, real sy);
const mat3 makeScale3(real sx, real sy, real sz);
const mat4 makeScale4(real sx, real sy, real sz);

// create translation transformation
const mat4 makeTranslation4(const vec3 &t);
const mat4 makeTranslation4(real tx, real ty, real tz);

// build and invert a homogeneous transformation
const mat4 makeTransform(const mat3 &r, const vec3 &t);
const mat4 makeTransform(const mat3 &r, const real s, const vec3 &t);
const mat4 makeTransform(const mat3 &r, const mat3 &s, const vec3 &t);

const mat4 makeRotateScale(const mat3 &r, const real s);
const mat4 makeRotateTranslate(const mat3 &r, const vec3 &t);
const mat4 makeScaleTranslate(const real s, const vec3 &t);

// utility to multiply 4D matrix with 3D vector
const vec3 multMat4Vec3(const mat4 &m, const vec3 &v);
const vec3 multMat4Vec3NoTranslate(const mat4 &m, const vec3 &v);

// optimized 4x4 transformation multiplication
const mat4 multTransformOptimized(const mat4 &m0, const mat4 &m1);

// compute the inverse matrix quickly for each type
const mat2 inverse(const mat2 &m);
const mat2 inverseNoScale(const mat2 &m);
const mat2 inverseUniformScale(const mat2 &m);

const mat3 inverse(const mat3 &m);
const mat3 inverseNoScale(const mat3 &m);
const mat3 inverseUniformScale(const mat3 &m);

const mat4 transformInverse(const mat4 &m);
const mat4 transformInverseNoScale(const mat4 &m);
const mat4 transformInverseUniformScale(const mat4 &m);

// utility to make basic projection matrix
const mat4 makePerspective(real fovyRadians, real aspectRatio, 
	real znear, real zfar);
const mat4 makePerspective(real left, real right, real bottom, real top, 
	real znear, real zfar);
const mat4 makeOrthographic(real width, real height, 
	real znear, real zfar);
const mat4 makeOrthographic(real left, real right, real bottom, real top, 
	real znear, real zfar);

// utility to build a Frenet frame given position and target
const mat3 makeFrenet3(const vec3 &center, const vec3 &target, const vec3 &defaultUp);
const mat4 makeFrenet4(const vec3 &center, const vec3 &target, const vec3 &defaultUp);
const mat3 makeFrenetInverse3(const vec3 &center, const vec3 &target, const vec3 &defaultUp);
const mat4 makeFrenetInverse4(const vec3 &center, const vec3 &target, const vec3 &defaultUp);


}
}


#ifdef _CBTK_OPEN_SOURCE
#include "cbmath/inl/cbtkMatrix2.inl"
#include "cbmath/inl/cbtkMatrix3.inl"
#include "cbmath/inl/cbtkMatrix4.inl"
#endif	// _CBTK_OPEN_SOURCE


#endif	// __CBTK_MATH_MATRIX_H