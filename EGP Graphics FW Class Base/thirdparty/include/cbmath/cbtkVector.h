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
	Cheezburger Toolkit Vector Classes
	By Dan Buckstein
	Standalone vector classes.
	Contains structures for 2D, 3D and 4D vector, with associated functions
		and operators.
*/


#ifndef	__CBTK_MATH_VECTOR_H
#define	__CBTK_MATH_VECTOR_H


#include "cbmath/cbtkMathUtils.h"


namespace cbtk
{
namespace cbmath
{


union vec2;
union vec3;
union vec4;


//-----------------------------------------------------------------------------
// structures

union vec2
{
	// ctor
	vec2();
	explicit vec2(real X, real Y);
	explicit vec2(real V);
	explicit vec2(const real *Vp);
	explicit vec2(const vec3 &v3);
	explicit vec2(const vec4 &v4);

	// set and reset
	vec2 &set();
	vec2 &set(real X, real Y);
	vec2 &set(real V);
	vec2 &set(const real *Vp);
	vec2 &set(const vec3 &v3);
	vec2 &set(const vec4 &v4);

	// data
	struct { real x,y; };
	struct { real s,t; };
	real v[2];
};
static const vec2 v2zero;
static const vec2 v2x(1, 0);
static const vec2 v2y(0, 1);


union vec3
{
	// ctor
	vec3();
	explicit vec3(real X, real Y, real Z);
	explicit vec3(real V);
	explicit vec3(const real *Vp);
	explicit vec3(const vec2 &v2, real Z=0.0f);
	explicit vec3(const vec4 &v4);

	// set and reset
	vec3 &set();
	vec3 &set(real X, real Y, real Z);
	vec3 &set(real V);
	vec3 &set(const real *Vp);
	vec3 &set(const vec2 &v2, real Z=0.0f);
	vec3 &set(const vec4 &v4);

	// data
	struct { real x,y,z; };
	struct { real r,g,b; };
	struct { real s,t,p; };
	struct { vec2 xy; };
	real v[3];
};
static const vec3 v3zero;
static const vec3 v3x(1, 0, 0);
static const vec3 v3y(0, 1, 0);
static const vec3 v3z(0, 0, 1);


union vec4
{
	// ctor
	vec4();
	explicit vec4(real X, real Y, real Z, real W);
	explicit vec4(real V);
	explicit vec4(const real *Vp);
	explicit vec4(const vec2 &v2, real Z=0.0f, real W=0.0f);
	explicit vec4(const vec3 &v3, real W=0.0f);

	// set and reset
	vec4 &set();
	vec4 &set(real X, real Y, real Z, real W);
	vec4 &set(real V);
	vec4 &set(const real *Vp);
	vec4 &set(const vec2 &v2, real Z=0.0f, real W=0.0f);
	vec4 &set(const vec3 &v3, real W=0.0f);

	// data
	struct { real x,y,z,w; };
	struct { real r,g,b,a; };
	struct { real s,t,p,q; };
	struct { vec2 xy; };
	struct { vec3 xyz; };
	real v[4];
};
static const vec4 v4zero;
static const vec4 v4x(1, 0, 0, 0);
static const vec4 v4y(0, 1, 0, 0);
static const vec4 v4z(0, 0, 1, 0);
static const vec4 v4w(0, 0, 0, 1);



//-----------------------------------------------------------------------------
// operators

real dot(const vec2 &v0, const vec2 &v1);
real dot(const vec3 &v0, const vec3 &v1);
real dot(const vec4 &v0, const vec4 &v1);


const vec2 operator +(const vec2 &v0, const vec2 &v1);
const vec3 operator +(const vec3 &v0, const vec3 &v1);
const vec4 operator +(const vec4 &v0, const vec4 &v1);

vec2 &operator +=(vec2 &v0, const vec2 &v1);
vec3 &operator +=(vec3 &v0, const vec3 &v1);
vec4 &operator +=(vec4 &v0, const vec4 &v1);

const vec2 operator +(const vec2 &v);
const vec3 operator +(const vec3 &v);
const vec4 operator +(const vec4 &v);

const vec2 operator -(const vec2 &v0, const vec2 &v1);
const vec3 operator -(const vec3 &v0, const vec3 &v1);
const vec4 operator -(const vec4 &v0, const vec4 &v1);

vec2 &operator -=(vec2 &v0, const vec2 &v1);
vec3 &operator -=(vec3 &v0, const vec3 &v1);
vec4 &operator -=(vec4 &v0, const vec4 &v1);

const vec2 operator -(const vec2 &v);
const vec3 operator -(const vec3 &v);
const vec4 operator -(const vec4 &v);

const vec2 operator *(const vec2 &v, const real s);
const vec3 operator *(const vec3 &v, const real s);
const vec4 operator *(const vec4 &v, const real s);

const vec2 operator *(const real s, const vec2 &v);
const vec3 operator *(const real s, const vec3 &v);
const vec4 operator *(const real s, const vec4 &v);

vec2 &operator *=(vec2 &v, const real s);
vec3 &operator *=(vec3 &v, const real s);
vec4 &operator *=(vec4 &v, const real s);

const vec2 operator /(const vec2 &v, const real s);
const vec3 operator /(const vec3 &v, const real s);
const vec4 operator /(const vec4 &v, const real s);

const vec2 operator /(const real s, const vec2 &v);
const vec3 operator /(const real s, const vec3 &v);
const vec4 operator /(const real s, const vec4 &v);

vec2 &operator /=(vec2 &v, const real s);
vec3 &operator /=(vec3 &v, const real s);
vec4 &operator /=(vec4 &v, const real s);

bool operator ==(const vec2 &v0, const vec2 &v1);
bool operator ==(const vec3 &v0, const vec3 &v1);
bool operator ==(const vec4 &v0, const vec4 &v1);

bool operator !=(const vec2 &v0, const vec2 &v1);
bool operator !=(const vec3 &v0, const vec3 &v1);
bool operator !=(const vec4 &v0, const vec4 &v1);



//-----------------------------------------------------------------------------
// special functions

// rotate a 2D vector given an angle in radians, or the trigonometry parts 
//	required to perform the rotation: cos(a) and sin(a)
const vec2 rotate(const real angle, const vec2 &v);
const vec2 rotate(const real cosine, const real sine, const vec2 &v);

// cross product for 3D vectors
const vec3 cross(const vec3 &v0, const vec3 &v1);
const vec3 crossNormalize(const vec3 &v0, const vec3 &v1);

// compute the normal given three spatial points
// normal is equal to the normalized version of (v1-v0)x(v2-v0)
const vec3 computeTriNormal(const vec3 &v0, const vec3 &v1, const vec3 &v2);

// compute the area of the triangle given three spatial points
// equal to the magnitude of (v1-v0)x(v2-v0)
real computeTriArea(const vec3 &v0, const vec3 &v1, const vec3 &v2);

// combination of the above two functions
real computeTriAreaNormal(const vec3 &v0, const vec3 &v1, const vec3 &v2, vec3 &normal);

// Gram-Schmidt orthogonalization algorithm
void GramSchmidt(const vec2 &base, vec2 &v);
void GramSchmidt(const vec3 &base, vec3 &v);
void GramSchmidt(const vec3 &base, vec3 &v0, vec3 &v1);


}
}


#ifdef _CBTK_OPEN_SOURCE
#include "cbmath/inl/cbtkVector2.inl"
#include "cbmath/inl/cbtkVector3.inl"
#include "cbmath/inl/cbtkVector4.inl"
#endif	// _CBTK_OPEN_SOURCE


#endif	// __CBTK_MATH_VECTOR_H