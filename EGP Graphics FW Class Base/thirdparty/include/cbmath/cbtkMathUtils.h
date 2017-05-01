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
	Cheezburger Toolkit Math Utilities
	By Dan Buckstein
	Basic helper functions for math stuff.
*/


#ifndef	__CBTK_MATH_MATHUTILS_H
#define	__CBTK_MATH_MATHUTILS_H


namespace cbtk
{
namespace cbmath
{


	// SHORTHAND TYPEDEF FOR REAL NUMBERS: "FLOATS" USED TO REPRESENT DECIMALS
typedef float real;


// random number generators
	// generates a random number as a real
real random();
	// generates a random number between [minv, maxv]
real random(const real minv, const real maxv);
	// generates a random number between [ 0, 1] (normalized)
real randomNorm();
	// generates a random number between [-1, 1] (symmetric normalized)
real randomSymm();


// statistical formulas and utilities
	// produce the summation of 1 to n
unsigned int summation(unsigned int n);
real summationf(unsigned int n);

	// produce the factorial of n (n!)
unsigned int factorial(unsigned int n);
real factorialf(unsigned int n);

	// compute the permutation of n permute k
unsigned int permutation(unsigned int n, unsigned int k);
real permutationf(unsigned int n, unsigned int k);

	// compute the combination of n choose k
unsigned int combination(unsigned int n, unsigned int k);
real combinationf(unsigned int n, unsigned int k);


// operations

	// transform interpolation parameters for easing
real easeInParam(real t);
real easeOutParam(real t);


	// safe division: prevents 'NaN' results by returning zero if the denominator is zero
real divide(const real numerator, const real denominator);


	// stand-in linear operations for reals
real absVal(const real n);
real sign(const real n);
real dot(const real n0, const real n1);
real length(const real n);
real normalize(const real n);

	// linear algebra operations
template <typename type>
real length(const type n);
template <typename type>
real lengthSq(const type n);
template <typename type>
real invLength(const type n);
template <typename type>
real invLengthSq(const type n);
template <typename type>
type normalize(const type n);
template <typename type>
real projScalar(const type n, const type onto);
template <typename type>
type proj(const type n, const type onto);
template <typename type>
type recip(const type n);
template <typename type>
type clamp(const type value, const type minimum, const type maximum);



// constants
#define _deg2rad		0.01745329251994329576923690768489
#define _rad2deg		57.295779513082320876798154814105
#define _pi				3.1415926535897932384626433832795
#define _halfpi			1.5707963267948966192313216916398
#define _twopi			6.283185307179586476925286766559
#define _randmax		32767.0
#define _invrandmax		3.0518509475997192297128208258309e-5

#define _deg2rad_f		0.01745329251994329576923690768489f
#define _rad2deg_f		57.295779513082320876798154814105f
#define _pi_f			3.1415926535897932384626433832795f
#define _halfpi_f		1.5707963267948966192313216916398f
#define _twopi_f		6.283185307179586476925286766559f
#define _randmax_f		32767.0f
#define _invrandmax_f	3.0518509475997192297128208258309e-5f

#define Deg2Rad(value)	(value*_deg2rad_f)
#define Rad2Deg(value)	(value*_rad2deg_f)
#define TwoPi()			(_twopi_f)
#define Pi()			(_pi_f)
#define HalfPi()		(_halfpi_f)
#define RandMax()		(_randmax_f)
#define InvRandMax()	(_invrandmax_f)


}
}


#include "cbmath/inl/cbtkMathUtils_open.inl"
#ifdef _CBTK_OPEN_SOURCE
#include "cbmath/inl/cbtkMathUtils.inl"
#endif	// _CBTK_OPEN_SOURCE


#endif	// __CBTK_MATH_MATHUTILS_H