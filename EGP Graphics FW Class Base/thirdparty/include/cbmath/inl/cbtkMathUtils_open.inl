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
	Cheezburger Toolkit Math Utilities, Open-Source Functions
	By Dan Buckstein
	Open-source utilities.
*/


using namespace cbtk;


//-----------------------------------------------------------------------------
//operations

template <typename type>
inline cbmath::real cbmath::length(const type n)
{
	return sqrtf(lengthSq(n));
}

template <typename type>
inline cbmath::real cbmath::lengthSq(const type n)
{
	return dot(n, n);
}

template <typename type>
inline cbmath::real cbmath::invLength(const type n)
{
	return divide(1.0f, length(n));
}

template <typename type>
inline cbmath::real cbmath::invLengthSq(const type n)
{
	return divide(1.0f, lengthSq(n));
}

template <typename type>
inline type cbmath::normalize(const type n)
{
	return (n*invLength(n));
}

template <typename type>
inline cbmath::real cbmath::projScalar(const type n, const type onto)
{
	return divide(dot(n, onto), lengthSq(onto));
}

template <typename type>
inline type cbmath::proj(const type n, const type onto)
{
	return (onto*projScalar(n, onto));
}


template <typename type>
inline type cbmath::recip(const type n)
{
	return (1.0f / n);
}


template <typename type>
inline type cbmath::clamp(const type value, const type minimum, const type maximum)
{
	return (value - minimum < 0 ? minimum : (value - maximum > 0 ? maximum : value));
}
