// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/math_functions.h"

#include <float.h>

_Static_assert( sizeof( int32_t ) == sizeof( int ), "Box2D expects int32_t and int to be the same" );

bool b2IsValidFloat( float a )
{
	if ( isnan( a ) )
	{
		return false;
	}

	if ( isinf( a ) )
	{
		return false;
	}

	return true;
}

bool b2IsValidVec2( b2Vec2 v )
{
	if ( isnan( v.x ) || isnan( v.y ) )
	{
		return false;
	}

	if ( isinf( v.x ) || isinf( v.y ) )
	{
		return false;
	}

	return true;
}

bool b2IsValidRotation( b2Rot q )
{
	if ( isnan( q.s ) || isnan( q.c ) )
	{
		return false;
	}

	if ( isinf( q.s ) || isinf( q.c ) )
	{
		return false;
	}

	return b2IsNormalizedRot( q );
}

bool b2IsValidPlane( b2Plane a )
{
	return b2IsValidVec2( a.normal ) && b2IsNormalized( a.normal ) && b2IsValidFloat( a.offset );
}

float b2Atan2( float y, float x )
{
	// Added check for (0,0) to match atan2f and avoid NaN
	if (x == 0.0f && y == 0.0f)
	{
		return 0.0f;
	}

	float ax = b2AbsFloat( x );
	float ay = b2AbsFloat( y );
	float mx = b2MaxFloat( ay, ax );
	float mn = b2MinFloat( ay, ax );
	float a = mn / mx;

	// Evaluate with second-order Horner grouping; preserve the rounding order.
	float s = a * a;
	float c = s * a;
	float q = s * s;
	float r = b2_atanC9 * q + b2_atanC5;
	float t = b2_atanC7 * q - b2_atanC3;
	r = r * s + t;
	r = r * c + a;

	// Map to full circle
	if ( ay > ax )
	{
		r = 0.5f * B2_PI - r;
	}

	if ( x < 0 )
	{
		r = B2_PI - r;
	}

	if ( y < 0 )
	{
		r = -r;
	}

	return r;
}

// Approximate cosine and sine for determinism. In my testing cosf and sinf produced
// the same results on x64 and ARM using MSVC, GCC, and Clang. However, I don't trust
// this result.
// https://en.wikipedia.org/wiki/Bh%C4%81skara_I%27s_sine_approximation_formula
b2CosSin b2ComputeCosSin( float radians )
{
	float x = b2UnwindAngle( radians );
	float pi2 = B2_PI * B2_PI;

	// cosine needs angle in [-pi/2, pi/2]
	float c;
	if ( x < -0.5f * B2_PI )
	{
		float y = x + B2_PI;
		float y2 = y * y;
		c = -( pi2 - 4.0f * y2 ) / ( pi2 + y2 );
	}
	else if ( x > 0.5f * B2_PI )
	{
		float y = x - B2_PI;
		float y2 = y * y;
		c = -( pi2 - 4.0f * y2 ) / ( pi2 + y2 );
	}
	else
	{
		float y2 = x * x;
		c = ( pi2 - 4.0f * y2 ) / ( pi2 + y2 );
	}

	// sine needs angle in [0, pi]
	float s;
	if ( x < 0.0f )
	{
		float y = x + B2_PI;
		s = -16.0f * y * ( B2_PI - y ) / ( 5.0f * pi2 - 4.0f * y * ( B2_PI - y ) );
	}
	else
	{
		s = 16.0f * x * ( B2_PI - x ) / ( 5.0f * pi2 - 4.0f * x * ( B2_PI - x ) );
	}

	float mag = sqrtf( s * s + c * c );
	float invMag = mag > 0.0 ? 1.0f / mag : 0.0f;
	b2CosSin cs = { c * invMag, s * invMag };
	return cs;
}

b2Rot b2ComputeRotationBetweenUnitVectors(b2Vec2 v1, b2Vec2 v2)
{
	B2_ASSERT( b2AbsFloat( 1.0f - b2Length( v1 ) ) < 100.0f * FLT_EPSILON );
	B2_ASSERT( b2AbsFloat( 1.0f - b2Length( v2 ) ) < 100.0f * FLT_EPSILON );

	b2Rot rot;
	rot.c = b2Dot( v1, v2 );
	rot.s = b2Cross( v1, v2 );
	return b2NormalizeRot( rot );
}
