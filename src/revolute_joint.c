// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#if defined( _MSC_VER ) && !defined( _CRT_SECURE_NO_WARNINGS )
#define _CRT_SECURE_NO_WARNINGS
#endif

// Assume masked floating-point exceptions so GCC can vectorize the guarded angle loop.
#if defined( __GNUC__ ) && !defined( __clang__ )
#pragma GCC optimize( "no-trapping-math" )
#endif

#include "body.h"
#include "core.h"
#include "joint.h"
#include "solver.h"
#include "solver_set.h"
#include "world.h"

// needed for dll export
#include "box2d/box2d.h"

#include <stdio.h>

void b2RevoluteJoint_EnableSpring( b2JointId jointId, bool enableSpring )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	if ( enableSpring != joint->revoluteJoint.enableSpring )
	{
		joint->revoluteJoint.enableSpring = enableSpring;
		joint->revoluteJoint.springImpulse = 0.0f;
	}
}

bool b2RevoluteJoint_IsSpringEnabled( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	return joint->revoluteJoint.enableSpring;
}

void b2RevoluteJoint_SetSpringHertz( b2JointId jointId, float hertz )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	joint->revoluteJoint.hertz = hertz;
}

float b2RevoluteJoint_GetSpringHertz( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	return joint->revoluteJoint.hertz;
}

void b2RevoluteJoint_SetSpringDampingRatio( b2JointId jointId, float dampingRatio )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	joint->revoluteJoint.dampingRatio = dampingRatio;
}

float b2RevoluteJoint_GetSpringDampingRatio( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	return joint->revoluteJoint.dampingRatio;
}

void b2RevoluteJoint_SetTargetAngle( b2JointId jointId, float angle )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	joint->revoluteJoint.targetAngle = angle;
}

float b2RevoluteJoint_GetTargetAngle( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	return joint->revoluteJoint.targetAngle;
}

float b2RevoluteJoint_GetAngle( b2JointId jointId )
{
	b2World* world = b2GetWorld( jointId.world0 );
	b2JointSim* jointSim = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	b2Transform transformA = b2GetBodyTransform( world, jointSim->bodyIdA );
	b2Transform transformB = b2GetBodyTransform( world, jointSim->bodyIdB );

	float angle = b2RelativeAngle( transformB.q, transformA.q ) - jointSim->revoluteJoint.referenceAngle;
	angle = b2UnwindAngle( angle );
	return angle;
}

void b2RevoluteJoint_EnableLimit( b2JointId jointId, bool enableLimit )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	if ( enableLimit != joint->revoluteJoint.enableLimit )
	{
		joint->revoluteJoint.enableLimit = enableLimit;
		joint->revoluteJoint.lowerImpulse = 0.0f;
		joint->revoluteJoint.upperImpulse = 0.0f;
	}
}

bool b2RevoluteJoint_IsLimitEnabled( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	return joint->revoluteJoint.enableLimit;
}

float b2RevoluteJoint_GetLowerLimit( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	return joint->revoluteJoint.lowerAngle;
}

float b2RevoluteJoint_GetUpperLimit( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	return joint->revoluteJoint.upperAngle;
}

void b2RevoluteJoint_SetLimits( b2JointId jointId, float lower, float upper )
{
	B2_ASSERT( lower <= upper );
	B2_ASSERT( lower >= -0.99f * B2_PI );
	B2_ASSERT( upper <= 0.99f * B2_PI );

	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	if ( lower != joint->revoluteJoint.lowerAngle || upper != joint->revoluteJoint.upperAngle )
	{
		joint->revoluteJoint.lowerAngle = b2MinFloat( lower, upper );
		joint->revoluteJoint.upperAngle = b2MaxFloat( lower, upper );
		joint->revoluteJoint.lowerImpulse = 0.0f;
		joint->revoluteJoint.upperImpulse = 0.0f;
	}
}

void b2RevoluteJoint_EnableMotor( b2JointId jointId, bool enableMotor )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	if ( enableMotor != joint->revoluteJoint.enableMotor )
	{
		joint->revoluteJoint.enableMotor = enableMotor;
		joint->revoluteJoint.motorImpulse = 0.0f;
	}
}

bool b2RevoluteJoint_IsMotorEnabled( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	return joint->revoluteJoint.enableMotor;
}

void b2RevoluteJoint_SetMotorSpeed( b2JointId jointId, float motorSpeed )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	joint->revoluteJoint.motorSpeed = motorSpeed;
}

float b2RevoluteJoint_GetMotorSpeed( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	return joint->revoluteJoint.motorSpeed;
}

float b2RevoluteJoint_GetMotorTorque( b2JointId jointId )
{
	b2World* world = b2GetWorld( jointId.world0 );
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	return world->inv_h * joint->revoluteJoint.motorImpulse;
}

void b2RevoluteJoint_SetMaxMotorTorque( b2JointId jointId, float torque )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	joint->revoluteJoint.maxMotorTorque = torque;
}

float b2RevoluteJoint_GetMaxMotorTorque( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	return joint->revoluteJoint.maxMotorTorque;
}

b2Vec2 b2GetRevoluteJointForce( b2World* world, b2JointSim* base )
{
	b2Vec2 force = b2MulSV( world->inv_h, base->revoluteJoint.linearImpulse );
	return force;
}

float b2GetRevoluteJointTorque( b2World* world, b2JointSim* base )
{
	const b2RevoluteJoint* revolute = &base->revoluteJoint;
	float torque = world->inv_h * ( revolute->motorImpulse + revolute->lowerImpulse - revolute->upperImpulse );
	return torque;
}

// Point-to-point constraint
// C = p2 - p1
// Cdot = v2 - v1
//      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
// J = [-I -r1_skew I r2_skew ]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

// Motor constraint
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
// K = invI1 + invI2

void b2PrepareRevoluteJoint( b2JointSim* base, b2StepContext* context )
{
	B2_ASSERT( base->type == b2_revoluteJoint );

	// chase body id to the solver set where the body lives
	int idA = base->bodyIdA;
	int idB = base->bodyIdB;

	b2World* world = context->world;

	b2Body* bodyA = b2BodyArray_Get( &world->bodies, idA );
	b2Body* bodyB = b2BodyArray_Get( &world->bodies, idB );

	B2_ASSERT( bodyA->setIndex == b2_awakeSet || bodyB->setIndex == b2_awakeSet );
	b2SolverSet* setA = b2SolverSetArray_Get( &world->solverSets, bodyA->setIndex );
	b2SolverSet* setB = b2SolverSetArray_Get( &world->solverSets, bodyB->setIndex );

	int localIndexA = bodyA->localIndex;
	int localIndexB = bodyB->localIndex;

	b2BodySim* bodySimA = b2BodySimArray_Get( &setA->bodySims, localIndexA );
	b2BodySim* bodySimB = b2BodySimArray_Get( &setB->bodySims, localIndexB );

	float mA = bodySimA->invMass;
	float iA = bodySimA->invInertia;
	float mB = bodySimB->invMass;
	float iB = bodySimB->invInertia;

	base->invMassA = mA;
	base->invMassB = mB;
	base->invIA = iA;
	base->invIB = iB;

	b2RevoluteJoint* joint = &base->revoluteJoint;

	joint->indexA = bodyA->setIndex == b2_awakeSet ? localIndexA : B2_NULL_INDEX;
	joint->indexB = bodyB->setIndex == b2_awakeSet ? localIndexB : B2_NULL_INDEX;

	// initial anchors in world space
	joint->anchorA = b2RotateVector( bodySimA->transform.q, b2Sub( base->localOriginAnchorA, bodySimA->localCenter ) );
	joint->anchorB = b2RotateVector( bodySimB->transform.q, b2Sub( base->localOriginAnchorB, bodySimB->localCenter ) );
	joint->deltaCenter = b2Sub( bodySimB->center, bodySimA->center );
	joint->deltaAngle = b2RelativeAngle( bodySimB->transform.q, bodySimA->transform.q );

	float k = iA + iB;
	joint->axialMass = k > 0.0f ? 1.0f / k : 0.0f;

	joint->springSoftness = b2MakeSoft( joint->hertz, joint->dampingRatio, context->h );
	joint->maxMotorImpulse = context->h * joint->maxMotorTorque;
	joint->relaxGeometryValid = false;

	if ( context->enableWarmStarting == false )
	{
		joint->linearImpulse = b2Vec2_zero;
		joint->springImpulse = 0.0f;
		joint->motorImpulse = 0.0f;
		joint->lowerImpulse = 0.0f;
		joint->upperImpulse = 0.0f;
	}
}

void b2WarmStartRevoluteJoint( b2JointSim* base, b2StepContext* context )
{
	B2_ASSERT( base->type == b2_revoluteJoint );

	float mA = base->invMassA;
	float mB = base->invMassB;
	float iA = base->invIA;
	float iB = base->invIB;

	// dummy state for static bodies
	b2BodyState dummyState = b2_identityBodyState;

	b2RevoluteJoint* joint = &base->revoluteJoint;
	b2BodyState* stateA = joint->indexA == B2_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b2BodyState* stateB = joint->indexB == B2_NULL_INDEX ? &dummyState : context->states + joint->indexB;

	b2Vec2 rA = b2RotateVector( stateA->deltaRotation, joint->anchorA );
	b2Vec2 rB = b2RotateVector( stateB->deltaRotation, joint->anchorB );

	float axialImpulse = joint->springImpulse + joint->motorImpulse + joint->lowerImpulse - joint->upperImpulse;

	stateA->linearVelocity = b2MulSub( stateA->linearVelocity, mA, joint->linearImpulse );
	stateA->angularVelocity -= iA * ( b2Cross( rA, joint->linearImpulse ) + axialImpulse );

	stateB->linearVelocity = b2MulAdd( stateB->linearVelocity, mB, joint->linearImpulse );
	stateB->angularVelocity += iB * ( b2Cross( rB, joint->linearImpulse ) + axialImpulse );
}

static void b2PrepareRevoluteGeometry( b2JointSim* base, b2StepContext* context, bool useBias,
                                     const b2BodyState* stateA, const b2BodyState* stateB )
{
	b2RevoluteJoint* joint = &base->revoluteJoint;
	float mA = base->invMassA, mB = base->invMassB;
	float iA = base->invIA, iB = base->invIB;
	bool fixedRotation = ( iA + iB == 0.0f );

	joint->relaxAnchorA = b2RotateVector( stateA->deltaRotation, joint->anchorA );
	joint->relaxAnchorB = b2RotateVector( stateB->deltaRotation, joint->anchorB );
	b2Vec2 rA = joint->relaxAnchorA;
	b2Vec2 rB = joint->relaxAnchorB;
	b2Mat22 K;
	K.cx.x = mA + mB + rA.y * rA.y * iA + rB.y * rB.y * iB;
	K.cy.x = -rA.y * rA.x * iA - rB.y * rB.x * iB;
	K.cx.y = K.cy.x;
	K.cy.y = mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB;
	joint->relaxK = K;
	float det = K.cx.x * K.cy.y - K.cy.x * K.cx.y;
	joint->relaxInvDet = det != 0.0f ? 1.0f / det : det;
	if ( ( joint->enableSpring || joint->enableLimit ) && fixedRotation == false )
	{
		float angle = b2RelativeAngle( stateB->deltaRotation, stateA->deltaRotation ) + joint->deltaAngle;
		if ( joint->enableSpring )
		{
			joint->relaxSpringBias = joint->springSoftness.biasRate * b2UnwindAngle( angle - joint->targetAngle );
		}
		if ( joint->enableLimit )
		{
			joint->relaxLimitAngle = b2UnwindAngle( angle - joint->referenceAngle );
			if ( useBias == false )
			{
				float lowerC = joint->relaxLimitAngle - joint->lowerAngle;
				float upperC = joint->upperAngle - joint->relaxLimitAngle;
				joint->relaxLowerBias = lowerC > 0.0f ? lowerC * context->inv_h : 0.0f;
				joint->relaxUpperBias = upperC > 0.0f ? upperC * context->inv_h : 0.0f;
			}
		}
	}
	joint->relaxGeometryValid = !useBias;
}

void b2SolveRevoluteJoint( b2JointSim* base, b2StepContext* context, bool useBias )
{
	B2_ASSERT( base->type == b2_revoluteJoint );

	float mA = base->invMassA;
	float mB = base->invMassB;
	float iA = base->invIA;
	float iB = base->invIB;

	// dummy state for static bodies
	b2BodyState dummyState = b2_identityBodyState;

	b2RevoluteJoint* joint = &base->revoluteJoint;

	b2BodyState* stateA = joint->indexA == B2_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b2BodyState* stateB = joint->indexB == B2_NULL_INDEX ? &dummyState : context->states + joint->indexB;

	b2Vec2 vA = stateA->linearVelocity;
	float wA = stateA->angularVelocity;
	b2Vec2 vB = stateB->linearVelocity;
	float wB = stateB->angularVelocity;

	bool fixedRotation = ( iA + iB == 0.0f );

	if ( joint->relaxGeometryValid == false )
	{
		b2PrepareRevoluteGeometry( base, context, useBias, stateA, stateB );
	}
	// Relaxation geometry survives velocity integration and warm starting.
	// Invalidate only before integration; extra biased passes share these positions.
	if ( useBias )
	{
		joint->relaxGeometryValid = context->biasedIteration + 1 < context->biasedIterations;
	}

	// Solve spring.
	if ( joint->enableSpring && fixedRotation == false )
	{
		float bias = joint->relaxSpringBias;
		float massScale = joint->springSoftness.massScale;
		float impulseScale = joint->springSoftness.impulseScale;

		float Cdot = wB - wA;
		float impulse = -massScale * joint->axialMass * ( Cdot + bias ) - impulseScale * joint->springImpulse;
		joint->springImpulse += impulse;

		wA -= iA * impulse;
		wB += iB * impulse;
	}

	// Solve motor constraint.
	if ( joint->enableMotor && fixedRotation == false )
	{
		float Cdot = wB - wA - joint->motorSpeed;
		float impulse = -joint->axialMass * Cdot;
		float oldImpulse = joint->motorImpulse;
		float maxImpulse = joint->maxMotorImpulse;
		float accumulatedImpulse = joint->motorImpulse + impulse;
		// Retain the original clamp when a negative torque limit reverses the bounds.
		joint->motorImpulse = maxImpulse < 0.0f
			? b2ClampFloat( accumulatedImpulse, -maxImpulse, maxImpulse )
			: b2MinFloat( maxImpulse, b2MaxFloat( -maxImpulse, accumulatedImpulse ) );
		impulse = joint->motorImpulse - oldImpulse;

		wA -= iA * impulse;
		wB += iB * impulse;
	}

	if ( joint->enableLimit && fixedRotation == false )
	{
		// Lower limit
		{
			float bias;
			float massScale = 1.0f;
			float impulseScale = 0.0f;
			if ( useBias )
			{
				float C = joint->relaxLimitAngle - joint->lowerAngle;
				if ( C > 0.0f )
				{
					// speculation
					bias = C * context->inv_h;
				}
				else
				{
					bias = base->constraintSoftness.biasRate * C;
					massScale = base->constraintSoftness.massScale;
					impulseScale = base->constraintSoftness.impulseScale;
				}
			}
			else
			{
				bias = joint->relaxLowerBias;
			}

			float Cdot = wB - wA;
			float oldImpulse = joint->lowerImpulse;
			float impulse = -massScale * joint->axialMass * ( Cdot + bias ) - impulseScale * oldImpulse;
			joint->lowerImpulse = b2MaxFloat( oldImpulse + impulse, 0.0f );
			impulse = joint->lowerImpulse - oldImpulse;

			wA -= iA * impulse;
			wB += iB * impulse;
		}

		// Upper limit
		// Note: signs are flipped to keep C positive when the constraint is satisfied.
		// This also keeps the impulse positive when the limit is active.
		{
			float bias;
			float massScale = 1.0f;
			float impulseScale = 0.0f;
			if ( useBias )
			{
				float C = joint->upperAngle - joint->relaxLimitAngle;
				if ( C > 0.0f )
				{
					// speculation
					bias = C * context->inv_h;
				}
				else
				{
					bias = base->constraintSoftness.biasRate * C;
					massScale = base->constraintSoftness.massScale;
					impulseScale = base->constraintSoftness.impulseScale;
				}
			}
			else
			{
				bias = joint->relaxUpperBias;
			}

			// sign flipped on Cdot
			float Cdot = wA - wB;
			float oldImpulse = joint->upperImpulse;
			float impulse = -massScale * joint->axialMass * ( Cdot + bias ) - impulseScale * oldImpulse;
			joint->upperImpulse = b2MaxFloat( oldImpulse + impulse, 0.0f );
			impulse = joint->upperImpulse - oldImpulse;

			// sign flipped on applied impulse
			wA += iA * impulse;
			wB -= iB * impulse;
		}
	}

	// Solve point-to-point constraint
	{
		// J = [-I -r1_skew I r2_skew]
		// r_skew = [-ry; rx]
		// K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x]
		//     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB]

		// current anchors
		b2Vec2 rA = joint->relaxAnchorA;
		b2Vec2 rB = joint->relaxAnchorB;

		b2Vec2 Cdot = b2Sub( b2Add( vB, b2CrossSV( wB, rB ) ), b2Add( vA, b2CrossSV( wA, rA ) ) );

		b2Vec2 bias = b2Vec2_zero;
		float massScale = 1.0f;
		float impulseScale = 0.0f;
		if ( useBias )
		{
			b2Vec2 dcA = stateA->deltaPosition;
			b2Vec2 dcB = stateB->deltaPosition;

			b2Vec2 separation = b2Add( b2Add( b2Sub( dcB, dcA ), b2Sub( rB, rA ) ), joint->deltaCenter );
			bias = b2MulSV( base->constraintSoftness.biasRate, separation );
			massScale = base->constraintSoftness.massScale;
			impulseScale = base->constraintSoftness.impulseScale;
		}

		b2Mat22 K = joint->relaxK;
		b2Vec2 rhs = b2Add( Cdot, bias );
		float det = joint->relaxInvDet;
		// Keep b2Solve22's arithmetic order instead of multiplying by an inverse matrix.
		b2Vec2 b = { det * ( K.cy.y * rhs.x - K.cy.x * rhs.y ), det * ( K.cx.x * rhs.y - K.cx.y * rhs.x ) };

		b2Vec2 impulse;
		impulse.x = -massScale * b.x - impulseScale * joint->linearImpulse.x;
		impulse.y = -massScale * b.y - impulseScale * joint->linearImpulse.y;
		joint->linearImpulse.x += impulse.x;
		joint->linearImpulse.y += impulse.y;

		vA = b2MulSub( vA, mA, impulse );
		wA -= iA * b2Cross( rA, impulse );
		vB = b2MulAdd( vB, mB, impulse );
		wB += iB * b2Cross( rB, impulse );
	}

	stateA->linearVelocity = vA;
	stateA->angularVelocity = wA;
	stateB->linearVelocity = vB;
	stateB->angularVelocity = wB;
}

#if 0
void b2RevoluteJoint::Dump()
{
	int32 indexA = joint->bodyA->joint->islandIndex;
	int32 indexB = joint->bodyB->joint->islandIndex;

	b2Dump("  b2RevoluteJointDef jd;\n");
	b2Dump("  jd.bodyA = bodies[%d];\n", indexA);
	b2Dump("  jd.bodyB = bodies[%d];\n", indexB);
	b2Dump("  jd.collideConnected = bool(%d);\n", joint->collideConnected);
	b2Dump("  jd.localAnchorA.Set(%.9g, %.9g);\n", joint->localAnchorA.x, joint->localAnchorA.y);
	b2Dump("  jd.localAnchorB.Set(%.9g, %.9g);\n", joint->localAnchorB.x, joint->localAnchorB.y);
	b2Dump("  jd.referenceAngle = %.9g;\n", joint->referenceAngle);
	b2Dump("  jd.enableLimit = bool(%d);\n", joint->enableLimit);
	b2Dump("  jd.lowerAngle = %.9g;\n", joint->lowerAngle);
	b2Dump("  jd.upperAngle = %.9g;\n", joint->upperAngle);
	b2Dump("  jd.enableMotor = bool(%d);\n", joint->enableMotor);
	b2Dump("  jd.motorSpeed = %.9g;\n", joint->motorSpeed);
	b2Dump("  jd.maxMotorTorque = %.9g;\n", joint->maxMotorTorque);
	b2Dump("  joints[%d] = joint->world->CreateJoint(&jd);\n", joint->index);
}
#endif

void b2DrawRevoluteJoint( b2DebugDraw* draw, b2JointSim* base, b2Transform transformA, b2Transform transformB, float drawSize )
{
	B2_ASSERT( base->type == b2_revoluteJoint );

	b2RevoluteJoint* joint = &base->revoluteJoint;

	b2Vec2 pA = b2TransformPoint( transformA, base->localOriginAnchorA );
	b2Vec2 pB = b2TransformPoint( transformB, base->localOriginAnchorB );

	b2HexColor c1 = b2_colorGray;
	b2HexColor c2 = b2_colorGreen;
	b2HexColor c3 = b2_colorRed;

	const float L = drawSize;
	// draw->drawPoint(pA, 3.0f, b2_colorGray40, draw->context);
	// draw->drawPoint(pB, 3.0f, b2_colorLightBlue, draw->context);
	draw->DrawCircleFcn( pB, L, c1, draw->context );

	float angle = b2RelativeAngle( transformB.q, transformA.q );

	b2Rot rot = b2MakeRot( angle );
	b2Vec2 r = { L * rot.c, L * rot.s };
	b2Vec2 pC = b2Add( pB, r );
	draw->DrawSegmentFcn( pB, pC, c1, draw->context );

	if ( draw->drawJointExtras )
	{
		float jointAngle = b2UnwindAngle( angle - joint->referenceAngle );
		char buffer[32];
		snprintf( buffer, 32, " %.1f deg", 180.0f * jointAngle / B2_PI );
		draw->DrawStringFcn( pC, buffer, b2_colorWhite, draw->context );
	}

	float lowerAngle = joint->lowerAngle + joint->referenceAngle;
	float upperAngle = joint->upperAngle + joint->referenceAngle;

	if ( joint->enableLimit )
	{
		b2Rot rotLo = b2MakeRot( lowerAngle );
		b2Vec2 rlo = { L * rotLo.c, L * rotLo.s };

		b2Rot rotHi = b2MakeRot( upperAngle );
		b2Vec2 rhi = { L * rotHi.c, L * rotHi.s };

		draw->DrawSegmentFcn( pB, b2Add( pB, rlo ), c2, draw->context );
		draw->DrawSegmentFcn( pB, b2Add( pB, rhi ), c3, draw->context );

		b2Rot rotRef = b2MakeRot( joint->referenceAngle );
		b2Vec2 ref = (b2Vec2){ L * rotRef.c, L * rotRef.s };
		draw->DrawSegmentFcn( pB, b2Add( pB, ref ), b2_colorBlue, draw->context );
	}

	b2HexColor color = b2_colorGold;
	draw->DrawSegmentFcn( transformA.p, pA, color, draw->context );
	draw->DrawSegmentFcn( pA, pB, color, draw->context );
	draw->DrawSegmentFcn( transformB.p, pB, color, draw->context );

	// char buffer[32];
	// sprintf(buffer, "%.1f", b2Length(joint->impulse));
	// draw->DrawString(pA, buffer, draw->context);
}

#if defined( B2_SIMD_SSE2 )
#include <emmintrin.h>

typedef union b2RevoluteWide
{
	__m128 v;
	float lane[4];
} b2RevoluteWide;

struct b2RevoluteGroup
{
	b2RevoluteWide mA, mB, iA, iB, axialMass, maxImpulse, speed;
	b2RevoluteWide rAx, rAy, rBx, rBy, k11, k12, k22, invDet, lowerBias, upperBias;
	b2RevoluteWide motor, lower, upper, linearX, linearY;
	b2RevoluteWide pointMass, pointScale, biasRate, biasX, biasY;
	int indexA[4], indexB[4];
	int eligible; // -1 until first solve, then fixed for this world step
	int remaining; // passes left in this phase; relaxation writes packed impulses back at zero
	b2RevoluteWide anchorAx, anchorAy, anchorBx, anchorBy, centerX, centerY;
	b2RevoluteWide lowerC, upperC; // pose errors retained independently of phase biases
};

int b2GetRevoluteGroupSize( void )
{
	return sizeof( b2RevoluteGroup );
}

void b2InitRevoluteGroup( b2RevoluteGroup* group )
{
	*group = (b2RevoluteGroup){ 0 };
	group->eligible = -1;
	for ( int lane = 0; lane < 4; ++lane )
	{
		group->indexA[lane] = B2_NULL_INDEX;
		group->indexB[lane] = B2_NULL_INDEX;
	}
}

static inline void b2GatherRevoluteVelocities( const b2BodyState* states, const int* indices, __m128* x, __m128* y,
									 __m128* w, __m128* flags )
{
	__m128 a = indices[0] == B2_NULL_INDEX ? _mm_setzero_ps() : _mm_loadu_ps( (const float*)( states + indices[0] ) );
	__m128 b = indices[1] == B2_NULL_INDEX ? _mm_setzero_ps() : _mm_loadu_ps( (const float*)( states + indices[1] ) );
	__m128 c = indices[2] == B2_NULL_INDEX ? _mm_setzero_ps() : _mm_loadu_ps( (const float*)( states + indices[2] ) );
	__m128 d = indices[3] == B2_NULL_INDEX ? _mm_setzero_ps() : _mm_loadu_ps( (const float*)( states + indices[3] ) );
	_MM_TRANSPOSE4_PS( a, b, c, d );
	*x = a;
	*y = b;
	*w = c;
	*flags = d;
}

static inline void b2ScatterRevoluteVelocities( b2BodyState* states, const int* indices, __m128 a, __m128 b, __m128 c, __m128 d )
{
	_MM_TRANSPOSE4_PS( a, b, c, d );
	if ( indices[0] != B2_NULL_INDEX )
	{
		_mm_storeu_ps( (float*)( states + indices[0] ), a );
	}
	if ( indices[1] != B2_NULL_INDEX )
	{
		_mm_storeu_ps( (float*)( states + indices[1] ), b );
	}
	if ( indices[2] != B2_NULL_INDEX )
	{
		_mm_storeu_ps( (float*)( states + indices[2] ), c );
	}
	if ( indices[3] != B2_NULL_INDEX )
	{
		_mm_storeu_ps( (float*)( states + indices[3] ), d );
	}
}

static __m128 b2NegRevoluteWide( __m128 a )
{
	return _mm_xor_ps( a, _mm_set1_ps( -0.0f ) );
}

static __m128 b2SelectRevoluteWide( __m128 mask, __m128 a, __m128 b )
{
	return _mm_or_ps( _mm_and_ps( mask, a ), _mm_andnot_ps( mask, b ) );
}

static inline void b2GatherRevolutePoses( const b2BodyState* states, const int* indices, __m128* x, __m128* y,
                                      __m128* c, __m128* s )
{
	__m128 identity = _mm_setr_ps( 0.0f, 0.0f, 1.0f, 0.0f );
	__m128 a = indices[0] == B2_NULL_INDEX ? identity : _mm_loadu_ps( &states[indices[0]].deltaPosition.x );
	__m128 b = indices[1] == B2_NULL_INDEX ? identity : _mm_loadu_ps( &states[indices[1]].deltaPosition.x );
	__m128 d = indices[2] == B2_NULL_INDEX ? identity : _mm_loadu_ps( &states[indices[2]].deltaPosition.x );
	__m128 e = indices[3] == B2_NULL_INDEX ? identity : _mm_loadu_ps( &states[indices[3]].deltaPosition.x );
	_MM_TRANSPOSE4_PS( a, b, d, e );
	*x = a;
	*y = b;
	*c = d;
	*s = e;
}

static void b2PrepareRevoluteGroupGeometry( b2RevoluteGroup* g, b2JointSim* joints, int count,
                                            b2StepContext* context )
{
	__m128 ax, ay, ac, as, bx, by, bc, bs;
	b2GatherRevolutePoses( context->states, g->indexA, &ax, &ay, &ac, &as );
	b2GatherRevolutePoses( context->states, g->indexB, &bx, &by, &bc, &bs );
	b2RevoluteWide cosA = { .v = ac }, sinA = { .v = as }, cosB = { .v = bc }, sinB = { .v = bs }, angles;
	// Unused lanes have identity rotations from the pose gather.
	for ( int lane = 0; lane < 4; ++lane )
	{
		b2Rot a = { cosA.lane[lane], sinA.lane[lane] };
		b2Rot b = { cosB.lane[lane], sinB.lane[lane] };
		angles.lane[lane] = b2RelativeAngle( b, a );
	}
	g->rAx.v = _mm_sub_ps( _mm_mul_ps( ac, g->anchorAx.v ), _mm_mul_ps( as, g->anchorAy.v ) );
	g->rAy.v = _mm_add_ps( _mm_mul_ps( as, g->anchorAx.v ), _mm_mul_ps( ac, g->anchorAy.v ) );
	g->rBx.v = _mm_sub_ps( _mm_mul_ps( bc, g->anchorBx.v ), _mm_mul_ps( bs, g->anchorBy.v ) );
	g->rBy.v = _mm_add_ps( _mm_mul_ps( bs, g->anchorBx.v ), _mm_mul_ps( bc, g->anchorBy.v ) );
	__m128 mass = _mm_add_ps( g->mA.v, g->mB.v );
	g->k11.v = _mm_add_ps( _mm_add_ps( mass, _mm_mul_ps( _mm_mul_ps( g->rAy.v, g->rAy.v ), g->iA.v ) ),
		_mm_mul_ps( _mm_mul_ps( g->rBy.v, g->rBy.v ), g->iB.v ) );
	g->k12.v = _mm_sub_ps( _mm_mul_ps( _mm_mul_ps( b2NegRevoluteWide( g->rAy.v ), g->rAx.v ), g->iA.v ),
		_mm_mul_ps( _mm_mul_ps( g->rBy.v, g->rBx.v ), g->iB.v ) );
	g->k22.v = _mm_add_ps( _mm_add_ps( mass, _mm_mul_ps( _mm_mul_ps( g->rAx.v, g->rAx.v ), g->iA.v ) ),
		_mm_mul_ps( _mm_mul_ps( g->rBx.v, g->rBx.v ), g->iB.v ) );
	__m128 det = _mm_sub_ps( _mm_mul_ps( g->k11.v, g->k22.v ), _mm_mul_ps( g->k12.v, g->k12.v ) );
	__m128 nonzero = _mm_cmpneq_ps( det, _mm_setzero_ps() );
	__m128 divisor = b2SelectRevoluteWide( nonzero, det, _mm_set1_ps( 1.0f ) );
	g->invDet.v = b2SelectRevoluteWide( nonzero, _mm_div_ps( _mm_set1_ps( 1.0f ), divisor ), det );
	g->biasX.v = _mm_mul_ps( g->biasRate.v,
		_mm_add_ps( _mm_add_ps( _mm_sub_ps( bx, ax ), _mm_sub_ps( g->rBx.v, g->rAx.v ) ), g->centerX.v ) );
	g->biasY.v = _mm_mul_ps( g->biasRate.v,
		_mm_add_ps( _mm_add_ps( _mm_sub_ps( by, ay ), _mm_sub_ps( g->rBy.v, g->rAy.v ) ), g->centerY.v ) );
	for ( int lane = 0; lane < count; ++lane )
	{
		b2RevoluteJoint* j = &joints[lane].revoluteJoint;
		float angle = angles.lane[lane] + j->deltaAngle;
		angle = b2UnwindAngle( angle - j->referenceAngle );
		g->lowerC.lane[lane] = angle - j->lowerAngle;
		g->upperC.lane[lane] = j->upperAngle - angle;
	}
}

static void b2PrepareRevoluteGroup( b2RevoluteGroup* g, b2JointSim* joints, int count,
                                    b2StepContext* context, bool useBias )
{
	// Geometry is initialized on the first solve and refreshed after position integration.
	// Velocity integration and warm starting preserve it for the next biased phase.
	if ( !useBias )
	{
		b2PrepareRevoluteGroupGeometry( g, joints, count, context );
	}
	// Later biased passes must retain the impulses accumulated in the packed group.
	if ( useBias && context->biasedIteration == 0 )
	{
		for ( int lane = 0; lane < count; ++lane )
		{
			b2RevoluteJoint* j = &joints[lane].revoluteJoint;
			g->motor.lane[lane] = j->motorImpulse;
			g->lower.lane[lane] = j->lowerImpulse;
			g->upper.lane[lane] = j->upperImpulse;
			g->linearX.lane[lane] = j->linearImpulse.x;
			g->linearY.lane[lane] = j->linearImpulse.y;
		}
	}
	g->lowerBias.v = g->lowerC.v;
	g->upperBias.v = g->upperC.v;
	if ( !useBias )
	{
		__m128 zero = _mm_setzero_ps();
		__m128 inv_h = _mm_set1_ps( context->inv_h );
		g->lowerBias.v = _mm_and_ps( _mm_cmpgt_ps( g->lowerBias.v, zero ), _mm_mul_ps( g->lowerBias.v, inv_h ) );
		g->upperBias.v = _mm_and_ps( _mm_cmpgt_ps( g->upperBias.v, zero ), _mm_mul_ps( g->upperBias.v, inv_h ) );
	}
	g->remaining = useBias ? 1 : context->world->relaxIterations;
}

void b2SolveRevoluteGroup( b2RevoluteGroup* g, b2JointSim* joints, int count, b2StepContext* context, bool useBias )
{
	// First biased solve follows joint preparation, which supplies the current body indices and masses.
	if ( g->eligible == -1 )
	{
		g->eligible = 1;
		for ( int lane = 0; lane < count; ++lane )
		{
			b2JointSim* base = joints + lane;
			b2RevoluteJoint* j = &base->revoluteJoint;
			if ( base->type != b2_revoluteJoint || !j->enableMotor || !j->enableLimit || j->enableSpring ||
				 base->invIA + base->invIB == 0.0f )
			{
				g->eligible = 0;
				break;
			}
			g->indexA[lane] = j->indexA;
			g->indexB[lane] = j->indexB;
			g->mA.lane[lane] = base->invMassA;
			g->mB.lane[lane] = base->invMassB;
			g->iA.lane[lane] = base->invIA;
			g->iB.lane[lane] = base->invIB;
			g->axialMass.lane[lane] = j->axialMass;
			g->maxImpulse.lane[lane] = j->maxMotorImpulse;
			g->speed.lane[lane] = j->motorSpeed;
			g->biasRate.lane[lane] = base->constraintSoftness.biasRate;
			g->pointMass.lane[lane] = -base->constraintSoftness.massScale;
			g->pointScale.lane[lane] = base->constraintSoftness.impulseScale;
			g->anchorAx.lane[lane] = j->anchorA.x;
			g->anchorAy.lane[lane] = j->anchorA.y;
			g->anchorBx.lane[lane] = j->anchorB.x;
			g->anchorBy.lane[lane] = j->anchorB.y;
			g->centerX.lane[lane] = j->deltaCenter.x;
			g->centerY.lane[lane] = j->deltaCenter.y;
		}
		if ( g->eligible )
		{
			b2PrepareRevoluteGroupGeometry( g, joints, count, context );
		}
	}
	if ( !g->eligible )
	{
		for ( int lane = 0; lane < count; ++lane )
		{
			b2SolveJoint( joints + lane, context, useBias );
		}
		g->remaining = 0;
		return;
	}
	if ( useBias || g->remaining == 0 )
	{
		b2PrepareRevoluteGroup( g, joints, count, context, useBias );
	}

	__m128 ax, ay, aw, af, bx, by, bw, bf;
	b2GatherRevoluteVelocities( context->states, g->indexA, &ax, &ay, &aw, &af );
	b2GatherRevoluteVelocities( context->states, g->indexB, &bx, &by, &bw, &bf );
	// Preserve scalar rounding and signed-zero operations; do not fuse or reassociate.
	__m128 zero = _mm_setzero_ps();
	__m128 negMass = b2NegRevoluteWide( g->axialMass.v );
	__m128 cdot = _mm_sub_ps( _mm_sub_ps( bw, aw ), g->speed.v );
	__m128 impulse = _mm_mul_ps( negMass, cdot );
	__m128 old = g->motor.v;
	g->motor.v = _mm_min_ps( g->maxImpulse.v,
		_mm_max_ps( b2NegRevoluteWide( g->maxImpulse.v ), _mm_add_ps( old, impulse ) ) );
	impulse = _mm_sub_ps( g->motor.v, old );
	aw = _mm_sub_ps( aw, _mm_mul_ps( g->iA.v, impulse ) );
	bw = _mm_add_ps( bw, _mm_mul_ps( g->iB.v, impulse ) );

	__m128 lowerMass = negMass, upperMass = negMass, lowerScale = zero, upperScale = zero;
	__m128 biasX = zero, biasY = zero, pointScale = zero;
	if ( useBias )
	{
		__m128 lowerMask = _mm_cmpgt_ps( g->lowerBias.v, zero );
		__m128 upperMask = _mm_cmpgt_ps( g->upperBias.v, zero );
		__m128 inv_h = _mm_set1_ps( context->inv_h );
		__m128 softMass = _mm_mul_ps( g->pointMass.v, g->axialMass.v );
		lowerMass = b2SelectRevoluteWide( lowerMask, negMass, softMass );
		upperMass = b2SelectRevoluteWide( upperMask, negMass, softMass );
		lowerScale = _mm_andnot_ps( lowerMask, g->pointScale.v );
		upperScale = _mm_andnot_ps( upperMask, g->pointScale.v );
		g->lowerBias.v = _mm_mul_ps( b2SelectRevoluteWide( lowerMask, inv_h, g->biasRate.v ), g->lowerBias.v );
		g->upperBias.v = _mm_mul_ps( b2SelectRevoluteWide( upperMask, inv_h, g->biasRate.v ), g->upperBias.v );
		biasX = g->biasX.v;
		biasY = g->biasY.v;
		pointScale = g->pointScale.v;
	}
	old = g->lower.v;
	cdot = _mm_sub_ps( bw, aw );
	impulse = _mm_sub_ps( _mm_mul_ps( lowerMass, _mm_add_ps( cdot, g->lowerBias.v ) ), _mm_mul_ps( lowerScale, old ) );
	g->lower.v = _mm_max_ps( _mm_add_ps( old, impulse ), zero );
	impulse = _mm_sub_ps( g->lower.v, old );
	aw = _mm_sub_ps( aw, _mm_mul_ps( g->iA.v, impulse ) );
	bw = _mm_add_ps( bw, _mm_mul_ps( g->iB.v, impulse ) );

	old = g->upper.v;
	cdot = _mm_sub_ps( aw, bw );
	impulse = _mm_sub_ps( _mm_mul_ps( upperMass, _mm_add_ps( cdot, g->upperBias.v ) ), _mm_mul_ps( upperScale, old ) );
	g->upper.v = _mm_max_ps( _mm_add_ps( old, impulse ), zero );
	impulse = _mm_sub_ps( g->upper.v, old );
	aw = _mm_add_ps( aw, _mm_mul_ps( g->iA.v, impulse ) );
	bw = _mm_sub_ps( bw, _mm_mul_ps( g->iB.v, impulse ) );

	__m128 rx = _mm_add_ps( _mm_sub_ps( _mm_add_ps( bx, _mm_mul_ps( b2NegRevoluteWide( bw ), g->rBy.v ) ),
		_mm_add_ps( ax, _mm_mul_ps( b2NegRevoluteWide( aw ), g->rAy.v ) ) ), biasX );
	__m128 ry = _mm_add_ps( _mm_sub_ps( _mm_add_ps( by, _mm_mul_ps( bw, g->rBx.v ) ),
		_mm_add_ps( ay, _mm_mul_ps( aw, g->rAx.v ) ) ), biasY );
	__m128 x = _mm_mul_ps( g->invDet.v, _mm_sub_ps( _mm_mul_ps( g->k22.v, rx ), _mm_mul_ps( g->k12.v, ry ) ) );
	__m128 y = _mm_mul_ps( g->invDet.v, _mm_sub_ps( _mm_mul_ps( g->k11.v, ry ), _mm_mul_ps( g->k12.v, rx ) ) );
	if ( useBias )
	{
		x = _mm_mul_ps( g->pointMass.v, x );
		y = _mm_mul_ps( g->pointMass.v, y );
	}
	else
	{
		x = b2NegRevoluteWide( x );
		y = b2NegRevoluteWide( y );
	}
	x = _mm_sub_ps( x, _mm_mul_ps( pointScale, g->linearX.v ) );
	y = _mm_sub_ps( y, _mm_mul_ps( pointScale, g->linearY.v ) );
	g->linearX.v = _mm_add_ps( g->linearX.v, x );
	g->linearY.v = _mm_add_ps( g->linearY.v, y );
	ax = _mm_sub_ps( ax, _mm_mul_ps( g->mA.v, x ) );
	ay = _mm_sub_ps( ay, _mm_mul_ps( g->mA.v, y ) );
	aw = _mm_sub_ps( aw, _mm_mul_ps( g->iA.v, _mm_sub_ps( _mm_mul_ps( g->rAx.v, y ), _mm_mul_ps( g->rAy.v, x ) ) ) );
	bx = _mm_add_ps( bx, _mm_mul_ps( g->mB.v, x ) );
	by = _mm_add_ps( by, _mm_mul_ps( g->mB.v, y ) );
	bw = _mm_add_ps( bw, _mm_mul_ps( g->iB.v, _mm_sub_ps( _mm_mul_ps( g->rBx.v, y ), _mm_mul_ps( g->rBy.v, x ) ) ) );
	b2ScatterRevoluteVelocities( context->states, g->indexA, ax, ay, aw, af );
	b2ScatterRevoluteVelocities( context->states, g->indexB, bx, by, bw, bf );

	if ( --g->remaining == 0 && !useBias )
	{
		for ( int lane = 0; lane < count; ++lane )
		{
			b2RevoluteJoint* j = &joints[lane].revoluteJoint;
			j->motorImpulse = g->motor.lane[lane];
			j->lowerImpulse = g->lower.lane[lane];
			j->upperImpulse = g->upper.lane[lane];
			j->linearImpulse = (b2Vec2){ g->linearX.lane[lane], g->linearY.lane[lane] };
		}
	}
}
#endif
