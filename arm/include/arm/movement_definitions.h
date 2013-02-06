/*
 * movement_definitions.h
 * Copyright 2013 University of Massachusetts Lowell
 * Author: Jonathan Hasenzahl
 */

#ifndef MOVEMENT_DEFINITIONS_H_
#define MOVEMENT_DEFINITIONS_H_

// Movement states
#define STOP 0
#define ARM_FORWARD 1
#define ARM_BACKWARD -1
#define ARM_LEFT 1
#define ARM_RIGHT -1
#define ARM_UP 1
#define ARM_DOWN -1
#define CLAW_YAW_LEFT 1
#define CLAW_YAW_RIGHT -1
#define CLAW_PITCH_UP 1
#define CLAW_PITCH_DOWN -1
#define CLAW_ROLL_LEFT -1
#define CLAW_ROLL_RIGHT 1
#define CLAW_GRIP_OPEN 1
#define CLAW_GRIP_CLOSE -1
#define LIFT_UP -1
#define LIFT_DOWN 1

// Movement axis indexes
#define X 0 // Forward/backward
#define Y 1 // Left/right
#define Z 2 // Up/down
#define YAW 3
#define PITCH 4
#define ROLL 5
#define GRIP 6
#define LIFT 7 // Not used with cartesian moves. Does not have a speed.

// Size of movement arrays
#define CART_MV_ARR_SZ 7	// Cartesian movement array
#define CONST_MV_ARR_SZ 8	// Constant movement array
#define SPD_ARR_SZ 7		// Speed array
#define FRM_ARR_SZ 8		// Can frame array

// Arm origin position
const float ORIGIN_POSITION[] = {  15000.0f,
		                           -4500.0f,
		                           19000.0f,
		                               0.0f,
		                               0.0f,
		                               0.0f,
		                          -10000.0f };
// Arm final position
const float FINAL_POSITION[][CART_MV_ARR_SZ] = { { 15000.0f,
								                   -4500.0f,
								                   19000.0f,
								                       0.0f,
								                       0.0f,
								                       0.0f,
								                  -10000.0f },
								                 {  8000.0f,
								                   -4500.0f,
								                    6000.0f,
								                       0.0f,
								                       0.0f,
								                       0.0f,
								                  -10000.0f } };

/*!
 * \brief Struct for holding Cartesian movement data
 */
struct CartesianMove
{
	float positions[CART_MV_ARR_SZ];
	int speeds[SPD_ARR_SZ];
};

/*!
 * \brief Struct for holding constant movement data
 */
struct ConstantMove
{
	int states[CONST_MV_ARR_SZ];
	int speeds[SPD_ARR_SZ]; // No speed for lift unit
};

#endif
