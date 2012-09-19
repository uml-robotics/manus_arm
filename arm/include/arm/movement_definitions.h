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
#define LIFT 0 // Not used with cartesian moves. Does not have a speed.
#define X 1 // Forward/backward
#define Y 2 // Left/right
#define Z 3 // Up/down
#define YAW 4
#define PITCH 5
#define ROLL 6
#define GRIP 7

// Size of a movement array
#define MOVE_ARR_SZ 8

// Arm origin position
const float origin_position[] = {     0.0f,
								   15000.0f,
		                           -4500.0f,
		                           19000.0f,
		                               0.0f,
		                               0.0f,
		                               0.0f,
		                          -10000.0f };
// Arm final position
const float final_position[][] = { {      0.0f,
								      15000.0f,
								      -4500.0f,
								      19000.0f,
								          0.0f,
								          0.0f,
								          0.0f,
								     -10000.0f },
								   {      0.0f,
								       8000.0f,
								      -4500.0f,
								       6000.0f,
								          0.0f,
								          0.0f,
								           0.0f,
								      -10000.0f } };

// Struct for holding Cartesian movement data
struct CartesianMove
{
	float positions[MOVE_ARR_SZ];
	int speeds[MOVE_ARR_SZ];
};

// Struct for holding constant movement data
struct ConstantMove
{
	int states[MOVE_ARR_SZ];
	int speeds[MOVE_ARR_SZ]; // No speed for lift unit
};

#endif
