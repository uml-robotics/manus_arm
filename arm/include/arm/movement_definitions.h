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

// Movement state array indexes (0-8)
// Position array indexes (0-6 only)
#define ARM_Z 0
#define ARM_X 1
#define ARM_Y 2
#define CLAW_YAW 3
#define CLAW_PITCH 4
#define CLAW_ROLL 5
#define CLAW_GRIP 6
#define LIFT_UNIT 7
#define SPEED 8

#define STATE_ARR_SZ 9
#define POS_ARR_SZ 7

namespace manus_arm
{
// Arrays of speeds
float stop[] = { 0, 0, 0, 0, 0, 0, 0 };

// Arrays of positions
float origin_position[] = { 14000, 0, 0, 0, 0, 0, -10000 };
float final_position[] = { 3400, -7300, 6200, 0, 0, 0, -10000 };
//float final_position[] = { 10000, 0, 0, 0, 0, 0, -10000 };

// Standard speed
int speed = 2;
}

#endif
