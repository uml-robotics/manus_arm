#ifndef KEY_CODES_H_
#define KEY_CODES_H_

// Arm control
#define KEYCODE_W         0x77 // ARM_FORWARD
#define KEYCODE_A         0x61 // ARM_LEFT
#define KEYCODE_S         0x73 // ARM_BACKWARD
#define KEYCODE_D         0x64 // ARM_RIGHT
#define KEYCODE_Z         0x7A // ARM_DOWN
#define KEYCODE_X         0x78 // ARM_UP
#define KEYCODE_F         0x66 // FOLD
#define KEYCODE_U         0x75 // UNFOLD

// Claw control
#define KEYCODE_NUM_4     0x34 // CLAW_YAW_LEFT
#define KEYCODE_NUM_6     0x36 // CLAW_YAW_RIGHT
#define KEYCODE_NUM_8     0x38 // CLAW_PITCH_UP
#define KEYCODE_NUM_2     0x32 // CLAW_PITCH_DOWN
#define KEYCODE_NUM_7     0x37 // CLAW_ROLL_LEFT
#define KEYCODE_NUM_9     0x39 // CLAW_ROLL_RIGHT
#define KEYCODE_NUM_MINUS 0x2D // CLAW_GRIP_CLOSE
#define KEYCODE_NUM_PLUS  0x2B // CLAW_GRIP_OPEN

// Lift control
#define KEYCODE_UP        0x41 // LIFT_UP
#define KEYCODE_DOWN      0x42 // LIFT_DOWN

// Other
#define KEYCODE_COMMA     0x2C // SPEED_DOWN
#define KEYCODE_PERIOD    0x2E // SPEED_UP
#define KEYCODE_BACKSPACE 0x7F // ALL_STOP
#define KEYCODE_TAB       0x09 // QUERY
#define KEYCODE_Q         0x71 // QUIT

#endif
