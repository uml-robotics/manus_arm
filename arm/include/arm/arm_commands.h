#ifndef ARM_COMMANDS_H_
#define ARM_COMMANDS_H_

// All possible commands to be sent to the ARM

enum commands { NONE,
       ARM_LEFT,
       ARM_RIGHT,
       ARM_UP,
       ARM_DOWN,
       ARM_FORWARD,
       ARM_BACKWARD,
       CLAW_YAW_LEFT,
       CLAW_YAW_RIGHT,
       CLAW_PITCH_UP,
       CLAW_PITCH_DOWN,
       CLAW_ROLL_LEFT,
       CLAW_ROLL_RIGHT,
       CLAW_GRIP_CLOSE,
       CLAW_GRIP_OPEN,
       LIFT_UP,
       LIFT_DOWN,
       FOLD,
       UNFOLD,
       SPEED_DOWN,
       SPEED_UP,
       ALL_STOP,
       QUERY,
       QUIT };

#endif
