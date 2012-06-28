/* Singleton class representing the arm.
 *
 */

#ifndef MANUS_ARM_HPP_
#define MANUS_ARM_HPP_

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <string>
#include <sstream>
#include <string.h> //C strings, for strcpy
#include <boost/thread.hpp>
#include <exception>
#include <vector>
#include <time.h>
#include "movement_definitions.h"

// from can_comm.cpp by ktsui
// MANUS STATUS; from Exact Dynamics
// TRANSPARENT MODE MANUAL; SECTION 6.9
#define STAT_NONE                                    0

#define STAT_WARNING                              1
#define MSG_GRIPPER_STUCK                    0
#define MSG_WRONG_AREA                       1
#define MSG_ARM_FOLDED_STRETCHED   2
#define MSG_BLOCKED_MOTOR                  3
#define MSG_MAX_ROT                               4

#define STAT_GENERAL                               2
#define MSG_FOLDED                                  0
#define MSG_UNFOLDED                             1
#define MSG_GRIPPER_INIT                         2
#define MSG_ABS_MEASURE                       3

#define STAT_ERROR                                   3
#define MSG_IO_80C552			                  1
#define MSG_ABS_ENCODER			              4
#define MSG_MOVE_WITHOUT_INPUT         15

#define CBOX_0_INIT                                   0x370
#define CBOX_1_CARTESIAN                       0x371
#define CBOX_4_JOINT                                 0x374
#define CBOX_5_UNFOLD                            0x375
#define CBOX_6_FOLD                                 0x376

/* Amount of error allowed to consider a movement "over"
 * In cartesian motion units (0.022 mm)
 */
#define CARTESIAN_SLOP 300

/* About a foot, in 0.022mm units
 */
#define CARTESIAN_FOOT 13854.54

// Indexes for can_frame.data array only
// Use the indexes defined in movement_states.h for movement state arrays and
// position state arrays
#define LIFT 0
#define Z 1
#define X 2
#define Y 3
#define YAW 4
#define PITCH 5
#define ROLL 6
#define GRIP 7

int sign(int n)
{
    return ((n >= 0) ? 1 : -1);
}

float sign(float n)
{
    return ((n >= 0) ? 1.0f : -1.0f);
}

/* for ARM state */
struct armState
{
	bool isValid;
	int jointPositions[7];
	int cbox;
	std::string message;
};

class ManusArm
{
private:
	/* Constructor, copy constructor, assignment operator are all private to make singleton */
	ManusArm();
	ManusArm(ManusArm const&){};
	ManusArm& operator=(ManusArm const&);
	/* Pointer to the existing instance */
	static ManusArm* armInstance;

	/* for SocketCan communication */
	int canSock;
	bool running;
	void pollCanSocket();

	/* State maintenance and thread safety */
	armState currState;
	boost::mutex stateMutex;
	void setCbox(int cbox, can_frame* frm);

	/* Sending messages to the arm */
	void sendFrames(can_frame frame);
	std::vector<can_frame> sendQueue;
	void enqueueFrame(can_frame toSend);

	//Mode setting commands
	void setCartesian();

	//Motion
	boost::thread motionThread;
	//void doMove(float target_position[], int speed_mode, void (*callback)()); // Original
	void doMove(float speeds[], void (*callback)()); // New version - for interruptible movement
	void doConstantMove(int movement_states[], void (*callback)()); // New version - for keyboard movement

public:
	static ManusArm* instance();
	int init(std::string interface);
	std::string getPrintState();
	std::string getCsvState();
	void getPosition(float position[]);

	//Motion commands
	void fold();
	void unfold();
	void moveConstant(int movement_states[], void (*callback)()); // New version - for keyboard movement
	void moveCartesian(float speeds[], void (*callback)()); // New version - for interruptible movement
	//void moveCartesian(float position[], int speed_mode, void (*callback)()); // Original
};

class ArmException: public std::exception
{
private:
	char* msg;
public:
	ArmException(char* message) throw();
	virtual const char* what() const throw();
};

#endif
