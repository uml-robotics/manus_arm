/*
 * ManusArm.hpp
 * Copyright 2013 University of Massachusetts Lowell
 * Authors: Abraham Shultz, Jonathan Hasenzahl
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

// Constant speed limits for each axis
const int SPEED_LIMITS[][5] = { { 3, 10, 17, 23, 30 },  // X
							    { 10, 30, 50, 70, 90 }, // Y
							    { 10, 30, 50, 70, 90 }, // Z
							    { 1, 3, 5, 7, 9 },      // Yaw
							    { 1, 3, 5, 7, 9 },      // Pitch
							    { 1, 3, 5, 7, 9 },      // Roll
							    { 1, 4, 7, 10, 14 } };  // Grip

int sign(int n)
{
    return ((n >= 0) ? 1 : -1);
}

float sign(float n)
{
    return ((n >= 0) ? 1.0f : -1.0f);
}

/*!
 * \brief Stores ARM state.
 */
struct armState
{
	bool isValid;
	int jointPositions[7];
	int cbox;
	std::string message;
};

/*!
 * \brief Singleton class representing the ARM.
 * \copyright Copyright 2013 University of Massachusetts Lowell
 * \author Abraham Shultz
 * \author Jonathan Hasenzahl
 */
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
	bool moveComplete;
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
	void doCartesianMove(const CartesianMove& cmd);
	void doConstantMove(const ConstantMove& cmd);

public:
	static ManusArm* instance();
	int init(std::string interface);
	std::string getPrintState();
	std::string getCsvState();
	void getPosition(float position[]);
	bool isMoveComplete() {	return moveComplete; }
	void setMoveComplete(bool move_complete)
	{
		boost::mutex::scoped_lock lock(stateMutex);
		moveComplete = move_complete;
	}

	//Motion commands
	void fold();
	void unfold();
	void moveCartesian(const CartesianMove& cmd);
	void moveConstant(const ConstantMove& cmd);
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
