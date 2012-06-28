#include "arm/ManusArm.hpp"
#include "arm/MoUT.hpp"

using namespace std;

/* Pointer to the existing instance */
ManusArm* ManusArm::armInstance = NULL;

int sign(int n)
{
	return ((n >= 0) ? 1 : -1);
}

float sign(float n)
{
	return ((n >= 0) ? 1.0f : -1.0f);
}

void ManusArm::getPosition(float fill[])
{
	//get current position
	{
		boost::mutex::scoped_lock lock(stateMutex);
		for (int i = 0; i < 7; i++)
		{
			fill[i] = currState.jointPositions[i];
			//cout << "currState.jointPositions[" << i << "] = " << currState.jointPositions[i] << endl;
		}
	}
}

string ManusArm::getCsvState()
{
	//Probably shouldn't lock here, as it only reads, and is used for debugging
	//boost::mutex::scoped_lock lock(stateMutex);
	stringstream sb;
	if (currState.isValid)
	{
		if (currState.cbox == CBOX_4_JOINT)
		{
			//Units are tenths of a degree
			sb << currState.jointPositions[0] * 0.1 << ",";
			sb << currState.jointPositions[1] * 0.1 << ",";
			sb << currState.jointPositions[2] * 0.1 << ",";
			sb << currState.jointPositions[3] * 0.1 << ",";
			sb << currState.jointPositions[4] * 0.1 << ",";
			sb << currState.jointPositions[5] * 0.1 << ",";
		}
		else
		{
			//Units are 0.022mm
			sb << currState.jointPositions[0] * 0.022 << ",";
			sb << currState.jointPositions[1] * 0.022 << ",";
			sb << currState.jointPositions[2] * 0.022 << ",";
			//Units are tenths of a degree
			sb << currState.jointPositions[3] * 0.1 << ",";
			sb << currState.jointPositions[4] * 0.1 << ",";
			sb << currState.jointPositions[5] * 0.1 << ",";
		}
		//Units are tenths of a millimeter
		sb << currState.jointPositions[6] * 0.1;
	}
	else
	{
		sb << "State not valid yet\n";
	}
	return sb.str();
}

string ManusArm::getPrintState()
{
	//Should not lock here, only reads, useful for debugging as arm moves
	//boost::mutex::scoped_lock lock(stateMutex);
	stringstream sb;
	sb << currState.message + "\n";
	if (currState.isValid)
	{
		if (currState.cbox == CBOX_4_JOINT)
		{
			//Units are tenths of a degree
			sb << "Axis 1: " << currState.jointPositions[0] * 0.1 << " degrees \n";
			sb << "Axis 2: " << currState.jointPositions[1] * 0.1 << " degrees \n";
			sb << "Axis 3: " << currState.jointPositions[2] * 0.1 << " degrees \n";
			sb << "Axis 4: " << currState.jointPositions[3] * 0.1 << " degrees \n";
			sb << "Axis 5: " << currState.jointPositions[4] * 0.1 << " degrees \n";
			sb << "Axis 6: " << currState.jointPositions[5] * 0.1 << " degrees \n";
		}
		else
		{
			//Units are 0.022mm
			sb << "X: " << currState.jointPositions[0] * 0.022 << " mm \n";
			sb << "Y: " << currState.jointPositions[1] * 0.022 << " mm \n";
			sb << "Z: " << currState.jointPositions[2] * 0.022 << " mm \n";
			//Units are tenths of a degree
			sb << "Yaw:   " << currState.jointPositions[3] * 0.1 << " degrees\n";
			sb << "Pitch: " << currState.jointPositions[4] * 0.1 << " degrees\n";
			sb << "Roll:   " << currState.jointPositions[5] * 0.1 << " degrees\n";
		}
		//Units are tenths of a millimeter
		sb << "Gripper: " << currState.jointPositions[6] * 0.1 << " mm \n";
	}
	else
	{
		sb << "State not valid yet\n";
	}

	return sb.str();
}

void ManusArm::setCbox(int cbox, can_frame* frm)
{
	boost::mutex::scoped_lock lock(stateMutex);
	currState.cbox = cbox;
	frm->can_id = currState.cbox;
}

/* Units are as follows
 * X, Y, Z are 0.022 mm every 20 ms, (1.1mm/sec) with a possible delay of 60ms on the first motion
 * So to move at 1 cm / second, you need to request a speed of 9 (9.9mm/sec) or 10 (11mm/sec),
 * depending on whether you want to be too fast or too slow. The valid range is 0-127.
 *
 * Roll, pitch, and yaw all refer to the gripper, and are in 0.1 degree per increments.
 * The valid range is 0-10 (0-1 degree/20msec)
 *
 * Gripper is opening and closing rates in units of 0.1mm/20msec.
 * The valid range is 0 to 15
 *
 *  We don't have a lift unit. The valid values are -1 (up), 0 (off), and 1 (down)
 */

// ORIGINAL PUBLIC MOVE METHOD
// The original implementation of the method is below.
/*
void ManusArm::moveCartesian(float target_position[], int speed_mode, void (*callback)())
{
    //boost::thread doMove(boost::bind(&ManusArm::doMove, this, target_position, callback));
    motionThread = boost::thread(boost::bind(&ManusArm::doMove, this, target_position, speed_mode, callback));
}
*/

void ManusArm::moveCartesian(float speeds[], void (*callback)())
{
	motionThread = boost::thread(boost::bind(&ManusArm::doMove, this, speeds, callback));
}

void ManusArm::doMove(float speeds[], void (*callback)())
{
    boost::this_thread::at_thread_exit(callback);

    struct can_frame move;
    setCbox(CBOX_1_CARTESIAN, &move);
    move.can_dlc = 8;
    move.data[LIFT] = 0;
    move.data[Z] = speeds[ARM_Z];
    move.data[X] = speeds[ARM_X];
    move.data[Y] = speeds[ARM_Y];
    move.data[YAW] = speeds[CLAW_YAW];
    move.data[PITCH] = speeds[CLAW_PITCH];
    move.data[ROLL] = speeds[CLAW_ROLL];
    move.data[GRIP] = speeds[CLAW_GRIP];
    enqueueFrame(move);

    // Wait 60 msec
    boost::this_thread::sleep(boost::posix_time::milliseconds(60));
}

// Added by Jon
void ManusArm::moveConstant(int movement_state[], void (*callback)())
{
    motionThread = boost::thread(boost::bind(&ManusArm::doConstantMove, this, movement_state, callback));
}

// Added by Jon
void ManusArm::doConstantMove(int movement_state[], void (*callback)())
{
    boost::this_thread::at_thread_exit(callback);
    
    // Speed limits
	const int linear_speed_limit[5] = { 10, 30, 50, 70, 90 };
	const int angular_speed_limit[5] = { 1, 3, 5, 7, 9 };
	const int grip_speed_limit[5] = { 1, 4, 7, 10, 14 };
	int linear_speed = linear_speed_limit[movement_state[SPEED]];
    int angular_speed = angular_speed_limit[movement_state[SPEED]];
    int grip_speed = grip_speed_limit[movement_state[SPEED]];
    
    struct can_frame move;
	setCbox(CBOX_1_CARTESIAN, &move);
	move.can_dlc = 8;
	move.data[LIFT] = movement_state[LIFT_UNIT];
	move.data[Z] = linear_speed * movement_state[ARM_Z];
	move.data[X] = linear_speed * movement_state[ARM_X];
	move.data[Y] = linear_speed * movement_state[ARM_Y];
	move.data[YAW] = angular_speed * movement_state[CLAW_YAW];
	move.data[PITCH] = angular_speed * movement_state[CLAW_PITCH];
	move.data[ROLL] = angular_speed * movement_state[CLAW_ROLL];
	move.data[GRIP] = grip_speed * movement_state[CLAW_GRIP];
	enqueueFrame(move);

	// Wait 60 msec
	boost::this_thread::sleep(boost::posix_time::milliseconds(60));
}

// ORIGINAL PRIVATE MOVE METHOD
// The original implementation of the method is below.
/*
void ManusArm::doMove(float target_position[], int speed_mode,
                      void (*callback)())
{
	boost::this_thread::at_thread_exit(callback);



	// a lot of the logic here is from can_comm.cpp's function pd_control()
	// Generally speaking, it works by:
	// 1. Calculating the difference between the current position and the desired position (the error)
	// 2. Multiplying the error by a speed constant
	// 3. Checking that the speeds are within limits
	// 4. Sending the speeds to the arm
	// Steps 1 and 2 ensure that the arm slows as it approaches the desired position

	//Speed constants for arm
	float Kp[6] =
	{ 5, 5, 5, 0.8, 0.7, 0.6 };

	//Constant speed limits
	int linear_speed_limit[5] =
	{ 10, 30, 50, 70, 90 };
	int angular_speed_limit[5] =
	{ 1, 3, 5, 7, 9 };
	// Which speed limits to use. 4 is full, kate's work used 2 when approaching user


	//error in position
	float pos_err[6];
	//previous position
	float prev_pos[6];
	//new speeds
	float newSpeeds[6];

	bool moveComplete = false;
	while (!moveComplete)
	{
		//get current position
		{
			boost::mutex::scoped_lock lock(stateMutex);
			for (int i = 0; i < 6; i++)
				prev_pos[i] = currState.jointPositions[i];
		}

		//Calculate the error and speeds
		register float tmp1, tmp2;
		for (int i = 0; i < 3; i++) // Currently only calculates for X Y Z
		{
			if (i == 5) // roll -180 ~ 180 : linear scaling
			{
				{
					tmp1 = (180.0f - target_position[i]) - (180.0f - prev_pos[i]);
				}

				if (tmp1 >= 0.0f)
					if (tmp1 <= 180.0f)
						tmp2 = tmp1;
					else
						tmp2 = tmp1 - 360.0f;
				else if (tmp1 > -180.0f)
					tmp2 = tmp1;
				else
					tmp2 = 360.0f + tmp1;

				pos_err[i] = tmp2;
			}
			else
				pos_err[i] = target_position[i] - (prev_pos[i]);

			float control_input = Kp[i] * pos_err[i];

			if (i < 3)
				newSpeeds[i] = (fabs(control_input) > linear_speed_limit[speed_mode]) ? sign(control_input) * linear_speed_limit[speed_mode] : control_input;
			else
				newSpeeds[i] = (fabs(control_input) > angular_speed_limit[speed_mode]) ? sign(control_input) * angular_speed_limit[speed_mode] : control_input;
		}

#ifdef DEBUG
		cout << "Target Position: ";
		for (int ii = 0; ii < 6; ii++)
		{
			cout << target_position[ii] << ", ";
		}
		cout << endl;

		cout << "Position: ";
		for (int ii = 0; ii < 6; ii++)
		{
			cout << prev_pos[ii] << ", ";
		}
		cout << endl;
		cout << "Error: ";
		for (int ii = 0; ii < 6; ii++)
		{
			cout << pos_err[ii] << ", ";
		}
		cout << endl;
		cout << "New speeds: ";
		for (int ii = 0; ii < 6; ii++)
		{
			cout << newSpeeds[ii] << ", ";
		}
		cout << endl << endl;
#endif

		struct can_frame move;
		setCbox(CBOX_1_CARTESIAN, &move);
		move.can_dlc = 8;
		move.data[LIFT] = 0; //Lift unit
		move.data[Z] = newSpeeds[ARM_Z];
		move.data[X] = newSpeeds[ARM_X];
		move.data[Y] = newSpeeds[ARM_Y];
		move.data[YAW] = 0; //newSpeeds[CLAW_YAW]; <-- only moves X Y Z
		move.data[PITCH] = 0; //newSpeeds[CLAW_PITCH]; <-- only moves X Y Z
		move.data[ROLL] = 0; //newSpeeds[CLAW_ROLL]; <-- only moves X Y Z
		move.data[GRIP] = 0; //Gripper open/close

		enqueueFrame(move);

		//wait 60 msec
		boost::this_thread::sleep(boost::posix_time::milliseconds(60));

		//Assume we are done
		moveComplete = true;
		for (int ii = 0; ii < 3; ii++) // Currently only calculates for X Y Z
		{
			if (fabs(pos_err[ii]) > CARTESIAN_SLOP)
			{
				//still have moving to do
				moveComplete = false;
				break;
			}
		}
	}

	//Stop the arm, otherwise it continues with whatever speeds it had when the move was done
	struct can_frame move;
	setCbox(CBOX_1_CARTESIAN, &move);
	move.can_dlc = 8;
	move.data[LIFT] = 0;
	move.data[X] = 0;
	move.data[Y] = 0;
	move.data[Z] = 0;
	move.data[YAW] = 0;
	move.data[PITCH] = 0;
	move.data[ROLL] = 0;
	move.data[GRIP] = 0; //Gripper open/close

	enqueueFrame(move);

	//wait 60 msec
	boost::this_thread::sleep(boost::posix_time::milliseconds(60));
}
*/


void ManusArm::setCartesian()
{
	struct can_frame frm;
	//Transition to Cartesian mode
	setCbox(CBOX_1_CARTESIAN, &frm);
	frm.can_dlc = 8;
	//Zero all motion rates
	for (int ii = 0; ii < frm.can_dlc; ii++)
	{
		frm.data[ii] = 0;
	}
	enqueueFrame(frm);
}

void ManusArm::fold()
{
	struct can_frame frm;
	setCbox(CBOX_6_FOLD, &frm);
	frm.can_dlc = 0;
	//Send twice to trigger folding
	enqueueFrame(frm);
	enqueueFrame(frm);
}

void ManusArm::unfold()
{
	struct can_frame frm;
	setCbox(CBOX_5_UNFOLD, &frm);
	frm.can_dlc = 0;
	//Should I zero the data here?
	//send twice to trigger unfolding
	enqueueFrame(frm);
	enqueueFrame(frm);
}

void ManusArm::enqueueFrame(can_frame toSend)
{
	if (!sendQueue.empty())
	{
		can_frame prevFrame = *(sendQueue.end() - 1);
		//Check if this message and the last message queued are the same type
		if (toSend.can_id == prevFrame.can_id)
		{
			/* If it is a motion packet, check if the data is the same. Otherwise,
			 * it is a fold out/fold in packet, and so can be repeated
			 */
			if (toSend.can_id == CBOX_1_CARTESIAN || toSend.can_id == CBOX_4_JOINT)
			{
				bool dataMatch = true;
				for (int ii = 0; ii < 6; ii++)
				{
					if (toSend.data[ii] != prevFrame.data[ii])
					{
						dataMatch = false;
						break;
					}
				}
				//All the data were the same, don't resend the packet
				if (dataMatch)
				{
					return;
				}
			}
		}
	}
	//None of the previous checks blocked the packet, add it to the queue
	sendQueue.insert(sendQueue.begin(), toSend);
}

void ManusArm::pollCanSocket()
{
	struct can_frame frame;
	while (running)
	{
		/* Read a message back from the CAN bus */
		uint bytes_read = recv(canSock, &frame, sizeof(struct can_frame), 0);

		if (bytes_read < sizeof(struct can_frame))
		{
			cerr << "Read: incomplete CAN frame" << endl;
		}

		//		printf("ID: %x \n", frame.can_id);
		//		printf("Length: %i \n", frame.can_dlc);
		//		for (int ii = 0; ii < frame.can_dlc; ii++)
		//		{
		//			printf("Data %x\n", frame.data[ii]);
		//		}

		//TODO copy the data out into a data structure in the ARM class
		if (frame.can_id == 0x350)
		{ //Status bytes (2 bytes) and axis 1-3 (6 bytes)
			//Lock to modify state
			boost::mutex::scoped_lock lock(stateMutex);

			//Update joint positions
			currState.jointPositions[0] = ((int8_t) frame.data[2] << 8) + (uint8_t) frame.data[3]; //X
			currState.jointPositions[1] = ((int8_t) frame.data[4] << 8) + (uint8_t) frame.data[5]; //Y
			currState.jointPositions[2] = ((int8_t) frame.data[6] << 8) + (uint8_t) frame.data[7]; //Z

			currState.message = "STATUS: Updated position(1): " + toString<canid_t> (frame.can_id, hex);

			//Deal with status bytes
			uint8_t status = frame.data[0];
			uint8_t mesg = frame.data[1];
			switch (status)
			{
			case STAT_NONE:
				//Do Nothing
				break;
			case STAT_WARNING:
				switch (mesg)
				{
				case MSG_GRIPPER_STUCK:
					currState.message = "WARNING: Gripper is stuck";
					break;
				case MSG_WRONG_AREA:
					currState.message = "WARNING: Wrong area";
					break;
				case MSG_ARM_FOLDED_STRETCHED:
					currState.message = "WARNING: ARM maximum folding or extension reached";
					break;
				case MSG_BLOCKED_MOTOR:
					currState.message = "WARNING: Motor blocked or weight limit exceeded";
					break;
				case MSG_MAX_ROT:
					currState.message = "WARNING: Maximum rotation of base motor";
					break;
				default:
					currState.message = "WARNING: Nonsense message byte: %x\n" + toString<uint8_t> (status, hex);
					break;
				}
				break;
			case STAT_GENERAL:
				switch (mesg)
				{
				case MSG_FOLDED:
					currState.message = "GENERAL_MESSAGE: ARM folding complete";
					break;
				case MSG_UNFOLDED:
					currState.message = "GENERAL_MESSAGE: ARM unfolding complete";
					break;
				case MSG_GRIPPER_INIT:
					currState.message = "GENERAL_MESSAGE: Gripper ready";
					break;
				case MSG_ABS_MEASURE:
					currState.message = "GENERAL_MESSAGE: Absolute measuring ready";
					break;
				default:
					currState.message = "GENERAL_MESSAGE: Nonsense message byte: %x\n" + toString<uint8_t> (status, hex);
					break;
				}
				//cout << currState.message << endl;
				break;
			case STAT_ERROR:
				switch (mesg)
				{
				case MSG_IO_80C552:
					currState.message = "ERROR: CAN I/O processor";
					break;
				case MSG_ABS_ENCODER:
					currState.message = "ERROR: Absolute encoder";
					break;
				case MSG_MOVE_WITHOUT_INPUT:
					currState.message = "ERROR: Move without input";
					break;
				default:
					currState.message = "ERROR: Nonsense message byte: " + toString<uint8_t> (status, hex);
					break;
				}
				break;
			default:
				currState.message = "STATUS: Nonsense status byte: %x\n" + toString<uint8_t> (status, hex);
				break;
			}
		}
		else if (frame.can_id == 0x360)
		{ //Axis 4-7 (8 bytes)
			//Lock to update state
			boost::mutex::scoped_lock lock(stateMutex);
			//Update joint positions
			currState.jointPositions[3] = ((int8_t) frame.data[0] << 8) + (uint8_t) frame.data[1]; //Yaw
			currState.jointPositions[4] = ((int8_t) frame.data[2] << 8) + (uint8_t) frame.data[3]; //Pitch
			currState.jointPositions[5] = ((int8_t) frame.data[4] << 8) + (uint8_t) frame.data[5]; //Roll
			currState.jointPositions[6] = ((int8_t) frame.data[6] << 8) + (uint8_t) frame.data[7]; //Gripper
			currState.message = "STATUS: Updated position(2): " + toString<canid_t> (frame.can_id, hex);
			//The state has been updated and is now valid
			if (currState.cbox == CBOX_1_CARTESIAN || currState.cbox == CBOX_4_JOINT)
			{
				currState.isValid = true;
			}
		}
		else if (frame.can_id == 0x37F)
		{ //Go to send reply
			boost::mutex::scoped_lock lock(stateMutex);
			currState.message = "STATUS: Ready for reply: " + toString<canid_t> (frame.can_id, hex);
			//if there is a reply ready, send it
			if (!sendQueue.empty())
			{
				//filled from front, sent from back
				can_frame frame = sendQueue.back();
				uint bytes_sent = write(canSock, &frame, sizeof(frame));
				if (bytes_sent < sizeof(struct can_frame))
				{
					throw ArmException((char*) "Failed to write CAN frame to arm:" + errno);
				}
				sendQueue.pop_back();
			};
		}
		else
		{
			boost::mutex::scoped_lock lock(stateMutex);
			currState.message = "WARNING: Nonsense ID value: " + toString<canid_t> (frame.can_id, hex);
		}
	}
}

ManusArm* ManusArm::instance()
{
	if (!armInstance)
	{
		armInstance = new ManusArm;
	}
	return armInstance;
}

int ManusArm::init(string interface)
{
	/* Create the socket */
	canSock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	if (canSock < 0)
	{
		throw ArmException((char*) "Failed to open socket:" + errno);
	}

	/* Locate the interface you wish to use */
	struct ifreq ifr;
	strcpy(ifr.ifr_name, interface.c_str());
	/* ifr.ifr_ifindex gets filled with that device's index */
	if (ioctl(canSock, SIOCGIFINDEX, &ifr) < 0)
	{
		throw ArmException((char*) "Ioctl failed:" + errno);
	}

	/* Select that CAN interface, and bind the socket to it. */
	struct sockaddr_can addr;
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;
	if (bind(canSock, (struct sockaddr*) &addr, sizeof(addr)) < 0)
	{
		throw ArmException((char*) "Bind failed:" + errno);
	}

	/* Send a message three times, wasting time
	 *  to get valid position data.
	 */
	struct can_frame frame;
	for (int ii = 0; ii < 4; ii++)
	{
		//Set the mode to cartesian
		setCartesian();

		//Send the mode setting CAN message
		while (1)
		{
			/* Read a message back from the CAN bus */
			uint bytes_read = recv(canSock, &frame, sizeof(struct can_frame), 0);

			if (bytes_read < sizeof(struct can_frame))
			{
				cerr << "Read: incomplete CAN frame" << endl;
			}

			if (frame.can_id == 0x37F)
			{ //Go to send reply
				boost::mutex::scoped_lock lock(stateMutex);
				currState.message = "STATUS: Ready for reply: " + toString<canid_t> (frame.can_id, hex);
				//if there is a reply ready, send it
				if (!sendQueue.empty())
				{
					//filled from front, sent from back
					can_frame frame = sendQueue.back();
					uint bytes_sent = write(canSock, &frame, sizeof(frame));
					if (bytes_sent < sizeof(struct can_frame))
					{
						throw ArmException((char*) "Failed to write CAN frame to arm:" + errno);
					}
					sendQueue.pop_back();
				}
				else
				{
					/* We have sent the message and received the 0x37F reply after it,
					 * which means we are now in cartesian mode and tthe position data
					 * will mean something. Otherwise, the position data is not well-defined,
					 * and causes problems.
					 */
					break;
				}
			}
		}
	}

	//Start the main CAN communication loop
	running = true;
	boost::thread doPoll(boost::bind(&ManusArm::pollCanSocket, this));

	//Wait for the main loop to update the state
	while (!currState.isValid)
	{
		sleep(1);
		//wait 120 msec
		//		timespec delay;
		//		timespec remaining;
		//		delay.tv_sec = 0;
		//		delay.tv_nsec = 120000000;
		//		nanosleep(&delay, &remaining);
	}
	cout << "ARM init done" << endl;
	return 0;
}

ManusArm::ManusArm()
{
	//Not valid until the polling thread updates the state
	currState.isValid = false;
}
/* Exceptions from the arm class */
ArmException::ArmException(char* message) throw ()
{
	msg = (char*) malloc(strlen(message));
	if (msg != NULL)
	{
		strncpy(msg, message, strlen(message));
	}
}

const char* ArmException::what() const throw ()
{
	return msg;
}
