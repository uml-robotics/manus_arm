/*
 * dish_viz.cpp
 *
 *  Created on: Jun 4, 2012
 *      Author: ams
 */

#include "dish_viz.h"

class DataHandler {
private:
	DishVisualizer dViz;

public:
	DataHandler() {
		dViz.init();
	}

	void messageCallback(const neuro_recv::dish_state::ConstPtr &msg) {
		for(int ii = 0; ii < 60; ii++)
		{
			ROS_INFO("%f", (double)msg->samples[ii]);
			dViz.update(ii, (double)msg->samples[ii]);
		}

	}
};

int main(int argc, char **argv) {
	// Set up ROS.
	ros::init(argc, argv, "listener");
	ros::NodeHandle n;

	// Declare variables that can be modified by launch file or command line.
	int rate;
	string topic;

	// Initialize node parameters from launch file or command line.
	// Use a private node handle so that multiple instances of the node can be run simultaneously
	// while using different parameters.
	ros::NodeHandle private_node_handle_("~");
	private_node_handle_.param("rate", rate, int(100));
	private_node_handle_.param("topic", topic, string("dish_states"));

	// Create a new DataHandler object.
	DataHandler *node_example = new DataHandler();

	// Create a subscriber.
	// Name the topic, message queue, callback function with class name, and object containing callback function.
	ros::Subscriber sub_message = n.subscribe(topic.c_str(), 1000,
			&DataHandler::messageCallback, node_example);

	// Tell ROS how fast to run this node.
	ros::Rate r(rate);

	// Main loop.
	while (n.ok()) {
		ros::spinOnce();
		r.sleep();
	}

	return 0;
} // end main()

