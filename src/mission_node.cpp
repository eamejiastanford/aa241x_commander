/**
 * This is a skeleton for a recommended second node that should contain your
 * "mission logic".  For more detail on the recommended structure, please see
 * the readme.
 */


// includes
#include <math.h>
#include <ros/ros.h>

// topic data
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/BatteryState.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>

#include <aa241x_mission/SensorMeasurement.h>
#include <aa241x_mission/MissionState.h>

/**
 * class to contain the functionality of the mission node.
 */
class MissionNode {

public:

	/**
	 * example constructor.
	 */
	MissionNode();

	/**
	 * the main loop to be run for this node (called by the `main` function)
	 * @return exit code
	 */
	int run();


private:

	// node handler
	ros::NodeHandle _nh;

	// data
	mavros_msgs::State _current_state;
	geometry_msgs::PoseStamped _current_local_pos;
	sensor_msgs::BatteryState _battery_state;

	// offset information
	float _e_offset = 0.0f;
	float _n_offset = 0.0f;
	float _u_offset = 0.0f;

	// subscribers
	ros::Subscriber _state_sub;			// the current state of the pixhawk
	ros::Subscriber _local_pos_sub;		// local position information
	ros::Subscriber _sensor_meas_sub;	// mission sensor measurement
	ros::Subscriber _mission_state_sub; // mission state
	ros::Subscriber _battery_sub;		// the current battery information
	// TODO: add subscribers here

	// publishers

	// TODO: you may want to have the mission node publish commands to your
	// control node.

	// callbacks

	/**
	 * callback for the current state of the pixhawk.
	 * @param msg mavros state message
	 */
	void stateCallback(const mavros_msgs::State::ConstPtr& msg);

	/**
	 * callback for the local position and orientation computed by the pixhawk.
	 * @param msg pose stamped message type
	 */
	void localPosCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

	/**
	 * callback for the sensor measurement for the AA241x mission
	 * NOTE: you may end up wanting to move this to a separate mission handling
	 * node
	 * @param msg the AA241x sensor measurement
	 */
	void sensorMeasCallback(const aa241x_mission::SensorMeasurement::ConstPtr& msg);

	/**
	 * callback for the mission state for the AA241x mission
	 * this includes the offset information for the lake lag coordinate frame
	 * @param msg mission state
	 */
	void missionStateCallback(const aa241x_mission::MissionState::ConstPtr& msg);

	/**
	 * callback for the battery information from the Pixhawk.
	 * @param msg the sensor message containing the battery data
	 *     (http://docs.ros.org/api/sensor_msgs/html/msg/BatteryState.html)
	 */
	void batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg);

	// TODO: add callbacks here

	// helper functions

};


MissionNode::MissionNode() {

	// subscribe to the desired topics
	_state_sub = _nh.subscribe<mavros_msgs::State>("mavros/state", 1, &MissionNode::stateCallback, this);
	_local_pos_sub = _nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, &MissionNode::localPosCallback, this);
	_sensor_meas_sub =_nh.subscribe<aa241x_mission::SensorMeasurement>("measurement", 10, &MissionNode::sensorMeasCallback, this);
	_battery_sub =_nh.subscribe<sensor_msgs::BatteryState>("mavros/battery", 10, &MissionNode::batteryCallback, this);

	// advertise the published detailed
}


void MissionNode::stateCallback(const mavros_msgs::State::ConstPtr& msg) {
	// save the state locally to be used in the main loop
	_current_state = *msg;
}


void MissionNode::localPosCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	// save the current local position locally to be used in the main loop
	// TODO: account for offset to convert from PX4 coordinate to lake lag frame
	_current_local_pos = *msg;

	// adjust the position with the offset to convert the saved local position
	// information into the Lake Lag frame
	_current_local_pos.pose.position.x += _e_offset;
	_current_local_pos.pose.position.y += _n_offset;
	_current_local_pos.pose.position.z += _u_offset;
}


void MissionNode::sensorMeasCallback(const aa241x_mission::SensorMeasurement::ConstPtr& msg) {
	// TODO: use the information from the measurement as desired
}


void MissionNode::missionStateCallback(const aa241x_mission::MissionState::ConstPtr& msg) {
	// save the offset information
	_e_offset = msg->e_offset;
	_n_offset = msg->n_offset;
	_u_offset = msg->u_offset;
}


void MissionNode::batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg) {
	_battery_state = *msg;

	// TODO: currently the callback is configured to just save the data
	//
	// you can either make decisions based on the battery information here (e.g.
	// change the state of the mission) or you can make those decisions in the
	// while loop in the run() function
}


int MissionNode::run() {

	// set the loop rate in [Hz]
	ros::Rate rate(10.0);

	// main loop
	while (ros::ok()) {

		// TODO: make high level decisions here

		// TODO: we recommend you publish high level commands (e.g. position or
		// general direction) here that your control node will use to actually
		// fly the given mission


		// remember need to always call spin once for the callbacks to trigger
		ros::spinOnce();
		rate.sleep();
	}

	// return  exit code
	return EXIT_SUCCESS;
}


int main(int argc, char **argv) {

	// initialize th enode
	ros::init(argc, argv, "mission_node");

	// get parameters from the launch file which define some mission
	// settings
	ros::NodeHandle private_nh("~");
	// TODO: determine settings

	// create the node
	MissionNode node;

	// run the node
	return node.run();
}