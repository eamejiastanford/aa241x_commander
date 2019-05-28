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

#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>

// Define states
const std::string NOTOFFBOARD = "NOTOFFBOARD";
const std::string TAKEOFF = "TAKEOFF";
const std::string LINE = "LINE";
const std::string LINE2 = "LINE2";
const std::string LINE3 = "LINE3";
const std::string GOHOME = "GOHOME";
const std::string LAND = "LAND";
const std::string Pt_Trajectory = "Pt_Trajectory";
const std::string Perimeter_Search = "Perimeter_Search";

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

        std::string _STATE;
        float _flight_alt;
        int _n_cycles;

	// data
	mavros_msgs::State _current_state;
	geometry_msgs::PoseStamped _current_local_pos;
	sensor_msgs::BatteryState _battery_state;

        std_msgs::String _droneState_msg;
        aa241x_mission::SensorMeasurement _beacon_msg;
        std_msgs::Float64 _flight_alt_msg;

	// offset information
	float _e_offset = 0.0f;
	float _n_offset = 0.0f;
	float _u_offset = 0.0f;

	// subscribers
        ros::Subscriber _state_sub;             // the current state of the pixhawk
	ros::Subscriber _local_pos_sub;		// local position information
	ros::Subscriber _sensor_meas_sub;	// mission sensor measurement
        ros::Subscriber _mission_state_sub;     // mission state
	ros::Subscriber _battery_sub;		// the current battery information
        ros::Subscriber _n_cycles_sub;          // number of cycles completed in perimeter search

	// TODO: add subscribers here

	// publishers
        ros::Publisher _droneState_pub;         // the current state of the drone
        ros::Publisher _beaconState_pub;        // the current state of the beacon information
        ros::Publisher _flight_alt_pub;         // the targeted flight altitude of the mission

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
        void missionStateCalxlback(const aa241x_mission::MissionState::ConstPtr& msg);

	/**
	 * callback for the battery information from the Pixhawk.
	 * @param msg the sensor message containing the battery data
	 *     (http://docs.ros.org/api/sensor_msgs/html/msg/BatteryState.html)
	 */
	void batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg);

        void nCyclesCallback(const std_msgs::Int64::ConstPtr& msg);

        void waitForFCUConnection();

	// TODO: add callbacks here

	// helper functions

};


MissionNode::MissionNode() {

	// subscribe to the desired topics
	_state_sub = _nh.subscribe<mavros_msgs::State>("mavros/state", 1, &MissionNode::stateCallback, this);
	_local_pos_sub = _nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, &MissionNode::localPosCallback, this);
	_sensor_meas_sub =_nh.subscribe<aa241x_mission::SensorMeasurement>("measurement", 10, &MissionNode::sensorMeasCallback, this);
	_battery_sub =_nh.subscribe<sensor_msgs::BatteryState>("mavros/battery", 10, &MissionNode::batteryCallback, this);
        _n_cycles_sub =_nh.subscribe<std_msgs::Int64>("n_cycles", 10, &MissionNode::nCyclesCallback, this);

	// advertise the published detailed
        _droneState_pub = _nh.advertise<std_msgs::String>("drone_state", 10);
        _beaconState_pub = _nh.advertise<aa241x_mission::SensorMeasurement>("beacon_state", 10);
        _flight_alt_pub = _nh.advertise<std_msgs::Float64>("flight_alt", 10);
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

void MissionNode::nCyclesCallback(const std_msgs::Int64::ConstPtr& msg) {
        _n_cycles = msg->data;
}

void MissionNode::waitForFCUConnection() {
        // wait for FCU connection by just spinning the callback until connected
        ros::Rate rate(5.0);
        while (ros::ok() && _current_state.connected) {
                ros::spinOnce();
                rate.sleep();
        }
}


int MissionNode::run() {

        // Initially, only valid state is takeoff
        _STATE = TAKEOFF;

        waitForFCUConnection();
        ROS_INFO("connected to the FCU");

	// set the loop rate in [Hz]
	ros::Rate rate(10.0);

	// main loop
	while (ros::ok()) {

            float x0;
            float y0;
            float z0;

            float xc = _current_local_pos.pose.position.x;
            float yc = _current_local_pos.pose.position.y;
            float zc = _current_local_pos.pose.position.z;
            float qx = _current_local_pos.pose.orientation.x;
            float qy = _current_local_pos.pose.orientation.y;
            float qz = _current_local_pos.pose.orientation.z;
            float qw = _current_local_pos.pose.orientation.w;

            // Record the takeoff position
            if (_current_state.mode != "OFFBOARD") {
                x0 = xc;
                y0 = yc;
                z0 = zc;

            }

            // Set flight parameters
            _flight_alt = 50.0;

            // State machine
            if     (_STATE == TAKEOFF) {
                // Check if we are close enough to finishing takeoff
                if(abs(zc - _flight_alt) < 0.1) {
                    _STATE = Perimeter_Search;
                }

            }
            else if(_STATE == Perimeter_Search) {
                // Check if we have completed enough cycles
                if(_n_cycles == 1) {//static_cast<int>(radius/radius_search)){ // completed two rotations
                    _STATE = GOHOME;
                }

            }
            else if(_STATE == GOHOME) {
                // Check if we are close enough to landing location
                if(abs(xc - x0) <= 0.1 && abs(yc - y0) <= 0.1) {
                    _STATE = LAND;
                    _flight_alt = 0.0;
                }

            }

            // Publish flight data
            _flight_alt_msg.data = _flight_alt;
            _flight_alt_pub.publish(_flight_alt_msg);

            // Publish drone mission state
            _droneState_msg.data = _STATE;
            _droneState_pub.publish(_droneState_msg);

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
