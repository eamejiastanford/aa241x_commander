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
#include <aa241x_mission/RequestLandingPosition.h>
#include <aa241x_mission/PersonEstimate.h>

#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>
#include <bits/stdc++.h>
#include <vector>
#include <algorithm>
#include <time.h>

using namespace std;

// Define states
const std::string NOTOFFBOARD = "NOTOFFBOARD";
const std::string TAKEOFF = "TAKEOFF";
const std::string LINE = "LINE";
const std::string LINE2 = "LINE2";
const std::string LINE3 = "LINE3";
const std::string GOHOME = "GOHOME";
const std::string DROP_ALT = "DROP_ALT";
const std::string LAND = "LAND";
const std::string LOITER = "LOITER";
const std::string Pt_Trajectory = "Pt_Trajectory";
const std::string Perimeter_Search = "Perimeter_Search";

// Define possible missions
const std::string SPIRAL = "SPIRAL";
const std::string LINEANDHOME = "LINEANDHOME";
const std::string OUTERPERIM = "OUTERPERIM";
const std::string HOVERTEST = "HOVERTEST";

/**
 * class to contain the functionality of the mission node.
 */
class MissionNode {

public:

	/**
	 * example constructor.
	 */
        MissionNode(std::string mission_type, float target_v, float flight_alt);

	/**
	 * the main loop to be run for this node (called by the `main` function)
	 * @return exit code
	 */
	int run();


private:

	// node handler
	ros::NodeHandle _nh;

        std::string _MISSIONTYPE;

        std::string _STATE;
        float _flight_alt;
        int _n_cycles = 0;
        float _target_v = 0.0;

	// data
	mavros_msgs::State _current_state;
	geometry_msgs::PoseStamped _current_local_pos;
	sensor_msgs::BatteryState _battery_state;

        std_msgs::String _droneState_msg;
        std_msgs::Float64 _flight_alt_msg;
        std_msgs::Float64 _dAlongLine_msg;
        std_msgs::Float64 _n_values_msg;
        std_msgs::Float64 _e_values_msg;
        std_msgs::Int64 _id_values_msg;
        aa241x_mission::PersonEstimate _person_found_msg;
        std_msgs::Float64 _speed_msg;

        // Beacon current information
        std::vector<int> _id;
        std::vector<float> _n;
        std::vector<float> _e;

        // Beacon total information
        std::vector<int> _id_total;
        vector<vector<float>> _n_total;
        vector<vector<float>> _e_total;

	// offset information
	float _e_offset = 0.0f;
	float _n_offset = 0.0f;
	float _u_offset = 0.0f;

	// landing position
	float _landing_e = 0.0f;
	float _landing_n = 0.0f;

        // Position
        float _xc;
        float _yc;
        float _zc;

        // Extra data for state machine
        time_t _start;
        time_t _end;
        float _dAlongLine = 0;

	// subscribers
        ros::Subscriber _state_sub;             // the current state of the pixhawk
	ros::Subscriber _local_pos_sub;		// local position information
	ros::Subscriber _sensor_meas_sub;	// mission sensor measurement
        ros::Subscriber _mission_state_sub;     // mission state
        ros::Subscriber _battery_sub;		// the curre = 0nt battery information
        ros::Subscriber _n_cycles_sub;          // number of cycles completed in perimeter search
        ros::Subscriber _dAlongLine_sub; // Subscribes to the traveled distance along the line

        // TODO: add subscribers here

	//service
	ros::ServiceClient _landing_loc_client;

	// publishers
        ros::Publisher _droneState_pub;         // the current state of the drone
        ros::Publisher _flight_alt_pub;         // the targeted flight altitude of the mission
        ros::Publisher _id_values_pub;
        ros::Publisher _e_values_pub;
        ros::Publisher _n_values_pub;
        ros::Publisher _person_found_pub;
        ros::Publisher _speed_pub;

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
         * @param msg mission state = 0
	 */
        void missionStateCallback(const aa241x_mission::MissionState::ConstPtr& msg);

	/**
	 * callback for the battery information from the Pixhawk.
	 * @param msg the sensor message containing the battery data
	 *     (http://docs.ros.org/api/sensor_msgs/html/msg/BatteryState.html)
	 */
	void batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg);

    void dAlongLineCallback(const std_msgs::Float64::ConstPtr& msg);

        void nCyclesCallback(const std_msgs::Int64::ConstPtr& msg);

        void waitForFCUConnection();

	// TODO: add callbacks here

	// helper functions

};


MissionNode::MissionNode(std::string mission_type, float target_v, float flight_alt) :
    _MISSIONTYPE(mission_type), _target_v(target_v), _flight_alt(flight_alt){

	// subscribe to the desired topics
	_state_sub = _nh.subscribe<mavros_msgs::State>("mavros/state", 1, &MissionNode::stateCallback, this);
	_local_pos_sub = _nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, &MissionNode::localPosCallback, this);
	_sensor_meas_sub =_nh.subscribe<aa241x_mission::SensorMeasurement>("measurement", 10, &MissionNode::sensorMeasCallback, this);
	_battery_sub =_nh.subscribe<sensor_msgs::BatteryState>("mavros/battery", 10, &MissionNode::batteryCallback, this);
        _n_cycles_sub =_nh.subscribe<std_msgs::Int64>("n_cycles", 10, &MissionNode::nCyclesCallback, this);
        _mission_state_sub = _nh.subscribe<aa241x_mission::MissionState>("mission_state", 10, &MissionNode::missionStateCallback, this);
        _dAlongLine_sub = _nh.subscribe<std_msgs::Float64>("dAlongLine", 10, &MissionNode::dAlongLineCallback, this);

	// service
	_landing_loc_client = _nh.serviceClient<aa241x_mission::RequestLandingPosition>("lake_lag_landing_loc");

	// advertise the published detailed
        _droneState_pub = _nh.advertise<std_msgs::String>("drone_state", 10);
        _flight_alt_pub = _nh.advertise<std_msgs::Float64>("flight_alt", 10);
        _e_values_pub = _nh.advertise<std_msgs::Float64>("e_value", 10);
        _n_values_pub = _nh.advertise<std_msgs::Float64>("n_value", 10);
        _id_values_pub = _nh.advertise<std_msgs::Int64>("id_value", 10);
        _person_found_pub = _nh.advertise<aa241x_mission::PersonEstimate>("person_found", 10);
        _speed_pub = _nh.advertise<std_msgs::Float64>("speed", 10);
}

void MissionNode::dAlongLineCallback(const std_msgs::Float64::ConstPtr& msg) {

    _dAlongLine_msg = *msg;

    _dAlongLine = _dAlongLine_msg.data;
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
        _current_local_pos.pose.position.z -= _u_offset;

        _xc = _current_local_pos.pose.position.x;
        _yc = _current_local_pos.pose.position.y;
        _zc = _current_local_pos.pose.position.z;
}


void MissionNode::sensorMeasCallback(const aa241x_mission::SensorMeasurement::ConstPtr& msg) {
    // TODO: use the information from the measurement as desired
    // NOTE: this callback is for an example of how to setup a callback, you may
    // want to move this information to a mission handling node
    _id = msg->id;
    _n = msg->n;
    _e = msg->e;
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

	// get the landing position
        aa241x_mission::RequestLandingPosition srv;
        if (_landing_loc_client.call(srv)) {
             // NOTE: saving the landing East and North coordinates to class member variables
             _landing_e = srv.response.east;
             _landing_n = srv.response.north;
             ROS_INFO("landing coordinate: (%0.2f, %0.2f)", _landing_n, _landing_e);
        } else {
             ROS_ERROR("unable to get landing location in Lake Lag frame!");
        }

	// set the loop rate in [Hz]
        ros::Rate rate(10.0);

        int _n_cycles_target = 0;
        float _target_time = 0.0;

        if (_MISSIONTYPE == SPIRAL) {
            _n_cycles_target = 6;
        }
        else if (_MISSIONTYPE == OUTERPERIM) {
            _n_cycles_target = 1;
        }

        if (_MISSIONTYPE == HOVERTEST) {
            _target_time = 300.0;
        }
        else {
            _target_time = 5.0;
        }

	// main loop
	while (ros::ok()) {


            float x0;
            float y0;
            float z0;
            bool new_beacon_found = false;

            // Loop through id's found (from mission node)
            for( int index = 0; index < _id.size(); ++index) {

                // Pull the current id to check
                int id_current = _id.at(index);

                // Loop through known id's to find a match
                int iMatch = _id_total.size();
                for(int i = 0; i < _id_total.size(); i++) {
                    if(_id_total[i] == id_current) {
                        iMatch = i;
                    }
                }
                _id_values_msg.data = id_current;
                _id_values_pub.publish(_id_values_msg);


                // Check if we've already seen this ID (beacon)
                if(iMatch != _id_total.size() ) {
                    int jMatch = _n_total[iMatch].size();
                    for(int i = 0; i < _n_total[iMatch].size(); i++) {
                        if(abs(_n_total[iMatch][i] - _n[index]) <= 1e-7) {
                            jMatch = i;
                        }
                    }
                    // Append reading to list of measurements for this beacon
                    if(jMatch == _n_total[iMatch].size()) {
                        _n_total[iMatch].push_back(_n[index]);
                        _e_total[iMatch].push_back(_e[index]);
                        _n_values_msg.data =_n[index];
                        _e_values_msg.data =_e[index];
                        _n_values_pub.publish(_n_values_msg);
                        _e_values_pub.publish(_e_values_msg);
                    }

                } else {
                    // Add beacon to list of found beacons with first measurement
                    new_beacon_found = true;
                    _id_total.push_back(id_current);
                    vector<float> n_current;
                    n_current.push_back(_n[index]);
                    _n_total.push_back(n_current);
                    vector<float> e_current;
                    e_current.push_back(_e[index]);
                    _e_total.push_back(e_current);

                }
            }

            // Record the takeoff position
            if (_current_state.mode != "OFFBOARD") {
                x0 = _xc;
                y0 = _yc;
                z0 = _zc;

            }

            // State machine
            if     (_STATE == TAKEOFF) {
                // Check if we are close enough to finishing takeoff
                if(abs(_zc - _flight_alt) < 1.0) {
                    if (_MISSIONTYPE == LINEANDHOME) {
                        _STATE = LINE;
                    }
                    else if (_MISSIONTYPE == HOVERTEST) {
                        _start = time(0);
                        _STATE = LOITER;
                    }
                    else {
                        _STATE = Perimeter_Search;
                    }
                }
                _speed_msg.data = _target_v;
                _speed_pub.publish(_speed_msg);
            }
            else if(_STATE == Perimeter_Search) {
                // Check if we have completed enough cycles
                if(new_beacon_found) {
                    _start = time(0);
                    _STATE = LOITER;
                }
                else if(_n_cycles == _n_cycles_target) {//static_cast<int>(radius/radius_search)){ // completed two rotations
                    _STATE = GOHOME;
                }

            }
            else if(_STATE == LOITER) {
                _end = time(NULL);
                if (_end - _start >= _target_time) {
                    if (_MISSIONTYPE == HOVERTEST) {
                        _STATE = GOHOME;
                    }
                    else {
                        _STATE = Perimeter_Search;
                    }
                }
            }
            else if(_STATE == GOHOME) {
                // Check if we are close enough to landing location
                if(abs(_xc - _landing_e) <= 1.0 && abs(_yc - _landing_n) <= 1.0) {
                    _STATE = DROP_ALT;

                    for( int index = 0; index < _id_total.size(); ++index) {
                        // Pull the current id to check
                        int id_current = _id_total[index];
                        vector<float> n_current = _n_total[index];
                        vector<float> e_current = _e_total[index];
                        float n_total = 0.0;
                        float e_total = 0.0;
                        for(int i=0; i<n_current.size(); ++i) {
                                n_total += n_current[i];
                                e_total += e_current[i];
                        }
                        float n_avg = n_total/n_current.size();
                        float e_avg = e_total/e_current.size();
                        _person_found_msg.header.stamp = ros::Time::now();
                        _person_found_msg.id = id_current;
                        _person_found_msg.n = n_avg;
                        _person_found_msg.e = e_avg;
                        _person_found_pub.publish(_person_found_msg);
                     }
                }
            }
            else if (_STATE == DROP_ALT){
                // If the altitude has dropped below 6.0 meters, switch to landing (slows down descent)
                if (abs(_zc - (3.0-_u_offset)) < 0.1 ){
                    _STATE = LAND;
                }
            }
            else if(_STATE == LINE) {
                // Check if we've travelled enough along the line
                if (abs(_dAlongLine) >= 30.0){
                        _STATE = GOHOME;
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
        // Specify Mission Type: OPTIONS: LINEANDHOME, OUTERPERIM, SPIRAL
        std::string mission_type = HOVERTEST;
        float target_v = 6.0;
        float flight_alt = 20.0;

	// create the node
        MissionNode node(mission_type, target_v, flight_alt);

	// run the node
	return node.run();
}
