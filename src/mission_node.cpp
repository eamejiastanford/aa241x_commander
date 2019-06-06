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
#include <std_msgs/Bool.h>
#include <bits/stdc++.h>
#include <vector>
#include <algorithm>
#include <time.h>

using namespace std;

// Define states
const std::string NOTOFFBOARD = "NOTOFFBOARD";
const std::string TAKEOFF = "TAKEOFF";
const std::string LINE = "LINE";
const std::string GOHOME = "GOHOME";
const std::string DROP_ALT = "DROP_ALT";
const std::string LAND = "LAND";
const std::string LOITER = "LOITER";
const std::string Pt_Trajectory = "Pt_Trajectory";
const std::string Perimeter_Search = "Perimeter_Search";
const std::string Navigate_to_land = "Navigate_to_land";
const std::string CAMERA_TEST = "CAMERA_TEST";

// Define possible missions
const std::string SPIRAL = "SPIRAL";
const std::string LINEANDHOME = "LINEANDHOME";
const std::string OUTERPERIM = "OUTERPERIM";
const std::string HOVERTEST = "HOVERTEST";
const std::string CAMERATEST = "CAMERATEST";

/**
 * class to contain the functionality of the mission node.
 */
class MissionNode {

public:

	/**
	 * example constructor.
	 */
        MissionNode(std::string mission_type, float target_v, float flight_alt, float loiter_t);

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
        float _flight_alt; // from center of lake lag
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
        std_msgs::Float64 _tag_abs_x_msg;
        std_msgs::Float64 _tag_abs_y_msg;
        std_msgs::Float64 _tag_abs_z_msg;



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

        // Position of april tag from the drone camera
        float _tag_rel_x = 0.0f; // Camera x
        float _tag_rel_y = 0.0f; // Camera y
        float _tag_rel_z = 0.0f; // Camera z
        bool _tag_found = false;

        // Position of april tag from the lake origin
        float _tag_abs_x = 0.0f; // N
        float _tag_abs_y = 0.0f; // E
        float _tag_abs_z = 0.0f; // U

        // Position
        float _xc = 0.0;
        float _yc = 0.0;
        float _zc = 0.0;

        // Orientation
        float _yaw = 0.0f;
        float _roll = 0.0f;
        float _pitch = 0.0f;

        // Extra data for state machine
        float _target_time = 0.0;
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
        ros::Subscriber _dAlongLine_sub;        // Subscribes to the traveled distance along the line
        ros::Subscriber _tag_rel_x_sub;
        ros::Subscriber _tag_rel_y_sub;
        ros::Subscriber _tag_rel_z_sub;
        ros::Subscriber _tag_found_sub;
        ros::Subscriber _pitch_sub;
        ros::Subscriber _yaw_sub;
        ros::Subscriber _roll_sub;
        ros::Subscriber _xc_sub;
        ros::Subscriber _yc_sub;
        ros::Subscriber _zc_sub;

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
        ros::Publisher _tag_abs_x_pub;
        ros::Publisher _tag_abs_y_pub;
        ros::Publisher _tag_abs_z_pub;

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
//	void localPosCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

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

        void tagRel_xCallback(const std_msgs::Float64::ConstPtr& msg);

        void tagRel_yCallback(const std_msgs::Float64::ConstPtr& msg);

        void tagRel_zCallback(const std_msgs::Float64::ConstPtr& msg);

        void tag_foundCallback(const std_msgs::Bool::ConstPtr& msg);

        void rollCallback(const std_msgs::Float64::ConstPtr& msg);

        void pitchCallback(const std_msgs::Float64::ConstPtr& msg);

        void yawCallback(const std_msgs::Float64::ConstPtr& msg);

        void xcCallback(const std_msgs::Float64::ConstPtr& msg);

        void ycCallback(const std_msgs::Float64::ConstPtr& msg);

        void zcCallback(const std_msgs::Float64::ConstPtr& msg);

        void waitForFCUConnection();

        void rotateCameraToLagFrame();
};


MissionNode::MissionNode(std::string mission_type, float target_v, float flight_alt, float loiter_t) :
    _MISSIONTYPE(mission_type), _target_v(target_v), _flight_alt(flight_alt), _target_time(loiter_t){

	// subscribe to the desired topics
	_state_sub = _nh.subscribe<mavros_msgs::State>("mavros/state", 1, &MissionNode::stateCallback, this);
        //_local_pos_sub = _nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, &MissionNode::localPosCallback, this);
	_sensor_meas_sub =_nh.subscribe<aa241x_mission::SensorMeasurement>("measurement", 10, &MissionNode::sensorMeasCallback, this);
	_battery_sub =_nh.subscribe<sensor_msgs::BatteryState>("mavros/battery", 10, &MissionNode::batteryCallback, this);
        _n_cycles_sub =_nh.subscribe<std_msgs::Int64>("n_cycles", 10, &MissionNode::nCyclesCallback, this);
        _mission_state_sub = _nh.subscribe<aa241x_mission::MissionState>("mission_state", 10, &MissionNode::missionStateCallback, this);
        _dAlongLine_sub = _nh.subscribe<std_msgs::Float64>("dAlongLine", 10, &MissionNode::dAlongLineCallback, this);
        _tag_rel_x_sub = _nh.subscribe<std_msgs::Float64>("tag_rel_x", 10, &MissionNode::tagRel_xCallback, this);
        _tag_rel_y_sub = _nh.subscribe<std_msgs::Float64>("tag_rel_y", 10, &MissionNode::tagRel_yCallback, this);
        _tag_rel_z_sub = _nh.subscribe<std_msgs::Float64>("tag_rel_z", 10, &MissionNode::tagRel_zCallback, this);
        _tag_found_sub = _nh.subscribe<std_msgs::Bool>("tagFound", 10, &MissionNode::tag_foundCallback, this);
        _roll_sub = _nh.subscribe<std_msgs::Float64>("roll", 10, &MissionNode::rollCallback, this);
        _pitch_sub = _nh.subscribe<std_msgs::Float64>("pitch", 10, &MissionNode::pitchCallback, this);
        _yaw_sub = _nh.subscribe<std_msgs::Float64>("yaw", 10, &MissionNode::yawCallback, this);
        _xc_sub = _nh.subscribe<std_msgs::Float64>("xc", 10, &MissionNode::xcCallback, this);
        _yc_sub = _nh.subscribe<std_msgs::Float64>("yc", 10, &MissionNode::ycCallback, this);
        _zc_sub = _nh.subscribe<std_msgs::Float64>("zc", 10, &MissionNode::zcCallback, this);

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
        _tag_abs_x_pub = _nh.advertise<std_msgs::Float64>("tag_abs_x", 10);
        _tag_abs_y_pub = _nh.advertise<std_msgs::Float64>("tag_abs_y", 10);
        _tag_abs_z_pub = _nh.advertise<std_msgs::Float64>("tag_abs_z", 10);
}

// Records the x measure of the position vector from the drone camera to the april tag (landing location)
void MissionNode::tagRel_xCallback(const std_msgs::Float64::ConstPtr& msg) {
    _tag_rel_x = msg->data;
}

// Records the y measure of the position vector from the drone camera to the april tag (landing location)
void MissionNode::tagRel_yCallback(const std_msgs::Float64::ConstPtr& msg) {
    _tag_rel_y = msg->data;
}

// Records the x measure of the position vector from the drone camera to the april tag (landing location)
void MissionNode::tagRel_zCallback(const std_msgs::Float64::ConstPtr& msg) {
    _tag_rel_z = msg->data;
}
void MissionNode::tag_foundCallback(const std_msgs::Bool::ConstPtr& msg) {
    _tag_found = msg->data;
}

void MissionNode::dAlongLineCallback(const std_msgs::Float64::ConstPtr& msg) {
    _dAlongLine_msg = *msg;
    _dAlongLine = _dAlongLine_msg.data;
}


void MissionNode::stateCallback(const mavros_msgs::State::ConstPtr& msg) {
	// save the state locally to be used in the main loop
	_current_state = *msg;
}

void MissionNode::xcCallback(const std_msgs::Float64::ConstPtr& msg) {
        // save the state locally to be used in the main loop
        _xc = msg ->data;
}
void MissionNode::ycCallback(const std_msgs::Float64::ConstPtr& msg) {
        // save the state locally to be used in the main loop
        _yc = msg ->data;
}
void MissionNode::zcCallback(const std_msgs::Float64::ConstPtr& msg) {
        // save the state locally to be used in the main loop
        _zc = msg->data;
}

void MissionNode::sensorMeasCallback(const aa241x_mission::SensorMeasurement::ConstPtr& msg) {
    // TODO: use the information from the measurement as desired
    // NOTE: this callback is for an example of how to setup a callback, you may
    // want to move this information to a mission handling node
    _id = msg->id;
    if (_id.size() >= 1) {
        _id_values_msg.data = _id[0];
        _id_values_pub.publish(_id_values_msg);
    }
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

void MissionNode::yawCallback(const std_msgs::Float64::ConstPtr& msg){
    _yaw = msg->data;
}

void MissionNode::pitchCallback(const std_msgs::Float64::ConstPtr& msg){
    _pitch = msg->data;
}

void MissionNode::rollCallback(const std_msgs::Float64::ConstPtr& msg){
    _roll = msg->data;
}

// Rotates tag position vector from camera frame to lake lag frame
void MissionNode::rotateCameraToLagFrame() {

    // Relative position from camera to tag (camera frame)
    float x = _tag_rel_x;
    float y = _tag_rel_y;
    float z = _tag_rel_z;

    // Dot products
    float bxDotnx = -cos(_pitch)*cos(_yaw);
    float bxDotny = -sin(_yaw)*cos(_pitch);
    float bxDotnz = sin(_pitch);
    float byDotnx = sin(_pitch)*sin(_roll)*cos(_yaw) - sin(_yaw)*cos(_roll);
    float byDotny = cos(_roll)*cos(_yaw) + sin(_pitch)*sin(_roll)*sin(_yaw);
    float byDotnz = sin(_roll)*cos(_pitch);
    float bzDotnx = -sin(_roll)*sin(_yaw) - sin(_pitch)*cos(_roll)*cos(_yaw);
    float bzDotny = sin(_roll)*cos(_yaw)-sin(_pitch)*sin(_yaw)*cos(_roll);
    float bzDotnz = -cos(_pitch)*cos(_roll);

    // Rotate the vector and move to lake frame (Distances from center of Lake Lag)
    _tag_abs_x = x * bxDotnx + y * byDotnx + z * bzDotnx + _xc;
    _tag_abs_y = x * bxDotny + y * byDotny + z * bzDotny + _yc;
    _tag_abs_z = x * bxDotnz + y * byDotnz + z * bzDotnz + _zc;
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

        if (_MISSIONTYPE == SPIRAL) {
            float outer_radius = 160.0;
            float diameter_search = (5.0/7.0)*(_flight_alt)+28.57; // Diameter of ground below drone which is realized
            _n_cycles_target = static_cast<int>((outer_radius/(0.8*diameter_search))+1);
        }
        else if (_MISSIONTYPE == OUTERPERIM) {
            _n_cycles_target = 1;
        }
        else if (_MISSIONTYPE == CAMERATEST) {
            _STATE = CAMERA_TEST;
        }

        if (_MISSIONTYPE == HOVERTEST) {
            _target_time = 300.0;
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
                        if(abs(_n_total[iMatch][i] - _n[index]) <= 1e-10) {
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
                    else if (_tag_found == true){
                        _STATE == Navigate_to_land;
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
                float landing_offset = _u_offset; // for now, land at the starting location
                if (abs(_zc - (3.0+landing_offset)) < 0.1 ){ // For landing at the landing location
                    _STATE = Navigate_to_land;
                    //_STATE = LAND;
                }
            }
            else if (_STATE == Navigate_to_land){
                // Updating tag absolute position in lake lag frame
                rotateCameraToLagFrame();
                _tag_abs_x_msg.data = _tag_abs_x;
                _tag_abs_y_msg.data = _tag_abs_y;
                _tag_abs_z_msg.data = _tag_abs_z;
                // Publish Absolute distances:
                _tag_abs_x_pub.publish(_tag_abs_x_msg);
                _tag_abs_y_pub.publish(_tag_abs_y_msg);
                _tag_abs_z_pub.publish(_tag_abs_z_msg);

                if (abs(_xc - _tag_abs_x) < 0.1 && abs(_yc - _tag_abs_y) < 0.1){
                    _STATE = LAND;
                }
                else if (_tag_found == true){
                    // currently this will loiter on every loop that sees a tag; needs to be changed to wait until drone navigates
                    // to where it thinks the final landing location is
                    _target_time = 10.0; // sets target time for hover over the April tag
                    _STATE = LOITER;
                }
            }
            else if(_STATE == LINE) {
                // Check if we've travelled enough along the line
                if (abs(_dAlongLine) >= 30.0){
                        _STATE = GOHOME;
                     }
            }
            else if(_STATE == CAMERA_TEST) {
                // Update tag absolute position in lake lag frame
                rotateCameraToLagFrame();
                _tag_abs_x_msg.data = _tag_abs_x;
                _tag_abs_y_msg.data = _tag_abs_y;
                _tag_abs_z_msg.data = _tag_abs_z;
                // Publish Absolute distances:
                _tag_abs_x_pub.publish(_tag_abs_x_msg);
                _tag_abs_y_pub.publish(_tag_abs_y_msg);
                _tag_abs_z_pub.publish(_tag_abs_z_msg);

            }
            
            // Publish flight data
            _flight_alt_msg.data = _flight_alt; // flight altitude from center of lake lag
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
        // Specify Mission Type: OPTIONS: LINEANDHOME, OUTERPERIM, SPIRAL, HOVERTEST, CAMERATEST
        std::string mission_type = CAMERATEST;
        float target_v = 4.0;
        float flight_alt = 45.0;
        float loiter_t = 32.0;

	// create the node
        MissionNode node(mission_type, target_v, flight_alt, loiter_t);

	// run the node
	return node.run();
}
