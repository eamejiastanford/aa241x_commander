/**
 * skeleton / example code for a node to do command and control of the pixhawk
 */

// includes
#include <math.h>
#include <ros/ros.h>
#include <string>
#include <iostream>

// topic data
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>

#include <aa241x_mission/SensorMeasurement.h>
#include <aa241x_mission/MissionState.h>
#include <aa241x_mission/RequestLandingPosition.h>

#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Bool.h>
#include <vector>

// Define states
const std::string TAKEOFF = "TAKEOFF";
const std::string LINE = "LINE";
const std::string GOHOME = "GOHOME";
const std::string LAND = "LAND";
const std::string LOITER = "LOITER";
const std::string DROP_ALT = "DROP_ALT";
const std::string Navigate_to_land = "Navigate_to_land";
const std::string Perimeter_Search = "Perimeter_Search";
const std::string CAMERA_TEST = "CAMERA_TEST";
const std::string MINISEARCH = "MINISEARCH";
const std::string Hover_Search = "Hover_Search";
const std::string GOHOME_LAND = "GOHOME_LAND";


/**
 * class to contain the functionality of the controller node.
 */
class ControlNode {

public:

        /**
         * example constructor.
         * @param flight_alt the desired altitude for the takeoff point.
         */
        ControlNode(float yaw_angle, float xLine, float yLine, float vDes);

        /**
         * the main loop to be run for this node (called by the `main` function)
         * @return exit code
         */
        int run();


private:
        // node handler
        ros::NodeHandle _nh;

        // TODO: add any settings, etc, here
        float _thetaLine = 0.0f;
        float _xLine = 0.0f;
        float _yLine = 0.0f;
        float _vDes = 0.0f;

        // data
        mavros_msgs::State _current_state;
        geometry_msgs::PoseStamped _current_local_pos;
        aa241x_mission::SensorMeasurement _current_sensor_meas;
        std::string _STATE;
        float _flight_alt = 0;

        // landing position
        float _landing_e = 0.0f;
        float _landing_n = 0.0f;
        float _tag_Alt = 0.0f;

        // GPS
        float _xGPS = 0.0;
        float _yGPS = 0.0;
        float _zGPS = 0.0;
        bool _useGPS = false;

        // Physical state variables
        float _xc;
        float _yc;
        float _zc;
        float _roll;
        float _pitch;
        float _yaw;
        float _vxc;
        float _vyc;
        float _vzc;

        // Takeoff position variables
        float _x0;
        float _y0;
        float _z0;
        float _xc0;
        float _yc0;
        float _zc0;

        // Custom message topics
        std_msgs::String _droneState_msg;
        std_msgs::Float64 _dPerp_msg;
        std_msgs::Float64 _dAlongLine_msg;
        std_msgs::Float64 _currentYaw_msg;
        std_msgs::Int64 _n_cycles_msg;
        std_msgs::Float64 _xc_msg;
        std_msgs::Float64 _yc_msg;
        std_msgs::Float64 _zc_msg;
        std_msgs::Float64 _z0_msg;
        std_msgs::Float64 _yaw_msg;
        std_msgs::Float64 _pitch_msg;
        std_msgs::Float64 _roll_msg;
        geometry_msgs::PoseStamped _gps_position_msg;

        geometry_msgs::TwistStamped _current_local_twist;

        // Beacon search initializations
        std::vector<int> _id;
        std::vector<float> _n;
        std::vector<float> _e;

        // Perimeter search variables
        int _angle = -10;
        int _count = 0;
        int _cycle = 0;
        float _outer_radius = 160.0;
        float _radius = _outer_radius; //160
        float radius_Alt_search = 0.5;

        // offset information
        float _e_offset = 0;
        float _n_offset = 0;
        float _u_offset = 0;
        float _current_lat = 0.0f;
        float _current_lon = 0.0f;

        // Loiter
        float _xLoiter = 0;
        float _yLoiter = 0;

        // actuator saturation
        float _vxMax;
        float _vyMax;
        float _vzMax = 1.0f;
        float _yawRate_max = 1.0f;
        float _vzTakeoff = 3.0f;
        bool _tag_found = false;
        float _tag_abs_x = 0.0f;
        float _tag_abs_y = 0.0f;
        float _tag_abs_z = 0.0f;

        float _e_home = 0.0;
        float _n_home = 0.0;

        // subscribers
        ros::Subscriber _state_sub;                 // the current state of the pixhawk
        ros::Subscriber _local_pos_sub;             // local position information
        ros::Subscriber _local_twist_sub;		// local twist information
        ros::Subscriber _sensor_meas_sub;           // mission sensor measurement
        ros::Subscriber _mission_state_sub;         // mission state
        ros::Subscriber _droneState_sub;            // the current state of the drone
        ros::Subscriber _beaconState_sub;           // the current state of the beacon information
        ros::Subscriber _flight_alt_sub;            // the targeted flight altitude of the mission
        ros::Subscriber _target_v_sub;
        ros::Subscriber _tag_found_sub;
        ros::Subscriber _tag_abs_x_sub;
        ros::Subscriber _tag_abs_y_sub;
        ros::Subscriber _tag_abs_z_sub;
        ros::Subscriber _e_home_sub;
        ros::Subscriber _n_home_sub;
        ros::Subscriber _xLoiter_sub;
        ros::Subscriber _yLoiter_sub;
        ros::Subscriber _tag_Alt_sub;
        ros::Subscriber _gps_position_sub;
        ros::Subscriber _useGPS_sub;


        // publishers
        ros::Publisher _cmd_pub;
        ros::Publisher _dPerp_pub;
        ros::Publisher _dAlongLine_pub;
        ros::Publisher _currentYaw_pub;
        ros::Publisher _n_cycles_pub;
        ros::Publisher _xc_pub;
        ros::Publisher _yc_pub;
        ros::Publisher _zc_pub;
        ros::Publisher _z0_pub;
        ros::Publisher _yaw_pub;
        ros::Publisher _roll_pub;
        ros::Publisher _pitch_pub;

        //service
        ros::ServiceClient _landing_loc_client;

        // callbacks

        void useGPSCallback(const std_msgs::Bool::ConstPtr& msg);

        void gpsPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

        void xLoiterCallback(const std_msgs::Float64::ConstPtr& msg);

        void yLoiterCallback(const std_msgs::Float64::ConstPtr& msg);

        void stateCallback(const mavros_msgs::State::ConstPtr& msg);

        /**
         * callback for the local position and orientation computed by the pixhawk.
         * @param msg pose stamped message type
         */
        void localPosCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void localTwistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);

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

        void beaconStateCallback(const aa241x_mission::SensorMeasurement::ConstPtr& msg);

        void droneStateCallback(const std_msgs::String::ConstPtr& msg);

        void flightAltCallback(const std_msgs::Float64::ConstPtr& msg);

        void targetVCallback(const std_msgs::Float64::ConstPtr& msg);


        void toEulerAngle(const float q[4]);

        // TODO: add callbacks here

        // helper functions

        /**
         * wait for the connection to the Pixhawk to be established.
         */
        void waitForFCUConnection();

        void saturateVelocities(geometry_msgs::Vector3& vel);

        void yawControl(mavros_msgs::PositionTarget& cmd);

        void takeoffControl(geometry_msgs::Vector3& vel);

        void perimeterSearchControl(geometry_msgs::Vector3& vel);

        void lineControl(geometry_msgs::Vector3& vel);

        void goHomeControl(geometry_msgs::Vector3& vel);

        void goHomeLandControl(geometry_msgs::Vector3& vel);

        void landControl(geometry_msgs::Vector3& vel);

        void dropAltControl(geometry_msgs::Vector3& vel);

        void navToLandControl(geometry_msgs::Vector3& vel);

        void loiterControl(geometry_msgs::Vector3& vel);

        void miniSearchControl(geometry_msgs::Vector3& vel);

        void hoverSearchControl(geometry_msgs::Vector3& vel);

        void tag_foundCallback(const std_msgs::Bool::ConstPtr& msg);
        void tag_abs_xCallback(const std_msgs::Float64::ConstPtr& msg);
        void tag_abs_yCallback(const std_msgs::Float64::ConstPtr& msg);
        void tag_abs_zCallback(const std_msgs::Float64::ConstPtr& msg);
        void tagAltCallback(const std_msgs::Float64::ConstPtr& msg);

        void e_homeCallback(const std_msgs::Float64::ConstPtr& msg);
        void n_homeCallback(const std_msgs::Float64::ConstPtr& msg);

};


ControlNode::ControlNode(float thetaLine, float xLine, float yLine, float vDes) :
_thetaLine(thetaLine), _xLine(xLine), _yLine(yLine), _vDes(vDes)
{

        // subscribe to the desired topics
        _state_sub = _nh.subscribe<mavros_msgs::State>("mavros/state", 1, &ControlNode::stateCallback, this);
        _local_pos_sub = _nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, &ControlNode::localPosCallback, this);
        _local_twist_sub = _nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local", 1, &ControlNode::localTwistCallback, this);
        _sensor_meas_sub =_nh.subscribe<aa241x_mission::SensorMeasurement>("measurement", 10, &ControlNode::sensorMeasCallback, this);
        _droneState_sub = _nh.subscribe<std_msgs::String>("drone_state", 10, &ControlNode::droneStateCallback, this);
        _beaconState_sub =_nh.subscribe<aa241x_mission::SensorMeasurement>("beacon_state", 10, &ControlNode::beaconStateCallback, this);
        _flight_alt_sub = _nh.subscribe<std_msgs::Float64>("flight_alt", 10, &ControlNode::flightAltCallback, this);
        _mission_state_sub = _nh.subscribe<aa241x_mission::MissionState>("mission_state", 10, &ControlNode::missionStateCallback, this);
        _target_v_sub = _nh.subscribe<std_msgs::Float64>("speed", 10, &ControlNode::targetVCallback, this);
        _xLoiter_sub = _nh.subscribe<std_msgs::Float64>("xLoiter", 10, &ControlNode::xLoiterCallback, this);
        _yLoiter_sub = _nh.subscribe<std_msgs::Float64>("yLoiter", 10, &ControlNode::yLoiterCallback, this);
        _gps_position_sub = _nh.subscribe<geometry_msgs::PoseStamped>("geodetic_based_lake_lag_pose", 10, &ControlNode::gpsPositionCallback, this);
        _useGPS_sub = _nh.subscribe<std_msgs::Bool>("useGPS", 10, &ControlNode::useGPSCallback, this);

        // April Tag Things:
        _tag_found_sub = _nh.subscribe<std_msgs::Bool>("tagFound", 10, &ControlNode::tag_foundCallback, this);
        _tag_abs_x_sub = _nh.subscribe<std_msgs::Float64>("tag_abs_x", 10, &ControlNode::tag_abs_xCallback, this);
        _tag_abs_y_sub = _nh.subscribe<std_msgs::Float64>("tag_abs_y", 10, &ControlNode::tag_abs_yCallback, this);
        _tag_abs_z_sub = _nh.subscribe<std_msgs::Float64>("tag_abs_z", 10, &ControlNode::tag_abs_zCallback, this);
        _e_home_sub = _nh.subscribe<std_msgs::Float64>("e_home", 10, &ControlNode::e_homeCallback, this);
        _n_home_sub = _nh.subscribe<std_msgs::Float64>("n_home", 10, &ControlNode::n_homeCallback, this);
        _tag_Alt_sub = _nh.subscribe<std_msgs::Float64>("tag_Alt", 10, &ControlNode::tagAltCallback, this);

        // advertise the published detailed

        // publish a PositionTarget to the `/mavros/setpoint_raw/local` topic which
        // mavros subscribes to in order to send commands to the pixhawk
        _cmd_pub = _nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 1);

        // Publisher for the drone state (TODO: MOVE TO STATE MACHINE NODE)
        //_droneState_pub = _nh.advertise<std_msgs::String>("drone_state", 10);

        // Publish data used in control
        _dPerp_pub = _nh.advertise<std_msgs::Float64>("dPerp", 10);
        _dAlongLine_pub = _nh.advertise<std_msgs::Float64>("dAlongLine", 10);
        _n_cycles_pub = _nh.advertise<std_msgs::Int64>("n_cycles", 10);
        _currentYaw_pub = _nh.advertise<std_msgs::Float64>("currentYaw", 10);
        _xc_pub = _nh.advertise<std_msgs::Float64>("xc", 10);
        _yc_pub = _nh.advertise<std_msgs::Float64>("yc", 10);
        _zc_pub = _nh.advertise<std_msgs::Float64>("zc", 10);
        _z0_pub = _nh.advertise<std_msgs::Float64>("z0", 10);
        _yaw_pub = _nh.advertise<std_msgs::Float64>("yaw", 10);
        _pitch_pub = _nh.advertise<std_msgs::Float64>("pitch", 10);
        _roll_pub = _nh.advertise<std_msgs::Float64>("roll", 10);

        // service
        _landing_loc_client = _nh.serviceClient<aa241x_mission::RequestLandingPosition>("lake_lag_landing_loc");

}

void ControlNode::useGPSCallback(const std_msgs::Bool::ConstPtr& msg) {
    _useGPS = msg->data;
}

void ControlNode::gpsPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    _gps_position_msg = *msg;
    _xGPS = _gps_position_msg.pose.position.x;
    _yGPS = _gps_position_msg.pose.position.y;
    _zGPS = _gps_position_msg.pose.position.z;

}

void ControlNode::xLoiterCallback(const std_msgs::Float64::ConstPtr& msg) {
    _xLoiter = msg->data;
}

void ControlNode::yLoiterCallback(const std_msgs::Float64::ConstPtr& msg) {
    _yLoiter = msg->data;
}

void ControlNode::stateCallback(const mavros_msgs::State::ConstPtr& msg) {
        // save the state locally to be used in the main loop
        _current_state = *msg;
}

void ControlNode::localPosCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        // save the current local position locally to be used in the main loop
        // TODO: account for offset to convert from PX4 coordinate to lake lag frame
        _current_local_pos = *msg;

        // Add the offsets to switch to lake lag (centered) frame
        _current_local_pos.pose.position.x += _e_offset;
        _current_local_pos.pose.position.y += _n_offset;
        _current_local_pos.pose.position.z += _u_offset;

        // Get the position
        _xc = _current_local_pos.pose.position.x; // N
        _yc = _current_local_pos.pose.position.y; // E
        _zc = _current_local_pos.pose.position.z; // U

        // Publish the centered position vector
        _xc_msg.data = _xc;
        _yc_msg.data = _yc;
        _zc_msg.data = _zc;
        _xc_pub.publish(_xc_msg);
        _yc_pub.publish(_yc_msg);
        _zc_pub.publish(_zc_msg);

        // Get the orientation
        float qx = _current_local_pos.pose.orientation.x;
        float qy = _current_local_pos.pose.orientation.y;
        float qz = _current_local_pos.pose.orientation.z;
        float qw = _current_local_pos.pose.orientation.w;

        // Map quaternions to Euler angltes
        float orient[4];
        orient[0] = qw;
        orient[1] = qx;
        orient[2] = qy;
        orient[3] = qz;
        toEulerAngle(orient);
}

void ControlNode::localTwistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
        // save the current local position locally to be used in the main loop
        // TODO: account for offset to convert from PX4 coordinate to lake lag frame
        _current_local_twist = *msg;

        // Get the velocity
        _vxc = _current_local_twist.twist.linear.x; // N
        _vyc = _current_local_twist.twist.linear.y; // E
        _vzc = _current_local_twist.twist.linear.z; // U

}

void ControlNode::sensorMeasCallback(const aa241x_mission::SensorMeasurement::ConstPtr& msg) {
        // TODO: use the information from the measurement as desired
        // NOTE: this callback is for an example of how to setup a callback, you may
        // want to move this information to a mission handling node
        _id = msg->id;
        _n = msg->n;
        _e = msg->e;
}

void ControlNode::missionStateCallback(const aa241x_mission::MissionState::ConstPtr& msg) {
        // save the offset information
        _e_offset = msg->e_offset;
        _n_offset = msg->n_offset;
        _u_offset = msg->u_offset;
}

void ControlNode::droneStateCallback(const std_msgs::String::ConstPtr& msg) {
        // save the state locally to be used in the main loop
        _STATE = msg->data;
}

void ControlNode::beaconStateCallback(const aa241x_mission::SensorMeasurement::ConstPtr& msg) {
        // save the offset information
        _id = msg->id;
        _n = msg->n;
        _e = msg->e;
}

void ControlNode::flightAltCallback(const std_msgs::Float64::ConstPtr& msg) {
        _flight_alt = msg->data;
}

void ControlNode::targetVCallback(const std_msgs::Float64::ConstPtr& msg) {
        _vyMax = msg->data;
        _vxMax = msg->data;
}


void ControlNode::waitForFCUConnection() {
        // wait for FCU connection by just spinning the callback until connected
        ros::Rate rate(5.0);
        while (ros::ok() && _current_state.connected) {
                ros::spinOnce();
                rate.sleep();
        }
}

// Records if tag has been found
void ControlNode::tag_foundCallback(const std_msgs::Bool::ConstPtr& msg) {
    _tag_found = msg->data;
}

void ControlNode::tag_abs_xCallback(const std_msgs::Float64::ConstPtr& msg){
    _tag_abs_x = msg->data;
}

void ControlNode::tag_abs_yCallback(const std_msgs::Float64::ConstPtr& msg){
    _tag_abs_y = msg->data;
}

void ControlNode::tag_abs_zCallback(const std_msgs::Float64::ConstPtr& msg){
    _tag_abs_z = msg->data;
}
void ControlNode::e_homeCallback(const std_msgs::Float64::ConstPtr& msg){
    _e_home = msg->data;
}
void ControlNode::n_homeCallback(const std_msgs::Float64::ConstPtr& msg){
    _n_home = msg->data;
}
void ControlNode::tagAltCallback(const std_msgs::Float64::ConstPtr& msg){
    _tag_Alt = msg->data;
}


// Computes euler angles from quaternions. Sequence is yaw, pitch, roll
void ControlNode::toEulerAngle(const float q[4])
{

    float qw = q[0];
    float qx = q[1];
    float qy = q[2];
    float qz = q[3];

    // Roll:
    double sinr_cosp = +2.0 * (qw * qx + qy * qz);
    double cosr_cosp = +1.0 - 2.0 * (qx * qx + qy * qy);
    _roll = atan2(sinr_cosp, cosr_cosp);

    // Pitch:
    double sinp = +2.0 * (qw * qy - qz * qx);
    if (fabs(sinp) >=1)
        _pitch = copysign(M_PI / 2, sinp); //use 90 degrees if out of range
    else
        _pitch = asin(sinp);

    // Yaw:
    double siny_cosp = +2.0 * (qw * qz + qx * qy);
    double cosy_cosp = +1.0 - 2.0 * (qy * qy + qz * qz);
    _yaw = atan2(siny_cosp, cosy_cosp);

    // Publish Roll, Pitch and Yaw:
    _yaw_msg.data = _yaw;
    _yaw_pub.publish(_yaw_msg);
    _pitch_msg.data = _pitch;
    _pitch_pub.publish(_pitch_msg);
    _roll_msg.data = _roll;
    _roll_pub.publish(_roll_msg);

}

// Gets the sign of a float (can't believe c++ doesn't have this...)
int sign(float x) {

    if (x >= 0) {
        return 1;
    }
    else {
        return -1;
    }
}

void ControlNode::saturateVelocities(geometry_msgs::Vector3 &vel) {

    // Saturate velocities
    if (abs(vel.x) > _vxMax) {
        vel.x = sign(vel.x) * _vxMax; // saturate vx
    }
    if (abs(vel.y) > _vyMax) {
        vel.y = sign(vel.y) * _vyMax; // saturate vy
    }
    if (abs(vel.z) > _vzMax) {
        vel.z = sign(vel.z) * _vzMax; // saturate vz
    }

}

void ControlNode::yawControl(mavros_msgs::PositionTarget& cmd) {

    float k_yaw = 0.5;

    // No yaw if we aren't moving very fast
    float vTotal = sqrt(pow(_vyc,2) + pow(_vxc,2));
    if( vTotal < 1.0) {
        cmd.yaw_rate = 0;
    }
    else {

        // Set desired yaw angle based on velocity in x-y N-E plane
        float yawDes = atan2(_vyc, _vxc);
        float mappedYaw;
        if (yawDes >= 0) {
            yawDes = atan2(_vyc, _vxc);
        }
        else {
            yawDes = 2 * M_PI + atan2(_vyc, _vxc); // map atan2 to 0 -> 360 degree angle
        }
        if (_yaw >= 0) { // yaw measurement is positive already
            mappedYaw = _yaw;
        }
        else { // yaw measurement is negative
            mappedYaw = 2 * M_PI + _yaw;
        }

        if (abs(mappedYaw-yawDes) <= M_PI){
            cmd.yaw_rate = -k_yaw * (mappedYaw-yawDes);
        }
        else {
            cmd.yaw_rate = sign(mappedYaw - yawDes) * k_yaw * (2*M_PI - abs(mappedYaw-yawDes));
        }

    }

    // Saturate the yaw rate
    if(abs(cmd.yaw_rate) > _yawRate_max) {
        cmd.yaw_rate = sign(cmd.yaw_rate) * _yawRate_max;
    }

}

void ControlNode::takeoffControl(geometry_msgs::Vector3& vel) {
    // Set gains for takeoff
    float kpz = 1.0;

    // Jump at max takeoff speed
    if (_zc <= (2.5+_u_offset)){ // need to account for _u_offset here..
        // Commnad velocities to control position
        vel.x = 0.0; // Don't translate laterally
        vel.y = 0.0; // Don't translate laterally
        vel.z = -kpz * (_zc - (_flight_alt));

        if (abs(vel.z) > _vzTakeoff) {
            vel.z = sign(vel.z) * _vzTakeoff; // saturate with takeoff speed
        }

    }
    else{
        // Commnad velocities to control position
        vel.x = 0.0; // Don't translate laterally
        vel.y = 0.0; // Don't translate laterally
        vel.z = -kpz * (_zc - (_flight_alt));

        // Saturate velocities
        saturateVelocities(vel);
    }

}

void ControlNode::perimeterSearchControl(geometry_msgs::Vector3& vel) {

    // Diameter of ground below drone which is realized
    float diameter_search = (5.0/7.0)*(_flight_alt)+28.57;
    float radius_search = diameter_search/2.0;

    float shifted_angle = _angle + 180;

    // Perform sweep of circle outer perimeter:
    float xL = (_radius-radius_search)*cos(shifted_angle*M_PI/180.0);
    float yL = (_radius-radius_search)*sin(shifted_angle*M_PI/180.0);

    // Distance between point and current location
    float xpt = xL-_xc;
    float ypt = yL-_yc;
    float pt_dist = sqrt(pow(xpt,2)+pow(ypt,2));

    // Define gains for point to point travel
    float kpx = 0.8;
    float kpy = 0.8;
    float kpz = 1.0;

    // Proportional position control
    vel.x = -kpx * (_xc - xL);
    vel.y = -kpy * (_yc - yL);
    vel.z = -kpz * (_zc - _flight_alt);

    // Saturate velocities
    saturateVelocities(vel);

    // If it reaches the objective point, switch points to acquire next point trajectory
    if (pt_dist <= 15.0){
        _angle = _angle+5;
    }
    if (_angle == 375){ // 360 + entry_angle){ // Incorporates angle at which the drone enters the ring
        _angle = 15;
        _radius = _radius-diameter_search*0.8;
        _cycle = _cycle + 1;
        _n_cycles_msg.data = _cycle;
        _n_cycles_pub.publish(_n_cycles_msg);
    }

}

void ControlNode::miniSearchControl(geometry_msgs::Vector3& vel) {

    // Decide which position estimates to use
    float xEst;
    float yEst;
    float zEst;
    if(_useGPS) {
        xEst = _xGPS;
        yEst = _yGPS;
        zEst = _zGPS;
    }
    else {
        xEst = _xc;
        yEst = _yc;
        zEst = _zc;
    }

    // Perform sweep of circle outer perimeter:
    float xL = (radius_Alt_search)*cos(_angle*M_PI/180.0) + _landing_e;
    float yL = (radius_Alt_search)*sin(_angle*M_PI/180.0) + _landing_n;

    // Distance between point and current location
    float xpt = xL-xEst;
    float ypt = yL-yEst;
    float pt_dist = sqrt(pow(xpt,2)+pow(ypt,2));

    // Define gains for point to point travel
    float kpx = 1.0;
    float kpy = 1.0;
    float kpz = 1.0;

    // Proportional position control
    vel.x = -kpx * (xEst - xL);
    vel.y = -kpy * (yEst - yL);
    vel.z = -kpz * (zEst - _tag_Alt);

    // Saturate velocities
    saturateVelocities(vel);

    // If it reaches the objective point, switch points to acquire next point trajectory
    if (pt_dist <= 0.5){
        _angle = _angle+5;
    }
    if (_angle >= 360){ // 360 + entry_angle){ // Incorporates angle at which the drone enters the ring
        _angle = 0;
        radius_Alt_search = radius_Alt_search + 1.0;
    }

}

// Loiters and looks for apriltags
void ControlNode::hoverSearchControl(geometry_msgs::Vector3& vel) {

    // Decide which position estimates to use
    float xEst;
    float yEst;
    float zEst;
    if(_useGPS) {
        xEst = _xGPS;
        yEst = _yGPS;
        zEst = _zGPS;
    }
    else {
        xEst = _xc;
        yEst = _yc;
        zEst = _zc;
    }

    float kp = 1.0;
    float kpz = 1.0;

    // Commnad velocities to control position
    vel.x = -kp * (xEst - _landing_e); // Hold position landing position
    vel.y = -kp * (yEst - _landing_n); // Don't translate laterally
    vel.z  = -kpz * (zEst - (_tag_Alt)); // Hold altitude

    // Saturate velocities
    saturateVelocities(vel);

}

void ControlNode::lineControl(geometry_msgs::Vector3& vel) {
    // Compute tangential distance to line
    float dperp = sin(_thetaLine)*(_yLine-_yc) + cos(_thetaLine)*(_xLine-_xc);
    float dAlongLine = -cos(_thetaLine)*(_y0-_yc) + sin(_thetaLine)*(_x0-_xc); // Position projection along line from drone starting point to current position

    // Define line distance gains
    float dp = 5;
    float kp = _vDes/dp;
    float kpz = 1;

    // Command speed towards line and along line
    float up = kp*dperp; // line y
    float vp = _vDes; // line x

    // Map to global with velocity saturation:
    vel.x = up*cos(_thetaLine) + vp*sin(_thetaLine);
    vel.y = up*sin(_thetaLine) - vp*cos(_thetaLine);
    vel.z = -kpz * (_zc - _flight_alt);

    // Saturate velocities
    saturateVelocities(vel);

    // Assign and publish the control data
    _dPerp_msg.data = dperp;
    _dAlongLine_msg.data = dAlongLine;
    _dPerp_pub.publish(_dPerp_msg);
    _dAlongLine_pub.publish(_dAlongLine_msg);
}

void ControlNode::goHomeControl(geometry_msgs::Vector3& vel) {

    // Decide which position estimates to use
    float xEst;
    float yEst;
    float zEst;
    if(_useGPS) {
        xEst = _xGPS;
        yEst = _yGPS;
        zEst = _zGPS;
    }
    else {
        xEst = _xc;
        yEst = _yc;
        zEst = _zc;
    }

    // Define gains for going home
    float kpx = 1.0;
    float kpy = 1.0;
    float kpz = 1.0;

    // Define controls on position
    vel.x = -kpx * (xEst - _landing_e);
    vel.y = -kpy * (yEst - _landing_n);
    vel.z = -kpz * (zEst - _flight_alt);

    // Saturate velocities
    saturateVelocities(vel);
}

void ControlNode::goHomeLandControl(geometry_msgs::Vector3& vel) {

    // Decide which position estimates to use
    float xEst;
    float yEst;
    float zEst;
    if(_useGPS) {
        xEst = _xGPS;
        yEst = _yGPS;
        zEst = _zGPS;
    }
    else {
        xEst = _xc;
        yEst = _yc;
        zEst = _zc;
    }

    // Define gains for going home
    float kpx = 1.0;
    float kpy = 1.0;
    float kpz = 1.0;

    // Define controls on position
    vel.x = -kpx * (xEst - _landing_e);
    vel.y = -kpy * (yEst - _landing_n);
    vel.z = -kpz * (zEst - _tag_Alt);

    // Saturate velocities
    saturateVelocities(vel);

    if (abs(xEst - _landing_e) <= 0.2 && abs(yEst - _landing_n) <= 0.2){
        vel.z = -kpz * (zEst-0.0);
    }

    // Slowing down the z-direction velocity for the last 5 meter drop
    if (abs(vel.z) > _vzMax/4.0) {
        vel.z = sign(vel.z) * _vzMax/4.0; // saturate vz
    }
}

void ControlNode::landControl(geometry_msgs::Vector3& vel) {

    // Decide which position estimates to use
    float xEst;
    float yEst;
    float zEst;
    if(_useGPS) {
        xEst = _xGPS;
        yEst = _yGPS;
        zEst = _zGPS;
    }
    else {
        xEst = _xc;
        yEst = _yc;
        zEst = _zc;
    }
    float kpz = 1.0;
    float kpx = 0.5;
    float kpy = 0.5;

    // Command velocities to control position
    vel.x = -kpx * (xEst - _tag_abs_x); // Don't translate laterally
    vel.y = -kpx * (yEst - _tag_abs_y); // Don't translate laterally
    vel.z = -kpz * (zEst - 0.0); //-kpz * (_zc - _z0); // Fix this with offsets

    // Saturate velocities
    saturateVelocities(vel);

    // Slowing down the z-direction velocity for the last 5 meter drop
    if (abs(vel.z) > _vzMax/4.0) {
        vel.z = sign(vel.z) * _vzMax/4.0; // saturate vz
    }

}

void ControlNode::dropAltControl(geometry_msgs::Vector3& vel) {

    // Decide which position estimates to use
    float xEst;
    float yEst;
    float zEst;
    if(_useGPS) {
        xEst = _xGPS;
        yEst = _yGPS;
        zEst = _zGPS;
    }
    else {
        xEst = _xc;
        yEst = _yc;
        zEst = _zc;
    }

    float kp = 1.0;
    float kpz = 1.0;

    // Commnad velocities to control position
    vel.x = -kp * (xEst - _landing_e); // Hold position landing position
    vel.y = -kp * (yEst - _landing_n); // Don't translate laterally
    vel.z  = -kpz * (zEst - (_tag_Alt)); // Drop to altitude of 5m //= -kpz * (_zc - (_flight_alt - 5.0)); // Drop 5 meters

    if (abs(vel.z) > _vzMax/2.0) {
        vel.z = sign(vel.z) * _vzMax/2.0; // saturate vz
    }

    // Saturate velocities
    saturateVelocities(vel);
}

void ControlNode::navToLandControl(geometry_msgs::Vector3& vel) {

    // Decide which position estimates to use
    float xEst;
    float yEst;
    float zEst;
    if(_useGPS) {
        xEst = _xGPS;
        yEst = _yGPS;
        zEst = _zGPS;
    }
    else {
        xEst = _xc;
        yEst = _yc;
        zEst = _zc;
    }
    float kpx = 1.0;
    float kpy = 1.0;
    float kpz = 1.0;

    // Command velocities to control position
    vel.x = -kpx * (xEst - _tag_abs_x);  // Move to allign the drone with camera x-direction
    vel.y = -kpy * (yEst - _tag_abs_y);  // Move to allign the drone with camera u-direction
    vel.z = -kpz * (zEst - (_tag_Alt) );

    // Saturate velocities
    saturateVelocities(vel);
}

void ControlNode::loiterControl(geometry_msgs::Vector3& vel) {
    float kp = 0.1;
    float kpz = 1.0;

    // Commnad velocities to control position
    vel.x = -kp * (_xc - _xLoiter); // Hold loiter position
    vel.y = -kp * (_yc - _yLoiter); // Hold loiter position
    vel.z = -kpz * (_zc - _flight_alt); // Don't translate vertically

    // Saturate velocities
    saturateVelocities(vel);
}

int ControlNode::run() {

        // wait for the controller connection
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

        // set up the general command parameters
        // NOTE: these will be true for all commands send
        mavros_msgs::PositionTarget cmd;
        cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;	// use the local frame

        // Always commanding velocities
        cmd.type_mask = (mavros_msgs::PositionTarget::IGNORE_PX |
                         mavros_msgs::PositionTarget::IGNORE_PY |
                         mavros_msgs::PositionTarget::IGNORE_PZ |
                         mavros_msgs::PositionTarget::IGNORE_AFX |
                         mavros_msgs::PositionTarget::IGNORE_AFY |
                         mavros_msgs::PositionTarget::IGNORE_AFZ |
                         mavros_msgs::PositionTarget::IGNORE_YAW);

        // the yaw information
        // NOTE: just keeping the heading north
        cmd.yaw_rate = 0;

        // the velocity information for the command
        // NOTE: this is defined in ENU
        geometry_msgs::Vector3 vel;
        vel.x = 0;	// E
        vel.y = 0;	// N
        vel.z = 0;	// U

        // set the loop rate in [Hz]
        // NOTE: must be faster than 2Hz
        ros::Rate rate(10.0);

        // main loops
        while (ros::ok()) {

                //Commanded velocities
                float vx;
                float vy;

                // if not in offboard mode, just keep waiting until we are and if not
                // enabled, then keep waiting
                //
                // NOTE: need to be streaming setpoints in order for offboard to be
                // allowed, hence the publishing of an empty command
                if (_current_state.mode != "OFFBOARD") {

                        // Don't move!
                        vel.x = 0;
                        vel.y = 0;
                        vel.z = 0;

                        // Define home position as position when offboard mode is initiated
                        _x0 = _xc;
                        _y0 = _yc;
                        _z0 = _zc; // fix with offsets

                        // timestamp the message and send it
                        cmd.header.stamp = ros::Time::now();
                        cmd.velocity = vel;
                        cmd.yaw_rate = 0;
                        _cmd_pub.publish(cmd);

                        // run the ros components
                        ros::spinOnce();
                        rate.sleep();
                        continue;

                }

                // Publish home position
                _z0_msg.data = _z0;
                _z0_pub.publish(_z0_msg);

                // at this point the pixhawk is in offboard control, so we can now fly
                // the drone as desired

                // Choose the control mode from the state machine
                if (_STATE == TAKEOFF) {
                    takeoffControl(vel);
                }
                else if (_STATE == Perimeter_Search){
                    perimeterSearchControl(vel);
                }
                else if(_STATE == LINE) {
                    lineControl(vel);
                }
                else if(_STATE == GOHOME) {
                    goHomeControl(vel);
                }
                else if(_STATE == GOHOME_LAND) {
                    goHomeLandControl(vel);
                }
                else if( _STATE == LAND){
                    landControl(vel);
                }
                else if( _STATE == DROP_ALT){
                    dropAltControl(vel);
                }
                else if (_STATE == Navigate_to_land){
                    navToLandControl(vel);
                }
                else if( _STATE == LOITER){
                    loiterControl(vel);
                }
                else if(_STATE == CAMERA_TEST) {
                    vel.x = 0;
                    vel.y = 0;
                    vel.z = 0;
                }
                else if (_STATE == MINISEARCH) {
                    miniSearchControl(vel);
                }
                else if (_STATE == Hover_Search) {
                    hoverSearchControl(vel);
                }

                // Assign velocity control
                cmd.header.stamp = ros::Time::now();
                cmd.velocity = vel;

                // Control the yaw to be in direction of velocity
                yawControl(cmd);

                // Update and publish yaw message
                _currentYaw_msg.data = _yaw * 180.0 / M_PI;
                _currentYaw_pub.publish(_currentYaw_msg);

                // Publish the control commands
                _cmd_pub.publish(cmd);

                // remember need to always call spin once for the callbacks to trigger
                ros::spinOnce();
                rate.sleep();
        }
        // return  exit code
        return EXIT_SUCCESS;
}


int main(int argc, char **argv) {

        // initialize th enode
        ros::init(argc, argv, "control_node");

        // get parameters from the launch file which define some mission
        // settings
        ros::NodeHandle private_nh("~");

        // Line to follow:
//        float thetaLine_desired = 225.0*M_PI/180.0;
        float thetaLine_desired = 0;
        float x0 = 50.0;
        float y0 = 50.0;


        // Desired forward speed
        float vDes = 2.0f;

        // create the node
        ControlNode node(thetaLine_desired, x0, y0, vDes);

        // run the node
        return node.run();
}
