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
#include <vector>

// Define states
const std::string TAKEOFF = "TAKEOFF";
const std::string LINE = "LINE";
const std::string GOHOME = "GOHOME";
const std::string LAND = "LAND";
const std::string LOITER = "LOITER";
const std::string DROP_ALT = "DROP_ALT";
const std::string Perimeter_Search = "Perimeter_Search";

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
        geometry_msgs::TwistStamped _current_local_twist;

        // Beacon search initializations
        std::vector<int> _id;
        std::vector<float> _n;
        std::vector<float> _e;

        // offset information
        float _e_offset = 0;
        float _n_offset = 0;
        float _u_offset = 0;
        float _current_lat = 0.0f;
        float _current_lon = 0.0f;

        // actuator saturation
        float _vxMax;
        float _vyMax;
        float _vzMax = 1.0f;
        float _vzTakeoff = 3.0f;

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

        // publishers
        ros::Publisher _cmd_pub;
        ros::Publisher _dPerp_pub;
        ros::Publisher _dAlongLine_pub;
        ros::Publisher _currentYaw_pub;
        ros::Publisher _n_cycles_pub;

        //service
        ros::ServiceClient _landing_loc_client;

        // callbacks

        /**
         * callback for the current state of the pixhawk.
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
         * @param msg mavros state message
         */
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
        //cmd.yaw_rate = -k_yaw * (yaw - yawDes);

        // TODO: add callbacks here

        // helper functions

        /**
         * wait for the connection to the Pixhawk to be established.
         */
        void waitForFCUConnection();


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

        // service
        _landing_loc_client = _nh.serviceClient<aa241x_mission::RequestLandingPosition>("lake_lag_landing_loc");

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
        _current_local_pos.pose.position.z -= _u_offset;

        // Get the position
        _xc = _current_local_pos.pose.position.x; // N
        _yc = _current_local_pos.pose.position.y; // E
        _zc = _current_local_pos.pose.position.z; // U

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


int ControlNode::run() {
        int angle = 50;
        int count = 0;
        int cycle = 0;
        float outer_radius = 160.0;
        float radius = outer_radius; //160

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

        // the yaw information
        // NOTE: just keeping the heading north
        cmd.yaw = 0;

        // the position information for the command
        // NOTE: this is defined in ENU
        geometry_msgs::Point pos;
        pos.x = 0;	// E
        pos.y = 0;	// N
        pos.z = 0;	// U

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

                float x1;
                float y1;

                //Pt Trajectory:
                float xPosVector[5];
                float yPosVector[5];

                //Commanded velocities
                float vx;
                float vy;
                float vz;

                float dPerpInit;

                // if not in offboard mode, just keep waiting until we are and if not
                // enabled, then keep waiting
                //
                // NOTE: need to be streaming setpoints in order for offboard to be
                // allowed, hence the publishing of an empty command
                if (_current_state.mode != "OFFBOARD") {

                        // send command to stay in the same position                if(abs(_zc - _flight_alt) < 0.1) {

                        // TODO: if doing position command in the lake lag frame, make
                        // sure these values match the initial position of the drone!
                        pos.x = 0;
                        pos.y = 0;
                        pos.z = 0;

                        // Define home position as position when offboard mode is initiated
                        _x0 = _xc;
                        _y0 = _yc;
                        _z0 = _zc;

                        // timestamp the message and send it
                        cmd.header.stamp = ros::Time::now();
                        cmd.position = pos;
                        cmd.velocity = vel;
                        _cmd_pub.publish(cmd);

                        // run the ros components
                        ros::spinOnce();
                        rate.sleep();
                        continue;

                }

                // at this point the pixhawk is in offboard control, so we can now fly
                // the drone as desired

                if (_STATE == TAKEOFF) {

                    cmd.type_mask = (mavros_msgs::PositionTarget::IGNORE_PX |
                                     mavros_msgs::PositionTarget::IGNORE_PY |
                                     mavros_msgs::PositionTarget::IGNORE_PZ |
                                     mavros_msgs::PositionTarget::IGNORE_AFX |
                                     mavros_msgs::PositionTarget::IGNORE_AFY |
                                     mavros_msgs::PositionTarget::IGNORE_AFZ |
                                     mavros_msgs::PositionTarget::IGNORE_YAW_RATE);

                    float kpz = 1.0;

                    if (_zc <= (2.5-_u_offset)){ // need to account for _u_offset here..
                        // Commnad velocities to control position
                        vel.x = 0.0; // Don't translate laterally
                        vel.y = 0.0; // Don't translate laterally
                        vel.z = -kpz * (_zc - _flight_alt);

                        // Saturate velocities
                        if (abs(vel.x) > _vxMax) {
                            vel.x = sign(vel.x) * _vxMax; // saturate vx
                        }
                        if (abs(vel.y) > _vyMax) {
                            vel.y = sign(vel.y) * _vyMax; // saturate vy
                        }
                        if (abs(vel.z) > _vzTakeoff) {
                            vel.z = sign(vel.z) * _vzTakeoff; // saturate vz
                        }

                    }
                    else{
                        // Commnad velocities to control position
                        vel.x = 0.0; // Don't translate laterally
                        vel.y = 0.0; // Don't translate laterally
                        vel.z = -kpz * (_zc - _flight_alt);

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


                    // publish the command
                    cmd.header.stamp = ros::Time::now();
                    cmd.position = pos;
                    cmd.velocity = vel;
                    cmd.yaw = _yaw;
                }

                else if (_STATE == Perimeter_Search){
                    cmd.type_mask = (mavros_msgs::PositionTarget::IGNORE_PX |
                                     mavros_msgs::PositionTarget::IGNORE_PY |
                                     mavros_msgs::PositionTarget::IGNORE_PZ |
                                     mavros_msgs::PositionTarget::IGNORE_AFX |
                                     mavros_msgs::PositionTarget::IGNORE_AFY |
                                     mavros_msgs::PositionTarget::IGNORE_AFZ |
                                     mavros_msgs::PositionTarget::IGNORE_YAW);
                    // Generalize this later to be based off longitude and lattitude, not distance from current position

                    float diameter_search = (5.0/7.0)*(_zc + _u_offset)+28.57; // Diameter of ground below drone which is realized
                    //double lake_ctr_lat = 37.4224444;		// [deg]
                    //double lake_ctr_lon = -122.1760917;	// [deg]                    //cmd.yaw_rate = -k_yaw * (yaw - yawDes);

                    // Given these lat and long coordinates, the center of the circle is found relative to the drone
                    float center_x = -_e_offset; //_current_lon - lake_ctr_lon; //-160;
                    float center_y = -_n_offset; //_current_lat - lake_ctr_lat; //-70;

                    float radius_search = diameter_search/2.0;

                    // Perform sweep of circle outer perimeter:
                    float xL = (radius-radius_search)*cos(angle*M_PI/180.0);// + center_x;
                    float yL = (radius-radius_search)*sin(angle*M_PI/180.0);// + center_y;

                    // Distance between point and current location
                    float xpt = xL-_xc;
                    float ypt = yL-_yc;
                    float pt_dist = sqrt(pow(xpt,2)+pow(ypt,2));

                    // Define gains for point to point travel
                    float kpx = 0.8;
                    float kpy = 0.8;
                    float kpz = 1.0;
                    float k_yaw = 0.5;

                    // Proportional position control
                    vel.x = -kpx * (_xc - xL);
                    vel.y = -kpy * (_yc - yL);
                    vel.z = -kpz * (_zc - _flight_alt);


                    // Saturation of velocities


//                    float V_max_pt2pt = sqrt(pow(_vxMax,2.0)+pow(_vyMax,2.0));

//                    if (abs(xpt)>abs(ypt)){
//                        // Larger distance in x-direction to close; set vx to its max then determine vy to keep constant Vmaxpt2pt
//                        if (abs(vel.x) > _vxMax) {
//                            vel.x = sign(vel.x) * _vxMax; // saturate vx
//                        }
//                        vel.y = sign(vel.y) * (sqrt(pow(V_max_pt2pt,2.0) - pow(vel.x,2.0)));
//                    }
//                    else {
//                        if (abs(vel.y) > _vyMax) {
//                            vel.y = sign(vel.y) * _vyMax; // saturate vx
//                        }
//                        vel.x = sign(vel.x) * (sqrt(pow(V_max_pt2pt,2.0) - pow(vel.y,2.0)));
//                    }


                    if (abs(vel.x) > _vxMax) {
                        vel.x = sign(vel.x) * _vxMax; // saturate vx
                    }
                    if (abs(vel.y) > _vyMax) {
                        vel.y = sign(vel.y) * _vyMax; // saturate vy
                    }

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
                    //cmd.yaw_rate = -k_yaw * (mappedYaw - yawDes);

                    if (abs(mappedYaw-yawDes) <= M_PI){
                        cmd.yaw_rate = -k_yaw * (mappedYaw-yawDes);
                    }
                    else {
                        cmd.yaw_rate = k_yaw * (2*M_PI - (mappedYaw-yawDes));
                    }

                    // publish the command
                    cmd.header.stamp = ros::Time::now();
                    cmd.position = pos;
                    cmd.velocity = vel;

                    // If it reaches the objective point, switch points to acquire next point trajectory
                    if (pt_dist <= 15.0){
                        angle = angle+5;
                    }
                    if (angle == 360){ // 360 + entry_angle){ // Incorporates angle at which the drone enters the ring
                        angle = 0;
                        radius = radius-diameter_search;
                        cycle = cycle + 1;
                        _n_cycles_msg.data = cycle;
                        _n_cycles_pub.publish(_n_cycles_msg);
                    }
                }
                else if(_STATE == LINE) {

                    cmd.type_mask = 2499;  // mask for Vx Vy and Pz control

                    // Compute tangential distance to line
                    float dperp = sin(_thetaLine)*(_yLine-_yc) + cos(_thetaLine)*(_xLine-_xc);
                    float dAlongLine = -cos(_thetaLine)*(_y0-_yc) + sin(_thetaLine)*(_x0-_xc); // Position projection along line from drone starting point to current position

                    // Define line distance gains
                    float dp = 5;
                    float kp = _vDes/dp;

                    // Command speed towards line and along line
                    float up = kp*dperp; // line y
                    float vp = _vDes; // line x

                    // Map to global with velocity saturation:
                    vx = up*cos(_thetaLine) + vp*sin(_thetaLine);
                    vy = up*sin(_thetaLine) - vp*cos(_thetaLine);
                    vel.x = vx;
                    vel.y = vy;
                    if (abs(vx) > _vxMax) {
                        vel.x = sign(vx) * _vxMax; // saturate vx
                    }
                    if (abs(vy) > _vyMax) {
                        vel.y = sign(vy) * _vyMax; // saturate vy
                    }

                    // Set the yaw angle and the position
                    //cmd.yaw = _thetaLine - M_PI/2.0; // yaw in Pixhawk measured CCW from EAST
                    cmd.yaw = atan2(vel.y,vel.x);
                    pos.z = _flight_alt+_u_offset;

                    // Assign the command to the command message
                    cmd.header.stamp = ros::Time::now();
                    cmd.position = pos;
                    cmd.velocity = vel;

                    // Assign and publish the control data
                    _dPerp_msg.data = dperp;
                    _dAlongLine_msg.data = dAlongLine;
                    _dPerp_pub.publish(_dPerp_msg);
                    _dAlongLine_pub.publish(_dAlongLine_msg);

                    if (abs(dAlongLine) >= 260.0){
                        //_STATE = LINE2;
                        //_STATE = GOHOME;
                        x1 = _xc;
                        y1 = _yc;
                        _xLine = _xc;
                        _yLine = _yc;
                        _thetaLine = _thetaLine + 17.0 * M_PI / 32.0;
                     }
                }
                else if(_STATE == GOHOME) {
                    cmd.type_mask = (mavros_msgs::PositionTarget::IGNORE_PX |
                                     mavros_msgs::PositionTarget::IGNORE_PY |
                                     //cmd.yaw_rate = -k_yaw * (yaw - yawDes);
                                     mavros_msgs::PositionTarget::IGNORE_PZ |
                                     mavros_msgs::PositionTarget::IGNORE_AFX |
                                     mavros_msgs::PositionTarget::IGNORE_AFY |
                                     mavros_msgs::PositionTarget::IGNORE_AFZ |
                                     mavros_msgs::PositionTarget::IGNORE_YAW_RATE);

                    // Define gains for going home
                    float kpx = 1.0;
                    float kpy = 1.0;
                    float kpz = 1.0;

                    // Define controls on position
                    vel.x = -kpx * (_xc - _landing_e);
                    vel.y = -kpy * (_yc - _landing_n);
                    vel.z = -kpz * (_zc - _flight_alt);

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

                    // publish the command
                    cmd.header.stamp = ros::Time::now();
                    cmd.position = pos;
                    cmd.velocity = vel;

                    // Compute distance to home
                    float xDistSq = pow((_xc - _landing_e),2);
                    float yDistSq = pow((_yc - _landing_n),2);
                    float distToHome = sqrt(xDistSq + yDistSq);

                    // Check if we should continue to command yaw
                    if( distToHome < 1.0) {
                        cmd.yaw = _yaw;
                    }
                    else {
                        cmd.yaw = atan2(vel.y,vel.x);
                    }


                }
                else if( _STATE == LAND){
                    cmd.type_mask = (mavros_msgs::PositionTarget::IGNORE_PX |
                                     mavros_msgs::PositionTarget::IGNORE_PY |
                                     mavros_msgs::PositionTarget::IGNORE_PZ |
                                     mavros_msgs::PositionTarget::IGNORE_AFX |
                                     mavros_msgs::PositionTarget::IGNORE_AFY |
                                     mavros_msgs::PositionTarget::IGNORE_AFZ |
                                     mavros_msgs::PositionTarget::IGNORE_YAW_RATE);

                    float kpz = 1.0;

                    // Command velocities to control position
                    vel.x = 0.0; // Don't translate laterally
                    vel.y = 0.0; // Don't translate laterally
                    vel.z = -kpz * (_zc - _z0);


                    // Saturate velocities
                    if (abs(vel.x) > _vxMax) {
                        vel.x = sign(vel.x) * _vxMax; // saturate vx
                    }
                    if (abs(vel.y) > _vyMax) {
                        vel.y = sign(vel.y) * _vyMax; // saturate vy
                    }
                    // Slowing down the z-direction velocity for the last 5 meter drop
                    if (abs(vel.z) > _vzMax/4.0) {
                        vel.z = sign(vel.z) * _vzMax/4.0; // saturate vz
                    }

                    // publish the command
                    cmd.header.stamp = ros::Time::now();
                    cmd.position = pos;
                    cmd.velocity = vel;
                    cmd.yaw = _yaw;
               }
                else if( _STATE == DROP_ALT){
                    cmd.type_mask = (mavros_msgs::PositionTarget::IGNORE_PX |
                                     mavros_msgs::PositionTarget::IGNORE_PY |
                                     mavros_msgs::PositionTarget::IGNORE_PZ |
                                     mavros_msgs::PositionTarget::IGNORE_AFX |
                                     mavros_msgs::PositionTarget::IGNORE_AFY |
                                     mavros_msgs::PositionTarget::IGNORE_AFZ |
                                     mavros_msgs::PositionTarget::IGNORE_YAW_RATE);

                    float kpz = 1.0;

                    // Commnad velocities to control position
                    vel.x = 0.0; // Don't translate laterally
                    vel.y = 0.0; // Don't translate laterally
                    vel.z  = -kpz * (_zc - (3.0-_u_offset)); // Drop to altitude of 5m //= -kpz * (_zc - (_flight_alt - 5.0)); // Drop 5 meters


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

                    // publish the command
                    cmd.header.stamp = ros::Time::now();
                    cmd.position = pos;
                    cmd.velocity = vel;
                    cmd.yaw = _yaw;

                    }
                else if( _STATE == LOITER){
                    cmd.type_mask = (mavros_msgs::PositionTarget::IGNORE_PX |
                                     mavros_msgs::PositionTarget::IGNORE_PY |
                                     mavros_msgs::PositionTarget::IGNORE_PZ |
                                     mavros_msgs::PositionTarget::IGNORE_AFX |
                                     mavros_msgs::PositionTarget::IGNORE_AFY |
                                     mavros_msgs::PositionTarget::IGNORE_AFZ |
                                     mavros_msgs::PositionTarget::IGNORE_YAW_RATE);

                    float kpz = 1.0;

                    // Commnad velocities to control position
                    vel.x = 0.0; // Don't translate laterally
                    vel.y = 0.0; // Don't translate laterally
                    vel.z = 0.0; // Don't translate vertically


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

                    // publish the command
                    cmd.header.stamp = ros::Time::now();
                    cmd.position = pos;
                    cmd.velocity = vel;
                    cmd.yaw = _yaw;

                    }
                // Update and publish yaw message
                _currentYaw_msg.data = _yaw * 180.0 / M_PI;
                _currentYaw_pub.publish(_currentYaw_msg);

                // Update state message
                //_droneState_msg.data = _STATE;

                _cmd_pub.publish(cmd);
                //_droneState_pub.publish(_droneState_msg);

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
        // TODO: determine settings

        // Line to follow:
        float thetaLine_desired = 225.0*M_PI/180.0;
        float x0 = 50.0;
        float y0 = 50.0;


        // Desired forward speed
        float vDes = 2.0f;

        // create the node
        ControlNode node(thetaLine_desired, x0, y0, vDes);

        // run the node
        return node.run();
}
