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
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>

#include <aa241x_mission/SensorMeasurement.h>
#include <aa241x_mission/MissionState.h>

#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>
#include <vector>

// Define states
const std::string TAKEOFF = "TAKEOFF";
const std::string LINE = "LINE";
const std::string LINE2 = "LINE2";
const std::string LINE3 = "LINE3";
const std::string GOHOME = "GOHOME";
const std::string LAND = "LAND";
const std::string Pt_Trajectory = "Pt_Trajectory";
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
        float _flight_alt;

        std_msgs::String _droneState_msg;
        std_msgs::Float64 _dPerp_msg;
        std_msgs::Float64 _dAlongLine_msg;
        std_msgs::Int64 _n_cycles_msg;

        // Beacon search initializations
        std::vector<int> _id;
        std::vector<float> _n;
        std::vector<float> _e;

        // offset information
        float _e_offset = 0.0f;
        float _n_offset = 0.0f;
        float _u_offset = 0.0f;

        // actuator saturation
        float _vxMax = 5.0f;
        float _vyMax = 5.0f;
        float _vzMax = 5.0f;

        // subscribers
        ros::Subscriber _state_sub;                 // the current state of the pixhawk
        ros::Subscriber _local_pos_sub;             // local position information
        ros::Subscriber _sensor_meas_sub;           // mission sensor measurement
        ros::Subscriber _mission_state_sub;         // mission state
        ros::Subscriber _droneState_sub;            // the current state of the drone
        ros::Subscriber _beaconState_sub;           // the current state of the beacon information
        ros::Subscriber _flight_alt_sub;            // the targeted flight altitude of the mission
        // TODO: add subscribers here

        // publishers
        ros::Publisher _cmd_pub;
//ros::Publisher _droneState_pub;
        ros::Publisher _dPerp_pub;
        ros::Publisher _dAlongLine_pub;
        ros::Publisher _n_cycles_pub;

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

        void beaconStateCallback(const aa241x_mission::SensorMeasurement::ConstPtr& msg);

        void droneStateCallback(const std_msgs::String::ConstPtr& msg);

        void flightAltCallback(const std_msgs::Float64::ConstPtr& msg);

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
        _sensor_meas_sub =_nh.subscribe<aa241x_mission::SensorMeasurement>("measurement", 10, &ControlNode::sensorMeasCallback, this);
        _droneState_sub = _nh.subscribe<std_msgs::String>("drone_state", 10, &ControlNode::droneStateCallback, this);
        _beaconState_sub =_nh.subscribe<aa241x_mission::SensorMeasurement>("beacon_state", 10, &ControlNode::beaconStateCallback, this);
        _flight_alt_sub = _nh.subscribe<std_msgs::Float64>("flight_alt", 10, &ControlNode::flightAltCallback, this);
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

}

void ControlNode::stateCallback(const mavros_msgs::State::ConstPtr& msg) {
        // save the state locally to be used in the main loop
        _current_state = *msg;
}

void ControlNode::localPosCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        // save the current local position locally to be used in the main loop
        // TODO: account for offset to convert from PX4 coordinate to lake lag frame
        _current_local_pos = *msg;
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


void ControlNode::waitForFCUConnection() {
        // wait for FCU connection by just spinning the callback until connected
        ros::Rate rate(5.0);
        while (ros::ok() && _current_state.connected) {
                ros::spinOnce();
                rate.sleep();
        }
}

// Computes euler angles from quaternions. Sequence is yaw, pitch, roll
static void toEulerAngle(const float q[4], float& roll, float& pitch, float& yaw)
{

    float qw = q[0];
    float qx = q[1];
    float qy = q[2];
    float qz = q[3];

    // Yaw:
    double sinr_cosp = +2.0 * (qw*qw + qy*qz);
    double cosr_cosp = +1.0 - 2.0*(qx*qx + qy*qy);
    yaw = atan2(sinr_cosp, cosr_cosp);

    // Pitch:
    double sinp = +2.0 * (qw*qy - qz*qx);
    if (fabs(sinp) >=1)
        pitch = copysign(M_PI / 2, sinp); //use 90 degrees if out of range
    else
        pitch = asin(sinp);

    //Roll:
    double siny_cosp = +2.0 * (qw*qz + qx*qy);
    double cosy_cosp = +1.0 - 2.0*(qy*qy + qz*qz);
    roll = atan2(siny_cosp,cosy_cosp);

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
        float radius = 150; //160

        // wait for the controller connection
        waitForFCUConnection();
        ROS_INFO("connected to the FCU");

        // set up the general command parameters
        // NOTE: these will be true for all commands send
        mavros_msgs::PositionTarget cmd;
        cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;	// use the local frame

        // configure the type mask to command only position information
        // NOTE: type mask sets the fields to IGNORE
        // TODO: need to add a link to the mask to explain the value

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


        // main loop
        while (ros::ok()) {
                // Get state for control
                float xc = _current_local_pos.pose.position.x;
                float yc = _current_local_pos.pose.position.y;
                float zc = _current_local_pos.pose.position.z;
                float qx = _current_local_pos.pose.orientation.x;
                float qy = _current_local_pos.pose.orientation.y;
                float qz = _current_local_pos.pose.orientation.z;
                float qw = _current_local_pos.pose.orientation.w;


                float orient[4];
                float roll;
                float pitch;
                float yaw;

                float x0;
                float y0;
                float z0;

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

                orient[0] = qw;
                orient[1] = qx;
                orient[2] = qy;
                orient[3] = qz;

                toEulerAngle(orient, roll, pitch, yaw);

                // if not in offboard mode, just keep waiting until we are and if not
                // enabled, then keep waiting
                //
                // NOTE: need to be streaming setpoints in order for offboard to be
                // allowed, hence the publishing of an empty command
                if (_STATE == "NOTOFFBOARD") {

                        // send command to stay in the same position
                        // TODO: if doing position command in the lake lag frame, make
                        // sure these values match the initial position of the drone!
                        pos.x = 0;
                        pos.y = 0;
                        pos.z = 0;

                        // Define home position as position when offboard mode is initiated
                        x0 = xc;
                        y0 = yc;
                        z0 = zc;

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

                // TODO: if drone is not armed at this point, need to send a command to
                // arm it
                //
                // NOTE: this can be done from either the callback or this main
                // function, so need to decide where I want to put it

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

                    // Commnad velocities to control position
                    vel.x = 0.0; // Don't translate laterally
                    vel.y = 0.0; // Don't translate laterally
                    vel.z = -kpz * (zc - _flight_alt);

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
                    cmd.yaw = yaw;

                }
                else if (_STATE == Perimeter_Search){
                    cmd.type_mask = (mavros_msgs::PositionTarget::IGNORE_PX |
                                     mavros_msgs::PositionTarget::IGNORE_PY |
                                     mavros_msgs::PositionTarget::IGNORE_PZ |
                                     mavros_msgs::PositionTarget::IGNORE_AFX |
                                     mavros_msgs::PositionTarget::IGNORE_AFY |
                                     mavros_msgs::PositionTarget::IGNORE_AFZ |
                                     mavros_msgs::PositionTarget::IGNORE_YAW_RATE);
                    // Generalize this later to be based off longitude and lattitude, not distance from current position
                    float center_x = -160;
                    float center_y = -70;


                    float diameter_search = (5/7)*zc+28.57; // Diameter of ground below drone which is realized
                    float radius_search = diameter_search/2;

                    // Perform sweep of circle outer perimeter:
                    float xL = (radius-radius_search)*cos(angle*M_PI/180.0)+center_x;
                    float yL = (radius-radius_search)*sin(angle*M_PI/180.0)+center_y;

                    // Distance between point and current location
                    float xpt = xL-xc;
                    float ypt = yL-yc;
                    float pt_dist = sqrt(pow(xpt,2)+pow(ypt,2));

                    // Define gains for point to point travel
                    float kpx = 1.0;
                    float kpy = 1.0;
                    float kpz = 1.0;

                    // Proportional position control
                    vel.x = -kpx * (xc - xL);
                    vel.y = -kpy * (yc - yL);
                    vel.z = -kpz * (zc - _flight_alt);

                    // Saturation of velocities
                    if (abs(vel.x) > _vxMax) {
                        vel.x = sign(vel.x) * _vxMax; // saturate vx
                    }
                    if (abs(vel.y) > _vyMax) {
                        vel.y = sign(vel.y) * _vyMax; // saturate vy
                    }

                    // Set the yaw angle and the position
                    cmd.yaw = atan2(vel.y,vel.x);
                    pos.z = _flight_alt;

                    // publish the command
                    cmd.header.stamp = ros::Time::now();
                    cmd.position = pos;
                    cmd.velocity = vel;
                    cmd.yaw = atan2(vel.y,vel.x);

                    // If it reaches the objective point, switch points to acquire next trajectory
                    if (pt_dist <= 2.0){
                        angle = angle+10;
                    }
                    if (angle == 360){
                        angle = 0;
                        radius = radius-diameter_search;
                        cycle = cycle + 1;
                        _n_cycles_msg.data = cycle;
                        _n_cycles_pub.publish(_n_cycles_msg);
                    }

                }
                else if(_STATE == Pt_Trajectory) {
                    cmd.type_mask = (mavros_msgs::PositionTarget::IGNORE_PX |
                                     mavros_msgs::PositionTarget::IGNORE_PY |
                                     mavros_msgs::PositionTarget::IGNORE_PZ |
                                     mavros_msgs::PositionTarget::IGNORE_AFX |
                                     mavros_msgs::PositionTarget::IGNORE_AFY |
                                     mavros_msgs::PositionTarget::IGNORE_AFZ |
                                     mavros_msgs::PositionTarget::IGNORE_YAW_RATE);


                    // Generalize this later to be based off longitude and lattitude, not distance from current position
                    float center_x = -160;
                    float center_y = -70;

                    float radius = 150; //160
                    // x Bounds: -10,-310; y Bounds: +80, -220

                    float diameter_search = (5/7)*zc+28.57; // Diameter of ground below drone which is realized
                    float radius_search = diameter_search/2;

                    // Perform sweep of circle outer perimeter:
                    // Initial sweep of outer circle to include bounds
                    xPosVector[0] = (radius-radius_search)*cos(angle*M_PI/180.0)+center_x;
                    yPosVector[0] = (radius-radius_search)*sin(angle*M_PI/180.0)+center_y;
                    //When angle gets to 360 degrees, switch to inner circle:
                    xPosVector[2] = -250;
                    yPosVector[2] = -60;

                    int size = 5; //length of x or y vector; make this generalized later

                    // Point of interest:
                    float xL = xPosVector[count];
                    float yL = yPosVector[count];

                    // Distance between point and current location
                    float xpt = xL-xc;
                    float ypt = yL-yc;
                    float pt_dist = sqrt(pow(xpt,2)+pow(ypt,2));

                    // Define gains for point to point travel
                    float kpx = 1.0;
                    float kpy = 1.0;
                    float kpz = 1.0;

                    // Proportional position control
                    vel.x = -kpx * (xc - xL);
                    vel.y = -kpy * (yc - yL);
                    vel.z = -kpz * (zc - _flight_alt);

                    // Saturation of velocities
                    if (abs(vel.x) > _vxMax) {
                        vel.x = sign(vel.x) * _vxMax; // saturate vx
                    }
                    if (abs(vel.y) > _vyMax) {
                        vel.y = sign(vel.y) * _vyMax; // saturate vy
                    }

                    // Set the yaw angle and the position
                    cmd.yaw = atan2(vel.y,vel.x);
                    pos.z = _flight_alt;

                    // publish the command
                    cmd.header.stamp = ros::Time::now();
                    cmd.position = pos;
                    cmd.velocity = vel;
                    cmd.yaw = atan2(vel.y,vel.x);

                    // If it reaches the objective point, switch points to acquire next trajectory
                    if (pt_dist <= 2.0){
                        count = count + 1;
                        if (count == size){ //Last point has been reached
                            _STATE = GOHOME;
                        }
                    }

                }

                else if(_STATE == LINE) {

                    cmd.type_mask = 2499;  // mask for Vx Vy and Pz control

                    // Compute tangential distance to line
                    float dperp = sin(_thetaLine)*(_yLine-yc) + cos(_thetaLine)*(_xLine-xc);
                    float dAlongLine = -cos(_thetaLine)*(y0-yc) + sin(_thetaLine)*(x0-xc); // Position projection along line from drone starting point to current position

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
                    pos.z = _flight_alt;

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
                        _STATE = GOHOME;
                        x1 = xc;
                        y1 = yc;
                        _xLine = xc;
                        _yLine = yc;
                        _thetaLine = _thetaLine + 17.0 * M_PI / 32.0;
                     }
                }
                else if(_STATE == LINE2) {

                    cmd.type_mask = 2499;  // mask for Vx Vy and Pz control

                    // Compute tangential distance to line
                    float dperp = sin(_thetaLine)*(_yLine-yc) + cos(_thetaLine)*(_xLine-xc);
                    float dAlongLine = -cos(_thetaLine)*(y0-yc) + sin(_thetaLine)*(x0-xc); // Position projection along line from drone starting point to current position

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
                    pos.z = _flight_alt;

                    // publish the command
                    cmd.header.stamp = ros::Time::now();
                    cmd.position = pos;
                    cmd.velocity = vel;

                    if (abs(dAlongLine) >= 230.0){
                        _STATE = LINE3;
                        x1 = xc;
                        y1 = yc;
                        _xLine = xc;
                        _yLine = yc;
                        _thetaLine = _thetaLine + 60 * M_PI / 180.0;
                     }
                }
                else if(_STATE == LINE3) {

                    cmd.type_mask = 2499;  // mask for Vx Vy and Pz control

                    // Compute tangential distance to line
                    float dperp = sin(_thetaLine)*(_yLine-yc) + cos(_thetaLine)*(_xLine-xc);
                    float dAlongLine = -cos(_thetaLine)*(y1-yc) + sin(_thetaLine)*(x1-xc); // Position projection along line from drone starting point to current position

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
                    pos.z = _flight_alt;

                    // publish the command
                    cmd.header.stamp = ros::Time::now();
                    cmd.position = pos;
                    cmd.velocity = vel;

                    if (abs(dAlongLine) >= 180.0){
                        _STATE = GOHOME;
                     }

                }
                else if(_STATE == GOHOME) {
                    cmd.type_mask = (mavros_msgs::PositionTarget::IGNORE_PX |
                                     mavros_msgs::PositionTarget::IGNORE_PY |
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
                    vel.x = -kpx * (xc - x0);
                    vel.y = -kpy * (yc - y0);
                    vel.z = -kpz * (zc - _flight_alt);

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
                    cmd.yaw = atan2(vel.y,vel.x);


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

                    // Commnad velocities to control position
                    vel.x = 0.0; // Don't translate laterally
                    vel.y = 0.0; // Don't translate laterally
                    vel.z = -kpz * (zc - z0);

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
                    cmd.yaw = yaw;
                    }

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
        float thetaLine_desired = 245.0*M_PI/180.0; // Measured CCW from SOUTH, defines forward
        float xLine= -10.0;
        float yLine = -10.0;

        // Desired forward speed
        float vDes = 2.0f;

        // create the node
        ControlNode node(thetaLine_desired, xLine, yLine, vDes);

        // run the node
        return node.run();
}
