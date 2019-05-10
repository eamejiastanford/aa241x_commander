/**
 * skeleton / example code for a node to do command and control of the pixhawk
 */

// includes
#include <math.h>
#include <ros/ros.h>

// topic data
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>

#include <aa241x_mission/SensorMeasurement.h>
#include <aa241x_mission/MissionState.h>

// Define states
const int TAKEOFF = 0;
const int LINE = 1;


/**
 * class to contain the functionality of the controller node.
 */
class ControlNode {

public:

	/**
	 * example constructor.
	 * @param flight_alt the desired altitude for the takeoff point.
	 */
        ControlNode(float flight_alt, float yaw_angle, float xLine, float yLine, float vDes);

	/**
	 * the main loop to be run for this node (called by the `main` function)
	 * @return exit code
	 */
	int run();


private:


	// node handler
	ros::NodeHandle _nh;

	// TODO: add any settings, etc, here
	float _flight_alt = 20.0f;		// desired flight altitude [m] AGL (above takeoff)
        float _thetaLine = 0.0f;
        float _xLine = 0.0f;
        float _yLine = 0.0f;
        float _vDes = 0.0f;
        int _STATE;


	// data
	mavros_msgs::State _current_state;
	geometry_msgs::PoseStamped _current_local_pos;

	// waypoint handling (example)
	int _wp_index = -1;
	int _n_waypoints = 1;
	float _target_alt = 0.0f;

	// offset information
	float _e_offset = 0.0f;
	float _n_offset = 0.0f;
	float _u_offset = 0.0f;

	// subscribers
	ros::Subscriber _state_sub;			// the current state of the pixhawk
	ros::Subscriber _local_pos_sub;		// local position information
	ros::Subscriber _sensor_meas_sub;	// mission sensor measurement
	ros::Subscriber _mission_state_sub; // mission state
	// TODO: add subscribers here

	// publishers
	ros::Publisher _cmd_pub;
	// TODO: recommend adding publishers for data you might want to log

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

	// TODO: add callbacks here

	// helper functions

	/**
	 * wait for the connection to the Pixhawk to be established.
	 */
	void waitForFCUConnection();


};


ControlNode::ControlNode(float flight_alt, float thetaLine, float xLine, float yLine, float vDes) :
_flight_alt(flight_alt), _thetaLine(thetaLine), _xLine(xLine), _yLine(yLine), _vDes(vDes)
{


	// subscribe to the desired topics
	_state_sub = _nh.subscribe<mavros_msgs::State>("mavros/state", 1, &ControlNode::stateCallback, this);
	_local_pos_sub = _nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, &ControlNode::localPosCallback, this);
	_sensor_meas_sub =_nh.subscribe<aa241x_mission::SensorMeasurement>("measurement", 10, &ControlNode::sensorMeasCallback, this);

	// advertise the published detailed

	// publish a PositionTarget to the `/mavros/setpoint_raw/local` topic which
	// mavros subscribes to in order to send commands to the pixhawk
	_cmd_pub = _nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 1);

}

void ControlNode::stateCallback(const mavros_msgs::State::ConstPtr& msg) {
	// save the state locally to be used in the main loop
	_current_state = *msg;
}

void ControlNode::localPosCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	// save the current local position locally to be used in the main loop
	// TODO: account for offset to convert from PX4 coordinate to lake lag frame
	_current_local_pos = *msg;



	// TODO: make sure to account for the offset if desiring to fly in the Lake Lag frame

	// check to see if have completed the waypoint
	// NOTE: for this case we only have a single waypoint
	if (_wp_index == 0) {
		float current_alt = _current_local_pos.pose.position.z;

		// check condition on being "close enough" to the waypoint
		if (abs(current_alt - _target_alt) < 0.1) {
			// update the target altitude to land, and increment the waypoint
                        //_target_alt = 0;
                        _STATE = LINE;
			_wp_index++;
		}
	}
}

void ControlNode::sensorMeasCallback(const aa241x_mission::SensorMeasurement::ConstPtr& msg) {
	// TODO: use the information from the measurement as desired

	// NOTE: this callback is for an example of how to setup a callback, you may
	// want to move this information to a mission handling node
}

void ControlNode::missionStateCallback(const aa241x_mission::MissionState::ConstPtr& msg) {
	// save the offset information
	_e_offset = msg->e_offset;
	_n_offset = msg->n_offset;
	_u_offset = msg->u_offset;
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


int ControlNode::run() {

        _STATE = TAKEOFF;

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

		// if not in offboard mode, just keep waiting until we are and if not
		// enabled, then keep waiting
		//
		// NOTE: need to be streaming setpoints in order for offboard to be
		// allowed, hence the publishing of an empty command
		if (_current_state.mode != "OFFBOARD") {
			// send command to stay in the same position
			// TODO: if doing position command in the lake lag frame, make
			// sure these values match the initial position of the drone!
			pos.x = 0;
			pos.y = 0;
			pos.z = 0;

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

		// set the first waypoint
		if (_wp_index < 0) {
			_wp_index = 0;
			_target_alt = _flight_alt;
		}

                if (_STATE == TAKEOFF) {

                    cmd.type_mask = (mavros_msgs::PositionTarget::IGNORE_VX |
                            mavros_msgs::PositionTarget::IGNORE_VY |
                            mavros_msgs::PositionTarget::IGNORE_VZ |
                            mavros_msgs::PositionTarget::IGNORE_AFX |
                            mavros_msgs::PositionTarget::IGNORE_AFY |
                            mavros_msgs::PositionTarget::IGNORE_AFZ |
                            mavros_msgs::PositionTarget::IGNORE_YAW_RATE);

                    // TODO: populate the control elements desired
                    //
                    // in this case, just asking the pixhawk to takeoff to the _target_alt
                    // height
                    pos.x = 0.0;
                    pos.y = 0.0;
                    pos.z = _target_alt;

                    // publish the command
                    cmd.header.stamp = ros::Time::now();
                    cmd.position = pos;
                    cmd.velocity = vel;

                }
                else if(_STATE == LINE) {

                    // Get state for control
                    float xc = _current_local_pos.pose.position.x;
                    float yc = _current_local_pos.pose.position.y;
                    float qx = _current_local_pos.pose.orientation.x;
                    float qy = _current_local_pos.pose.orientation.y;
                    float qz = _current_local_pos.pose.orientation.z;
                    float qw = _current_local_pos.pose.orientation.w;


                    float orient[4];
                    float roll;
                    float pitch;
                    float yaw;

                    orient[0] = qw;
                    orient[1] = qx;
                    orient[2] = qy;
                    orient[3] = qz;

                    toEulerAngle(orient, roll, pitch, yaw);


                    cmd.type_mask = 2499;  // mask for Vx Vy and Pz control

//                    cmd.type_mask = (mavros_msgs::PositionTarget::IGNORE_PX |
//                            mavros_msgs::PositionTarget::IGNORE_PY |
//                            mavros_msgs::PositionTarget::IGNORE_VZ |
//                            mavros_msgs::PositionTarget::IGNORE_AFX |
//                            mavros_msgs::PositionTarget::IGNORE_AFY |
//                            mavros_msgs::PositionTarget::IGNORE_AFZ |
//                            mavros_msgs::PositionTarget::IGNORE_YAW_RATE);

                    // Compute tangential distance to line origin

                    //float h = sqrt((_xLine-xc)*(_xLine-xc) + (_yLine-yc)*(_yLine-yc));

                    // Compute perpendicular distance to line
                    //float mp = -tan(_thetaLine);

                    //float mc = (_yLine-yc)/(_xLine-xc);

                    //float psi = atan2(-(mc-mp), (1+mp*mc));

                    float dperp = sin(_thetaLine)*(_yLine-yc) + cos(_thetaLine)*(_xLine-xc);
//                    if (abs(_xLine-xc) <= 0.01) {
//                            dperp = _yLine-yc;
//                    }
//                    else{
//                        dperp = sin(psi)*h;
//                    }

                    // Define line distance gains
                    float dp = 5;
                    float kp = _vDes/dp;

                    // Command forward speed

                    // Command lateral speed to approach line
                    //float vy = sqrt(kp*kp*dperp*dperp - _vDes*_vDes);
                    float up = kp*dperp; // line y
                    float vp = _vDes; // line x

                    // Map to global:
                    vel.x = up*cos(_thetaLine) + vp*sin(_thetaLine);
                    vel.y = up*sin(_thetaLine) - vp*cos(_thetaLine);

                    // TODO: populate the control elements desired
                    //
                    // in this case, just asking the pixhawk to takeoff to the _target_alt
                    // height
                    cmd.yaw = _thetaLine;
                    pos.z = _target_alt;

                    // publish the command
                    cmd.header.stamp = ros::Time::now();
                    cmd.position = pos;
                    cmd.velocity = vel;

                }


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
	// TODO: determine settings

        float alt_desired = 5.0;
        // Line to follow:
        float thetaLine_desired = 30.0*M_PI/180.0;
        float x0 = 50.0;
        float y0 = -10.0;

        // Desired forward speed
        float vDes = 1.0f;

	// create the node
        ControlNode node(alt_desired,thetaLine_desired, x0, y0, vDes);

	// run the node
	return node.run();
}
