/**
 * this file will contain the skeleton / example node for reading from the picam
 * using the C++ library
 *
 * maybe I'll also have it demonstrate the ability to find the AprilTag in the
 * image
 *
 * TODO: figure out how much the latency is and see if it will really cause
 * problems.  I think if they have an estimator running for the position of the
 * truck bed, it will help with any latency??? -> actually probably not...
 */
#include <math.h>
#include <iostream>

#include <ros/ros.h>
#include <string>

//#include <raspicam/raspicam_cv.h>
//#include <image_transport/image_transport.h>
//#include <opencv2/highgui/highgui.hpp>
//#include <cv_bridge/cv_bridge.h>

//extern "C" {
//#include <apriltag/apriltag.h>
//#include <apriltag/apriltag_pose.h>
//#include <apriltag/tag16h5.h>
//#include <apriltag/tag36h11.h>
//}

#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <random>

#include <aa241x_mission/RequestLandingPosition.h>

const std::string MINISEARCH = "MINISEARCH";
const std::string Hover_Search = "Hover_Search";
const std::string LAND = "LAND";
const std::string Navigate_to_land = "Navigate_to_land";

using namespace std;

// TODO: remove aa241x from the node name
class VisionNode {

public:

//	VisionNode(int frame_width, int frame_height, bool publish_image);
        VisionNode(int frame_width, int frame_height);

	// main loop
	int run();


private:

	// node handler
        ros::NodeHandle _nh;


	// settings, etc
	int _frame_width;		// image width to use in [px]
	int _frame_height;		// image height to use in [px]

	// camera stuff
//	raspicam::RaspiCam_Cv _camera;	// TODO: use the correct class name here

        // Drone state
        std::string _STATE;

        // Tag relative position (camera frame) from the drone
        float _xr;
        float _yr;
        float _zr;

        // Drone position (for sim)
        float _xc;
        float _yc;
        float _zc;

        // Drone orientation (for sim)
        float _yaw;
        float _pitch;
        float _roll;

        // Landing position (for sim)
        float _landing_e;
        float _landing_n;
        float _z0;

        // Subscribers
        ros::Subscriber _droneState_sub;
        ros::Subscriber _xc_sub;
        ros::Subscriber _yc_sub;
        ros::Subscriber _zc_sub;
        ros::Subscriber _yaw_sub;
        ros::Subscriber _pitch_sub;
        ros::Subscriber _roll_sub;
        ros::Subscriber _z0_sub;

        // Messages
        std_msgs::Float64 _tag_relative_x_msg;
        std_msgs::Float64 _tag_relative_y_msg;
        std_msgs::Float64 _tag_relative_z_msg;
        std_msgs::Bool _tag_found_msg;
        std_msgs::String _droneState_msg;

	// publishers
        ros::Publisher _tag_relative_x_pub;	// the relative position vector to the truck
        ros::Publisher _tag_relative_y_pub;	// the relative position vector to the truck
        ros::Publisher _tag_relative_z_pub;	// the relative position vector to the truck
        ros::Publisher _tag_found_pub;
        ros::Publisher _tag_details_pub;	// the raw tag details (for debugging)

        //service
        ros::ServiceClient _landing_loc_client;

        // Callbacks
        void droneStateCallback(const std_msgs::String::ConstPtr& msg);

        void xcCallback(const std_msgs::Float64::ConstPtr& msg);

        void ycCallback(const std_msgs::Float64::ConstPtr& msg);

        void zcCallback(const std_msgs::Float64::ConstPtr& msg);

        void yawCallback(const std_msgs::Float64::ConstPtr& msg);

        void pitchCallback(const std_msgs::Float64::ConstPtr& msg);

        void rollCallback(const std_msgs::Float64::ConstPtr& msg);

        void z0Callback(const std_msgs::Float64::ConstPtr& msg);

        void simTagPosition();

};


//VisionNode::VisionNode(int frame_width, int frame_height, bool publish_image) :
VisionNode::VisionNode(int frame_width, int frame_height) :
_frame_width(frame_width),
_frame_height(frame_height)//,
{
    // Publishers
    _tag_relative_x_pub = _nh.advertise<std_msgs::Float64>("tag_rel_x", 10);
    _tag_relative_y_pub = _nh.advertise<std_msgs::Float64>("tag_rel_y", 10);
    _tag_relative_z_pub = _nh.advertise<std_msgs::Float64>("tag_rel_z", 10);
    _tag_found_pub      = _nh.advertise<std_msgs::Bool>("tagFound",10);

    // Subscribers
    _droneState_sub = _nh.subscribe<std_msgs::String>("drone_state", 10, &VisionNode::droneStateCallback, this);
    _xc_sub = _nh.subscribe<std_msgs::Float64>("xc", 10, &VisionNode::xcCallback, this);
    _yc_sub = _nh.subscribe<std_msgs::Float64>("yc", 10, &VisionNode::ycCallback, this);
    _zc_sub =  _nh.subscribe<std_msgs::Float64>("zc", 10, &VisionNode::zcCallback, this);
    _yaw_sub = _nh.subscribe<std_msgs::Float64>("yaw", 10, &VisionNode::yawCallback, this);
    _pitch_sub = _nh.subscribe<std_msgs::Float64>("pitch", 10, &VisionNode::pitchCallback, this);
    _roll_sub = _nh.subscribe<std_msgs::Float64>("roll", 10, &VisionNode::rollCallback, this);
    _z0_sub = _nh.subscribe<std_msgs::Float64>("z0", 10, &VisionNode::z0Callback, this);

    // service
    _landing_loc_client = _nh.serviceClient<aa241x_mission::RequestLandingPosition>("lake_lag_landing_loc");

//    // configure the camera
//    _camera.set(CV_CAP_PROP_FORMAT, CV_8UC1);
//    _camera.set(cv::CAP_PROP_FRAME_WIDTH, _frame_width);
//    _camera.set(cv::CAP_PROP_FRAME_HEIGHT, _frame_height);
//    _camera.set(cv::CAP_PROP_FORMAT, CV_8UC1);

}

void VisionNode::droneStateCallback(const std_msgs::String::ConstPtr& msg) {
        // save the state locally to be used in the main loop
        _STATE = msg->data;
}

void VisionNode::xcCallback(const std_msgs::Float64::ConstPtr& msg) {
    _xc = msg->data;
}

void VisionNode::ycCallback(const std_msgs::Float64::ConstPtr &msg) {
    _yc = msg->data;
}

void VisionNode::zcCallback(const std_msgs::Float64::ConstPtr& msg) {
    _zc = msg->data;
}

void VisionNode::yawCallback(const std_msgs::Float64::ConstPtr &msg) {
    _yaw = msg->data;
}

void VisionNode::pitchCallback(const std_msgs::Float64::ConstPtr &msg) {
    _pitch = msg->data;
}

void VisionNode::rollCallback(const std_msgs::Float64::ConstPtr &msg) {
    _roll = msg->data;
}

void VisionNode::z0Callback(const std_msgs::Float64::ConstPtr& msg) {
    _z0 = msg->data;
}
// Simulates the position of the april tag
void VisionNode::simTagPosition() {

    float le = _landing_e;
    float ln = _landing_n;
    float lu = _z0;

    _xr = -cos(_pitch)*cos(_yaw)*(le-_xc) - (ln-_yc)*(sin(_yaw)*cos(_roll)+sin(_pitch)*sin(_roll)*cos(_yaw)) - (lu-_zc)*(sin(_roll)*sin(_yaw)-sin(_pitch)*cos(_roll)*cos(_yaw));

    _yr = (lu-_zc)*(sin(_roll)*cos(_yaw)+sin(_pitch)*sin(_yaw)*cos(_roll)) + (ln-_yc)*(cos(_roll)*cos(_yaw)-sin(_pitch)*sin(_roll)*sin(_yaw)) - sin(_yaw)*cos(_pitch)*(le-_xc);

    _zr = sin(_roll)*cos(_pitch)*(ln-_yc) - sin(_pitch)*(le-_xc) - cos(_pitch)*cos(_roll)*(lu-_zc);
}


int VisionNode::run() {

//    // apriltag handling setup
//    //apriltag_family_t *tf = tag16h5_create();
//    apriltag_family_t *tf = tag36h11_create();
//    apriltag_detector_t *td = apriltag_detector_create();
//    apriltag_detector_add_family(td, tf);
//    td->quad_decimate = 3.0;
//    td->quad_sigma = 0.0;
//    td->refine_edges = 0;
//    //td->decode_sharpening = 0.25;

//    cv::Mat frame_gray;		// the image in grayscale

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
    float test_value = 1.0;

    // loop while the node should be running
    while (ros::ok()) {

        // Check if we should use the camera..
        if (_STATE == Hover_Search || _STATE == MINISEARCH || _STATE == Navigate_to_land || _STATE == LAND) {

//            // open the camera
//            ROS_INFO("opening camera");
//            if (!_camera.open()) {
//                ROS_ERROR("Error opening the camera");
//                std::cerr << "Error opening the camera" << std::endl;
//                return -1;
//            }

//            _camera.grab();
//            _camera.retrieve(frame_gray);

//            image_u8_t im = { .width = frame_gray.cols,
//                        .height = frame_gray.rows,
//                        .stride = frame_gray.cols,
//                        .buf = frame_gray.data
//            };

//            zarray_t *detections = apriltag_detector_detect(td, &im);
//            ROS_INFO("%d tags detected", zarray_size(detections));

            //if (test_value == 1.0){
                //test_value = 2.0;
               // _tag_found_msg.data = false;
            //}
            //else{
                _tag_found_msg.data = true;
           // }

//            for (int i = 0; i < zarray_size(detections); i++) {
//                apriltag_detection_t *det;
//                zarray_get(detections, i, &det);

//                // Define camera parameters struct
//                apriltag_detection_info_t info;
//                info.det = det;
//                info.tagsize = 0.16;
//                info.fx = 1.0007824174077226e+03;
//                info.fy = 1.0007824174077226e+03;
//                info.cx = 640.;
//                info.cy = 360.;

                // Estimate pose
//                apriltag_pose_t pose;
//                double err = estimate_tag_pose(&info, &pose);
//                matd_t* t = pose.t;
//                double *tData = t->data;
//                double x = tData[0];
//                double y = tData[1];
//                double z = tData[2];


                // Publish the vector from the drone to the april tag (camera frame)
            simTagPosition();
            std::default_random_engine generator;
            std::normal_distribution<float> noise(0.0, 0.0);
            _tag_relative_x_msg.data = _xr + noise(generator);
            //_tag_relative_x_msg.data = _xr;
            _tag_relative_y_msg.data = _yr+ noise(generator);
            _tag_relative_z_msg.data = _zr+ noise(generator);

            _tag_relative_x_pub.publish(_tag_relative_x_msg);
            _tag_relative_y_pub.publish(_tag_relative_y_msg);
            _tag_relative_z_pub.publish(_tag_relative_z_msg);
            _tag_found_pub.publish(_tag_found_msg);

//            }
            // clean up the detections
//            zarray_destroy(detections);
        }

        ros::spinOnce();

    }

    // need to stop the camera
    ROS_INFO("stopping camera");
//    _camera.release();

    // remove apriltag stuff
//    apriltag_detector_destroy(td);
    //tag16h5_destroy(tf);
//    tag36h11_destroy(tf);

}



int main(int argc, char **argv) {

	// initialize th enode
	ros::init(argc, argv, "vision_node");

	// get parameters from the launch file which define some mission
	// settings
	ros::NodeHandle private_nh("~");
	// TODO: determine settings
	int frame_width, frame_height;
	bool publish_image;
	private_nh.param("frame_width", frame_width, 640);
	private_nh.param("frame_height", frame_height, 512);
	private_nh.param("publish_image", publish_image, false);

	// create the node
//	VisionNode node(frame_width, frame_height, publish_image);
        VisionNode node(frame_width, frame_height);

	// run the node
	return node.run();
}
