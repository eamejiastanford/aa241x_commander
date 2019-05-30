# AA241x Student Commander Example #

This ROS package is a skeleton for the teams of Spring 2019's AA241x course.  The purpose of this package is mission handling / command and control of the drone through the Pixhawk 4.  This package has been created separately from the image handling nodes as the dependencies for the nodes related to the camera and imagine are quite different than the dependencies for the mission / command and control related nodes.  Furthermore, the separation allows teams to test the mission / command and control nodes on the desktops running Gazebo (i.e. not the Raspberry Pi 3B+) without any compilation problems that would occur due to the fact that the imaging handling nodes are very specific to the Raspberry Pi 3B+ hardware (i.e. the PiCam v2) and these nodes can be more generalized to any hardware interacting with the PX4 code and the world environment.

This skeleton package currently contains the following nodes:

 - [`control_node`](#control-node): a skeleton node to help get you started with the command and control of the Pixhawk 4 onboard the drone through the [MavROS](http://wiki.ros.org/mavros) interface.  Currently the node executes a takeoff to a given altitude and once that altitude is reached, commands a landing using position commands.
 - [`mission_node`](#mission-node): a skeleton node to help present a recommended structure for your mission logic.  [See below for details on the recommendations](#recommendations)


This README is also a bit of a guide to help you get started and running the elements and is broken down as follows:

 - [Getting Started](#getting-started) - help you get everything up and running 
 - [Nodes](#nodes) - details on the nodes that exist within the skeleton code
 - [Offboard Control](#offboard-control) - details on how to configure your Pixhawk for `OFFBOARD` control (the level flight mode required to give control to ROS)
 - [Recommendations](#recommendations) - some recommendations and hints for how to structure your code as your mission elements get more complex.

## Getting Started ##

The follow sections will help you get started with this ROS package.

### Getting the Code ###

It is recommended that you first [fork](https://help.github.com/en/articles/fork-a-repo) the repository to allow you to make changes and build off the code as desired in your own fork of the code.

Once forked, you will be able to clone your repository on to your team's Raspberry Pi 3B+.  This is a ROS package that has been designed with the `catkin_ws` that is used with ROS Melodic, so make sure to clone this repository into the `catkin_ws/src` directory, where ROS packages live.

For example:

```sh
cd ~/catkin_ws/src
git clone https://github.com/<your-github-handle>/aa241x_commander.git
```

Once you have it cloned, you can build the code using `catkin_make`:

```sh
cd ~/catkin_ws/
catkin_make
```

### Running the Code ###

You will be running your ROS code in 2 different environments:

 - [In simulation](#in-simulation)
 - [On your drone](#on-your-drone)

Some of the commands for running your code in these two environments will be the same, so the first sub-section here addresses the general commands needed to get your code running and then more specifically how to run it in the different environment.

#### Running Nodes in General ####

Each of the nodes of this package can be run using the basic [`rosrun`](http://wiki.ros.org/rosbash#rosrun) framework or can be run using the example launch file using `roslaunch`.  For a tutorial on `roslaunch` check out either [this resource](http://www.clearpathrobotics.com/assets/guides/ros/Launch%20Files.html) or [this resource](http://wiki.ros.org/roslaunch#Tutorials).  The example launch file (`controller_only.launch`) will start the connection to the gazebo simulation (using the help of a launch file contained in [`aa241x_mission`](https://github.com/aa241x/aa241x_mission)) and the control node.

To run using the launch file, you first need to make sure that you have the `aa241x_mission` package.  You will most likely not be modifying the `aa241x_mission` package, so feel free to just [clone the package](https://github.com/aa241x/aa241x_mission), but if you are interested in making your own modification, you can also fork the package.

Once you have the `aa241x_mission` package, you can launch the `controller_only.launch` file:

```sh
cd ~/catkin_ws/
source devel/setup.bash
roslaunch aa241x_commander controller_only.launch
```

Remember to `catkin_make`!

**Note:** for more details on the connection to the gazebo simulation using mavros, see the `aa241x_mission` [documentation](https://github.com/aa241x/aa241x_mission).

**Note:** You'll notice that if you run the above commands in a vacuum without anything else running on your machine you will most likely run into an error. This is because the `controller_only.launch` file is trying to connect to a pixhawk, and if one is not connected, it will fail.  See the two sections below (for running [i simulation](#in-simulation) and [on your drone](#on-your-drone)).

##### Additional Launch File #####

This skeleton node also contains a launch file to demonstrate the use of [`rosbag`](http://wiki.ros.org/rosbag) to be able to log data published to topics.  This is a very helpful tool to be able to log all the data and enable replaying data in ROS after a flight.

**Note:** For the logging launch file to work, you will need to make sure to have the following directory `~/rosbags/` (if needed `mkdir ~/rosbags`) as that is where the logs will be saved.  More details on `rosbag` (logging, playback, etc), check out the [general documentation for the `aa241x_mission` node](https://github.com/aa241x/aa241x_mission).

#### In Simulation ####

To run your code with a simulated version of PX4 (what we call Software in the Loop), we have created a [detailed set of documentation](https://github.com/aa241x/scripts/blob/master/running-sitl.md) that will walk you through setting up an emulation of PX4 that your ROS environment will be able to connect to.


#### On Your Drone ####

[There is a whole guide for this with detailed steps to help you get in the air.](https://github.com/aa241x/aa241x_commander/blob/master/flying.md)


### Dependencies ###

This code has the following dependencies:

 - [MavROS](http://wiki.ros.org/mavros) - this is a ROS package that handles the communication with PX4 using the Mavlink protocol.  It handles the communication and exposes the information by publishing the information using the ROS topic framework.


## Nodes ##

Here is a more detailed description of the control node.

### Control Node ###

The control node serves both as a skeleton for the development of team's controllers for the Spring 2019's AA241x mission and as an example for controlling a drone through using the MavROS framework for sending control commands to PX4.  The example controller sends position commands to the PX4 to execute a takeoff and once the drone reaches the specified altitude commands the drone to land, again through a position control command.  To achieve this, the node subscribes to several pieces of critical information and publishes commands to a topic that the MavROS node subscribes to (see below for details on the subscriptions and publications).

#### Subscribes To ####

The control node subscribes to the following information:

 - `mavros/state` - this topic, of type [`mavros_msgs::State`](http://docs.ros.org/melodic/api/mavros_msgs/html/msg/State.html), contains information on the state of the PX4 code.  The node uses 2 pieces of information from the state:
     + `connected` - a boolean for whether or not the PX4 has completed the bootup process and has connected to the offboard computer (e.g. your Raspberry Pi 3B+).
     + `mode` - a string stating the current control mode the PX4 is set to (e.g. "MANUAL" or "OFFBOARD")

 - `/mavros/local_position/pose` - this topic, of type [`geometry_msgs::PoseStamped`](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseStamped.html), contains the local position (ENU) (defined with the origin as the take-off position of the drone) and the orientation of the drone computed by PX4.

 - `measurement` - this topic, of type [`aa241x_mission::SensorMeasurement`](https://github.com/aa241x/aa241x_mission/blob/master/msg/SensorMeasurement.msg), contains the sensor information for the AA241x mission.  For more details on the sensor measurement, check out the [documentation for the `aa241x_mission` node](https://github.com/aa241x/aa241x_mission).

 - `mission_state` - this topic, of type [`aa241x_mission::MissionState`](https://github.com/aa241x/aa241x_mission/blob/master/msg/MissionState.msg), contains general state information for the mission, and the offset needed to go from the ENU frame as computed by PX4 (with origin as the take-off position) to the Lake Lag ENU frame.  For more details on the Lake Lag frame and how to use the offset information, check out the [documentation for the `aa241x_mission` node](https://github.com/aa241x/aa241x_mission).

**Note:** You will most likely find that you need to subscribe to additional information, so check out the [full MavROS documentation](http://wiki.ros.org/mavros) for additional topics that are published.  To help see what topics and data are publishes, check out [`rostopic`](http://wiki.ros.org/rostopic), which enables viewing the published topic names and data (among other things).

#### Publishes ####

 - `mavros/setpoint_raw/local` - this topic, of type [`mavros_msgs::PositionTarget`](http://docs.ros.org/api/mavros_msgs/html/msg/PositionTarget.html), provides the control information to send to PX4 for controlling the drone.  The command is made up of several key elements:
     + `type_mask` - a bitfield that tells PX4 which fields to **ignore**.  For the most part the bitfield can be built using the constants defined in the topic type (the only notable exception is an altitude hold velocity control (specify Pz, Vx, Vy) which has a mask value of `2499`).
     + the data fields: `position`, `velocity`, `acceleration_or_force`, `yaw`, and `yaw_rate`.  These fields contain the respective information for the command to send to PX4.  **Note:** if you set data to the `velocity` fields, but the `type_mask` bitfield specifies to ignore the `velocity` fields, PX4 will ignore the fields, regardless of the data contained within the field.

### Mission Node ###

This node serves as a skeleton and example for how you might want to structure your code as your mission logic gets more complex ([see recommendations below](#recommendations)).

#### Subscribes To ####

The mission node subscribes to the following information:

 - `mavros/state` - this topic, of type [`mavros_msgs::State`](http://docs.ros.org/melodic/api/mavros_msgs/html/msg/State.html), contains information on the state of the PX4 code.  The node uses 2 pieces of information from the state:
     + `connected` - a boolean for whether or not the PX4 has completed the bootup process and has connected to the offboard computer (e.g. your Raspberry Pi 3B+).
     + `mode` - a string stating the current control mode the PX4 is set to (e.g. "MANUAL" or "OFFBOARD")

 - `/mavros/local_position/pose` - this topic, of type [`geometry_msgs::PoseStamped`](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseStamped.html), contains the local position (ENU) (defined with the origin as the take-off position of the drone) and the orientation of the drone computed by PX4.

 - `/avros/battery` - this topic, of type [`sensor_msgs::BatteryState`](http://docs.ros.org/api/sensor_msgs/html/msg/BatteryState.html), constains the battery information (voltage, current draw, etc.) as measured by the Pixhawk.

 - `measurement` - this topic, of type [`aa241x_mission::SensorMeasurement`](https://github.com/aa241x/aa241x_mission/blob/master/msg/SensorMeasurement.msg), contains the sensor information for the AA241x mission.  For more details on the sensor measurement, check out the [documentation for the `aa241x_mission` node](https://github.com/aa241x/aa241x_mission).

 - `mission_state` - this topic, of type [`aa241x_mission::MissionState`](https://github.com/aa241x/aa241x_mission/blob/master/msg/MissionState.msg), contains general state information for the mission, and the offset needed to go from the ENU frame as computed by PX4 (with origin as the take-off position) to the Lake Lag ENU frame.  For more details on the Lake Lag frame and how to use the offset information, check out the [documentation for the `aa241x_mission` node](https://github.com/aa241x/aa241x_mission).

**Note:** You will most likely find that you need to subscribe to additional information, so check out the [full MavROS documentation](http://wiki.ros.org/mavros) for additional topics that are published.  To help see what topics and data are publishes, check out [`rostopic`](http://wiki.ros.org/rostopic), which enables viewing the published topic names and data (among other things).

#### Publishes ####

Currently the node does not publish any data, however we recommend that this node should publish high level commands for your controller to act upon.

## Offboard Control ##

For more detail on PX4's definition of offboard, etc, check out the [PX4 documentation explaining offboard mode](https://docs.px4.io/en/flight_modes/offboard.html)

### Configuring the Pixhawk ###

There are 2 required steps to configure the Pixhawk to enable the use of `OFFBOARD` mode.

 1. [Enable a switch to toggle into `OFFBOARD` mode](enable-offboard).
 2. [Enable the mavlink telemetry link over the `telem2` connection for a companion computer](enable-companion-link).

#### Enable OFFBOARD ####

Make sure that `OFFBOARD` control is possibly by mapping an RC switch to the `OFFBOARD` mode.  This can be done in one of two ways:

 - **Option 1:** setting `OFFBOARD` to the third position of your already configured mode switch.  In the *radio configuration* setting page, select `OFFBOARD` for the control level of the last position switch.
 - **Option 2:** enable another switch on the remote control and use that new switch to toggle `OFFBOARD` mode.  There are guides online that help explain how to use the Taranis radio to enable another switch.


#### Enable Companion Link ####

To enable the companion link, go to the full parameters list and select the *system* tab.  In the *system* set of parameters, you should see a `SYS_COMPANION` parameter (you can also use the search feature on the parameters page to find this parameter).  Set the `SYS_COMPANION` parameter to `921600` (may be called `companion (921600)`).

This will now enable a [mavlink](https://mavlink.io/en/) connection over the `telem2` port, which is where we will be connecting the USB to Serial connector.  To validate this connection, plug in the non-USB end of the connector into the `telem2` port and plug in the USB end to the raspberry Pi 3B+.  You should now see a new device called `/dev/ttyUSB0` (use the command `ls /dev/tty*` in the terminal to see the list of devices) from the Raspberry Pi 3B+.

For a final confirmation that everything is working, you can run the following `aa241x_mission` launch file for [connecting to the pixhawk](https://github.com/aa241x/aa241x_mission#communicating-with-px4):

```sh
cd ~/catkin_ws/
source devel/setup.bash
roslaunch aa241x_mission mavros_pixhawk.launch
```


## Recommendations ##

**This section contains several recommendations to help you get started and navigating the task of designing an approach for handling the mission in a ROS environment.  These are only recommendations and you may find that you much prefer a different approach.  Please do not feel like you have to structure your code as we lay out here if something else makes more sense to you.**

### Code Structure ###

We recommend that you break down your handling of the overall mission into at least 2 different nodes:

 - **mission logic** - A node that handles the high level logic that is associated with the mission.  Things that you might want to consider doing within the scope of this node are (a sekelton code for this node and these tasks can be found in `mission_node.cpp`):

     + Keeping track of the current "mission phase" (e.g. have you taken off yet?  Are you in a search phase?  Moving back for a landing? - you can imagine that in each of these phase, you might want your system to behave differently)

     + Keeping track of battery usage to make decisions on the current "mission phase" (e.g. should you sotp searching and go back to land?)

     + "Looking" for people (i.e. subscribing to the `/measurement` topic) and running your estimation for their locations

     + Adjusting your flight plan accordingly when people have been spotted (e.g. having the `mission_node` publish high level position commands to be used by your `control_node`)

 - **control** - A node that handles the acutal control loops and publishing commands to the appropriate mavros topic to command the Pixhawk (e.g. `/mavros/setpoint_raw/local`).  As you integrate more mission phases into your system you may find it helpful for your `control_node` to subscribe to different topics that will send it high level information for where to go based on the mission phase, for example:

     + Subscribe to high level position information published by your mission logic node that is to be used during the search phase of the mission

     + Subscribe to information derived from your camera that is to be used during the landing phase of the mission


