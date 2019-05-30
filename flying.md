# Flying #

This guide will contain everything you need to know and do to get your drones up in the air running your controllers and other code you have developed in ROS.

## Before Your First Flight ##

Before your first flight, you will need to have completed the following setup steps in the comfort of the lab:

 - [Configure your flight modes](#configure-flight-modes) to enable `OFFBOARD` control from your remote control's flight mode switch

 - [Enable the companion computer](#enable-companion-computer) from the Pixhawk, which configures the `telem2` port to send the autopilot data over 

 - [Create rosbags directory](#create-rosbags-directory) which is where all the logs are configured to be saved in the skeleton code provided.

 - [Download scripts](#download-scripts) to be able to access the helper start / stop scripts we have provided for you to run ROS on the Raspberry Pi

 - [Download your code](#download-your-code) to your Raspberry Pi

The first 3 configuration steps will only need to be done once before your very first flight and once set, those configurations should persist.

### Configure Flight Modes ###

You will need to make sure that your flight mode switch is able to toggle your Pixhawk into `OFFBOARD` mode.  In the **Flight Modes** tab of the settings page in QGroundControl (with your Pixhawk connected), you are able to change the assigned modes to the different positions of your flight mode switch.

If you toggle your switch, you should be able to see the highlighted flight mode changes in the **Flight Modes** tab as you toggle through the positions of your switch.

We recommend you set your 3 flight mode options to be: `Manual` in the top position, `Altitude` in the middle position, and `Offboard` in the bottom position.

Without the Raspberry Pi connected and ROS running, if you switch to the `OFFBOARD` state, QGroundControl should display an error saying that the change is not possible at this time (it's not receiving any commands so it does not believe anything is attached and therefore will not change to `OFFBOARD`).

### Enable Companion Computer ###

The next step is to ensure that data can be sent from the Pixhawk to the Raspberry Pi.  You will be using the `telem2` port for this purpose.  To enable this data stream, you will need to configure the `SYS_COMPANION` parameter.

In the setting page in QGroundControl (with your Pixhawk connected), navigate to the **Parameters** tab.  Within the **Parameters** tab, navigate to the **System** tab, which will display the system related parameters.  One of the parameters in the list should be `SYS_COMPANION`.  Make sure the value for `SYS_COMPANION` is set to `Companion link (921600 8N1)`.  **Note: the exact wording may be slightly different, but the key number to be looking for is `921600` which sets the rate of communication over the `telem2` link.**

For more specific details on this configuration, you can [check out the PX4 documentation](https://dev.px4.io/en/ros/offboard_control.html#2-enable-the-companion-computer-interface).

### Create Rosbags Directory ###

The example launch files provided to you are all configured to save the rosbag log information into a `~/rosbags` directory.  Unfortunately `rosbag` isn't smart enough to create the directories if they don't exist so you will need to make sure you have a `~/rosbags` directory:

```sh
mkdir ~/rosbags
```

### Download Scripts ###

In addition to some of the SITL scripts, the scripts directory also contains helper scripts to start and stop ROS on your Raspberry Pi when at the field (`start.sh` and `stop.sh`) so it is important you get this directory on your Raspberry Pi.

To download the scripts, you can simply clone the directory to your Raspberry Pi:

```sh
cd ~
git clone https://github.com/aa241x/scripts.git
```

Once downloaded, you will want to make sure that the scripts are executable with the following commands:

```sh
cd ~/scripts
sudo chmod +x start.sh
sudo chmod +x stop.sh
```

For more detail on the `start.sh` and `stop.sh` scripts, see [the section below that explains modifications you might want to make](#modifying-the-startup-script) and if you're really curious about what the script is actually doing, see the [details on start / stop]().

### Download Your Code ###

Finally, you'll want to make sure that you have cloned your code on to your Raspberry Pi and that you have compiled it with `catkin_make`.

In addition to your code (`aa241x_commander` and `aa241x_vision`) you will also need to make sure that you have `aa241x_mission` cloned to your Raspberry Pi as that contains important elements to be able to establish the connection with MavROS and run all the elements of what makes the mission the mission.


## Before Every Flight ##

This is more of a checklist to make sure you have everything you need before you make the "treck" out to Lake Lag to only realize you forgot something.

 - Ensure you have all the components needed:
     + propellers
     + battery (charged)
     + remote control (charged)
     + Raspberry Pi
     + Power adapter for the Raspberry Pi
     + USB-to-serial connector for the connection between the Raspberry Pi and the Pixhawk
     + USB-to-Ethernet adapter
     + Ethernet cable
 - Your code is on the Raspberry Pi is up to date with what you want to fly and that it has been compiled
 - You have a launch file configured with the nodes you want to run
 - Your startup script(s) reflects the launch file you plan to run


## Startup Procedure ##

When you get to the field, these are the steps you will need to take in order to get all the elements up and running before you take off.

### 1. Make sure that all parts are securely mounted and all connections are made ###

This includes the propellers, the battery, the Raspberry Pi, etc.  Basically anything that comes on and off your drone pretty regularly should be checked.

For the connections, ensure that the USB-to-serial cable between the Raspberry Pi and the Pixhawk is connected and securely attached.

### 2. Turn everything on ###

 - Make sure your remote control is turned on
 - Plug in the battery to the power adapter cable
 - Plug in the power adapter cable to both the Raspberry Pi (through the micro-USB port) and to the drone (the XT-60 connector)

At this point you should also be able to have your telemetry radio plugged in to your laptop and a connection should be established to display information to QGroundControl.

On the USB-to-serial adapter you should also now see 2 red lights: one solid and one mostly solid (flashing very quickly some say it looks solid).  The solid light shows there is power to the adapter and the flashing light shows that data is flowing from the Pixhawk to the Raspberry Pi.  **If you don't see the flashing light, make sure that you have [configured the companion computer link](#enable-companion-computer).

### 3. Start ROS ###

*Note: This is one of the more involved steps of the process.*

We will be breaking this step down into several parts:

 1. connect your computer to the Raspberry Pi through the Ethernet link (USB-to-Ethernet adapter connected to your laptop and Ethernet cable connected from the adapter to the Ethernet port of the Raspberry Pi)

 2. ssh to your Raspberry Pi (this effectively remotely logs you into the Raspberry Pi and allows you interact with the command line on the Raspberry Pi).  Remember that your Raspberry Pi's Ethernet address was set to `192.168.1.41` and has a username of `aa241x`.  With that information you can ssh by opening your windows command line and typing the following:

```sh
ssh aa241x@192.168.1.41
```

 3. [Make sure that the startup script reflects what you want to run](#modifying-the-startup-script).  The startup script is configured to launch a specific launch file, so you will need to make sure it is launching the file of interest.  As an important observation here, this means that you will need a single launch file that runs all the elements you want.

 4. Run the startup script:

```sh
cd ~/scripts
./start.sh
```

 5. *(optional)* You can check to make sure that ROS is running properly by typing `rostopic list` in the command line at this point.  If all started correctly, it should be able to connect to the master and list out all of the topics that are currently advertised.  If you want to check to make sure a specific element is working, you can also call `rostopic echo <name-of-topic-of-interest>` to ensure that data is indeed flowing of the topic of interest.  As a second check, on the USB-to-serial adapter, you should see one solid red light, one mostly solid red light (flashing so fast it looks solid), and one more visibly flashing red (some say orange-ish) light (so 3 different light sources), which signals that the adapter has power, data is flowing from the Pixhawk to the Raspberry Pi, and data is flowing from the Raspberry Pi to the Pixhawk (respectively).

 6. You're just about all set to fly!  The last step here is to exit out of the ssh connection (type `exit` in the command line) and disconnect your Ethernet cable from the Raspberry Pi.

At this point your drone should only be connected to its own elements and you should be able to walk away to a safe distance before getting everything started!

### 4. Arm your drone ###

Before turning control over to the Raspberry Pi, you will need to arm the drone with the remote control.

### 5. Transition to `OFFBOARD` control ###

*Note: there are 2 different options at this point.*

 - If your control code is designed to handle takeoff, then all you need to do is toggle your flight mode switch to the position for `OFFBOARD`.  You should then see that change reflected on QGroundControl and ideally, if all goes well, your drone should **safely** takeoff!

 - If your control code assumes you are already in the air (e.g. you are focusing on acquiring a line in the air), then takeoff either manually or in your favorite flight mode to your desired altitude.  Once at or near your desired altitude, toggle the flight mode switch to the position for `OFFBOARD`.  You should then see that change reflected on QGroundControl and ideally, if all goes well, your drone should start moving exactly as expected!


## Shutdown Procedure ##

Once you have landed (either fully autonomously or after having taken back control - toggled from `OFFBOARD` back to the mode of your choice) and the drone is disarmed, you will be able to stop ROS on the Raspberry Pi.

*Note: if you really want, you could just unplug the Raspberry Pi, but that is most definitely not recommended as it will corrupt some of the log data.  It is recoverable, so if you accidentally do it, don't panic.*

 1. Connect to the Raspberry Pi over Ethernet (connect the cables and ssh into the Raspberry Pi as done in the startup procedure)

 2. Navigate to the `~/scripts` directory and run the `stop.sh` script (this should never need to be modified for its basic functionality):

```sh
cd ~/scripts
./stop.sh
```

 3. *(optional but recommended if you are flying again immediately)* To make sure all went well with the logging and to make sure you know which log corresponds with this flight, head over to the `~/rosbags` and look at the files in the directory (`ls` command).  Note the date and timestamp corresponding to the newest log (the one you don't recognize - the date and time should be more or less correct, but we have seen some oddities on the time).  You can double check that it indeed contains the information you expect by running `rosbag info <log-name>` (make sure you type the actual log name where it says `<log-name>`), which will show you the details of which messages were logged and how many of each message was logged.

 4. If you're done for the day, then you can shutdown the computer by running `sudo shutdown now`.  This command will trigger the shutdown and automatically disconnect your ssh connection.  If you're not done for the day, go back to the [startup procedures](#startup-procedure).  **Note: if you are changing the battery out, this will remove power from the Raspberry Pi, so make sure you run the shutdown command before doing so.**


## FAQ ##

**I'm at the Lake and we forgot to pull our newest code!  Do I really have to walk back to Durand?**

 >Nope!  There are a couple ways that you can get your code up to date on your Raspberry Pi.  If you happen to have the most up to date code on your laptop, you can use FTP (e.g. with FileZilla) to simply move the files from your computer to the Raspberry Pi.  If you don't, the Elliot Program Center (building near the Lake) does have WiFi and you can get the code on your laptop over that connection.  If you have your Raspberry Pi registered on the network, you may find that you'll even be able to call `git pull` if you are close enough to the Elliot Program Center.


**I don't like the startup script, can I do my own thing?**

 > The startup script is meant to help you with some of the oddities of working with Linux and companion computers.  If you are more comfortable with different ways to run background tasks, then please go ahead and do that.  Note however that simply running `roslaunch <package-name> <launch-file>` and then closing the terminal will not work as that is a foreground task that is immediately terminated when your connection to the Raspberry Pi is closed.


## Troubleshooting ##

### Unable to SSH into the Raspberry Pi ###

Check your USB-to-Ethernet adapter settings.  You might have forgotten to configure the adapter for a fixed IP address (e.g. `192.168.1.40` with a netmask of `255.255.255.0`).

Did you edit any of the network settings on the Raspberry Pi itself?  There is a chance that the Raspberry Pi is not configured for a local connection but rather a regular automatically assigned connection.  If this is the case, then unfortunately you will have to plug in your Raspberry Pi with a mouse / keyboard / monitor to fix the problem.

### `rostopic list` isn't showing anything when start is run ###

When you run the `./start.sh` script, the output that is typically displayed to the terminal is saved to a file called `nohup.out`.  This file should exist in the `~/scripts` directory if you followed the exact commands above, otherwise it will exist in the directory from which you called the script. You can look at the contents of the file either through your favorite command line editor or with the `cat` command (`cat nohup.out`).  This will display the contents of what would have appeared in the terminal with the `roslaunch` command and should indicate why ROS crashed.


## Helpful Hints ##


### Modifying the Startup Script ###

You will find that you will eventually want to modify the startup script to call your desired launch file when starting.  Looking at the startup script itself we can see it does the following steps:

 1. Moves to the catkin directory
 2. Executes the `source` command we all know and love
 3. Calls `roslaunch` within a specific framework that allows it to run as a background task
 4. Saves ID of that background task so that we can eventually "safely" stop the task

Of most interest is **Step 3** where the standard call to `roslaunch` can be found.  To launch your own custom launch file, or specify your own parameters, etc, you can modify this line of the script you reflect your desired `roslaunch` command.  For example, the current line says:

```sh
nohup roslaunch aa241x_commander controller_mission.launch &
```

But let's say we want to start a launch file called `fly_all_the_things.launch`, we can change the line to be:

```sh
nohup roslaunch aa241x_commander fly_all_the_things.launch &
```


### Editing On The Fly ###

Unfortunately editing any ROS code while in the air is not just inadvisable, it is not possible when relying on the wired connection to edit/run things on the Raspberry Pi.  However, that doesn't mean you can't do anything when you're out at the field.  You have 2 main options for editing code while out at the field:

 - using a command line editor (e.g. `vim`)
 - editing the files on your own machine and using FTP to move the files to your Raspberry Pi (e.g. using FileZilla to transfer the files directly to the Raspberry Pi)

**Remember: if you are editing source code (`.cpp` files) you will need to run `catkin_make` for your changes to be built and be seen, otherwise you will still be running the old version of the code!**
