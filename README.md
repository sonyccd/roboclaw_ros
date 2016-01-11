# roboclaw_ros
This is the ROS driver for the Roboclaw motor controllers. http://www.ionmc.com/

## Usage
This is just a folder that contains the driver. You can clone it into your catkin workspace.
```bash
cd <workspace>/src
git clone https://github.com/sonyccd/roboclaw_ros.git
cd <workspace>
catkin_make
source devel/setup.bash
roslaunch roboclaw_node roboclaw.launch
```

## Parameters
The launch file can be configure at the command line with arguments, by changing the value in the launch file or through the rosparam server.

|Parameter|Default|Definition|
|-----|----------|-------|
|dev|/dev/ttyACM0|Dev that is the Robotclaw|
|baud|115200|Baud rate the Roboclaw is configured for|
|address|128|The address the Roboclaw is set to, 128 is 0x80|
|max_speed|2.0|Max speed allowed for motors in meters per second|
|ticks_per_meter|4342.2|The number of encoder ticks per meter of movement|
|base_width|0.315|Width from one wheel edge to another in meters|

## Topics
###Subscribed
/cmd_vel (http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html)  
  Velocity commands for the mobile base.
###Published
/odom (http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html)  
  Odometry output from the mobile base.
