# roboclaw_ros
This is the ROS driver for the Roboclaw motor controllers made by [Ion Motion Control](http://www.ionmc.com/).

#HELP: I have been busy with another project that is not using robo claw. Message me if you want to become a contributer and help keep this thing alive!

## Before you begin
Before you use this package you need to calibrate the velocity PID on the Roboclaw.  This will requare the
installation of the free software [IonMotion](http://downloads.ionmc.com/software/IonMotion/ionmotion.htm) (Windows only).
You do not need to tune for position just velocity.

From the user [manual](http://downloads.ionmc.com/docs/roboclaw_user_manual.pdf):
>IonMotion includes the option to autotune velocity and or position values. To use these options
you should first make sure your encoder and motor are running in the correct direction and that
basic PWM control of the motor works as expected. To do this go to the PWM Settings screen in
IonMotion. Slide the motor slider up to start moving the motor forward. Check the encoder is
increasing in value. If it is not either reverse the motor wires or the encoder wires.  

>If you are using autotune for Position control you must first set the motors QPPS value. Unlike
Velocity autotune the QPPS value will not be automatically measured. This is because most
position control systems have a limited range of movement. Once you have manually set the
motors QPPS value(eg the maximum speed the motor can run at) you can continue with Position
autotuning.  

>Then just click the autotune button for the motor you want to tune. The autotune function will
try to determine the best settings for the motor. In the Velocity settings window it will autotune
for velocity. In the Position Settings window you have the option to tune a simple PD position
controller, a PID position controller or a cascaded Position/Velocity controller(PIV). The cascaded
tune will determine both the velocity and position values for the motor but still requires the QPPS
be manually set for the motor before starting. Autotune functions usually return reasonable
values but in most cases you will still need to manually adjust them for optimum performance.

## Usage
Just clone the repo into your catkin workspace. It contains the ROS package and the motor controller driver.  Remmeber to make sure ROS has permisions to use the dev port you give it.
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
|dev|/dev/ttyACM0|Dev that is the Roboclaw|
|baud|115200|Baud rate the Roboclaw is configured for|
|address|128|The address the Roboclaw is set to, 128 is 0x80|
|max_speed|2.0|Max speed allowed for motors in meters per second|
|ticks_per_meter|4342.2|The number of encoder ticks per meter of movement|
|base_width|0.315|Width from one wheel edge to another in meters|

## Topics
###Subscribed
/cmd_vel [(geometry_msgs/Twist)](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html)  
Velocity commands for the mobile base.
###Published
/odom [(nav_msgs/Odometry)](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html)  
Odometry output from the mobile base.

#IF SOMETHING IS BROEKN:
Please file an issue, it makes it far easier to keep track of what needs to be fixed. It also allows others that might have solved the problem to contribute.  If you are confused feel free to email me, I might have overlooked something in my readme.
