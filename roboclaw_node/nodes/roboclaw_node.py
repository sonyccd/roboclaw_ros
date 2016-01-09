#!/usr/bin/env python
from math import pi, cos, sin

import roboclaw_driver.roboclaw_driver as roboclaw
import rospy
import tf
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry


class EncoderOdom:
    def __init__(self, ticks_per_meter, base_width):
        self.TICKS_PER_METER = ticks_per_meter
        self.BASE_WIDTH = base_width
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1)
        self.cur_x = 0
        self.cur_y = 0
        self.cur_theta = 0.0
        self.last_enc_left = 0
        self.last_enc_right = 0
        self.last_enc_time = rospy.Time.now()

    def normalize_angle(angle):
        while angle > pi:
            angle -= 2.0 * pi
        while angle < -pi:
            angle += 2.0 * pi
        return angle

    def update(self, enc_left, enc_right):
        left_ticks = enc_left - self.last_enc_left
        right_ticks = enc_right - self.last_enc_right
        self.last_enc_left = enc_left
        self.last_enc_right = enc_right

        dist_left = left_ticks / self.TICKS_PER_METER
        dist_right = right_ticks / self.TICKS_PER_METER
        dist = (dist_right + dist_left) / 2.0

        current_time = rospy.Time.now()
        d_time = (current_time - self.last_enc_time).to_sec()
        self.last_enc_time = current_time

        if left_ticks == right_ticks:
            d_theta = 0.0
            self.cur_x += dist * cos(self.cur_theta)
            self.cur_y += dist * sin(self.cur_theta)
        else:
            d_theta = (dist_right - dist_left) / self.BASE_WIDTH
            R = dist / d_theta
            self.cur_x += R * (sin(d_theta + self.cur_theta) - sin(self.cur_theta))
            self.cur_y -= R * (cos(d_theta + self.cur_theta) - cos(self.cur_theta))
            self.cur_theta = self.normalize_angle(self.cur_theta + d_theta)

        if abs(d_time) < 0.000001:
            vel_x = 0.0
            vel_theta = 0.0
        else:
            vel_x = dist / d_time
            vel_theta = d_theta / d_time

        return vel_x, vel_theta

    def update_publish(self, enc_left, enc_right):
        # 2106 per 0.1 seconds is max speed, error in the 16th bit is 32768
        if abs(enc_left - self.last_enc_left) > 20000:
            rospy.logerr("Ignoring left encoder jump: cur %d, last %d" % (enc_left, self.last_enc_left))
        elif abs(enc_right - self.last_enc_right) > 20000:
            rospy.logerr("Ignoring right encoder jump: cur %d, last %d" % (enc_right, self.last_enc_right))
        else:
            vel_x, vel_theta = self.update(enc_left, enc_right)
            self.publish_odom(self.cur_x, self.cur_y, self.cur_theta, vel_x, vel_theta)

    def publish_odom(self, cur_x, cur_y, cur_theta, vx, vth):
        quat = tf.transformations.quaternion_from_euler(0, 0, cur_theta)
        current_time = rospy.Time.now()

        br = tf.TransformBroadcaster()
        br.sendTransform((cur_x, cur_y, 0),
                         tf.transformations.quaternion_from_euler(0, 0, cur_theta),
                         current_time,
                         "base_footprint",
                         "odom")

        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = 'odom'

        odom.pose.pose.position.x = cur_x
        odom.pose.pose.position.y = cur_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(*quat)

        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[14] = 99999
        odom.pose.covariance[21] = 99999
        odom.pose.covariance[28] = 99999
        odom.pose.covariance[35] = 0.01

        odom.child_frame_id = 'base_link'
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = 0
        odom.twist.twist.angular.z = vth
        odom.twist.covariance = odom.pose.covariance

        self.odom_pub.publish(odom)


class Node:
    def __init__(self):
        rospy.init_node('roboclaw_node')
        rospy.loginfo("Connecting to roboclaw")
        roboclaw.Open("/dev/ttyACM0", 115200)
        rospy.on_shutdown(self.shutdown)
        self.address = 0x80
        version = roboclaw.ReadVersion(self.address)
        if not version[0]:
            rospy.logwarn("Could not get version from roboclaw")
        else:
            rospy.loginfo(repr(version[1]))
        self.MAX_SPEED = 2.0
        self.TICKS_PER_METER =

        self.encodm = EncoderOdom(100, 100)
        self.last_set_speed_time = rospy.get_rostime()
        rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback)

    def run(self):
        rospy.loginfo("Starting motor drive")
        r_time = rospy.Rate(10)
        while not rospy.is_shutdown():
            enc1, status1, crc1 = roboclaw.ReadEncM1(self.address)
            enc2, status2, crc2 = roboclaw.ReadEncM2(self.address)
            rospy.logdebug("Encoders %d %d" % (enc1, enc2))
            self.encodm.update_publish(enc1, enc2)
            r_time.sleep()

    def cmd_vel_callback(self, twist):
        if not self.paused:
            self.last_set_speed_time = rospy.get_rostime()

            linear_x = twist.linear.x
            if linear_x > self.MAX_SPEED: linear_x = self.MAX_SPEED
            if linear_x < -self.MAX_SPEED: linear_x = -self.MAX_SPEED

            Vr = linear_x + twist.angular.z * self.BASE_WIDTH / 2.0  # m/s
            Vl = linear_x - twist.angular.z * self.BASE_WIDTH / 2.0

            Vr_ticks = int(Vr * self.TICKS_PER_METER)  # ticks/s
            Vl_ticks = int(Vl * self.TICKS_PER_METER)

            roboclaw.SpeedM1M2(Vr_ticks, Vl_ticks)

    def shutdown(self):
        rospy.loginfo("Shutting down")
        roboclaw.ForwardM1(self.address, 0)
        roboclaw.ForwardM2(self.address, 0)


if __name__ == "__main__":
    try:
        node = Node()
        node.run()
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("Exiting")
