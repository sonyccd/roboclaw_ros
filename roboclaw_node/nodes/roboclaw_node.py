#!/usr/bin/env python3
import threading
from numbers import Number
from math import pi, cos, sin

import diagnostic_msgs
import diagnostic_updater
from roboclaw_driver.roboclaw_3 import Roboclaw as roboclaw

import rospy
import tf
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry

__author__ = "bwbazemore@uga.edu (Brad Bazemore)"


# TODO need to find some better was of handling OSerror 11 or preventing it, any ideas?

class EncoderOdom:
    def __init__(self, ticks_per_meter, base_width):
        self.TICKS_PER_METER = ticks_per_meter
        self.BASE_WIDTH = base_width
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.cur_x = 0
        self.cur_y = 0
        self.cur_theta = 0.0
        self.last_enc_left = 0
        self.last_enc_right = 0
        self.last_enc_time = rospy.Time.now()

    @staticmethod
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

        # TODO find better what to determine going straight, this means slight deviation is accounted
        if left_ticks == right_ticks:
            d_theta = 0.0
            self.cur_x += dist * cos(self.cur_theta)
            self.cur_y += dist * sin(self.cur_theta)
        else:
            d_theta = (dist_right - dist_left) / self.BASE_WIDTH
            r = dist / d_theta
            self.cur_x += r * (sin(d_theta + self.cur_theta) - sin(self.cur_theta))
            self.cur_y -= r * (cos(d_theta + self.cur_theta) - cos(self.cur_theta))
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
        # TODO lets find a better way to deal with this error
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
                         "base_link",
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

        self.ERRORS = {0x000000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "Normal"),
                        0x000001: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "E-Stop"),
                        0x000002: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Temperature1"),
                        0x000004: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Temperature2"),
                        0x000008: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Main Voltage High"),
                        0x000010: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Logic Voltage High"),
                        0x000020: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Logic Voltage Low"),
                        0x000040: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "M1 Driver Fault"),
                        0x000080: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "M2 Driver Fault"),
                        0x000100: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "M1 Speed"),
                        0x000200: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "M2 Speed"),
                        0x000400: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "M1 Position"),
                        0x000800: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "M2 Position"),
                        0x001000: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "M1 Current"),
                        0x002000: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "M2 Current"),
                        0x010000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M1 Over Current"),
                        0x020000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M2 Over Current"),
                        0x040000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Main Voltage High"),
                        0x080000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Main Voltage Low"),
                        0x100000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Temperature1"),
                        0x200000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Temperature2"),
                        0x400000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "S4 Signal Triggered"),
                        0x800000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "S5 Signal Triggered"),
                        0x01000000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Speed Error Limit"),
                        0x02000000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Position Error Limit")}

        self.mutex = threading.Lock()
        rospy.init_node("roboclaw_node", disable_signals=True)
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Connecting to roboclaw")
        dev_name = rospy.get_param("~dev", "/dev/ttyACM0")
        baud_rate = int(rospy.get_param("~baud", "115200"))

        self.address = int(rospy.get_param("~address", "128"))
        if self.address > 0x87 or self.address < 0x80:
            rospy.logfatal("Address out of range")
            rospy.signal_shutdown("Address out of range")

        # TODO need someway to check if address is correct
        self.roboclaw = roboclaw(dev_name, baud_rate)
        
        status = None
        try:
            status = self.roboclaw.Open()
        except Exception as e:
            rospy.logerr("Could not connect to Roboclaw")
            rospy.logdebug(e)
            rospy.signal_shutdown("Could not connect to Roboclaw")
        
        if status:
            rospy.loginfo("Sucessfully open connection to RoboClaw")
        else:
            rospy.signal_shutdown("Could not connect to Roboclaw")

        self.updater = diagnostic_updater.Updater()
        self.updater.setHardwareID("Roboclaw")
        self.updater.add(diagnostic_updater.
                         FunctionDiagnosticTask("Vitals", self.check_vitals))
        
        rospy.sleep(1)
        try:
            with self.mutex:
                version = self.roboclaw.ReadVersion(self.address)
        except Exception as e:
            rospy.logwarn("Problem getting roboclaw version")
            rospy.logdebug(e)
            rospy.signal_shutdown("Failed to read roboclaw version, controller function improperly!!!")

        if not version[0]:
            rospy.logwarn("Could not get version from roboclaw")
            rospy.signal_shutdown("Failed to read roboclaw version, controller function improperly!!!")
        else:
            rospy.loginfo(repr(version[1]))
        
        with self.mutex:
            self.roboclaw.SpeedM1M2(self.address, 0, 0)
            self.roboclaw.ResetEncoders(self.address)

        self.MAX_SPEED          = rospy.get_param("~max_speed", 2.0)
        self.TICKS_PER_METER    = rospy.get_param("~ticks_per_meter", 4342.2)
        self.BASE_WIDTH         = rospy.get_param("~base_width", 0.315)
        self.CMD_FREQ           = rospy.get_param("~cmd_frequency", 10)

        self.encodm                 = EncoderOdom(self.TICKS_PER_METER, self.BASE_WIDTH)
        self.last_set_speed_time    = rospy.get_rostime()

        rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback)

        rospy.logdebug("dev %s", dev_name)
        rospy.logdebug("baud %d", baud_rate)
        rospy.logdebug("address %d", self.address)
        rospy.logdebug("max_speed %f", self.MAX_SPEED)
        rospy.logdebug("ticks_per_meter %f", self.TICKS_PER_METER)
        rospy.logdebug("base_width %f", self.BASE_WIDTH)

    def run(self):
        rospy.loginfo("Starting motor drive")
        r_time = rospy.Rate(self.CMD_FREQ)
        while not rospy.is_shutdown():

            if (rospy.get_rostime() - self.last_set_speed_time).to_sec() > 1./float(self.CMD_FREQ):
                # rospy.loginfo("Did not get command for 1 second, stopping")
                try:
                    with self.mutex:
                        self.roboclaw.ForwardM1(self.address, 0)
                        self.roboclaw.ForwardM2(self.address, 0)
                except OSError as e:
                    rospy.logerr("Could not stop")
                    rospy.logdebug(e)

            # TODO need find solution to the OSError11 looks like sync problem with serial
            status1, enc1, crc1 = None, None, None
            status2, enc2, crc2 = None, None, None

            try:
                with self.mutex:
                    status1, enc1, crc1 = self.roboclaw.ReadEncM1(self.address)
            except ValueError:
                pass
            except OSError as e:
                rospy.logwarn("ReadEncM1 OSError: %d", e.errno)
                rospy.logdebug(e)

            try:
                with self.mutex:
                    status2, enc2, crc2 = self.roboclaw.ReadEncM2(self.address)
            except ValueError:
                pass
            except OSError as e:
                rospy.logwarn("ReadEncM2 OSError: %d", e.errno)
                rospy.logdebug(e)

            if ((isinstance(enc1,Number) and isinstance(enc2,Number))):
                rospy.logdebug(" Encoders %d %d" % (enc1, enc2))
                self.encodm.update_publish(enc2, enc1)

                self.updater.update()
            r_time.sleep()

    def cmd_vel_callback(self, twist):
        self.last_set_speed_time = rospy.get_rostime()

        linear_x = twist.linear.x
        if linear_x > self.MAX_SPEED:
            linear_x = self.MAX_SPEED
        if linear_x < -self.MAX_SPEED:
            linear_x = -self.MAX_SPEED

        vr = linear_x + twist.angular.z * self.BASE_WIDTH / 2.0  # m/s
        vl = linear_x - twist.angular.z * self.BASE_WIDTH / 2.0

        vr_ticks = int(vr * self.TICKS_PER_METER)  # ticks/s
        vl_ticks = int(vl * self.TICKS_PER_METER)

        rospy.loginfo("vr_ticks:%8d vl_ticks: %8d", vr_ticks, vl_ticks)

        try:
            # This is a hack way to keep a poorly tuned PID from making noise at speed 0
            with self.mutex:
                if vr_ticks == 0 and vl_ticks == 0:
                    self.roboclaw.ForwardM1(self.address, 0)
                    self.roboclaw.ForwardM2(self.address, 0)
                else:
                    self.roboclaw.SpeedM1M2(self.address, vr_ticks, vl_ticks)
        except OSError as e:
            rospy.logwarn("SpeedM1M2 OSError: %d", e.errno)
            rospy.logdebug(e)

    # TODO: Need to make this work when more than one error is raised
    def check_vitals(self, stat):
        try:
            with self.mutex:
                status = self.roboclaw.ReadError(self.address)[1]
        except OSError as e:
            rospy.logwarn("Diagnostics OSError: %d", e.errno)
            rospy.logdebug(e)
            return
        
        try:
            state, message = self.ERRORS[status]
        except KeyError:
            state = diagnostic_msgs.msg.DiagnosticStatus.ERROR
            message = "Unknown or various errors: 0x{0:x}".format(status)

        stat.summary(state, message)
        
        try:
            with self.mutex:
                stat.add("Main Batt V:", float(self.roboclaw.ReadMainBatteryVoltage(self.address)[1] / 10))
                stat.add("Logic Batt V:", float(self.roboclaw.ReadLogicBatteryVoltage(self.address)[1] / 10))
                stat.add("Temp1 C:", float(self.roboclaw.ReadTemp(self.address)[1] / 10))
                stat.add("Temp2 C:", float(self.roboclaw.ReadTemp2(self.address)[1] / 10))
        except OSError as e:
            rospy.logwarn("Diagnostics OSError: %d", e.errno)
            rospy.logdebug(e)
        return stat

    # TODO: need clean shutdown so motors stop even if new msgs are arriving
    def shutdown(self):
        rospy.loginfo("Shutting down")
        try:
            with self.mutex:
                self.roboclaw.ForwardM1(self.address, 0)
                self.roboclaw.ForwardM2(self.address, 0)
        
        except OSError:
            rospy.logerr("Shutdown did not work trying again")
            try:
                with self.mutex:
                    self.roboclaw.ForwardM1(self.address, 0)
                    self.roboclaw.ForwardM2(self.address, 0)
            except OSError as e:
                rospy.logerr("Could not shutdown motors!!!!")
                rospy.logdebug(e)
        self.roboclaw._port.flushOutput()
        self.roboclaw._port.flushInput()
        self.roboclaw._port.flush()

if __name__ == "__main__":
    try:
        node = Node()
        node.run()
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("Exiting")
