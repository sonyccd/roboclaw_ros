import roboclaw
import rospy


class Node:
    def __init__(self):

        rospy.init_node('roboclaw_node_ros')

        # Init roboclaw
        rospy.loginfo("Connecting to roboclaw")
        roboclaw.Open("/dev/ttyACM0", 115200)
        self.address = 0x80
        version = roboclaw.ReadVersion(self.address)
        if not version[0]:
            rospy.logwarn("Could not get version from roboclaw")
        else:
            rospy.loginfo(repr(version[1]))

    def run(self):
        rospy.loginfo("Starting motor drive")
        r_time = rospy.Rate(5)
        while not rospy.is_shutdown():
            rospy.loginfo(roboclaw.ReadMainBatteryVoltage(self.address))
            r_time.sleep()

if __name__ == "__main__":
    try:
        node = Node()
        node.run()
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("Exiting")