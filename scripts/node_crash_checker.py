#!/usr/bin/python3

import rospy
import rosnode

from std_msgs.msg import String as ROSString

class NodeCrashChecker:

    def __init__(self):

        rospy.init_node('node_crash_checker', anonymous=True)

        uav_name = rospy.get_param('~uav_name')

        publisher_status = rospy.Publisher('~status_out', ROSString, queue_size=1)

        rate = rospy.Rate(5.0)

        registered_nodes = []

        crash_count = 0

        while not rospy.is_shutdown():

            node_names = rosnode.get_node_names()

            uav_node_names = []

            for node_name in node_names:

                if node_name.startswith("/" + uav_name):

                    uav_node_names.append(node_name)

            for node_name in uav_node_names:

                if not node_name in registered_nodes:

                    rospy.loginfo('registering ' + node_name)

                    registered_nodes.append(node_name)

            for node_name in registered_nodes:

                if not node_name in uav_node_names:

                    rospy.logerr('missing ' + node_name + '! it probably crached')

                    registered_nodes.remove(node_name)

                    crash_count += 1

            msg = ROSString();
            if crash_count == 0:
              msg.data = "{} crashes".format(crash_count)
            else:
              msg.data = "-R {} crashes".format(crash_count)

            publisher_status.publish(msg)

            rate.sleep();

if __name__ == '__main__':
    try:
        node_crash_checker = NodeCrashChecker()
    except rospy.ROSInterruptException:
        pass
