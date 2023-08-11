#!/usr/bin/python3

import rospy
import rosnode
import subprocess

from std_msgs.msg import String as ROSString

class RosbagChecker:

    def __init__(self):

        rospy.init_node('rosbag_checker', anonymous=True)

        uav_name = rospy.get_param('~uav_name')

        disk_timeout_threshold = 100.0 # miliseconds

        disk_timeout_count = 0 # how many times in a row has disk ping been higher than threshold
        disk_timeout_count_threshold = 3 # how many times in a row disk ping has to be higher than threshold to trigger rosbag stop
        # publisher_status = rospy.Publisher('~status_out', ROSString, queue_size=1)

        rate = rospy.Rate(2.0)

        print("Starting rosbag checker")

        while not rospy.is_shutdown():


            bashCommand = "ioping -D . -c 1 -B -w 500ms"
            process = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
            output, error = process.communicate()

            time = str(output).split()[-1]
            time = time.split("\\")[0]

            ms_time = float(time)/1000000

            if (ms_time < disk_timeout_threshold):
                print("Disk ping time: " + str(ms_time) + " ms")
                disk_timeout_count = 0

            else:
                disk_timeout_count += 1
                print("Disk ping time over threshold " + str(disk_timeout_count) + " times! " + str(ms_time) + " ms /" + str(disk_timeout_threshold) + " ms")
                if disk_timeout_count >= disk_timeout_threshold:
                    print("!!stopping rosbag!!")

                    msg = ROSString();
                    msg.data = "-R STOPPING ROSBAG"
                    publisher_status.publish(msg)
                    rosbag_node_name = str(uav_name) + "/rosbag_record"
                    rosnode.kill_nodes(rosbag_node_name)

            # result = str(output).find("time=")
            # print(str(output)[result])
            # print(str(output)[result + 1])
            # for text in out:
            #     if "time=" in text:
            #         print(text)
            # print("pes5")
            # node_names = rosnode.get_node_names()

            # uav_node_names = []

            # for node_name in node_names:

            #     if node_name.startswith("/" + uav_name):

            #         uav_node_names.append(node_name)

            # for node_name in uav_node_names:

            #     if not node_name in registered_nodes:

            #         rospy.loginfo('registering ' + node_name)

            #         registered_nodes.append(node_name)

            # for node_name in registered_nodes:

            #     if not node_name in uav_node_names:

            #         rospy.logerr('missing ' + node_name + '! it probably crached')

            #         registered_nodes.remove(node_name)

            #         crash_count += 1


            rate.sleep();

if __name__ == '__main__':
    try:
        rosbag_checker = RosbagChecker()
    except rospy.ROSInterruptException:
        pass
