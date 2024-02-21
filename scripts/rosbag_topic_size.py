#!/usr/bin/env python
# Prints total cumulative serialized msg size in bytes per topic
# Calculation takes approximately 1sec/GB
# Usage:   python rosbag_size_by_topic.py BAG_FILE_PATH
# source: https://answers.ros.org/question/318667/using-rosbag-to-get-size-of-each-topic/

import rosbag
import sys
import argparse
import os

parser=argparse.ArgumentParser(
    description='''Display topics recorded in a rosbag along with their cumulative sizes.''')
parser.add_argument('path_to_rosbag', help='path to rosbag - e.g. ~/bag_files/latest/_bag_name.bag')
args=parser.parse_args()

file_stats = os.stat(args.path_to_rosbag)

topic_size_dict = {}
total_size = 0
size_part = 0
file_size = file_stats.st_size
for topic, msg, time in rosbag.Bag(args.path_to_rosbag, "r").read_messages(raw=True):
    topic_size_dict[topic] = topic_size_dict.get(topic, 0) + len(msg[1])
    total_size += len(msg[1])
    size_part += len(msg[1])
    if size_part > file_size / 10:
        print(f"Processed {total_size / 1000**2} / {file_size / 1000**2} MB.")
        size_part %= file_size / 10

topic_size = list(topic_size_dict.items())
topic_size.sort(key=lambda x: x[1], reverse=False)

for topic, size in topic_size:
    if size < 1000:
        print("{:7d}B  {}".format(size, topic))
    elif size >= 1000000000000:
        print("{:7.2f}T  {}".format(size/1000000000000, topic))
    elif size >= 1000000000:
        print("{:7.2f}G  {}".format(size/1000000000, topic))
    elif size >= 1000000:
        print("{:7.2f}M  {}".format(size/1000000, topic))
    elif size >= 1000:
        print("{:7.2f}K  {}".format(size/1000, topic))
