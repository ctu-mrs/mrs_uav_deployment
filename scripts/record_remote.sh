#!/bin/bash

filename=`mktemp`

include=(
'(.*)odometry/odom_main(.*)'
'(.*)control_manager/cmd_odom(.*)'
'(.*)control_manager/mpc_tracker/debug_set_trajectory'
'(.*)control_manager/diagnostics'
'(.*)odometry/diagnostics'
'/rosout'
)

# file's header
echo "<launch>" > "$filename"
echo "<arg name=\"UAV_NAME\" default=\"\$(env UAV_NAME)\" />" >> "$filename"
echo "<group ns=\"\$(arg UAV_NAME)\">" >> "$filename"

echo -n "<node pkg=\"rosbag\" type=\"record\" name=\"rosbag_record_remote\" args=\"-o /home/\$(optenv USER mrs)/bag_files/latest/" >> "$filename"

# if there is anything to exclude
if [ "${#include[*]}" -gt 0 ]; then

  echo -n " -e " >> "$filename"

  # list all the string and separate the with |
  for ((i=0; i < ${#include[*]}; i++));
  do
    echo -n "${include[$i]}" >> "$filename"
    if [ "$i" -lt "$( expr ${#include[*]} - 1)" ]; then
      echo -n "|" >> "$filename"
    fi
  done

fi

echo "\" />" >> "$filename"

# file's footer
echo "</group>" >> "$filename"
echo "</launch>" >> "$filename"

cat $filename
roslaunch $filename
