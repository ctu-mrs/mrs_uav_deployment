#!/bin/bash

path="~/bag_files/"

# By default, we record everything.
# Except for this list of EXCLUDED topics:
exclude=(

# IN GENERAL, DON'T RECORD CAMERAS
#
# If you want to record cameras, create a copy of this script
# and place it at your tmux session.
#
# Please, seek an advice of a senior researcher of MRS about
# what can be recorded. Recording too much data can lead to
# ROS communication hiccups, which can lead to eland, failsafe
# or just a CRASH.
)

# file's header
filename=`mktemp`

echo -n "cd $path; ros2 bag record -a" >> "$filename"

# if there is anything to exclude
if [ "${#exclude[*]}" -gt 0 ]; then

  echo -n " --exclude-regex " >> "$filename"

  # list all the strings and separate the with |
  for ((i=0; i < ${#exclude[*]}; i++));
  do
    echo "\"${exclude[$i]}\"" >> "$filename"
    if [ "$i" -lt "$( expr ${#exclude[*]} - 1)" ]; then
      echo -n " " >> "$filename"
    fi
  done

fi

cat $filename

eval `cat $filename`
