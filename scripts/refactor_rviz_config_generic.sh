#!/bin/bash

if [ "$#" -ne 2 ]; then
    echo "Usage: $0 <rviz_config_file> <uav_name>"
    exit 1
fi

RVIZ_CONFIG_FILE="$1"
UAV_NAME="$2"

cp $RVIZ_CONFIG_FILE /tmp/default.rviz

sed -i "s/uav[0-9]\+/$UAV_NAME/g" /tmp/default.rviz

rviz -d /tmp/default.rviz
