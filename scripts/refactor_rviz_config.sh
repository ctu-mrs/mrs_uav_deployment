#!/bin/bash

PACKAGE_PATH=$(rospack find mrs_uav_deployment)

cp $PACKAGE_PATH/config/rviz/default.rviz /tmp/default.rviz

sed -i "s/uav[0-9]/$UAV_NAME/g" /tmp/default.rviz
