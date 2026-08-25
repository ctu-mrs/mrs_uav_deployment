#!/bin/bash

# Target UAV to check for Zenoh router connection
UAV_NAME="${UAV_NAME:-uav1}"
UAV_IP=$(getent hosts "$UAV_NAME" 2> /dev/null | awk '{print $1}')
PATTERN="${UAV_NAME}${UAV_IP:+|$UAV_IP}"

# Check ZENOH_ROUTER_CONFIG_URI file or ZENOH_CONFIG_OVERRIDE variable
if grep -Eqs "$PATTERN" "$ZENOH_ROUTER_CONFIG_URI" || grep -Eq "$PATTERN" <<< "$ZENOH_CONFIG_OVERRIDE"; then
  ros2 run rmw_zenoh_cpp rmw_zenohd
else
  echo -e "\e[33mWarning: Zenoh router not started.\e[0m"
  echo -e "\e[33mNeither '$UAV_NAME'${UAV_IP:+ (IP: $UAV_IP)} was found in:\e[0m"
  echo -e "\e[33m  - ZENOH_ROUTER_CONFIG_URI: '${ZENOH_ROUTER_CONFIG_URI:-unset}'\e[0m"
  echo -e "\e[33m  - ZENOH_CONFIG_OVERRIDE: '${ZENOH_CONFIG_OVERRIDE:-unset}'\e[0m"
fi
