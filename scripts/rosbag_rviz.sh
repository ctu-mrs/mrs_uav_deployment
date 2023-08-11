#!/bin/bash

if [ -f /usr/local/bin/tmux ]; then
  TMUX_BIN=/usr/local/bin/tmux
else
  TMUX_BIN=/usr/bin/tmux
fi

# get path to script
SCRIPT_PATH="$( cd "$(dirname "$0")" ; pwd -P )"

UAV_NAME=$1
PROJECT_NAME=monitor_$UAV_NAME

# following commands will be executed first, in each window
pre_input="export UAV_NAME=$UAV_NAME"

# define commands
# 'name' 'command'
input=(
  'Rviz' "waitForRos; roscd mrs_uav_testing; ${SCRIPT_PATH}/change_uav_control.sh $UAV_NAME; rosrun rviz rviz -d ${SCRIPT_PATH}/../rviz/remote_log.rviz
"
  'Layout' "waitForRos; sleep 2; ~/.i3/layout_manager.sh ${SCRIPT_PATH}/../layouts/remote_log.json
"
)

init_window="Rosout"

###########################
### DO NOT MODIFY BELOW ###
###########################

SESSION_NAME=$PROJECT_NAME

MAIN_DIR=~/"bag_files"

# Absolute path to this script. /home/user/bin/foo.sh
SCRIPT=$(readlink -f $0)
# Absolute path this script is in. /home/user/bin
SCRIPTPATH=`dirname $SCRIPT`

if [ -z ${TMUX} ];
then
  TMUX= $TMUX_BIN new-session -s "$SESSION_NAME" -d
  echo "Starting new session."
else
  echo "Already in tmux, leave it first."
  exit
fi

# create arrays of names and commands
for ((i=0; i < ${#input[*]}; i++));
do
  ((i%2==0)) && names[$i/2]="${input[$i]}"
  ((i%2==1)) && cmds[$i/2]="${input[$i]}"
done

# run tmux windows
for ((i=0; i < ${#names[*]}; i++));
do
  $TMUX_BIN new-window -t $SESSION_NAME:$(($i+1)) -n "${names[$i]}"
done

sleep 1

# send commands
for ((i=0; i < ${#cmds[*]}; i++));
do
  tmux send-keys -t $SESSION_NAME:$(($i+1)) "cd $SCRIPTPATH;${pre_input};${cmds[$i]}"
done

# identify the index of the init window
init_index=0
for ((i=0; i < ((${#names[*]})); i++));
do
  if [ ${names[$i]} == "$init_window" ]; then
    init_index=$(expr $i + 1)
  fi
done

$TMUX_BIN -2 attach-session -t $SESSION_NAME

clear
