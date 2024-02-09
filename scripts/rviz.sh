#!/bin/bash

# get path to script
SCRIPT_PATH="$( cd "$(dirname "$0")" ; pwd -P )"

UAV_NAME=$1
PROJECT_NAME=rviz_$UAV_NAME

# ~/.i3/detacher.sh 1 "~/.scripts/set_ros_master_uri.sh $UAV_NAME"

# following commands will be executed first, in each window
pre_input="export UAV_NAME=$UAV_NAME; export ROS_MASTER_URI=http://$UAV_NAME:11311; unset ROS_HOSTNAME; export ROS_IP=$(hostname -I | awk '{print $1}')"

# define commands
# 'name' 'command'
input=(
  'Rviz' "waitForRos; roslaunch mrs_uav_deployment rviz.launch
"
  'RvizInterface' "waitForRos; roslaunch mrs_rviz_plugins rviz_interface.launch
"
  # 'Layout' "waitForRos; sleep 2; ~/.i3/layout_manager.sh layout.json"
)

init_window="RvizInterface"

###########################
### DO NOT MODIFY BELOW ###
###########################

attach=true

export TMUX_BIN="/usr/bin/tmux -L mrs -f /etc/ctu-mrs/tmux.conf"

SESSION_NAME=$PROJECT_NAME

MAIN_DIR=~/"bag_files"

# Absolute path to this script. /home/user/bin/foo.sh
SCRIPT=$(readlink -f $0)
# Absolute path this script is in. /home/user/bin
SCRIPTPATH=`dirname $SCRIPT`

TMUX= $TMUX_BIN new-session -s "$SESSION_NAME" -d
echo "Starting new session."

# get the iterator
ITERATOR_FILE="$MAIN_DIR/$PROJECT_NAME"/iterator.txt
if [ -e "$ITERATOR_FILE" ]
then
  ITERATOR=`cat "$ITERATOR_FILE"`
  ITERATOR=$(($ITERATOR+1))
else
  echo "iterator.txt does not exist, creating it"
  mkdir -p "$MAIN_DIR/$PROJECT_NAME"
  touch "$ITERATOR_FILE"
  ITERATOR="1"
fi
echo "$ITERATOR" > "$ITERATOR_FILE"

# create file for logging terminals' output
LOG_DIR="$MAIN_DIR/$PROJECT_NAME/"
SUFFIX=$(date +"%Y_%m_%d_%H_%M_%S")
SUBLOG_DIR="$LOG_DIR/"$ITERATOR"_"$SUFFIX""
TMUX_DIR="$SUBLOG_DIR/tmux"
mkdir -p "$SUBLOG_DIR"
mkdir -p "$TMUX_DIR"

# link the "latest" folder to the recently created one
rm "$LOG_DIR/latest" > /dev/null 2>&1
rm "$MAIN_DIR/latest" > /dev/null 2>&1
ln -sf "$SUBLOG_DIR" "$LOG_DIR/latest"
ln -sf "$SUBLOG_DIR" "$MAIN_DIR/latest"

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

sleep 3

# start loggers
for ((i=0; i < ${#names[*]}; i++));
do
  $TMUX_BIN pipe-pane -t $SESSION_NAME:$(($i+1)) -o "ts | cat >> $TMUX_DIR/$(($i+1))_${names[$i]}.log"
done

# send commands
for ((i=0; i < ${#cmds[*]}; i++));
do
  $TMUX_BIN send-keys -t $SESSION_NAME:$(($i+1)) "cd $SCRIPTPATH;${pre_input};${cmds[$i]}"
done

# identify the index of the init window
init_index=0
for ((i=0; i < ((${#names[*]})); i++));
do
  if [ ${names[$i]} == "$init_window" ]; then
    init_index=$(expr $i + 1)
  fi
done

$TMUX_BIN select-window -t $SESSION_NAME:$init_index

if $attach; then

  if [ -z ${TMUX} ];
  then
    $TMUX_BIN -2 attach-session -t $SESSION_NAME
  else
    tmux detach-client -E "tmux -L mrs a -t $SESSION_NAME"
  fi
else
  echo "The session was started"
  echo "You can later attach by calling:"
  echo "  tmux -L mrs a -t $SESSION_NAME"
fi
