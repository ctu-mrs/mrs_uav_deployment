#!/bin/bash

MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`

# Get RC file 
PNAME=$( ps -p "$$" -o comm= )
SNAME=$( echo "$SHELL" | grep -Eo '[^/]+/?$' )
if [ "$PNAME" != "$SNAME" ]; then
  exec "$SHELL" -i "$0" "$@"
  exit "$?"
else
  case $- in
    *i*) ;;
    *)
      exec "$SHELL" -i "$0" "$@"
      exit "$?"
      ;;
  esac
  source ~/."$SNAME"rc
fi

RCFILE=~/."$SNAME"rc

cd $MY_PATH

# Get the broadcast ip address
echo "Determining broadcast IP address"
if var1=$(ifconfig | grep -A1 wlan0:); then

  ip=$(echo $var1 |sed -n -e 's/^.*broadcast //p')

  if [ -z "$ip" ]; then
    success=false
    echo "Could not determine the broadcast IP address"
  else
    success=true
    echo "Broadcast address:"
    echo "$ip"
  fi

else
  success=false
  echo "Could not find the interface wlan0"
fi

echo "Setting BROADCAST_IP variable in RC file"

if var1=$(cat ~/.bashrc | grep BROADCAST_IP); then

  if [ -x "$(whereis nvim | awk '{print $2}')" ]; then
    VIM_BIN="$(whereis nvim | awk '{print $2}')"
    HEADLESS="--headless"
  elif [ -x "$(whereis vim | awk '{print $2}')" ]; then
    VIM_BIN="$(whereis vim | awk '{print $2}')"
    HEADLESS=""
  fi

$VIM_BIN -u "$GIT_PATH/linux-setup/submodules/profile_manager/epigen/epigen.vimrc" $HEADLESS -Ens -c "%g/BROADCAST_IP.*/norm ^/BROADCAST_IPc\$BROADCAST_IP=$ip" -c "wqa" -- ~/.bashrc
echo "BROADCAST_IP changed to $ip"

else 
  ~/git/uav_core/miscellaneous/scripts/get_set_rc_variable.sh "$HOME/.bashrc" "BROADCAST_IP" "$ip" "The broadcast IP address used for Nimbro network transport"
  echo "BROADCAST_IP set to $ip"
fi

source ~/.bashrc
