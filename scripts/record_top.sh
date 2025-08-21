#!/bin/bash

INTERVAL=0.5
echo "Recording ps output every $INTERVAL seconds"
echo "Saving to top.log"
while true; do
    echo "Timestamp: $(date '+%Y-%m-%d %H:%M:%S')"
    ps -eo pid,ppid,%cpu,%mem,cmd --sort=-%cpu
    sleep $INTERVAL
done > top.log
