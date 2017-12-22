#!/bin/bash

if [ $# != 1 ] ; then
    echo "$(tput setaf 1)parameters error, you should use like this: ./run.sh <mode>$(tput setaf 7)"
    echo "<mode> should be play or observation, default is observation"
    exit -1
fi

if [ $1 != "play" ] && [ $1 != "observation" ]; then
    echo "$(tput setaf 1)parameters error, you should use like this: ./run.sh <mode>$(tput setaf 7)"
    echo "<mode> should be play or observation, default is observation"
    exit -1
fi

echo ""
echo "HOSTNAME"
echo "======="
echo `hostname`

# start roscore if not running
if ! pidof -x "roscore" >/dev/null; then
    echo ""
    echo "ROSCORE STARTING"
    echo "======="
    setsid roscore &
    # while [! pidof -x "roscore" >/dev/null]
    # do
    #     echo "ROSCORE STARTING"
    #     echo "======="
    # done
    # echo ""
    # echo "ROSCORE STARTED"
    # echo "======="
fi

# start rviz if not running
if ! pidof -x "rviz" >/dev/null; then
    echo ""
    echo "RVIZ STARTING"
    echo "======="
    setsid rosrun rviz rviz -d kitti.rviz &
fi

if [ $# -ge 1 ] && [ $1 == "play" ] ; then
    echo ""
    echo "MODE play"
    echo "======="
    python input_velodyne.py play
else
    echo ""
    echo "MODE observation"
    echo "======="
    python input_velodyne.py
fi

