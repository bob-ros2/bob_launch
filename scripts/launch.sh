#!/bin/bash

CONF=$1
NODESCONF=$2

[ -z "$NODESCONF" ] || NODESCONF="config_nodes:=$NODESCONF"

export BOB_LAUNCH_AUTOABORT=${BOB_LAUNCH_AUTOABORT:-1}
export BOB_LAUNCH_CONFIG=$CONF
ros2 launch bob_launch generic.launch.py $NODESCONF
