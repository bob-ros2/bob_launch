#!/bin/sh

CONF=$1
NODESCONF=$2
LARGS=--debug

usage_exit() { # [<msg> [<rc>]]
    [ -z "$1" ] || echo "$1"
    cat <<EOF
Wrapper script to start ROS package bob_launch generic.launch.py with the given parameter.
This script can also read YAML launch <config> from stdin.

Usage: $(basename $0) [<config> [<nodes-config>]]
       cat *.yaml | $(basename $0) [<nodes-config>]

EOF
exit $2
}

[ "$CONF" != "-h" ] || usage_exit

export BOB_LAUNCH_AUTOABORT=${BOB_LAUNCH_AUTOABORT:-1}
export BOB_LAUNCH_CONFIG=$CONF

if [ -t 0 ]; then
    [ -n "$BOB_LAUNCH_CONFIG" ] || usage_exit "Missing Input!" 1
    [ -z "$NODESCONF" ] || NODESCONF="config_nodes:=$NODESCONF"
    ros2 launch bob_launch generic.launch.py $NODESCONF $LARGS
else
    temp=$(mktemp --suffix .yaml)
    trap "rm -f $temp 2>/dev/null" EXIT
    while IFS= read l; do
        printf "%s\n" "$l" >> $temp
    done
    export BOB_LAUNCH_CONFIG=$temp
    [ -z "$CONF" ] || CONF="config_nodes:=$CONF"
    ros2 launch bob_launch generic.launch.py $CONF $LARGS
fi