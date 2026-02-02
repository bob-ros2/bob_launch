#!/bin/bash

CONF=$1
NODESCONF=$2

usage_exit() { # [<msg> [<rc>]]
  [ -z "$1" ] || echo "$1"
  cat <<EOF
  Wrapper script to start ROS package bob_launch generic.launch.py with the given parameter.
  This script can also read YAML launch <config> from stdin.

  Usage: $(basename $0) <config> [<nodes-config>]
         cat *.yaml | $(basename $0) [<nodes-config>]

EOF
exit $2
}

[ "$CONF" != "-h" ] || usage_exit

export BOB_LAUNCH_AUTOABORT=${BOB_LAUNCH_AUTOABORT:-1}

if [ -t 0 ]; then
  # Stdin is a terminal (No pipe)
  # Usage: launch.sh <config> [<nodes-config>]
  [ -n "$CONF" ] || usage_exit "Missing config argument!" 1
  export BOB_LAUNCH_CONFIG=$CONF
  [ -z "$NODESCONF" ] || NODESCONF="config_nodes:=$NODESCONF"
  ros2 launch bob_launch generic.launch.py $NODESCONF
else
  # Stdin is a pipe
  # Usage: cat config.yaml | launch.sh [<nodes-config>]
  temp=$(mktemp --suffix .yaml)
  trap "rm -f $temp 2>/dev/null" EXIT
  cat > "$temp"
  export BOB_LAUNCH_CONFIG=$temp
  # If piped, the first argument (if any) is the nodes-config
  [ -z "$CONF" ] || CONF="config_nodes:=$CONF"
  ros2 launch bob_launch generic.launch.py $CONF
fi
