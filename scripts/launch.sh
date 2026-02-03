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
export BOB_LAUNCH_CONFIG=${CONF:-$BOB_LAUNCH_CONFIG}
export BOB_LAUNCH_CONFIG_NODES=${NODESCONF:-$BOB_LAUNCH_CONFIG_NODES}

if [ -t 0 ]; then
  # Stdin is a terminal (No pipe)
  # Usage: launch.sh <config> [<nodes-config>]
  [ -n "$BOB_LAUNCH_CONFIG" ] || usage_exit "Missing config argument!" 1
  ros2 launch bob_launch generic.launch.py
else
  # Stdin is a pipe
  # Usage: cat config.yaml | launch.sh [<nodes-config>]
  temp=$(mktemp --suffix .yaml)
  trap "rm -f $temp 2>/dev/null" EXIT
  cat > "$temp"
  export BOB_LAUNCH_CONFIG=$temp
  # If piped, the first argument is used as BOB_LAUNCH_CONFIG_NODES if provided
  [ -z "$CONF" ] || export BOB_LAUNCH_CONFIG_NODES=$CONF
  ros2 launch bob_launch generic.launch.py
fi
