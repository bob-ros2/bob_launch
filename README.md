# ROS Package [bob_launch](https://github.com/bob-ros2/bob_launch)
This repository is part of Bob's ROS Packages.

## Launch File generic.launch.py

This generic ROS launch file is able to spawn Ros Nodes or other ROS Launch files from a YAML configuration file.

Since the ROS Launch System has the property of working context-related, this launch file must be started in a special way to be able to use a config to dynamically generate the final launch description.

Ultimately, this kind of working simplifies the startup configuration compared to the common ROS startup files, which are rather inconvenient for hassle free routing of huge amounts of ROS topics.

### Environment Variables
* `BOB_LAUNCH_CONFIG` - Path to the YAML config.
* `BOB_LAUNCH_AUTOABORT` - Wether to stop launch script if one of the nodes exits. This is useful when running in a docker container. If variable is not existing the default is: 1

### Launch Arguments:
* `config_nodes` - An optional Ros Nodes parameter file can be provided which will be passed to all contained nodes

### Starting generic Launch file
```bash
# start everything from config
BOB_LAUNCH_CONFIG=/path/to/config.yaml ros2 launch bob_launch generic.launch.py

# start everything from config 
# in addition pass a typical ROS parameter config file to all contained ros nodes within our config
BOB_LAUNCH_CONFIG=/path/to/config.yaml ros2 launch bob_launch generic.launch.py config_nodes:=nodes_config.yaml
```

### Example config starting Nodes
```YAML
# image view node
- name: image_view
  package: image_view
  executable: image_view
  arguments:
    - --ros-args
    - -p
    - image:=/image_topic
    - -r
    - __ns:=/bob_v2

# start usb_cam node with a nodes parameter file
# sudo apt-get install ros-<ros2-distro>-usb-cam
- name: usb_cam
  package: usb_cam
  executable: usb_cam_node_exe
  arguments:
    - --ros-args
    - --params-file
    - /path/to/nodes_params.yaml
```

### Example config starting Launch file
```YAML
#  launch example from a package with push to namespace
- launch_file: 
    package_name: bob_llama_cpp
    launch_name: llm.launch.py
  # optional
  launch_args:
    terminal: 'true'
    ns: ''
  # optional
  launch_ns: /llmtest/a_sub_group

# launch example using just a path
- launch_file: /path/to/my.launch.py

# start in addition rqt nodes
- name: rqt_graph
  package: rqt_graph
  executable: rqt_graph
```

### Example using YAML features Anchors and Aliases
The YAML format has features to reuse variables or even complex sub structures across the YAML file. See the YAML documentation for further information.
```YAML
# Anchors can be defined like this. 
# That works because the launch script identifies nodes 
# or launch configs from their properties names.
- &ns __ns:=/bob_v2

- name: voice
  package: rosspeaks
  executable: speak
  arguments:
    - --ros-args
    - -r
    - *ns

- name: llm
  package: bob_llama_cpp
  executable: llm
  arguments:
    - --ros-args
    - -r
    - llm_out:=speak
    - -r
    - *ns
```

## Helper Script launch.sh
```bash
# help output
ros2 run bob_launch launch.sh -h
Wrapper script to start ROS package bob_launch generic.launch.py with the given parameter.
This script can also read YAML launch <config> from stdin.

Usage: launch.sh [<config> [<nodes-config>]]
       cat *.yaml | launch.sh [<nodes-config>]
```
```bash
# start with helper script
# by default the launch will abort if one node exists
ros2 run bob_launch launch.sh /path/to/launch.yaml nodes_config.yaml

# start with helper script
# don't abort everything if one node is exiting
export BOB_LAUNCH_AUTOABORT=0
ros2 run bob_launch launch.sh /path/to/launch.yaml

# start with helper script and launch config from stdin
cat *.yaml | ros2 run bob_launch launch.sh /path/to/nodes_config.yaml
```
