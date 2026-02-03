# ROS Package [bob_launch](https://github.com/bob-ros2/bob_launch)

`bob_launch` is a powerful meta-launcher for ROS 2 that enables dynamic system orchestration using YAML or JSON configuration files. It allows you to define, compose, and launch complex ROS 2 graphs without writing boilerplate Python launch files.

## High-Level Capabilities

- **Dynamic Orchestration**: Spawn nodes or include other launch files based on external configuration.
- **Composition over Coding**: Define your entire ROS system in simple YAML.
- **Streamlined Workflow**: Use the `launch.sh` wrapper or native ROS 2 launch arguments.
- **Auto-Abort Protection**: Optionally shut down the entire launch tree if any critical node exits (ideal for Docker/CI).
- **Global Parameters**: Inject a shared parameter file into all nodes launched within a configuration.

## Quick Start: `launch.sh`

The `launch.sh` script is the primary entry point. It simplifies execution by handling environment variables and temporary file management for you.

### 1. Launch from a file
```bash
ros2 run bob_launch launch.sh my_config.yaml
```

### 2. Launch with a global parameter file
```bash
ros2 run bob_launch launch.sh my_config.yaml global_params.yaml
```

### 3. Launch from a JSON config file
```bash
ros2 run bob_launch launch.sh my_config.json
```

## Native ROS 2 Usage (Preferred)

You can call the launch file directly using standard ROS 2 launch arguments:

```bash
ros2 launch bob_launch generic.launch.py config:=my_config.yaml
```

To pass an optional global node parameter file:
```bash
ros2 launch bob_launch generic.launch.py config:=my_config.yaml config_nodes:=node_params.yaml
```

*Note: You can also use the environment variable `BOB_LAUNCH_CONFIG=./my_config.yaml` as an alternative to the `config:=` argument.*

## Advanced Usage

`bob_launch` is designed to be highly scriptable, making it an ideal tool for AI Agents needing to spawn ROS components on the fly.

### Pipe a String Directly (Dynamic Spawning)
An agent can generate a YAML string and pipe it directly to the launcher without creating a persistent file:
```bash
echo "- name: talker
  package: demo_nodes_cpp
  executable: talker" | ros2 run bob_launch launch.sh
```

### Multi-File Composition
You can quickly composite different system "layers" (e.g., base drivers + perception + mission logic) by concatenating files into the pipe:
```bash
cat base_robot.yaml nav2_stack.yaml custom_logic.yaml | ros2 run bob_launch launch.sh
```

## Configuration Schema

### Spawning Nodes
```yaml
- name: usb_cam               # Optional: defaults to executable_name + random suffix
  package: usb_cam
  executable: usb_cam_node_exe
  arguments:                  # Optional: list of CLI arguments
    - --ros-args
    - --params-file
    - /path/to/params.yaml
  respawn: false              # Optional: default is false
  output: log                 # Optional: 'log' or 'screen'
```

### Including Launch Files
```yaml
- launch_file: 
    package_name: bob_llm
    launch_name: llm.launch.py
  launch_args:                # Optional: arguments passed to the included launch
    model: 'gpt-4'
  launch_ns: /llm_group       # Optional: pushes everything into this namespace
```

### Advanced YAML: Anchors & Aliases
Great for reusing common namespaces or remappings across many nodes:
```yaml
- &common_ns __ns:=/robot_1

- name: drive_node
  package: my_pkg
  executable: driver
  arguments: ["--ros-args", "-r", *common_ns]

- name: sensor_node
  package: my_pkg
  executable: sensor
  arguments: ["--ros-args", "-r", *common_ns]
```

### Launch Arguments & Environment Variables
| Argument / Variable | Description | Default |
| :--- | :--- | :--- |
| `config` | Path to YAML/JSON config file or raw string. | `BOB_LAUNCH_CONFIG` |
| `config_nodes` | Path to an optional global parameter file. | `BOB_LAUNCH_CONFIG_NODES` |
| `BOB_LAUNCH_AUTOABORT` | Environment variable: if `1`, shuts down if any node exits. | `1` |
