# ROS Package [bob_launch](https://github.com/bob-ros2/bob_launch)
[![ROS 2 CI](https://github.com/bob-ros2/bob_launch/actions/workflows/ros2_ci.yml/badge.svg)](https://github.com/bob-ros2/bob_launch/actions/workflows/ros2_ci.yml)
[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

`bob_launch` is a powerful meta-launcher for ROS 2 that enables dynamic system orchestration using YAML or JSON configuration files. It allows you to define, compose, and launch complex ROS 2 graphs without writing boilerplate Python launch files.

## High-Level Capabilities

- **Dynamic Orchestration**: Spawn nodes or include other launch files based on external configuration.
- **Composition over Coding**: Define your entire ROS system in simple YAML.
- **Streamlined Workflow**: Use the `launch.sh` wrapper or native ROS 2 launch arguments.
- **Auto-Abort Protection**: Optionally shut down the entire launch tree if any critical node exits (ideal for Docker/CI).
- **Global Parameters**: Inject a shared parameter file into all nodes launched within a configuration.
- **Path Placeholders**: Use `//PKGSHARE/` (current node package) or `//PKGSHARE:pkg/` to dynamically resolve ROS 2 package share directories in parameters and arguments.
- **Environment Substitution**: Support for shell-like `${VAR}` or `${VAR:-default}` syntax in all configuration fields and relevant launch arguments.

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

## Fast Execution: `launch.sh`

The `launch.sh` script is a convenient wrapper that simplifies execution by handling environment variables and temporary file management for you.

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

### Dynamic Substitution & Placeholders
`bob_launch` supports powerful dynamic string substitution in all configuration fields and launcher arguments (like `config:=` and `config_nodes:=`).

#### Environment Variables (`${VAR}`)
Supports shell-like syntax for dynamic values:
- `${MYVAR}`: Replaces with the value or an empty string.
- `${MYVAR:-/default/path/}`: Replaces with the value or the specified default if unset.

#### Path Placeholders (`//PKGSHARE`)
Avoid hardcoded absolute paths by dynamically resolving ROS 2 package share directories:
- `//PKGSHARE/`: Resolves to the share directory of the package defined for the current entity (implicit).
- `//PKGSHARE:pkg_name/`: Resolves to the share directory of `pkg_name` (explicit).

#### Combination & CLI Usage
These mechanisms can be combined and even used directly in the `ros2 launch` command arguments:

```bash
ros2 launch bob_launch generic.launch.py \
  config:='//PKGSHARE:my_pkg/config/${ROBOT_ENV:-dev}.yaml'
```

To disable environment substitution, set `BOB_SUBSTITUTE_ENV_VARS=0`.

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
  prefix: 'xterm -e'          # Optional: runs node in xterm (or gdb, nice, etc.)
  parameters:                 # Optional: inline node parameters
    background_r: 255
    background_g: 0
    background_b: 0
    scalar_param: 1.5
    some_string: "hello bob"
  remappings:                 # Optional: list of pairs [old, new] or dictionary
    - ["input_topic", "/bot/input_topic"]
    - ["internal/data", "~/input_topic"]
  output: log                 # Optional: 'log' or 'screen'
```

### Topic Remappings
Nodes can be launched with custom topic remappings using the `remappings:` field. You can provide them as a **list of pairs** (recommended for dynamic remappings) or as a **dictionary**.

```yaml
- name: my_node
  package: my_pkg
  executable: my_exe
  remappings:
    - ["/old/chatter", "/new/chatter"]  # List of pairs [from, to]
    - ["~/status", "${STATUS_TOPIC}"]   # List format supports placeholders in both fields
    # OR Dictionary format:
    # chatter: /global/chatter          # Note: placeholders work in values only
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
| `BOB_SUBSTITUTE_ENV_VARS` | Environment variable: if `1`, resolves `${VAR}` placeholders in config. | `1` |
