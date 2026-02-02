# bob_launch

`bob_launch` is a powerful meta-launcher for ROS 2 that enables dynamic system orchestration using YAML or JSON configuration files. It allows you to define, compose, and launch complex ROS 2 graphs without writing boilerplate Python launch files.

## High-Level Capabilities

- **Dynamic Orchestration**: Spawn nodes or include other launch files based on external configuration.
- **Composition over Coding**: Define your entire ROS system in simple YAML.
- **Streamlined Workflow**: Use the `launch.sh` wrapper to supply configurations via files, pipes, or direct strings.
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

## Advanced Usage & AI Agent Integration

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

### Mixing Pipes and Arguments
Pipe the main logic while providing a shared parameter file as an argument:
```bash
cat dynamic_nodes.yaml | ros2 run bob_launch launch.sh shared_params.yaml
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

## Technical Design: Why Environment Variables?

Unlike standard ROS launch files that rely solely on `LaunchArguments`, `bob_launch` uses a hybrid approach to enable **Dynamic Generation**.

1.  **The Problem**: ROS `LaunchConfiguration` substitutions are evaluated at **Runtime**. This is too late if you want to use the config data to decide *which* nodes to even create.
2.  **The Solution**: `bob_launch` reads the `BOB_LAUNCH_CONFIG` environment variable during the **Generation Phase**. 
3.  **The Result**: The Python script parses the YAML, validates the structure, and constructs the final `LaunchDescription` before the ROS execution engine starts.

### Environment Variables Reference
| Variable | Description | Default |
| :--- | :--- | :--- |
| `BOB_LAUNCH_CONFIG` | Full path to the YAML/JSON config file (or raw string). | None (Required) |
| `BOB_LAUNCH_AUTOABORT` | If `1`, shuts down the launch if any node exits. | `1` |
