# Copyright 2026 BobRos
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""This generic ROS launch file spawns Nodes or Launch files from YAML config."""
import os
import sys
import json
import yaml
import string
import secrets

from typing import Union
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.actions import Shutdown
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace
from ament_index_python.packages import get_package_share_directory


def create_launcher(
        launch_file: Union[str, dict],
        launch_arguments: dict = None,
        ns: str = None) -> LaunchDescription:

    if isinstance(launch_file, dict):
        file = os.path.join(
            get_package_share_directory(launch_file['package_name']),
            'launch', launch_file['launch_name'])
    else:
        file = launch_file

    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([file]),
        launch_arguments=(None if not launch_arguments
                          else launch_arguments.items()))
    if ns:
        description = GroupAction(
            actions=[
                PushRosNamespace(ns),
                description])
    return description


def create_node(args: dict, config_nodes: LaunchConfiguration) -> Node:
    chars = string.ascii_letters + string.digits
    return Node(
        package=args['package'],
        executable=args['executable'],
        name=(args['executable']
              + '_' + ''.join(secrets.choice(chars) for i in range(8))
              if 'name' not in args else args['name']),
        arguments=None if 'arguments' not in args else args['arguments'],
        parameters=[config_nodes],
        respawn=False if 'respawn' not in args else args['respawn'],
        output='log' if 'output' not in args else args['output'],
        on_exit=None if not int(os.getenv('BOB_LAUNCH_AUTOABORT', '1')) else [
            LogInfo(
                msg=[f"ROS Node {args['executable']} ended. "
                     "Stopping everything... "]),
            LogInfo(
                msg=["To disable this behaviour set env variable "
                     "BOB_LAUNCH_AUTOABORT=0"]),
            Shutdown(reason='launch is shutting down')])


def create_entities(config: list, config_nodes: LaunchConfiguration) -> list:
    entities = list()
    for entity in config:
        if 'executable' in entity:
            entities.append(create_node(entity, config_nodes))
        elif 'launch_file' in entity:
            entities.append(
                create_launcher(
                    entity['launch_file'],
                    (None if 'launch_args' not in entity
                     else entity['launch_args']),
                    (None if 'launch_ns' not in entity
                     else entity['launch_ns'])))
    return entities


def generate_launch_description() -> LaunchDescription:

    launch_config = os.getenv('BOB_LAUNCH_CONFIG', '')

    if not launch_config:
        print("[ERROR] Environment variable BOB_LAUNCH_CONFIG not set!",
              file=sys.stderr)
        sys.exit(1)

    initial_entities = [
        DeclareLaunchArgument('config_nodes',
                              default_value='')
    ]

    try:
        with open(launch_config, 'r') as file:
            launch_config = file.read()
    except IOError:
        pass

    try:
        config = json.loads(launch_config)
        initial_entities.append(
            LogInfo(msg="JSON config loaded"))
    except (json.JSONDecodeError, ValueError):
        try:
            config = yaml.safe_load(launch_config)
            if isinstance(config, str):
                raise ValueError("Can't load YAML!")
            initial_entities.append(
                LogInfo(msg="YAML config loaded"))
        except (yaml.YAMLError, ValueError):
            print("[ERROR] Config is no file or not JSON/YAML parsable! "
                  "Check the environment variable BOB_LAUNCH_CONFIG",
                  file=sys.stderr)
            sys.exit(1)
    finally:
        initial_entities += create_entities(
            config, LaunchConfiguration('config_nodes'))

    return LaunchDescription(initial_entities)
