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

"""This generic ROS launch file spawns Nodes or Launch files from YAML or JSON config."""

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
from launch.actions import OpaqueFunction
from launch_ros.actions import PushRosNamespace
from ament_index_python.packages import get_package_share_directory


def create_launcher(
        launch_file: Union[str, dict],
        launch_arguments: dict = None,
        ns: str = None) -> LaunchDescription:
    """
    Create a LaunchDescription for an included launch file.

    :param launch_file: Path to launch file or dict with package_name and launch_name.
    :param launch_arguments: Arguments to pass to the included launch file.
    :param ns: Namespace to push the included launch items into.
    :return: A LaunchDescription entity.
    """
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


def create_node(args: dict, config_nodes_path: str) -> Node:
    """
    Create a ROS 2 Node from a configuration dictionary.

    :param args: Dictionary containing node parameters (package, executable, name, etc.).
    :param config_nodes_path: Path to a global parameter file to apply.
    :return: A Node entity.
    """
    chars = string.ascii_letters + string.digits
    params = [config_nodes_path] if config_nodes_path else []

    return Node(
        package=args['package'],
        executable=args['executable'],
        name=(args['executable']
              + '_' + ''.join(secrets.choice(chars) for i in range(8))
              if 'name' not in args else args['name']),
        arguments=None if 'arguments' not in args else args['arguments'],
        parameters=params,
        respawn=False if 'respawn' not in args else args['respawn'],
        output='log' if 'output' not in args else args['output'],
        on_exit=None if not int(os.getenv('BOB_LAUNCH_AUTOABORT', '1')) else [
            LogInfo(
                msg=[f'ROS Node {args["executable"]} ended. ',
                     'Stopping everything... ']),
            LogInfo(
                msg=['To disable this behaviour set env variable ',
                     'BOB_LAUNCH_AUTOABORT=0']),
            Shutdown(reason='launch is shutting down')])


def create_entities(config: list, config_nodes_path: str) -> list:
    """
    Parse a list of configurations and create corresponding launch entities.

    :param config: List of dictionaries defining nodes or launch files.
    :param config_nodes_path: Global parameter file to apply to all nodes.
    :return: List of launch entities (Nodes or LaunchDescriptions).
    """
    entities = list()
    for entity in config:
        if 'executable' in entity:
            entities.append(create_node(entity, config_nodes_path))
        elif 'launch_file' in entity:
            entities.append(
                create_launcher(
                    entity['launch_file'],
                    (None if 'launch_args' not in entity
                     else entity['launch_args']),
                    (None if 'launch_ns' not in entity
                     else entity['launch_ns'])))
    return entities


def launch_setup(context, *args, **kwargs):
    """
    Execute the launch setup using the current launch context.

    This function resolves launch arguments and environment variables at runtime
    to dynamically build the system graph.

    :param context: The ROS 2 LaunchContext.
    :return: List of initial launch entities.
    """
    config_nodes_path = context.perform_substitution(
        LaunchConfiguration('config_nodes'))
    launch_config = context.perform_substitution(
        LaunchConfiguration('config'))

    if not launch_config:
        print('[ERROR] No configuration provided! Use \'config:=\' or '
              'set environment variable BOB_LAUNCH_CONFIG',
              file=sys.stderr)
        sys.exit(1)

    try:
        with open(launch_config, 'r') as file:
            launch_config_content = file.read()
    except IOError:
        launch_config_content = launch_config

    initial_entities = []
    try:
        config = json.loads(launch_config_content)
        initial_entities.append(LogInfo(msg='JSON config loaded'))
    except (json.JSONDecodeError, ValueError):
        try:
            config = yaml.safe_load(launch_config_content)
            if isinstance(config, str):
                raise ValueError('Can\'t load YAML!')
            initial_entities.append(LogInfo(msg='YAML config loaded'))
        except (yaml.YAMLError, ValueError):
            print('[ERROR] Config is no file or not JSON/YAML parsable! '
                  'Check the \'config\' argument or BOB_LAUNCH_CONFIG',
                  file=sys.stderr)
            sys.exit(1)

    # Check if config_nodes file exists, otherwise ignore it
    if config_nodes_path and not os.path.isfile(config_nodes_path):
        config_nodes_path = ''

    initial_entities += create_entities(config, config_nodes_path)
    return initial_entities


def generate_launch_description() -> LaunchDescription:
    """
    Generate the top-level LaunchDescription for the package.

    :return: The complete LaunchDescription.
    """
    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=os.getenv('BOB_LAUNCH_CONFIG', ''),
            description='Path to YAML/JSON config file or raw string'),
        DeclareLaunchArgument(
            'config_nodes',
            default_value=os.getenv('BOB_LAUNCH_CONFIG_NODES', ''),
            description='Optional global parameter file'),
        OpaqueFunction(function=launch_setup)
    ])
