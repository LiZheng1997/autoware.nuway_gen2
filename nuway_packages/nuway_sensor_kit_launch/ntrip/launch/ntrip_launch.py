"""Launch the NTRIP client.

Credentials come from the environment, never from the repository:

    export NTRIP_USER='<your AUSCORS username>'
    read -s NTRIP_PASS && export NTRIP_PASS

NTRIP_HOST and NTRIP_MOUNTPOINT may also be set to override the defaults in
config/ntrip-param.yaml, which point at AUSCORS and its nearest mountpoint to
UWA. Parameters listed after the file override the values in it.
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    log_level = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='logging level for the ntrip client')

    params_file = Path(get_package_share_directory('ntrip'), 'config', 'ntrip-param.yaml')

    overrides = {}
    for env_name, param_name in (('NTRIP_USER', 'username'),
                                 ('NTRIP_PASS', 'password'),
                                 ('NTRIP_HOST', 'host'),
                                 ('NTRIP_MOUNTPOINT', 'mountpoint')):
        value = os.environ.get(env_name)
        if value:
            overrides[param_name] = value

    ntrip_node = Node(
        package='ntrip',
        executable='ntrip',
        name='ntrip_client',
        output='screen',
        parameters=[params_file] + ([overrides] if overrides else []),
        remappings=[
            ('nmea', 'nmea'),   # GGA in, published by the GNSS driver
            ('rtcm', 'rtcm'),   # corrections out, consumed by the GNSS driver
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )

    return LaunchDescription([log_level, ntrip_node])
