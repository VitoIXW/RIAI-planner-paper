# MIT License

# Copyright (c) 2025 Manuel Domínguez

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def px4_gz_positioning_setup(context, *args, **kwargs):

    num_vehicles = int(LaunchConfiguration('num_vehicles').perform(context))

    actions = []
    for i in range(1, num_vehicles + 1):
        node = Node(
            package='px4_gz_positioning',
            executable='px4_gz_positioning_node',
            name=f'px4_gz_positioning_{i}',
            output='screen',
            parameters=[{
                'gz_topic': f'/model/x500_vision_{i}/pose',
                'px4_topic': f'/px4_{i}/fmu/in/vehicle_visual_odometry',
            }]
        )
        actions.append(node)

    return actions


def generate_launch_description():

    num_vehicles_arg = DeclareLaunchArgument(
        'num_vehicles',
        default_value='1',
        description='Número de vehículos a lanzar'
    )

    ld = LaunchDescription()

    ld.add_action(num_vehicles_arg)
    ld.add_action(OpaqueFunction(function=px4_gz_positioning_setup))

    ld.add_action(Node(
        package='tracking',
        executable='tracking_node',
        name='tracking_1',
        output='screen'
    ))

    ld.add_action(Node(
        package='mission',
        executable='mission_node',
        name='mission_1',
        output='screen'
    ))

    return ld
