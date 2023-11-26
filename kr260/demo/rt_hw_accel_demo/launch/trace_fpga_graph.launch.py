# Copyright 2022 Víctor Mayoral-Vilches
# All rights reserved.

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from tracetools_launch.action import Trace
from tracetools_trace.tools.names import DEFAULT_EVENTS_ROS
from tracetools_trace.tools.names import DEFAULT_EVENTS_KERNEL
from tracetools_trace.tools.names import DEFAULT_CONTEXT


def generate_launch_description():
     # Trace
    trace = Trace(
        session_name="trace_rt_hw_accel_demo_fpga",
        events_ust=[
            "ros2_image_pipeline:*",
        ]
        + DEFAULT_EVENTS_ROS,
        context_fields={
                'kernel': [],
                'userspace': ['vpid', 'vtid', 'procname'],
        },
        # events_kernel=DEFAULT_EVENTS_KERNEL,
    )

    # Use a multi-threaded executor defined in the Node
    fpga_graph_node = Node(
        package="rt_hw_accel_demo",
        executable="fpga_graph",
        name="fpga_graph",
        remappings=[
            ("image", "/camera/image_raw"),
            ("camera_info", "/camera/camera_info"),
        ],
        parameters=[
            {
                "scale_height": 2.0,
                "scale_width": 2.0,
            }
        ],
    )

    return LaunchDescription([
        # LTTng tracing
        trace,
        # image pipeline
        # perception_container
        fpga_graph_node
    ])