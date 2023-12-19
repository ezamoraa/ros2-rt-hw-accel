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
import sys
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
        session_name="trace_rt_hw_accel_demo_cpu",
        events_ust=[
            "ros2_image_pipeline:*",
            # "lttng_ust_cyg_profile*",
            # "lttng_ust_statedump*",
            # "liblttng-ust-libc-wrapper",
        ]
        + DEFAULT_EVENTS_ROS,
        context_fields={
                'kernel': [],
                'userspace': ['vpid', 'vtid', 'procname'],
        },
        # events_kernel=DEFAULT_EVENTS_KERNEL,
        # context_names=DEFAULT_CONTEXT,
    )

    node_args = []
    for arg in sys.argv:
        if arg.startswith("sched:="):
            arg_val = str(arg.split(":=")[1])
            node_args.extend(['--sched', arg_val])
        elif arg.startswith("priority:="):
            arg_val = str(arg.split(":=")[1])
            node_args.extend(['--priority', arg_val])

    cpu_graph_node = Node(
       package="rt_hw_accel_demo",
       executable="cpu_graph",
       name="cpu_graph",
       remappings=[
           ("image", "/camera/image_raw"),
           ("camera_info", "/camera/camera_info"),
       ],
       arguments=node_args,
    )

    # rt_hw_accel_demo_container = ComposableNodeContainer(
    #     name="rt_hw_accel_demo_container",
    #     namespace="",
    #     package="rclcpp_components",
    #     executable="component_container",
    #     composable_node_descriptions=[
    #         ComposableNode(
    #             package="rt_hw_accel_demo",
    #             plugin="rt_hw_accel_demo::HarrisNodeCPU",
    #             name="cpu_harris_node",
    #             remappings=[
    #                 ("image", "/camera/image_raw"),
    #                 ("camera_info", "/camera/camera_info"),
    #             ],
    #         ),
    #     ],
    #     output="screen",
    # )

    return LaunchDescription([
        # LTTng tracing
        trace,
        # image pipeline
        cpu_graph_node,
        # rt_hw_accel_demo_container
    ])
