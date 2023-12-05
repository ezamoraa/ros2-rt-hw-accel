cmake_minimum_required(VERSION 3.5)
project(rt_hw_accel_demo)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unknown-pragmas)
endif()

find_package(ament_vitis QUIET)
find_package(ament_cmake_auto REQUIRED)
find_package(rt_hw_accel_msgs REQUIRED)
ament_auto_find_build_dependencies()

include_directories(include)

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")

# LTTng flame chart and graphs
# see https://archive.eclipse.org/tracecompass/doc/stable/org.eclipse.tracecompass.doc.user/LTTng-UST-Analyses.html#Flame_Chart_View
set(CMAKE_CXX_FLAGS "-g -O2 -finstrument-functions")

if (DEFINED ROS_VITIS)
  find_package(vitis_common)
  find_package(OpenCL REQUIRED)

  #####################
  # Components
  #####################

  # cpu_harris_node component, dependency with hls_stream.h Vitis-related header
  ament_auto_add_library(cpu_harris_node SHARED src/cpu_harris_node.cpp)
  ament_target_dependencies(cpu_harris_node tracetools_image_pipeline rt_hw_accel_msgs)
  target_include_directories(cpu_harris_node PUBLIC
    # $<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/include/vitis_common>
    $ENV{XILINX_HLS}/common/technology/autopilot
    $ENV{XILINX_HLS}/include)
  target_compile_definitions(cpu_harris_node PRIVATE "COMPOSITION_BUILDING_DLL")
  rclcpp_components_register_nodes(cpu_harris_node "rt_hw_accel_demo::HarrisNodeCPU")
  set(node_plugins "${node_plugins}rt_hw_accel_demo::HarrisNodeCPU;$<TARGET_FILE:cpu_harris_node>\n")

  # fpga_harris_node component (with OpenCL)
  ament_auto_add_library(fpga_harris_node SHARED src/fpga_harris_node.cpp)
  ament_target_dependencies(fpga_harris_node tracetools_image_pipeline rt_hw_accel_msgs)
  target_include_directories(fpga_harris_node PUBLIC
    # $<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/include/vitis_common>
    $ENV{XILINX_HLS}/common/technology/autopilot
    $ENV{XILINX_HLS}/include)
  target_link_libraries(fpga_harris_node ${OpenCL_LIBRARY})
  target_compile_definitions(fpga_harris_node PRIVATE "COMPOSITION_BUILDING_DLL")
  rclcpp_components_register_nodes(fpga_harris_node "rt_hw_accel_demo::HarrisNodeFPGA")
  set(node_plugins "${node_plugins}rt_hw_accel_demo::HarrisNodeFPGA;$<TARGET_FILE:fpga_harris_node>\n")

  # cornerHarris_accel kernel synthesis
  vitis_acceleration_kernel(
    NAME cornerHarris_accel
    FILE src/kernels/xf_harris_accel.cpp
    CONFIG cfg/kr260_fpga_graph.cfg
    INCLUDE
      include/image_proc
      ${CMAKE_INSTALL_PREFIX}/include
    TYPE hw
  )

  #####################
  # Applications
  #####################

  # cpu_graph
  # Define the APU target
  add_executable(cpu_graph src/cpu_graph.cpp)
  target_link_libraries(cpu_graph cpu_harris_node)
  ament_target_dependencies(cpu_graph rclcpp class_loader image_proc)

  # fpga_graph
  # Place and route FPGA kernels
  vitis_link_kernel(
    OUTPUT fpga_graph
    KERNELS cornerHarris_accel
    CONFIG cfg/kr260_fpga_graph.cfg
  )
  # Define the APU target
  add_executable(fpga_graph src/fpga_graph.cpp)
  target_link_libraries(fpga_graph fpga_harris_node)
  ament_target_dependencies(fpga_graph rclcpp class_loader)

  set(install_targets cpu_graph fpga_graph)
endif()  # ROS_VITIS

install(
  TARGETS ${install_targets}
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(INSTALL_TO_SHARE launch)
