/* Modification Copyright (c) 2023, Acceleration Robotics®
   Author: Alejandra Martínez Fariña <alex@accelerationrobotics.com>
   Based on:
*/

// Copyright 2022 Víctor Mayoral-Vilches
// All rights reserved.

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef IMAGE_PROC__HARRIS_FPGA_HPP_
#define IMAGE_PROC__HARRIS_FPGA_HPP_

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <ament_index_cpp/get_resource.hpp>

#include <cstring>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <thread>

#include <vitis_common/common/ros_opencl_120.hpp>
#include "opencv2/imgproc.hpp"

#include "rt_hw_accel_msgs/msg/point.hpp"
#include "rt_hw_accel_msgs/msg/point_array.hpp"

namespace rt_hw_accel_demo
{
  using Corner = rt_hw_accel_msgs::msg::Point;
  using CornersMessage = rt_hw_accel_msgs::msg::PointArray;
  using CornersPublisher = rclcpp::Publisher<CornersMessage>;

class HarrisNodeFPGA
  : public rclcpp::Node
{

public:
  explicit HarrisNodeFPGA(const rclcpp::NodeOptions &);

protected:
  image_transport::Publisher pub_image_;
  image_transport::Subscriber sub_image_;
  CornersPublisher::SharedPtr pub_corners_;

  int myHarris_qualityLevel;
  int max_qualityLevel;
  int blockSize_harris;
  int apertureSize;

  cl::Kernel* krnl_;
  cl::Context* context_;
  cl::CommandQueue* queue_;

  std::mutex connect_mutex_;

  size_t get_msg_size(sensor_msgs::msg::Image::ConstSharedPtr image_msg);
  size_t get_msg_size(sensor_msgs::msg::CameraInfo::ConstSharedPtr info_msg);

  void imageCb(sensor_msgs::msg::Image::ConstSharedPtr image_msg);

  void harrisImage_fpga(const cv::Mat& in_img, cv::Mat& harris_img,
                        CornersMessage& corners_msg) const;
};

}  // namespace rt_hw_accel_demo

#endif  // IMAGE_PROC__HARRIS_FPGA_HPP_
