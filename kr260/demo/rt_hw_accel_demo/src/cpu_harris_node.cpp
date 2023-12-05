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

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <memory>
#include <mutex>
#include <vector>

#include "tracetools_image_pipeline/tracetools.h"
#include "cpu_harris_node.hpp"
#include "xf_ocv_ref.hpp"
#include <rclcpp/serialization.hpp>

#define FILTER_WIDTH 3
#define BLOCK_WIDTH 3
#define NMS_RADIUS 1
#define MAXCORNERS 256 // 1024

namespace rt_hw_accel_demo
{

HarrisNodeCPU::HarrisNodeCPU(const rclcpp::NodeOptions & options)
: rclcpp::Node("HarrisNodeCPU", options)
{
  // Create image pub
  pub_image_ = image_transport::create_publisher(this, "harris");
  // Create image sub
  sub_image_ = image_transport::create_subscription(
    this, "image", std::bind(&HarrisNodeCPU::imageCb, this, std::placeholders::_1), "raw");
  // Create corners pub
  pub_corners_ = this->create_publisher<CornersMessage>("/microROS/corners", 10);

  myHarris_qualityLevel = this->declare_parameter<int>("myHarris_qualityLevel", 50);
  max_qualityLevel = this->declare_parameter<int>("max_qualityLevel", 100);
  blockSize_harris = this->declare_parameter<int>("blockSize_harris", 3);
  apertureSize = this->declare_parameter<int>("apertureSize", 3);
  do_pub_corners = this->declare_parameter<bool>("pubCorners", true);
  do_pub_image = this->declare_parameter<bool>("pubImage", true);
}

size_t HarrisNodeCPU::get_msg_size(sensor_msgs::msg::Image::ConstSharedPtr image_msg){
  //Serialize the Image and CameraInfo messages
  rclcpp::SerializedMessage serialized_data_img;
  rclcpp::Serialization<sensor_msgs::msg::Image> image_serialization;
  const void* image_ptr = reinterpret_cast<const void*>(image_msg.get());
  image_serialization.serialize_message(image_ptr, &serialized_data_img);
  size_t image_msg_size = serialized_data_img.size();
  return image_msg_size;
}

size_t HarrisNodeCPU::get_msg_size(sensor_msgs::msg::CameraInfo::ConstSharedPtr info_msg){
  rclcpp::SerializedMessage serialized_data_info;
  rclcpp::Serialization<sensor_msgs::msg::CameraInfo> info_serialization;
  const void* info_ptr = reinterpret_cast<const void*>(info_msg.get());
  info_serialization.serialize_message(info_ptr, &serialized_data_info);
  size_t info_msg_size = serialized_data_info.size();
  return info_msg_size;
}

void HarrisNodeCPU::harrisImage(const cv::Mat& in_img,
                                cv::Mat& harris_img,
                                CornersMessage& corners_msg) const
{
  cv::Mat ocv_out_img, img_gray;
  cv::RNG rng(12345);

  float Th;
  if (FILTER_WIDTH == 3) {
      Th = 30532960.00;
  } else if (FILTER_WIDTH == 5) {
      Th = 902753878016.0;
  } else if (FILTER_WIDTH == 7) {
      Th = 41151168289701888.000000;
  }

  // Convert rgb into grayscale
  cvtColor(in_img, img_gray, CV_BGR2GRAY);

  ocv_out_img.create(img_gray.rows, img_gray.cols, CV_8U); // create memory for opencv output image  // NOLINT
  ocv_ref(img_gray, ocv_out_img, Th);

  if (do_pub_corners) {
    // Populate corners_msg with the detected corners (max MAXCORNERS)
    Corner corner;
    corners_msg.points.reserve(MAXCORNERS); // Allocate N entries

    for (int j = 1; j < ocv_out_img.rows - 1; j++) {
        for (int i = 1; i < ocv_out_img.cols - 1; i++) {
            if ((int) ocv_out_img.at<unsigned char>(j, i)) {
              corner.x = i;
              corner.y = j;
              corners_msg.points.push_back(corner);
              if (corners_msg.points.size() >= MAXCORNERS) {
                break;
              }
            }
        }
    }
  }

  if (do_pub_image) {
    /// Drawing a circle around corners
    harris_img = in_img.clone();

    for (int j = 1; j < ocv_out_img.rows - 1; j++) {
        for (int i = 1; i < ocv_out_img.cols - 1; i++) {
            if ((int) ocv_out_img.at<unsigned char>(j, i)) {
              cv::circle(
                harris_img,
                cv::Point(i, j),
                4,
                cv::Scalar(
                  rng.uniform(0, 255),
                  rng.uniform(0, 255),
                  rng.uniform(0, 255)),
                -1, 8, 0);
            }
        }
    }
  }
}

void HarrisNodeCPU::imageCb(sensor_msgs::msg::Image::ConstSharedPtr image_msg)
{
  TRACEPOINT(
    image_proc_harris_cb_init,
    static_cast<const void *>(this),
    static_cast<const void *>(&(*image_msg)),
    nullptr,
    image_msg->header.stamp.nanosec,
    image_msg->header.stamp.sec,
    get_msg_size(image_msg),
    0);

  // if (pub_image_.getNumSubscribers() < 1) {
  //   TRACEPOINT(
  //     image_proc_harris_cb_fini,
  //     static_cast<const void *>(this),
  //     static_cast<const void *>(&(*image_msg)),
  //     nullptr,
  //     image_msg->header.stamp.nanosec,
  //     image_msg->header.stamp.sec,
  //     get_msg_size(image_msg),
  //     0);
  //   return;
  // }

  cv_bridge::CvImagePtr cv_ptr;

  try {
    cv_ptr = cv_bridge::toCvCopy(image_msg);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    TRACEPOINT(
      image_proc_harris_cb_fini,
      static_cast<const void *>(this),
      static_cast<const void *>(&(*image_msg)),
      nullptr,
      image_msg->header.stamp.nanosec,
      image_msg->header.stamp.sec,
      get_msg_size(image_msg),
      0);
    return;
  }

  TRACEPOINT(
    image_proc_harris_init,
    static_cast<const void *>(this),
    static_cast<const void *>(&(*image_msg)),
    nullptr,
    image_msg->header.stamp.nanosec,
    image_msg->header.stamp.sec);

  cv::Mat in_img, ocv_out_img;
  CornersMessage corners_msg;
  cv_ptr->image.copyTo(in_img);
  harrisImage(in_img, ocv_out_img, corners_msg);

  TRACEPOINT(
    image_proc_harris_fini,
    static_cast<const void *>(this),
    static_cast<const void *>(&(*image_msg)),
    nullptr,
    image_msg->header.stamp.nanosec,
    image_msg->header.stamp.sec);

  // Allocate new image message
  sensor_msgs::msg::Image::SharedPtr harris_msg = cv_bridge::CvImage(
      image_msg->header,
      image_msg->encoding,
      ocv_out_img).toImageMsg();

  if (do_pub_image) {
    pub_image_.publish(harris_msg);
  }
  if (do_pub_corners) {
    corners_msg.stamp = image_msg->header.stamp;
    pub_corners_->publish(corners_msg);
  }

  TRACEPOINT(
    image_proc_harris_cb_fini,
    static_cast<const void *>(this),
    static_cast<const void *>(&(*harris_msg)),
    nullptr,
    image_msg->header.stamp.nanosec,
    image_msg->header.stamp.sec,
    get_msg_size(harris_msg),
    0);
}

}  // namespace rt_hw_accel_demo

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the
// component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rt_hw_accel_demo::HarrisNodeCPU)
