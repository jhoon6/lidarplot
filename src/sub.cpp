/*
 *  SLLIDAR ROS2 CLIENT
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2022 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <math.h>
#include "opencv2/opencv.hpp"

#define RAD2DEG(x) ((x)*180./M_PI)

static void scanCb(sensor_msgs::msg::LaserScan::SharedPtr scan) {
  int count = scan->scan_time / scan->time_increment;
  printf("[SLLIDAR INFO]: I heard a laser scan %s[%d]:\n", scan->header.frame_id.c_str(), count);
  printf("[SLLIDAR INFO]: angle_range : [%f, %f]\n", RAD2DEG(scan->angle_min),
         RAD2DEG(scan->angle_max));

  cv::Mat img(cv::Size(500, 500), CV_8UC3, cv::Scalar(255, 255, 255));
  cv::drawMarker(img, cv::Point(250, 250), cv::Scalar(0, 0, 0), cv::MARKER_CROSS, 10, 1, cv::LINE_4);

  for (int i = 0; i < count; i++) {
    float angle = scan->angle_min + scan->angle_increment * i;
    float degree = RAD2DEG(angle);
    float distance = scan->ranges[i];

    if (std::isnan(distance) || std::isinf(distance)) continue;
    int x = 250 + (distance * 100.0 * sin(angle));
    int y = 250 + (distance * 100.0 * cos(angle));

    cv::drawMarker(img, cv::Point(x, y), cv::Scalar(0, 0, 255), cv::MARKER_SQUARE, 2, 2, cv::LINE_4);

    //printf("[SLLIDAR INFO]: angle-distance : [%f, %f]\n", degree, distance);
  }
  
  cv::imshow("scan", img);
  cv::waitKey(1);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("sub");

  auto lidar_info_sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
                        "scan", rclcpp::SensorDataQoS(), scanCb);

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
