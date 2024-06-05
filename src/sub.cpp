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

const int def_speed = 100;
const float k = 70.f;
const int horizontalWallDetectionScore = 750;

float calc_point(float distance) {
    if (distance == 0) return 10.0;
    return 10.0 / distance;
}

static void scanCb(sensor_msgs::msg::LaserScan::SharedPtr scan) {
  static cv::Point center(250, 250);

  int count = scan->scan_time / scan->time_increment;
  printf("[SLLIDAR INFO]: I heard a laser scan %s[%d]:\n", scan->header.frame_id.c_str(), count);
  printf("[SLLIDAR INFO]: angle_range : [%f, %f]\n", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));

  cv::Mat img(cv::Size(500, 500), CV_8UC3, cv::Scalar(0, 0, 0));
  //cv::drawMarker(img, cv::Point(250, 250), cv::Scalar(255, 255, 255), cv::MARKER_CROSS, 10, 1, cv::LINE_4);

  cv::drawMarker(img, cv::Point(250, 250), cv::Scalar(0, 100, 0), cv::MARKER_CROSS, 500, 1, cv::LINE_4);
  cv::drawMarker(img, cv::Point(250, 250), cv::Scalar(0, 100, 0), cv::MARKER_STAR, 500, 1, cv::LINE_4);
  for (float r = 50; r <= 250; r += 50) cv::circle(img, cv::Point(250, 250), r, cv::Scalar(0, 100, 0), 1);

  float scoreFrontLeft = 0;
  float scoreFrontRight = 0;

  for (int i = 0; i < count; i++) {
    float angle = scan->angle_min + scan->angle_increment * i;
    float degree = RAD2DEG(angle);
    float distance = scan->ranges[i];

    if (std::isnan(distance) || std::isinf(distance) || distance >= 3) continue;
    int x = 250 + (distance * 100.0 * sin(angle));
    int y = 250 + (distance * 100.0 * cos(angle));

    cv::drawMarker(img, cv::Point(x, y), cv::Scalar(0, 255, 0), cv::MARKER_SQUARE, 2, 2, cv::LINE_4);

    if (degree >= -180 && degree < -120 /* -90 */ && distance < 1.5) {
      scoreFrontLeft += calc_point(distance);
    } else if (degree >= 120 /* 90 */ && degree < 180 && distance < 1.5) {
      scoreFrontRight += calc_point(distance);
    }

    int color = distance * 100;
    if (color >= 255) color = 255;
    cv::drawMarker(img, cv::Point(x, y), cv::Scalar(color, 255, color), cv::MARKER_SQUARE, 2, 2, cv::LINE_4);

    //printf("[SLLIDAR INFO]: angle-distance : [%f, %f]\n", degree, distance);
  }

  float add = abs(scoreFrontLeft - scoreFrontRight) / k;
  printf("절댓값: %f\n", add);

  if (add < 7 && scoreFrontLeft >= horizontalWallDetectionScore && scoreFrontRight >= horizontalWallDetectionScore)
  {
    printf("벽 감지!\n");
    add = 80;
  }

  float add_plot;
  if (add >= 90.f) add_plot = 90.f;
  else add_plot = add; 

  if (scoreFrontLeft > scoreFrontRight){
    printf("좌가 점수많음 %f %f\n", scoreFrontLeft, scoreFrontRight);
    cv::Point arrow_end(center.x + 100 * cos((90.f - add_plot) * CV_PI / 180), center.y - 100 * sin((90.f - add_plot) * CV_PI / 180));
    cv::arrowedLine(img, center, arrow_end, cv::Scalar(255, 255, 0), 1);
  } 
  else if (scoreFrontLeft <= scoreFrontRight) {
    printf("우가 점수많음 %f %f\n", scoreFrontLeft, scoreFrontRight);
    cv::Point arrow_end(center.x + 100 * cos((90.f + add_plot) * CV_PI / 180), center.y - 100 * sin((90.f + add_plot) * CV_PI / 180));
    cv::arrowedLine(img, center, arrow_end, cv::Scalar(255, 255, 0), 1);
    add = -1 * add;
  }

  float vel1 = def_speed + add;
  float vel2 = -1 * (def_speed - add);

  printf("Speed: %lf, %lf\n", vel1, vel2);
  
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
