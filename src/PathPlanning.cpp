/**
 *  MIT License
 *
 *  Copyright (c) 2018 Saimouli Katragadda, Saurav Kumar
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a
 *  copy of this software and associated documentation files (the "Software"),
 *  to deal in the Software without restriction, including without
 *  limitation the rights to use, copy, modify, merge, publish, distribute,
 *  sublicense, and/or sell copies of the Software, and to permit persons to
 *  whom the Software is furnished to do so, subject to the following
 *  conditions:
 *
 *  The above copyright notice and this permission notice shall be included
 *  in all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 *  THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 *  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 *  DEALINGS IN THE SOFTWARE.
 */

/**
 *@file         PathPlanning.cpp
 *@author       Saimouli Katragadda
 *@author       Saurav Kumar
 *@copyright    MIT License
 *@brief        implements the PathPlanning class methods
 */
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <frontier_exploration_turtlebot/PathPlanning.h>

PathPlanning::PathPlanning() {
  ros::Rate loop_rate(1);
  ROS_INFO("Creating the Explorer behaviour...");
  // Set speed parameters
  linearSpeed = 0.0;
  angularSpeed = 0.0;
  // register to publish topic on /cmd_vel
  // to send move velocity commands to the turtlebot
  pubVel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
  // creating Subscriber sub  subscribing to scan topic and calling
  // lasercallback function of CollisionDetector class

  // Define the initial velocity message
  msg.linear.x = 0.0;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = 0.0;
  // Stop the turtlebot
  pubVel.publish(msg);
  ros::spinOnce();
  // Sleep for the remaining time until we hit our 1 Hz rate
  loop_rate.sleep();
}

PathPlanning::~PathPlanning() {
  // Stop the turtlebot before exiting
  msg.linear.x = 0.0;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = 0.0;
  // Stop the turtlebot
  pubVel.publish(msg);
}

void PathPlanning::PathGenerator() {
  ros::Rate loop_rate(1);
  //  then rotate in CCW
  msg.linear.x = linearSpeed;
  msg.angular.z = angularSpeed;


  // Publish the cmd message to anyone listening
  pubVel.publish(msg);
  // "Spin" a callback in case we set up any callbacks
  ros::spinOnce();
  // Sleep for the remaining time until we hit our 1 Hz rate
  loop_rate.sleep();
}

void PathPlanning::setangularspeed
(float angular_first, float angular_second, float dt) {
  double dtheeta = (angular_second -angular_first)*M_PI/180;
  if ((angular_second- angular_first)>= 180) {
    dtheeta = (angular_second -angular_first-360)*M_PI/180;
  }
  if ((angular_second- angular_first)<= -180) {
      dtheeta = (angular_second -angular_first+360)*M_PI/180;
    }
  angularSpeed = dtheeta/dt;
}
void PathPlanning::setlinearspeed
(float linear_x1, float linear_y1, float linear_x2, float linear_y2, float dt) {
  double delta_x = linear_x2 - linear_x1;
  double delta_y = linear_y2 - linear_y1;
  linearSpeed = (sqrt(pow(delta_x, 2)+pow(delta_y, 2)))/(100*dt);
}

float PathPlanning::getangularspeed() {
  return angularSpeed;
}
float PathPlanning::getlinearspeed() {
  return linearSpeed;
}
