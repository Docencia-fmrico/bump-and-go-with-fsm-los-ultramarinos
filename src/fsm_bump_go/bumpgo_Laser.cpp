// Copyright 2022 los ultramarinos
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cmath>
#include "ros/ros.h"
#include "fsm_bump_go/bumpgo_Laser.h"

namespace fsm_bump_go
{
BumpGo_Laser::BumpGo_Laser(bool laser_flipped) : state_(GOING_FORWARD), laser_flipped_(laser_flipped)
{
  obstacle_left_ = obstacle_front_ = obstacle_right_ = false;
  sub_laser_ = n_.subscribe("/scan_filtered", 1, &BumpGo_Laser::laserCallback, this);
  pub_vel_ = n_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
}

void BumpGo_Laser::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  float angle_zero = 0.f;

  float angle_sides = ANGLE_SIDE * M_PI / 180.f;
  float angle_front = ANGLE_FRONT * M_PI / 180.f;
  float angle = msg->angle_min;
  bool obstacle_found = false;

  if (laser_flipped_)
  {
    angle_zero = M_PI / 2.f;
    // ROS_INFO("ANGULO 0: %f", angle_zero);
  }
  // ROS_INFO("ANGULO IZQ: %f", (msg->angle_min)*180/M_PI);
  // ROS_INFO("ANGULO DCH: %f", (msg->angle_max)*180/M_PI);
  

  obstacle_left_ = obstacle_front_ = obstacle_right_ = false;

  for (int i = 0; i < msg->ranges.size() && !obstacle_found; i++)
  {
    if (angle >= (angle_zero - angle_sides) && angle < (angle_zero - angle_front))  // left zone
    {
      if (msg->ranges.at(i) <= MINIMUM_ALLOWED_OBJECT_DISTANCE)
      {
        obstacle_found = obstacle_left_ = true;
        // ROS_INFO("IZQUIERDA %f", angle);
      }
    }
    else if (angle >= (angle_zero - angle_front) && angle < (angle_zero + angle_front))  // front zone
    {
      if (msg->ranges.at(i) <= MINIMUM_ALLOWED_OBJECT_DISTANCE)
      {
        obstacle_found = obstacle_front_ = true;
        // ROS_INFO("DE FRENTE %f", angle);
      }
    }
    else if (angle >= (angle_zero + angle_front) && angle < (angle_zero + angle_sides))  // right zone
    {
      if (msg->ranges.at(i) <= MINIMUM_ALLOWED_OBJECT_DISTANCE)
      {
        obstacle_found = obstacle_right_ = true;
        // ROS_INFO("DERECHA %f", angle);
      }
    }
    angle += msg->angle_increment;
  }
}

void BumpGo_Laser::step()
{
  geometry_msgs::Twist cmd;

  switch (state_)
  {
    case GOING_FORWARD:
      cmd.linear.x = 0.2;
      cmd.angular.z = 0;

      if (obstacle_front_ || obstacle_right_ || obstacle_left_)
      {
        pressed_ts_ = ros::Time::now();
        state_ = GOING_BACK;
        ROS_INFO("RETROCEDIENDO");
      }
      break;

    case GOING_BACK:
      cmd.linear.x = -0.2;
      cmd.angular.z = 0;

      if ((ros::Time::now() - pressed_ts_).toSec() > BACKING_TIME )
      {
        turn_ts_ = ros::Time::now();
        if (obstacle_left_)
        {
          state_ = TURNING_RIGHT;
          ROS_INFO("GIRANDO DERECHA");
        }
        else
        {
          state_ = TURNING_LEFT;
          ROS_INFO("GIRANDO IZQUIERDA");
        }
      }
      break;

    case TURNING_LEFT:
      cmd.linear.x = 0;
      cmd.angular.z = 0.4;

      if ((ros::Time::now()-turn_ts_).toSec() > TURNING_TIME )
      {
        state_ = GOING_FORWARD;
        ROS_INFO("AVANZANDO");
      }
      break;
    case TURNING_RIGHT:
      cmd.linear.x = 0;
      cmd.angular.z = -0.4;

      if ((ros::Time::now()-turn_ts_).toSec() > TURNING_TIME )
      {
        state_ = GOING_FORWARD;
        ROS_INFO("AVANZANDO");
      }
      break;
  }
  pub_vel_.publish(cmd);
}
}  // namespace fsm_bump_go
