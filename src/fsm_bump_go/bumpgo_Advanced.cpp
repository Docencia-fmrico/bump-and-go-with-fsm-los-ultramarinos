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

#include "fsm_bump_go/bumpgo_Advanced.h"

#include "kobuki_msgs/BumperEvent.h"
#include "geometry_msgs/Twist.h"

#include "ros/ros.h"

namespace fsm_bump_go
{
BumpGo_Advanced::BumpGo_Advanced() : state_(GOING_FORWARD), pressed_(false)
{
  sub_bumber_ = n_.subscribe("/mobile_base/events/bumper", 1, &BumpGo_Advanced::bumperCallback, this);
  pub_vel_ = n_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
}

void BumpGo_Advanced::bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
  pressed_ = msg->state == kobuki_msgs::BumperEvent::PRESSED;
  direction_ =  msg->bumper;
}

void BumpGo_Advanced::step()
{
  geometry_msgs::Twist cmd;

  switch (state_)
  {
    case GOING_FORWARD:
      cmd.linear.x = 0.2;
      cmd.angular.z = 0;

      if (pressed_)
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
        if (direction_ == PRESSED_LEFT)
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
