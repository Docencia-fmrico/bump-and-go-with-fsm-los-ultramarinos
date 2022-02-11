// Copyright 2022 Intelligent Robotics Lab
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

#include "fsm_bump_go/BumpGo.h"
#include "kobuki_msgs/BumperEvent.h"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"

namespace fsm_bump_go
{

BumpGo::BumpGo()
: state_(GOING_FORWARD),
obstacle_(NO_OBSTACLE_DETECTED) 
//obstacle_detected_left_(false), 
//obstacle_detected_right_(false)

{
  //sub_sensor_ = BumpGo::subscribirSensor(n_);
  //sub_sensor_ = n_.subscribe("/mobile_base/events/bumper", 10, &BumpGo::sensorCallback, this);
  pub_vel_ = n_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);
}



//void
//BumpGo::sensorCallback(const kobuki_msgs::BumperEvent::ConstPtr&  msg)
//{
  
    //obstacle_detected_left_ = detectObstacleLeft();
   
    //obstacle_detected_right_ = detectObstacleRight();

//obstacle_ = detectObstacle();
 
    //if(obstacle_detected_left_){
      //  ROS_INFO("OBSTACLE DETECTED LEFT, TURNING RIGHT");
    //}
    //if(obstacle_detected_right_){
      //  ROS_INFO("OBSTACLE DETECTED RIGHT, TURNING LEFT");
    //}
//}

void
BumpGo::step()
{
  geometry_msgs::Twist cmd;

  float linearV = 0.2 ;
  float angularW = 0.4 ;

  switch (state_)
  {
    case GOING_FORWARD:

      cmd.linear.x = linearV ;
      cmd.angular.z = 0 ;
      

      if (obstacle_ != NO_OBSTACLE_DETECTED)
      {
        press_ts_ = ros::Time::now();
        state_ = GOING_BACK;
        ROS_INFO("GOING_FORWARD -> GOING_BACK");
      }

      break;
    case GOING_BACK:

        cmd.linear.x = -linearV  ;
        cmd.angular.z = 0 ;

      if ((ros::Time::now() - press_ts_).toSec() > BACKING_TIME)
      {
        turn_ts_ = ros::Time::now();
        state_ = obstacle_;
        ROS_INFO("GOING_BACK -> TURNING");
      }

      break;
    case TURNING_LEFT:

        cmd.linear.x = 0 ;
        cmd.angular.z = angularW ;

      if ((ros::Time::now()-turn_ts_).toSec() > TURNING_TIME )
      {
        state_ = GOING_FORWARD;
        ROS_INFO("TURNING -> GOING_FORWARD");
      }
      break;

    case TURNING_RIGHT:

        cmd.linear.x = 0 ;
        cmd.angular.z = -angularW ;

      if ((ros::Time::now()-turn_ts_).toSec() > TURNING_TIME )
      {
        state_ = GOING_FORWARD;
        ROS_INFO("TURNING -> GOING_FORWARD");
      }
      break;
    }

    pub_vel_.publish(cmd);
}

//class Bumper
//{
//
 // void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr&  msg)
 // {
  //   if (msg -> bumper == kobuki_msgs::BumperEvent::LEFT && msg -> state == kobuki_msgs::BumperEvent::PRESSED)
   //  {
   //    BumpGo::obstacle_ = OBSTACLE_DETECTED_LEFT;
  //     ROS_INFO("OBSTACLE DETECTED IN LEFT BUMPER, BACKING, THEN TURNING RIGHT");
  //   }
  //   if (msg -> bumper == kobuki_msgs::BumperEvent::RIGHT && msg -> state == kobuki_msgs::BumperEvent::PRESSED)
  //   {
  //     obstacle_ = OBSTACLE_DETECTED_RIGHT;
  //     ROS_INFO("OBSTACLE DETECTED IN RIGHT BUMPER, BACKING, THEN TURNING LEFT");
  //   }
  //   if (msg -> bumper == kobuki_msgs::BumperEvent::CENTER && msg -> state == kobuki_msgs::BumperEvent::PRESSED)
  //   {
  //     obstacle_ = OBSTACLE_DETECTED_RIGHT;
  //     ROS_INFO("OBSTACLE DETECTED IN FRONT BUMPER, BACKING, THEN TURNING LEFT");
 //    }
 // }
//}

//ros::Subscriber BumpGo::subscribirSensor(ros::NodeHandle nodo)
//  {
//    ros::Subscriber sub = nodo.subscribe("/mobile_base/events/bumper", 10, &Bumper::bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr&  msg), this); 
//    return sub; 
//  }

} //namespace fsm_bump_go
