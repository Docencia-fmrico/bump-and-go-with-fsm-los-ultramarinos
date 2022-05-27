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

#ifndef FSM_BUMP_GO_BUMPGO_LASER_H
#define FSM_BUMP_GO_BUMPGO_LASER_H

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

namespace fsm_bump_go
{
class BumpGo_Laser
{
public:
  explicit BumpGo_Laser(bool laser_flipped);

  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void step();

private:
  ros::NodeHandle n_;

  static const int GOING_FORWARD = 0;
  static const int GOING_BACK = 1;
  static const int TURNING_LEFT = 2;
  static const int TURNING_RIGHT = 3;

  const float ANGLE_FRONT = 10.f;
  const float ANGLE_SIDE = 60.f;
  const float MINIMUM_ALLOWED_OBJECT_DISTANCE = 0.5f;

  static constexpr double TURNING_TIME = 3.0;
  static constexpr double BACKING_TIME = 3.0;

  int state_;

  bool obstacle_front_;
  bool obstacle_left_;
  bool obstacle_right_;

  bool laser_flipped_;

  ros::Time pressed_ts_;
  ros::Time turn_ts_;

  ros::Subscriber sub_laser_;
  ros::Publisher pub_vel_;
};
}  // namespace fsm_bump_go
#endif  // FSM_BUMP_GO_BUMPGO_LASER_H
