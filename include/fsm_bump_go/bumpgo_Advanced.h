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

#ifndef FSM_BUMP_GO_BUMPGO_ADVANCED_H
#define FSM_BUMP_GO_BUMPGO_ADVANCED_H

#include "ros/ros.h"
#include "kobuki_msgs/BumperEvent.h"
#include "geometry_msgs/Twist.h"

namespace fsm_bump_go
{

class BumpGo_Advanced
{
public:
  BumpGo_Advanced();

  void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg);
  void step();

protected:
  ros::NodeHandle n_;

  static const int GOING_FORWARD = 0;
  static const int GOING_BACK = 1;
  static const int TURNING_LEFT = 2;
  static const int TURNING_RIGHT = 3;

  static const int PRESSED_LEFT = 0;
  static const int PRESSED_FRONT = 1;
  static const int PRESSED_RIGHT = 2;

  static constexpr double TURNING_TIME = 5.0;
  static constexpr double BACKING_TIME = 3.0;

  int state_;
  int direction_;

  bool pressed_;

  ros::Time pressed_ts_;
  ros::Time turn_ts_;

  ros::Subscriber sub_bumber_;
  ros::Publisher pub_vel_;
};

}  // namespace fsm_bump_go
#endif  // FSM_BUMP_GO_BUMPGO_ADVANCED_H
