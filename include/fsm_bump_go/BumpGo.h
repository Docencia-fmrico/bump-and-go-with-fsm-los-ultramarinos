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

#ifndef FSM_BUMP_GO_BUMPGO_H
#define FSM_BUMP_GO_BUMPGO_H

#include "ros/ros.h"
#include "kobuki_msgs/BumperEvent.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

namespace fsm_bump_go
{

class BumpGo
{
public:
  BumpGo();

  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

//------------------------------------------------------------
  bool hayObstaculo(std::vector<float> &arr,float rango);  
  std::vector<std::vector<float>> divisionVector(std::vector<float> &arr);
  bool valorApto(float v );
  float hacerMedia(std::vector<float> &arr);


  void step();

private:
  std::vector<float> mediciones;
  float rango_deteccion;
  float linearV;
  float angularW;
  float  min;
  float  max;
  float apertura;

  ros::NodeHandle n_;

  static const int GOING_FORWARD = 0;
  static const int GOING_BACK = 1;
  static const int TURNING_RIGHT = 2;
  static const int TURNING_LEFT = 3;

  static constexpr double TURNING_TIME = 5.0;
  static constexpr double BACKING_TIME = 3.0;


  const float PI = 3.1415926535897 ;

  int state_;

  int sentido_;

  float media;
  float mediaIzquierda;
  float mediaDerecha;

  std::vector<std::vector<float>> semiplanos;
  std::vector<float> semiplanoIzquierdo;
  std::vector<float> semiplanoDerecho;

  bool obstacle_left_;
  bool obstacle_right_;

  ros::Time pressed_ts_;
  ros::Time turn_ts_;
  ros::Time scan_ts_;

  ros::Subscriber sub_bumber_;
  ros::Subscriber pub_astra_;
  ros::Publisher pub_vel_;
};
}  // namespace fsm_bump_go

#endif  // FSM_BUMP_GO_BUMPGO_H
