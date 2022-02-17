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
  pressed_(false),
  sentido_(1),
  angulo_(PI/2.0),
  TURNING_TIME(5.0)
{

  // el nombre del topic para recibir los mensajes del bumper es --> /mobile_base/events/bumper
  // subscribe('nombreTopic',frecuencia,funcion de CallBack);
   
  //ros::Subscriber vsub_bumber_ = sus.subscribe("/mobile_base/events/bumper",10,&BumpGo::bumperCallback,this);

  // el nombre del topic para publibar en los motores es --> mobile_base/commands/velocity
  // n_.advertise< Paquete :: Tipo del mensaje >('Nombre del topic',frecuencia);

  //ros::Publisher pub_vel_ = pub.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity",10);

  sub_bumber_ = n_.subscribe("/mobile_base/events/bumper",10,&BumpGo::bumperCallback,this);
  pub_vel_ = n_.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity",10);
}

void
BumpGo::bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
  int a = msg->PRESSED;
   
  pressed_ = msg->state == kobuki_msgs::BumperEvent::PRESSED;

  if (pressed_)
  {
    switch (msg->bumper)
    {
      case kobuki_msgs::BumperEvent::LEFT:
        sentido_=TURNING_RIGHT;
        angulo_=PI/4.0;
        ROS_INFO("OUCH  POR LA IZQ");
        break;
      case kobuki_msgs::BumperEvent::RIGHT:
        sentido_=TURNING_LEFT;
        angulo_=PI/4.0;
        ROS_INFO("OUCH POR LA DRCH");
        break;
      case kobuki_msgs::BumperEvent::CENTER:
        angulo_=PI/3.0;
        ROS_INFO("OUCH DE FRENTE");
        break;
    }
  }
}

void
BumpGo::step()
{
  geometry_msgs::Twist cmd;

  float linearV = 0.2 ;
  float angularW = 0.8 ;

  switch (state_)
  {
    case GOING_FORWARD:

      cmd.linear.x = linearV ;
      cmd.angular.z = 0 ;
      

      if (pressed_)
      {
        press_ts_ = ros::Time::now();
        state_ = GOING_BACK;
        ROS_INFO("GOING_FORWARD -> GOING_BACK");
      }

      break;
    case GOING_BACK:

        cmd.linear.x = -linearV  ;
        cmd.angular.z = 0 ;

      if ((ros::Time::now() - press_ts_).toSec() > BACKING_TIME )
      {
        turn_ts_ = ros::Time::now();
        state_ = TURNING;
        ROS_INFO("GOING_BACK -> TURNING");
      }

      break;
    case TURNING:
        TURNING_TIME = PI * angulo_/(angularW);
        cmd.linear.x = 0 ;
        cmd.angular.z = sentido_*angularW ;

      if ((ros::Time::now()-turn_ts_).toSec() > TURNING_TIME )
      {
        state_ = GOING_FORWARD;
        ROS_INFO("TURNING -> GOING_FORWARD");
      }
      break;
    }

    pub_vel_.publish(cmd);
}

}  // namespace fsm_bump_go
