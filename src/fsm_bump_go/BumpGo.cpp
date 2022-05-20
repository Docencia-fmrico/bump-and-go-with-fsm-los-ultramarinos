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
#include "sensor_msgs/LaserScan.h"
#include "ros/ros.h"
#include "stdio.h"

namespace fsm_bump_go
{

BumpGo::BumpGo(): state_(GOING_FORWARD), obstacle_detected_(false), sentido_(1)
{
  // parametros santi 
  std::vector<float> mediciones = {10000};
  rango_deteccion = 0.7;
  linearV  = 0.2;
  angularW  = 1.0;
  min =  0.15 ;//rango_deteccion*3/4;
  max = rango_deteccion*1.5;
  apertura = PI/4;
 
  // publicadores y suscriptores 
  pub_astra_ = n_.subscribe("/scan",10,&BumpGo::laserCallback,this);
  pub_vel_ = n_.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity",10);
}

bool BumpGo::valorApto(float v )
{
  return  !(v > max || v < min);
} 

float BumpGo::hacerMedia(std::vector<float> &arr)
{
  float media = 0;
  int n = 0;
 
  for (int i = 0; i < arr.size(); i++)
  {
    float valorActual = arr[i];
    //ROS_INFO_STREAM("VALOR = " << valorActual);

    if (valorApto(valorActual))
    {
      //ROS_INFO_STREAM("VALOR = " << valorActual);
      media+=valorActual;
      n++;
    } 
  }
  //ROS_INFO_STREAM("size: " << arr.size());
  //ROS_INFO_STREAM("VALORES = " << n);
  return media/n;
}

bool BumpGo::hayObstaculo(std::vector<float> &arr,float rango)
{
  float media = hacerMedia(arr);
  ROS_INFO_STREAM("MEDIA = " << media);
  return  media < rango;
}

std::vector<std::vector<float>> BumpGo::divisionVector(std::vector<float> &arr){
     
     std::vector<float> semiplanoDerecho;
     std::vector<float> semiplanoIzquierdo;
     std::vector<float> semiplanoCentral;

     std::vector<std::vector<float>> semiplanos;

     for(int i = 0 ; i < arr.size() ; i++){

         float valorActual = arr[i];

         if( i < arr.size()/3 ){
            semiplanoIzquierdo.push_back(valorActual);
         }
         else if ( i < 2* arr.size()/3 ){
            semiplanoCentral.push_back(valorActual);
         }
         else{
            semiplanoDerecho.push_back(valorActual);
         }

     }
     semiplanos.push_back(semiplanoIzquierdo);
     semiplanos.push_back(semiplanoCentral);
     semiplanos.push_back(semiplanoDerecho);

     return semiplanos ;
}

int BumpGo::semiplanoConObstaculo(std::vector<float> &izq,std::vector<float> &der){
    float mediaDerecha = hacerMedia(izq);
    float mediaIzquierda = hacerMedia(der);

    if (mediaDerecha > mediaIzquierda)
    {
      return -1;
    }
    else
    {
      return 1;
    }
} 

void BumpGo::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
  
  //int ranges_size = msg->ranges.size();
  // pasa de array a std::vector 

  float min_ = msg->angle_min;
  float max_ = msg->angle_max; 
  float paso = msg->angle_increment;
  //ROS_INFO_STREAM("min: " << min_);
  //ROS_INFO_STREAM("max: " << max_);
  //ROS_INFO_STREAM("inc: " << paso);
  //ROS_INFO_STREAM("apertura: " << apertura);
  

  int indiceMin = (apertura )/ paso;
  int indiceMax = (max_ - apertura )/ paso;
  
  //ROS_INFO_STREAM("indice min: " << indiceMin);
  //ROS_INFO_STREAM("indice max: " << indiceMax);



  int n = msg->ranges.size();
  std::vector<float> medidas;
  //ROS_INFO_STREAM("size: " << n);


  for( int i = 0 ; i < indiceMin ; i++ ){

     medidas.push_back(msg->ranges[i]);
  }

  for (int i = indiceMax; i < n ; i++){

     medidas.push_back(msg->ranges[i]);

  }

  mediciones = medidas;
  //ROS_INFO_STREAM("size: " << medidas.size());
  
}

void BumpGo::step(){

  geometry_msgs::Twist cmd;
  
  // ROS_INFO("bucle");
  
      std::vector<std::vector<float>> semiplanos = divisionVector(mediciones);
      std::vector<float> semiplanoIzquierdo = semiplanos[0];
      std::vector<float> semiplanoCentral = semiplanos[1];
      std::vector<float> semiplanoDerecho = semiplanos[2];

  obstacle_front_ = hayObstaculo(mediciones,semiplanoCentral);
  obstacle_left_ = hayObstaculo(mediciones,semiplanoIzquierdo);
  obstacle_right_ = hayObstaculo(mediciones,semiplanoDerecho);

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

}  // namespace fsm_bump_go
