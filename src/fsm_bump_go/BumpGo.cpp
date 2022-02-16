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

BumpGo::BumpGo()
: state_(GOING_FORWARD),
  pressed_(false),
  sentido_(1)
{

  // el nombre del topic para recibir los mensajes del bumper es --> /mobile_base/events/bumper
  // subscribe('nombreTopic',frecuencia,funcion de CallBack);

  //ros::Subscriber vsub_bumber_ = sus.subscribe("/mobile_base/events/bumper",10,&BumpGo::bumperCallback,this);

  // el nombre del topic para publibar en los motores es --> mobile_base/commands/velocity
  // n_.advertise< Paquete :: Tipo del mensaje >('Nombre del topic',frecuencia);

  //ros::Publisher pub_vel_ = pub.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity",10);

  // Suscriptor del bumper
  //sub_bumber_ = n_.subscribe("/mobile_base/events/bumper",10,&BumpGo::bumperCallback,this);

  // Suscriptor del laser
  pub_astra_ = n_.subscribe("/scan",10,&BumpGo::laserCallback,this);
  pub_vel_ = n_.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity",10);
}

bool BumpGo::valorApto(float v ){
    float  min =  0;
    float  max = 1;
    return  !(v > max || v < min)  ;
} 

float BumpGo::hacerMedia(std::vector<float> &arr){

  float media = 0 ;
  int n = 0 ;
 
  for (int i = 0; i < arr.size() ; i++)
  {
    float valorActual = arr[i] ;

    if ( valorApto(valorActual) ){
        media+=valorActual;
        n++;
    }  
  }
  return media/n ;
}

bool BumpGo::hayObstaculo(std::vector<float> &arr,float rango){
     float media = hacerMedia(arr) ;
     return  media < rango ;
}

std::vector<std::vector<float>> BumpGo::divisionVector(std::vector<float> &arr){
     
     std::vector<float> semiplanoDerecho ;
     std::vector<float> semiplanoIzquierdo ;

     std::vector<std::vector<float>> semiplanos ;

     for(int i = 0 ; i < arr.size() ; i++){

         float valorActual = arr[i] ;

         if( i < arr.size()/2 ){
            semiplanoIzquierdo.push_back(valorActual) ;
         }else{
            semiplanoDerecho.push_back(valorActual) ;
         }

     }
     semiplanos.push_back(semiplanoIzquierdo) ;
     semiplanos.push_back(semiplanoDerecho)  ;

     return semiplanos ;
}

void
BumpGo::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  /* Version 1: leemos las medidas y vamos hacia atras si detectamos un obstáculo
    a una distancia d m
  */
  int i = 0;
  double d = 1.0;
  

  //############# PARAMETROS ######################
  float rango = 2 ; // 

  // #############################################

  // paso del array medidas a un std::vector 
  int ranges_size = msg->ranges.size();
  
  
  //float media = hacerMedia(a) ;

  double distacnia_media_izq = 0.0;
  double distacnia_media_der = 0.0;


  for (i = 0; i < ranges_size/2; i++)
  {
    if (msg->ranges[i] > 0 && msg->ranges[i] < 1)
        distacnia_media_izq+=msg->ranges[i];

  }
  for (i = ranges_size/2; i < ranges_size; i++)
  {
    if (msg->ranges[i] > 0 && msg->ranges[i] < 1)
        distacnia_media_der+=msg->ranges[i];
  }

  distacnia_media_der = distacnia_media_der / (ranges_size/2);
  distacnia_media_izq = distacnia_media_izq / (ranges_size/2);

  ROS_INFO("Distancia media: %lf %lf", distacnia_media_izq, distacnia_media_der);

  if (distacnia_media_izq < 0.2) {
    pressed_ = true;
    sentido_ = 1;
  }
  else if (distacnia_media_der < 0.2) {
    pressed_ = true;
    sentido_ = -1;
  }
  else {
    pressed_ = false;
  }

}


// Bumper
void
BumpGo::bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
  int a = msg->PRESSED ;

  pressed_ = msg->state == kobuki_msgs::BumperEvent::PRESSED;

  if (pressed_)
  {
    switch (msg->bumper)
    {
      case kobuki_msgs::BumperEvent::LEFT:
        sentido_=TURNING_RIGHT;
        ROS_INFO("OUCH  POR LA IZQ");
        break;
      case kobuki_msgs::BumperEvent::RIGHT:
        sentido_=TURNING_LEFT;
        ROS_INFO("OUCH POR LA DRCH");
        break;
      case kobuki_msgs::BumperEvent::CENTER:
        sentido_=TURNING_LEFT;
        ROS_INFO("OUCH DE FRENTE");
        break;
    }
  }
}
//

void
BumpGo::step()
{
  geometry_msgs::Twist cmd;

  float linearV = 0.2 ;
  float angularW = 1 ;

  bool obstacle_detected_ = true ;

  

  if(obstacle_detected_)
  {
  state_ = TURNING;
  }
  else
  {
  state_ = GOING_FORWARD ;
  }
  
  switch (state_)
  {
  case GOING_FORWARD :
  {
    cmd.linear.x = linearV;
    cmd.linear.z = 0;
  }
  case TURNING :
  {
    cmd.linear.x = 0;
    cmd.linear.z = sentido_*angularW;
  }
  }

  

    pub_vel_.publish(cmd);
}

}  // namespace fsm_bump_go
