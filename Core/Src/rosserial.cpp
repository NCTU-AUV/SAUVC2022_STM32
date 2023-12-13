﻿/*
 * main.cpp

 *
 *  Created on: 2018/01/17
 *      Author: yoneken
 */
#include "rosserial.h"
#include "std_msgs/Float32MultiArray.h"


ros::NodeHandle nh;

// parameters
geometry::Vector* ex_pointer;
geometry::Vector* ev_pointer;
Dynamics* state_pointer;
float* yaw_pointer;
float rec;
std_msgs::Float32MultiArray pub_msg;
Quaternion q_camera2AUV;
extern Dynamics state;
double rec_msg[13];


/* ----subscriber parameters
- 0~3:   state.orientation (quaternion) w,x,y,z
- 4-6:   state.velocity.angular: x,y,z
- 7-9:   ex (position error): x,y,z
- 10-12: ev (velocity error): x,y,z


*/
void callback(const std_msgs::Float32MultiArray& msg){
  state_pointer->orientation.w = msg.data[0];
  state_pointer->orientation.x = msg.data[1];
  state_pointer->orientation.y = msg.data[2];
  state_pointer->orientation.z = msg.data[3];
  //state.orientation.w = msg.data[0];


  geometry::Vector ex;
  geometry::Vector ev;
  //Dynamics state;
  
  //Quaternion camera(msg.data[0], msg.data[1], msg.data[2], msg.data[3]);
  /*q_camera2AUV.x = 1;
  q_camera2AUV.y = 0;
  q_camera2AUV.z = 0;
  q_camera2AUV.w = 0;
  state_pointer->orientation = q_camera2AUV.conjugate() * camera * q_camera2AUV;*/ 
  
  //state_pointer->orientation = camera;
  state_pointer->velocity.angular.x = msg.data[4];
  state_pointer->velocity.angular.y = msg.data[5];
  state_pointer->velocity.angular.z = msg.data[6]; 

  ex.x = msg.data[7];
  ex.y = msg.data[8];
  //*(ex.z) = msg.data[9];
  ev.x = msg.data[10];
  ev.y = msg.data[11];
  //*(ev.z) = msg.data[12];
}
ros::Publisher pub("stm32_to_rpi", &pub_msg);
ros::Subscriber<std_msgs::Float32MultiArray> sub("rpi_to_stm32", callback);


void rosserial_subscribe(){
    nh.spinOnce();
}

void rosserial_publish(double depth){
  // publish data
  pub_msg.data_length = 3;
  float array[3] = {0};
  
  
  array[0] = state_pointer->orientation.w;
  array[1] = state.orientation.w;
  array[2] = depth;
  //pub_msg.data[0] = depth;
  pub_msg.data = array;
  pub.publish(&pub_msg);
  nh.spinOnce();
}

void rosserial_init(geometry::Vector* ex_p, Dynamics* s_p, float* yaw)
{
  ex_pointer = ex_p;
  state_pointer = s_p;
  //yaw_pointer= yaw_p;
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);
  //int pid = fork();
  //if(pid==0){ nh.spinOnce(); HAL_Delay(1); }
  
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->reset_rbuf();
}


// Create class of rosserial
