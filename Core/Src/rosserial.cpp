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
geometry::Vector eR;
float* yaw_pointer;

std_msgs::Float32MultiArray pub_msg;
Quaternion q_camera2AUV;


int arm_state = 0;
float desired_depth = 0;


/* ----subscriber parameters
- 0-2:   error angle x,y,z
- 3-4:   ex          x,y
- 5:     desire depth
- 6:     robot arm state

*/
void callback(const std_msgs::Float32MultiArray& msg){
  /*state_pointer->orientation.w = msg.data[0];
  state_pointer->orientation.x = msg.data[1];
  state_pointer->orientation.y = msg.data[2];
  state_pointer->orientation.z = msg.data[3];*/
  //state.orientation.w = msg.data[0];


 
  

  
  /*Quaternion camera(msg.data[0], msg.data[1], msg.data[2], msg.data[3]);
  q_camera2AUV.x = 1;
  q_camera2AUV.y = 0;
  q_camera2AUV.z = 0;
  q_camera2AUV.w = 0;
  state_pointer->orientation = q_camera2AUV.conjugate() * camera * q_camera2AUV; */
  
  //state_pointer->orientation = camera;
  /*state_pointer->velocity.angular.x = msg.data[4];
  state_pointer->velocity.angular.y = msg.data[5];
  state_pointer->velocity.angular.z = msg.data[6];*/ 
  eR.x = msg.data[0];
  eR.y = msg.data[1];
  eR.z = msg.data[2];
  ex_pointer->x = msg.data[3];
  ex_pointer->y = msg.data[4];
  desired_depth = msg.data[5];
  arm_state = msg.data[6];
  
  
  /*ev_pointer->x = msg.data[10];
  ev_pointer->y = msg.data[11];
  ev_pointer->z = msg.data[12];
  operate = msg.data[13]; //0 -> interrupt*/
  
}

ros::Publisher pub("stm32_to_rpi", &pub_msg);
ros::Subscriber<std_msgs::Float32MultiArray> sub("rpi_to_stm32", callback);


void rosserial_subscribe(){
    nh.spinOnce();
}

void rosserial_publish(float q_w, float q_x, float q_y, float q_z){
  // publish data
  pub_msg.data_length = 4;
  float array[4] = {0};
  
  
  array[0] = q_w;
  array[1] = q_x;
  array[2] = q_y;
  array[3] = q_z;
  
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
