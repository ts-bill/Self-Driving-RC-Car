#include <ros/ros.h>
#include "blocking_reader.h"
#include <iostream>
#include <string>
#include <sstream>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
boost::asio::io_service io_s;
boost::asio::serial_port sp(io_s);

std_msgs::String numberclass;
std_msgs::String size;
bool incomemsg,_incomemsg;

uint8_t cmd_1; //16
uint8_t cmd_2[8];//RA up-down
uint8_t cmd_3=0;//RA right-left
uint8_t cmd_4;//LA up-down
uint8_t cmd_5;//LA right-lefto
char cmd_str[8] = ""; //6
int8_t button[12];
//int8_t axes[6];
#define deadzone 0.05f
float axes[7];
int16_t axes_val;



std::ostringstream cmd;
uint32_t dir = 0; //16

void msgCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
    numberclass.data=msg->bounding_boxes[0].Class;
     if(numberclass.data=="stopsign"){
        cmd_3=1;
        //ROS_INFO("sssss");
     }
     else if (numberclass.data=="redlight")cmd_3=2;
     else if (numberclass.data=="greenlight")cmd_3=3;
     else if (numberclass.data=="yellowlight")cmd_3=4;
     else if (numberclass.data=="nolight")cmd_3=5;
     else if (numberclass.data=="turnleftsign")cmd_3=6;
     else if (numberclass.data=="turnrightsign")cmd_3=7;
      //std::cout << numberclass.data;
}
void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{ 
  for(int8_t n = 3, j=0; n < 8; n++)
  {
    if(n!=5){
    int8_t m=0,k=0;
    float val = joy->axes[n];
    ROS_INFO("axes[%d] : %lf" ,n ,joy->axes[n]);
    if(val > deadzone )
    {
      m = 1;
      k = 0;
      //ROS_INFO("check+");
    }
    else if(val < (-1*deadzone) )
    {
      m = 0;
      k = 1;
      //ROS_INFO("check-")
    } 
    else
    {
      m = 0;
      k = 0;
      //ROS_INFO("not thing");
    }
    axes_val += pow(2,7-j)*m;
    ++j;
    axes_val += pow(2,7-j)*k;
    ++j;

  }
  }
     for(size_t i =0,k=0 ;i< 8; ++i)
   {
     if(joy->axes[i]>deadzone)
     {
       cmd_2[k]=255*(joy->axes[i]);
     }
     else if(joy->axes[i]<(-1*deadzone))
     {
        cmd_2[k]  =255*(-(joy->axes[i]));
     }
     else {
       cmd_2[k]=0;
     }
     ++k;
  }
  dir += axes_val;
  //ROS_INFO("%d\r\n",dir);
  cmd_1 = dir;
  // unsigned char data_array[sizeof(cmd_1)]={0};
  // unsigned char analog_array[sizeof(cmd_2)]={0};
  // memcpy(data_array,&cmd_1,sizeof(cmd_1));
  // memcpy(analog_array,&cmd_2,sizeof(cmd_2));
  // ROS_INFO("%d\r\n",sizeof(cmd_1));
  // cmd<<'@'<<data_array[0]<<data_array[1]<<analog_array[0]<<analog_array[1]<<analog_array[2]<<analog_array[3]<<'$';
  // sp.write_some(boost::asio::buffer(cmd.str()));
  // std::cout << cmd.str();
  //cmd_1 = 0;
  //cmd_2 = 0;
  dir = 0;
  axes_val = 0;
  //cmd.str("");
}

void sendCommand(const ros::TimerEvent& event)
{
  unsigned char data_array[sizeof(cmd_1)]={0};
  unsigned char analog_array[sizeof(cmd_2)]={0};
  unsigned char sign_array[sizeof(cmd_3)]={0};
  memcpy(data_array,&cmd_1,sizeof(cmd_1));
  memcpy(analog_array,&cmd_2,sizeof(cmd_2));
  memcpy(sign_array,&cmd_3,sizeof(cmd_3));
  ROS_INFO("%d\r\n",sizeof(cmd_3));
  cmd<<'@'<<data_array[0]<<sign_array[0]<<analog_array[3]<<analog_array[7]<<'$';
  sp.write_some(boost::asio::buffer(cmd.str()));
  std::cout << cmd.str();
  //cmd_3=0;
  cmd.clear();
  cmd.str("");
  
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "irap_control");
  ROS_INFO("%s", "initial irap_control");
  ros::MultiThreadedSpinner spinner(4);
  ros::NodeHandle nh_;
  ros::Subscriber joy_sub_;
  ros::Subscriber sign_sub;
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 1, &joyCallback);
  sign_sub = nh_.subscribe<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes",1,&msgCallback);
  ros::Timer timer1 = nh_.createTimer(ros::Duration(0.1),sendCommand);
    try
    {
	      sp.open("/dev/ttyACM0");
        //sp.open("/dev/ttyS1");
        //sp.open("/dev/ttyUSB0");
        sp.set_option(boost::asio::serial_port::baud_rate(115200));
    }
    catch(...)
    {
        throw std::runtime_error("Unable to open serial_port");
        ROS_INFO("Success");
    }
    while(nh_.ok())
    {
      //ros::spin();
      spinner.spin();
    }
    return 0;
}

