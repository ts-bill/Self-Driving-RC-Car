#include <ros/ros.h>
#include "blocking_reader.h"
#include <iostream>
#include <string>
#include <sstream>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
//#include <sensor_msgs/Joy.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/ByteMultiArray.h"
char Arr[2];
void autodriveCallback(const std_msgs::ByteMultiArray::ConstPtr& array);

boost::asio::io_service io_s;
boost::asio::serial_port sp(io_s);

std_msgs::String numberclass;
std_msgs::String size;
bool incomemsg,_incomemsg;

uint8_t cmd_1; //16
uint8_t cmd_2[4];//RA up-down
uint8_t cmd_3=0;//RA right-left
uint8_t cmd_4;//LA up-down
uint8_t cmd_5;//LA right-lefto
char cmd_6[2];
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

void autodriveCallback(const std_msgs::ByteMultiArray::ConstPtr& array)
{
  int i = 0;
	for(std::vector<signed char>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
  {
       cmd_6[i] = *it;
       ROS_INFO("%d",cmd_6[i]);
       i++;

  }

}
  
  void sendCommand(const ros::TimerEvent& event)
{
  unsigned char data_array[sizeof(cmd_6)]={0};
  unsigned char sign_array[sizeof(cmd_3)]={0};
  memcpy(data_array,&cmd_6,sizeof(cmd_6));
  memcpy(sign_array,&cmd_3,sizeof(cmd_3));
  //ROS_INFO("%d\r\n",sizeof(cmd_3));
  cmd<<'A'<<data_array[0]<<data_array[1]<<sign_array[0]<<'$';
  sp.write_some(boost::asio::buffer(cmd.str()));
  //std::cout << cmd.str();
  cmd_6[0]=0;
  cmd_6[1]=0;
  cmd_3=0;
  cmd.clear();
  cmd.str("");
  
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "irap_control");
  ROS_INFO("%s", "initial irap_control");
  ros::MultiThreadedSpinner spinner(4);
  ros::NodeHandle nh_;
  ros::Subscriber autodrive_sub;
  ros::Subscriber sign_sub;
  autodrive_sub = nh_.subscribe<std_msgs::ByteMultiArray>("/autodrive" , 1, &autodriveCallback);
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

