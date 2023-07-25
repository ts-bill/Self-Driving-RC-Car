#include "ros/ros.h"
#include <sstream>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
std_msgs::String numberclass;
bool incomemsg;
void msgCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
    numberclass.data=msg->bounding_boxes[0].Class;
    incomemsg=1;
}
int main(int argc, char **argv)
{
ros::init(argc,argv,"coord_pixel_subscriber");
ros::NodeHandle nh;
ros::Publisher classdetected_pub = nh.advertise<std_msgs::String>("/Classdetected", 1000);
ros::Subscriber sub = nh.subscribe("/darknet_ros/bounding_boxes",100,msgCallback); 
ros::Rate loop_rate(10);
while(ros::ok())
{
std_msgs::String senddata;
std::stringstream ss;
ss << "no object";
if(incomemsg) {
  senddata.data=numberclass.data;
}
else {
senddata.data=ss.str();
}
classdetected_pub.publish(senddata);
incomemsg=0;
ros::spinOnce();
loop_rate.sleep();
//loop_rate.sleep(10);
}
return 0;
}

