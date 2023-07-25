#include "ros/ros.h"
#include <sstream>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
std_msgs::String numberclass;
std_msgs::String size;
bool incomemsg,_incomemsg;

void msgCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
    numberclass.data=msg->bounding_boxes[0].Class;
    //size.data = msg->bounding_boxes[0].xmin + "," +msg->bounding_boxes[0].xmax + "," + msg->bounding_boxes[0].ymin + "," + msg->bounding_boxes[0].ymax;
    //ROS_INFO("%s", size.data);
    //cout<<"Bouding Boxes (xmax):" << msg->bounding_boxes[0].xmax <<endl;
    //cout<<"Bouding Boxes (ymin):" << msg->bounding_boxes[0].ymin <<endl;
    //cout<<"Bouding Boxes (ymax):" << msg->bounding_boxes[0].ymax <<endl;
    incomemsg=1;
}

int main(int argc, char **argv)
{
ros::init(argc,argv,"coord_pixel_subscriber");
ros::NodeHandle nh;
ros::Publisher classdetected_pub = nh.advertise<std_msgs::String>("/Classdetected", 1000);
ros::Subscriber sub = nh.subscribe("/darknet_ros/bounding_boxes",1000,msgCallback);
ros::Rate loop_rate(10);
while(ros::ok())
{
std_msgs::String senddata;
std::stringstream ss;
ss << "no object";
if(incomemsg && (incomemsg != _incomemsg)) {
  senddata.data=numberclass.data;
  classdetected_pub.publish(senddata);
}
else if(!incomemsg && (incomemsg != _incomemsg)){
senddata.data=ss.str();
classdetected_pub.publish(senddata);
}
_incomemsg=incomemsg;
incomemsg=0;
ros::spinOnce();
loop_rate.sleep();
//loop_rate.sleep(10);
}
return 0;
}


