#include "ros/ros.h"
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
std_msgs::String numberclass;
bool incomemsg;
class Subarduinoyolo
{
public:
  Subarduinoyolo()
  {
    classdetected_pub = nh.advertise<std_msgs::String>("/Classdetected", 1000);
    sub = nh.subscribe("/darknet_ros/bounding_boxes",100,&Subarduinoyolo::msgCallback, this);
  }

void msgCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
    //std::cout << "------------------------------" << endl;
    numberclass.data=msg->bounding_boxes[0].Class;
    classdetected_pub.publish(numberclass);
    //std::cout<<"Bouding Boxes (Class):" << numberclass <<endl;
    

}
private:
  ros::NodeHandle nh;
  ros::Publisher classdetected_pub;
  ros::Subscriber sub;
};
//void numberclassCallback(const std_msgs::Int8::ConstPtr& msg1)
//{
 // numberofclass = msg1->data;
//  ROS_INFO("I heard: [%i]", numberofclass);
//}
int main(int argc, char **argv)
{
ros::init(argc,argv,"coord_pixel_subscriber");
Subarduinoyolo SAYobject;
ros::spin();
return 0;
}

