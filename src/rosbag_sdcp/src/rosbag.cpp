// #include <message_filters/subscriber.h>
// #include <message_filters/time_synchronizer.h>
// #include <message_filters/synchronizer.h>
// #include <message_filters/sync_policies/exact_time.h>
// #include <sensor_msgs/Image.h>
// #include <sensor_msgs/Joy.h>

// using namespace sensor_msgs;
// using namespace message_filters;

// // void callback(const ImageConstPtr& image, const sensor_msgs::Joy::ConstPtr& msg)
// // {
// //   ROS_INFO("gogoprojectgo");
// // }

// void callback(const ImageConstPtr& image, const JoyConstPtr& joy)
//  {
//    ROS_INFO("gogoprojectgo");
//  }

// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, "rosbag_node");
//   ROS_INFO("go");
//   ros::NodeHandle nh;
//   message_filters::Subscriber<Image> imagelane_sub(nh,"/lanedetectcam/usb_cam/image_raw",1);
//   //message_filters::Subscriber<Image> imagesign_sub(nh,"/lanedetectcam/usb_cam/image_raw",1);
//   //message_filters::Subscriber<CameraInfo> info_sub(nh,"/lanedetectcam/usb_cam/camera_info",1);
//   message_filters::Subscriber<Joy> joy_sub(nh, "/joy",1);
//   ApproximateTimeSynchronizeTimeSynchronizer<Image,Joy> sync(imagelane_sub, joy_sub , 10);
//   sync.registerCallback(boost::bind(&callback, _1, _2));

//   ros::spin();

//   return 0;
// }

#include "std_msgs/Int32.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Joy.h>
#include "rosbag/bag.h"

using namespace sensor_msgs;
using namespace message_filters;
//rosbag::Bag bag("test.bag", rosbag::bagmode::Write);
//int i=0;
ros::Publisher image1_pub;
ros::Publisher image2_pub;
ros::Publisher joy_pub;
int k;
int i;


void callback(const CompressedImageConstPtr& _image1, const CompressedImageConstPtr& _image2 ,const JoyConstPtr& _joy)
{
    image1_pub.publish(_image1);
    image2_pub.publish(_image2);
    joy_pub.publish(_joy);
    //i.data = 42;
    //bag.write('image_record',image->_image);
    //bag.write('joy_record',joy->_joy);
  i++;
  ROS_INFO("%d",i);
  //bag.write("numbers", ros::Time::now(), i);

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rosbag_node");
  ros::NodeHandle nh;
  ROS_INFO("go");
  message_filters::Subscriber<CompressedImage> image1_sub(nh,"/lanedetectcam/usb_cam/image_raw/compressed", 1);
  message_filters::Subscriber<CompressedImage> image2_sub(nh, "/signdetectcam/usb_cam/image_raw/compressed", 1);
  message_filters::Subscriber<Joy> joy_sub(nh, "/joy",1);

  image1_pub = nh.advertise<CompressedImage>("/synchronized/lanedetectcam/usb_cam/image_raw/compressed" , 100);
  image2_pub = nh.advertise<CompressedImage>("/synchronized/signdetectcam/usb_cam/image_raw/compressed" , 100);
  joy_pub = nh.advertise<Joy>("/synchronized/joy" , 100);
  
  //TimeSynchronizer<Image, Joy> sync(image1_sub, joy_sub, 10);
  typedef sync_policies::ApproximateTime<CompressedImage,CompressedImage,Joy> MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(30), image1_sub, image2_sub, joy_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2,_3));
//while (ros::ok())
  //{
  ros::spin();
  //}
  //}
  //bag.close();

  return 0;
}
    // #include <rosbag/bag.h>
    // #include <std_msgs/Int32.h>
    // #include <std_msgs/String.h>

    // rosbag::Bag bag;
    // bag.open("test.bag", rosbag::bagmode::Write);

    // std_msgs::String str;
    // str.data = std::string("foo");

    // std_msgs::Int32 i;
    // i.data = 42;

    // bag.write("chatter", ros::Time::now(), str);
    // bag.write("numbers", ros::Time::now(), i);

    // bag.close();