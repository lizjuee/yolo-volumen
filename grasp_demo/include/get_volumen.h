#ifndef GET_VOLUMEN_H
#define GET_VOLUMEN_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <std_msgs/Float64.h>


using namespace cv;
using namespace std;

class ImageConverter
{
private:
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_depth; //接收深度图像
	image_transport::Subscriber image_sub_color; //接收彩色图像

	ros::Subscriber camera_info_sub_; //接收深度图像对应的相机参数话题
	ros::Subscriber object_sub; // Yolo的话题订阅
	ros::Publisher volumen_pub; // 发布物体大小的topic
	double real_x_min, real_xz_min, real_x_max, real_xz_max;
	double real_y_min, real_yz_min, real_y_max, real_yz_max;
	double Length, Width;
	double depth, depth_min, depth_max;
	double Volumen;
	int xmin, xmax, ymin, ymax;
	bool get_obj;
	string Object_class;

	sensor_msgs::CameraInfo camera_info;
	std_msgs::Float64 volumen_msg;
	/* Mat depthImage,colorImage; */
	Mat colorImage;
	Mat depthImage = Mat::zeros(480, 640, CV_16UC1); //camera_info

public:
	ImageConverter();

	~ImageConverter();

    void cameraInfoCb(const sensor_msgs::CameraInfo &msg);

    void imageDepthCb(const sensor_msgs::ImageConstPtr &msg);

    void imageColorCb(const sensor_msgs::ImageConstPtr &msg);
    
    void YoloCb(const darknet_ros_msgs::BoundingBoxes &yolo_temp);

};

#endif
