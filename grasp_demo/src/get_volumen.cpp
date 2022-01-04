#include "get_volumen.h"

ImageConverter::ImageConverter() : it_(nh_), xmin(0), xmax(0), ymin(0), ymax(0), get_obj(false)
{
	// topic sub:
	image_sub_depth = it_.subscribe("/cam_1/aligned_depth_to_color/image_raw",
									1, &ImageConverter::imageDepthCb, this);

	image_sub_color = it_.subscribe("/cam_1/color/image_raw", 1,
									&ImageConverter::imageColorCb, this);
	camera_info_sub_ =
		nh_.subscribe("/cam_1/aligned_depth_to_color/camera_info", 1,
					  &ImageConverter::cameraInfoCb, this);

	object_sub = nh_.subscribe("/darknet_ros/bounding_boxes", 1, &ImageConverter::YoloCb, this);
	volumen_pub = nh_.advertise<std_msgs::Float64>("volumen", 1);
}

ImageConverter::~ImageConverter()
{
}

void ImageConverter::cameraInfoCb(const sensor_msgs::CameraInfo &msg)
{
	camera_info = msg;
}

void ImageConverter::imageDepthCb(const sensor_msgs::ImageConstPtr &msg)
{
	cv_bridge::CvImagePtr cv_ptr;

	try
	{
		cv_ptr =
			cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
		depthImage = cv_ptr->image;
	}
	catch (cv_bridge::Exception &e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
}

void ImageConverter::imageColorCb(const sensor_msgs::ImageConstPtr &msg)
{
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		colorImage = cv_ptr->image;
	}
	catch (cv_bridge::Exception &e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	if (get_obj)
	{
		//get Length
		real_xz_min = 0.001 * depthImage.at<u_int16_t>((ymin + ymax) / 2, xmin);
		real_x_min =
			(xmin - camera_info.K.at(2)) / camera_info.K.at(0) * real_xz_min;
		real_xz_max = 0.001 * depthImage.at<u_int16_t>((ymin + ymax) / 2, xmax);
		real_x_max =
			(xmax - camera_info.K.at(2)) / camera_info.K.at(0) * real_xz_max;
		Length = real_x_max - real_x_min ;
		//get Width
		real_yz_min = 0.001 * depthImage.at<u_int16_t>(ymin ,(xmin + xmax) / 2 );
		real_y_min =
			(ymin - camera_info.K.at(2)) / camera_info.K.at(0) * real_yz_min;
		real_yz_max = 0.001 * depthImage.at<u_int16_t>(ymax, (xmin + xmax) / 2 );
		real_y_max =
			(ymax - camera_info.K.at(2)) / camera_info.K.at(0) * real_yz_max;
		Width = real_y_max - real_y_min ;
		//get Height
		double depth_center = 0.001 * depthImage.at<u_int16_t>((ymin + ymax) / 2, (xmin + xmax) / 2);
		depth_min = depth_center;
		depth_max = depth_center;
		for (int i = xmin - 20; i < xmax + 20; i = i + 10)
		{
			double depth = 0.001 * depthImage.at<u_int16_t>((ymin + ymax) / 2, i);
			if (depth > depth_max && depth - depth_max < 0.05)
			{
				depth_max = depth;
			}
			if (depth < depth_min && depth != 0 )
			{
				depth_min = depth;
			}
		}
		depth = depth_max - depth_min;
		Volumen = depth * (Length / 2.0) * (Width / 2.0) * (4.0/3.0)*3.14;
		std:: cout << Volumen << std::endl;
		volumen_msg.data = Volumen;
		volumen_pub.publish(volumen_msg);
	}
	else
	{
		volumen_msg.data = 0.0;
		volumen_pub.publish(volumen_msg);
	}
	get_obj = false;
}

void ImageConverter::YoloCb(const darknet_ros_msgs::BoundingBoxes &yolo_temp)
{
	get_obj = true;
	xmin = yolo_temp.bounding_boxes[0].xmin;
	xmax = yolo_temp.bounding_boxes[0].xmax;
	ymin = yolo_temp.bounding_boxes[0].ymin;
	ymax = yolo_temp.bounding_boxes[0].ymax;
	std:: cout << xmin << std:: endl;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "get_obj_volumen");
	ImageConverter imageconverter;
	ros::spin();
	return (0);
}
