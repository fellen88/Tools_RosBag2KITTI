#include "map_generation_node.h"
//This code is to read the .bag files in ROS and decode them into .png and .pcd
//Coordinate system transformation function is optional. 
using namespace cv;

double trans_roll_  = 0.0;
double trans_pitch_ = 0.0;
double trans_yaw_   = 0.0;
double trans_tx_    = 0.0;
double trans_ty_    = 0.0;
double trans_tz_    = 0.0;
Eigen::Affine3f transform_matrix_ = Eigen::Affine3f::Identity();

MapGenerationNode::MapGenerationNode():	lidar_index(0),camera_index(0), camera_captured(false), it(nh),
										init_camera_time(false), init_lidar_time(false)
{
    sub_lidar = nh.subscribe("/rslidar_points", 1000, &MapGenerationNode::lidarCallback, this);

	//ros::param::set("~image_transport", "compressed");
	sub_camera200 = nh.subscribe("/usb_cam200/image_raw", 1000, &MapGenerationNode::camera200Callback, this);

	sub_camera1 = nh.subscribe("/usb_cam1/image_raw", 1000, &MapGenerationNode::camera1Callback, this);
	sub_camera2 = nh.subscribe("/usb_cam2/image_raw", 1000, &MapGenerationNode::camera2Callback, this);
	sub_camera3 = nh.subscribe("/usb_cam3/image_raw", 1000, &MapGenerationNode::camera3Callback, this);
	sub_camera4 = nh.subscribe("/usb_cam4/image_raw", 1000, &MapGenerationNode::camera4Callback, this);

    ros::spin();
}

//About Lidar Points Cloud (Read & Trans & Save)
void MapGenerationNode::lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& lidar)
{
	if(!init_lidar_time)
	{
		lidar_base_time = lidar->header.stamp.sec * 1e3 + lidar->header.stamp.nsec / 1e6;
		init_lidar_time = true;
	}

	long long lidar_delta_time = lidar->header.stamp.sec * 1e3 + lidar->header.stamp.nsec / 1e6 - lidar_base_time;
	std::cout<<lidar->header.stamp<<std::endl;
	ROS_INFO("get lidar : %lld ms", lidar_delta_time);
	lidar_derta_time_gelobal = lidar_delta_time;

	char s[200];
	sprintf(s, "/home/fellen/data_process/src/Tools_RosBag2KITTI/catkin_ws/output/pcd/%06lld.pcd", lidar_index); 
	++lidar_index;
	camera_captured = false;

	pcl::fromROSMsg(*lidar, lidar_cloud);

	//Building the transformation matrix
	transform_matrix_.translation() << trans_tx_, trans_ty_, trans_tz_;
  	transform_matrix_.rotate(Eigen::AngleAxisf(trans_yaw_ * M_PI / 180, Eigen::Vector3f::UnitZ()));
  	transform_matrix_.rotate(Eigen::AngleAxisf(trans_pitch_ * M_PI / 180, Eigen::Vector3f::UnitY()));
  	transform_matrix_.rotate(Eigen::AngleAxisf(trans_roll_ * M_PI / 180, Eigen::Vector3f::UnitX()));

  	//Transformation
	pcl::PointCloud<pcl::PointXYZI>::Ptr trans_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  	pcl::transformPointCloud(lidar_cloud, *trans_cloud_ptr, transform_matrix_);
	//Save in .pcd
	pcl::io::savePCDFileASCII (s, lidar_cloud);
}

//About Visual Image (Read & Save)
void MapGenerationNode::camera200Callback(const sensor_msgs::ImageConstPtr& camera)
{
	if(!init_lidar_time)
		return;

	if(!init_camera_time)
	{
		camera_base_time = camera->header.stamp.sec * 1e3 + camera->header.stamp.nsec / 1e6;
		init_camera_time = true;
	}

	long long camera_delta_time = camera->header.stamp.sec * 1e3 + camera->header.stamp.nsec / 1e6 - lidar_base_time;
	if(camera_captured)
	{
		// ROS_INFO("discard camera:"/home/fellen/LiDAR_Detection/data_process/src/Tools_RosBag2KITTI/catkin_ws/output %lld ms", camera_delta_time);
		return;
	}

	ROS_INFO("get camera200: %lld ms", camera_delta_time);
	char s[200];
	sprintf(s, "/home/fellen/data_process/src/Tools_RosBag2KITTI/catkin_ws/output/png/%06lld.png", lidar_index--); 
	lidar_index ++;
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(camera, sensor_msgs::image_encodings::BGR8); 
		//cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", camera->encoding.c_str());
	}
	Mat img_rgb = cv_ptr->image;
	imwrite(s, img_rgb);

	if(abs(camera_delta_time - lidar_derta_time_gelobal) > 10)
	{
		lidar_index--;
	}

	camera_captured = true;
}

//Camera 1 (Read & Save)
void MapGenerationNode::camera1Callback(const sensor_msgs::ImageConstPtr& camera)
{
	if(!init_camera_time)
	{
		camera_base_time = camera->header.stamp.sec * 1e3 + camera->header.stamp.nsec / 1e6;
		init_camera_time = true;
	}

	long long camera_delta_time = camera->header.stamp.sec * 1e3 + camera->header.stamp.nsec / 1e6 - camera_base_time;
	if(camera_captured)
	{
		// ROS_INFO("discard camera:"/home/fellen/LiDAR_Detection/data_process/src/Tools_RosBag2KITTI/catkin_ws/output %lld ms", camera_delta_time);
		return;
	}

	ROS_INFO("get camera1: %lld ms", camera_delta_time);
	char s[200];
	sprintf(s, "/home/fellen/data_process/src/Tools_RosBag2KITTI/catkin_ws/output/camera1/%06lld.png", camera_index);
	camera_index ++;

	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(camera, sensor_msgs::image_encodings::BGR8); 
		//cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", camera->encoding.c_str());
	}
	Mat img_rgb = cv_ptr->image;
	imwrite(s, img_rgb);
}

//Camera 2 (Read & Save)
void MapGenerationNode::camera2Callback(const sensor_msgs::ImageConstPtr& camera)
{
	if(!init_camera_time)
	{
		camera_base_time = camera->header.stamp.sec * 1e3 + camera->header.stamp.nsec / 1e6;
		init_camera_time = true;
	}

	long long camera_delta_time = camera->header.stamp.sec * 1e3 + camera->header.stamp.nsec / 1e6 - camera_base_time;
	if(camera_captured)
	{
		// ROS_INFO("discard camera:"/home/fellen/LiDAR_Detection/data_process/src/Tools_RosBag2KITTI/catkin_ws/output %lld ms", camera_delta_time);
		return;
	}

	ROS_INFO("get camera2: %lld ms", camera_delta_time);
	char s[200];
	sprintf(s, "/home/fellen/data_process/src/Tools_RosBag2KITTI/catkin_ws/output/camera2/%06lld.png", camera_index);
	camera_index ++;

	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(camera, sensor_msgs::image_encodings::BGR8); 
		//cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", camera->encoding.c_str());
	}
	Mat img_rgb = cv_ptr->image;
	imwrite(s, img_rgb);
}

//Camera 3 (Read & Save)
void MapGenerationNode::camera3Callback(const sensor_msgs::ImageConstPtr& camera)
{
	if(!init_camera_time)
	{
		camera_base_time = camera->header.stamp.sec * 1e3 + camera->header.stamp.nsec / 1e6;
		init_camera_time = true;
	}

	long long camera_delta_time = camera->header.stamp.sec * 1e3 + camera->header.stamp.nsec / 1e6 - camera_base_time;
	if(camera_captured)
	{
		// ROS_INFO("discard camera:"/home/fellen/LiDAR_Detection/data_process/src/Tools_RosBag2KITTI/catkin_ws/output %lld ms", camera_delta_time);
		return;
	}

	ROS_INFO("get camera3: %lld ms", camera_delta_time);
	char s[200];
	sprintf(s, "/home/fellen/data_process/src/Tools_RosBag2KITTI/catkin_ws/output/camera3/%06lld.png", camera_index);
	camera_index ++;

	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(camera, sensor_msgs::image_encodings::BGR8); 
		//cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", camera->encoding.c_str());
	}
	Mat img_rgb = cv_ptr->image;
	imwrite(s, img_rgb);
}

//Camera 4 (Read & Save)
void MapGenerationNode::camera4Callback(const sensor_msgs::ImageConstPtr& camera)
{
	if(!init_camera_time)
	{
		camera_base_time = camera->header.stamp.sec * 1e3 + camera->header.stamp.nsec / 1e6;
		init_camera_time = true;
	}

	long long camera_delta_time = camera->header.stamp.sec * 1e3 + camera->header.stamp.nsec / 1e6 - camera_base_time;
	if(camera_captured)
	{
		// ROS_INFO("discard camera:"/home/fellen/LiDAR_Detection/data_process/src/Tools_RosBag2KITTI/catkin_ws/output %lld ms", camera_delta_time);
		return;
	}

	ROS_INFO("get camera4: %lld ms", camera_delta_time);
	char s[200];
	sprintf(s, "/home/fellen/data_process/src/Tools_RosBag2KITTI/catkin_ws/output/camera4/%06lld.png", camera_index);
	camera_index ++;

	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(camera, sensor_msgs::image_encodings::BGR8); 
		//cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", camera->encoding.c_str());
	}
	Mat img_rgb = cv_ptr->image;
	imwrite(s, img_rgb);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_generation");
    MapGenerationNode mapgeneration;
    return 0;
}
