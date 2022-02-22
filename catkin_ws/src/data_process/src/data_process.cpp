#include "data_process.h"
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

char* save_data;
char* undistort_image;

MapGenerationNode::MapGenerationNode():	lidar_index(0),camera_index(0), camera_captured(false), it(nh),
										init_camera_time(false), init_lidar_time(false)
{
    sub_lidar = nh.subscribe("/rslidar_points", 1000, &MapGenerationNode::lidarCallback, this);

	//ros::param::set("~image_transport", "compressed");

	sub_camera3 = nh.subscribe("/usb_cam3/image_raw", 1000, &MapGenerationNode::camera3Callback, this);
	//send message ***topic name should be different from original***
	image_publisher = nh.advertise<sensor_msgs::Image>("/usb_cam2/image_raw",1);

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
	sprintf(s, "./output/pcd/%06lld.pcd", lidar_index); 
	++lidar_index;
	camera_captured = false;

	pcl::fromROSMsg(*lidar, lidar_cloud);
	//remove nan and inf
	// The mapping tells you to what points of the old cloud the new ones correspond,
	// but we will not use it.
	std::vector<int> mapping;
	pcl::removeNaNFromPointCloud(lidar_cloud, lidar_cloud, mapping);

	//Building the transformation matrix
	transform_matrix_.translation() << trans_tx_, trans_ty_, trans_tz_;
  	transform_matrix_.rotate(Eigen::AngleAxisf(trans_yaw_ * M_PI / 180, Eigen::Vector3f::UnitZ()));
  	transform_matrix_.rotate(Eigen::AngleAxisf(trans_pitch_ * M_PI / 180, Eigen::Vector3f::UnitY()));
  	transform_matrix_.rotate(Eigen::AngleAxisf(trans_roll_ * M_PI / 180, Eigen::Vector3f::UnitX()));

  	//Transformation
	pcl::PointCloud<pcl::PointXYZI>::Ptr trans_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  	pcl::transformPointCloud(lidar_cloud, *trans_cloud_ptr, transform_matrix_);
	
	if(save_data)
	{
		//Save in .pcd
		pcl::io::savePCDFileASCII (s, lidar_cloud);
	}
}

//About Visual Image (Read & Save)
void MapGenerationNode::camera3Callback(const sensor_msgs::ImageConstPtr& camera)
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
	sprintf(s, "./output/png/%06lld.jpg", lidar_index--); 
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

	// 读取一副图片，不改变图片本身的颜色类型
	Mat src = cv_ptr->image;
	Mat distortion = src.clone();
	Mat camera_matrix = Mat(3, 3, CV_32FC1);
	Mat distortion_coefficients;
	Mat img_gray = cv_ptr->image;

	//导入相机内参和畸变系数矩阵
	FileStorage file_storage("./camera_data.xml", FileStorage::READ);
	file_storage["Camera_Matrix"] >> camera_matrix;
	file_storage["Distortion_Coefficients"] >> distortion_coefficients;
	file_storage.release();

	//矫正
	undistort(src, distortion, camera_matrix, distortion_coefficients);
	// cv::imshow("img", src);
	// cv::imshow("undistort", distortion);
	//cvtColor(distortion, img_gray, COLOR_RGB2GRAY);
  	sensor_msgs::ImagePtr output = cv_bridge::CvImage(std_msgs::Header(), "bgr8", distortion).toImageMsg();

    output->header.stamp = ros::Time::now();
	image_publisher.publish(output);


	if(save_data)
	{		
		imwrite(s, distortion);

		if(abs(camera_delta_time - lidar_derta_time_gelobal) > 10)
		{
			lidar_index--;
		}
	}
	camera_captured = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_generation");
    MapGenerationNode mapgeneration;
    return 0;
}

