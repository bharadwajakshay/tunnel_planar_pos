/*
 * tunnel_planar_pos_utils.h
 *
 *  Created on: Sep 30, 2016
 *      Author: nuc
 */

#ifndef TUNNEL_PLANAR_POS_INCLUDE_TUNNEL_PLANAR_POS_UTILS_H_
#define TUNNEL_PLANAR_POS_INCLUDE_TUNNEL_PLANAR_POS_UTILS_H_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Imu.h"

#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/OccupancyGrid.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <std_msgs/Float64.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/Imu.h>

#include <pcl/common/common_headers.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac_model_parallel_line.h>
#include <pcl/octree/octree.h>
#include "pcl_ros/point_cloud.h"

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/registration/icp.h>
#include "pcl_ros/point_cloud.h"

#include <Eigen/SVD>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <math.h>
#include <xsens_slim/imuX.h>
#include <vector>


#include <sstream>
#include <vector>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/point_cloud_image_extractors.h>

//#include <boost/circular_buffer/base.hpp>

extern std::string topic_pc;
extern std::string topic_img;
extern std::string topic_imu;

#define IMU_TIME_DELAY 1400
#define SIZE_OF_VECTOR 500
class tunnel_planar_pos
{
public:
	tunnel_planar_pos();
	void callbackpointclouds(const sensor_msgs::PointCloud2::ConstPtr& msg);
	void pc_segmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud, Eigen::Vector4f min_pt, Eigen::Vector4f max_pt);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr imu_correction(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud);
	void image_segmentation(pcl::PointCloud<pcl::PointXYZRGB>  input_cloud);
	void callbackpointimu(const xsens_slim::imuX::ConstPtr& msg);
	void comp_hist_col_img(cv::Mat image);
	pcl::ModelCoefficients plane_est_svd(pcl::PointCloud<pcl::PointXYZRGB> point_cloud);
	double angle_btw_planes(pcl::ModelCoefficients plane1, pcl::ModelCoefficients plane2);
	double distance_frm_point_2_plane(pcl::PointXYZ point, pcl::ModelCoefficients plane);
	void extract_inliers(pcl::ModelCoefficients xmin, pcl::ModelCoefficients xmax, pcl::ModelCoefficients ymin,pcl::ModelCoefficients ymax);
private:
	ros::NodeHandle nh;
	//Topics to subscribe
	ros::Subscriber sub_pc;
	ros::Subscriber sub_imu;
	ros::Subscriber sub_img;
	ros::Publisher pub_slices;
	ros::Publisher pub_corrected_pc;
	ros::Publisher pub_xminp,pub_yminp,pub_xmaxp,pub_ymaxp;
	pcl::PointCloud<pcl::PointXYZRGB> xmin_plane, ymin_plane,xmax_plane,ymax_plane;
	pcl::PointCloud<pcl::PointXYZRGB> xmin_pc, ymin_pc,xmax_pc,ymax_pc,slices_pc;
	bool xmin,xmax,ymin,ymax;
	bool coeff_valid;
	std::vector <pcl::PointXYZRGB> xmin_points, xmax_points,ymin_points,ymax_points;
	bool first_imu_msg;
	geometry_msgs::Vector3 rpy;
	int imu_loop_itr;
	std::deque<xsens_slim::imuX> imu_message;

};



#endif /* TUNNEL_PLANAR_POS_INCLUDE_TUNNEL_PLANAR_POS_UTILS_H_ */
