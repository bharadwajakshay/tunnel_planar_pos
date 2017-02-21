/*
 * tunnel_planar_pos_utils.h
 *
 *  Created on: Sep 30, 2016
 *      Author: nuc
 */

#ifndef TUNNEL_PLANAR_POS_INCLUDE_TUNNEL_PLANAR_POS_UTILS_H_
#define TUNNEL_PLANAR_POS_INCLUDE_TUNNEL_PLANAR_POS_UTILS_H_
#include <iostream>
#include <fstream>

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

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/point_cloud_image_extractors.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/centroid.h>


///#include <chrono>


//#include <boost/circular_buffer/base.hpp>

extern std::string topic_pc;
extern std::string topic_img;
extern std::string topic_imu;

#define IMU_TIME_DELAY 1400
#define SIZE_OF_VECTOR 500
#define PLANAR_FIT 1
#define LINE_FIT 2
#define NO_OF_SLICES 20
#define NO_OF_SLICES_X 20
#define MEDIAN_ELEMENT 9
#define IMAGE_WIDTH 640
#define IMAGE_HEIGHT 480
#define IMAGE_DEPTH 3

#define _DISP_ALL_POINTS
class tunnel_planar_pos
{
public:
	tunnel_planar_pos();
	~tunnel_planar_pos();
	void callbackpointclouds(const sensor_msgs::PointCloud2::ConstPtr& msg);
	void pc_segmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud,Eigen::Vector4f pt_min,Eigen::Vector4f pt_max,
			Eigen::Vector4f& xmin_coeff,Eigen::Vector4f& xmax_coeff,Eigen::Vector4f& ymin_coeff,Eigen::Vector4f& ymax_coeff, std::vector<Eigen::Vector4f> Z_normals);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr imu_correction(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud);
	void image_segmentation(pcl::PointCloud<pcl::PointXYZRGB>  input_cloud);
	void callbackpointimu(const xsens_slim::imuX::ConstPtr& msg);
	void comp_hist_col_img(cv::Mat image);
	pcl::ModelCoefficients plane_est_svd(pcl::PointCloud<pcl::PointXYZRGB> point_cloud, int func);
	pcl::ModelCoefficients plane_est_svd(Eigen::MatrixXf points);
	double angle_btw_planes(pcl::ModelCoefficients plane1, pcl::ModelCoefficients plane2);
	double distance_frm_point_2_plane(pcl::PointXYZI point, pcl::ModelCoefficients plane);
	void extract_inliers(pcl::ModelCoefficients xmin, pcl::ModelCoefficients xmax, pcl::ModelCoefficients ymin,pcl::ModelCoefficients ymax);
	std::vector<double> chng_angl_orient(pcl::ModelCoefficients current_coeff, pcl::ModelCoefficients prev_coeff, pcl::ModelCoefficients first_coeff);
	void normal_extrct_org_pc(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud);
	cv::Mat ExtractEdgeImg(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);
	cv::Mat ApplyHoughLineTranform(cv::Mat input_image);
	void PlanarExtraction(pcl::PointCloud<pcl::PointXYZRGB> point_cloud);
	std::vector<pcl::ModelCoefficients> process_single_slice(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud);
	pcl::PointCloud<pcl::PointXYZRGB> pc_segmentation_y(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud, Eigen::Vector4f min_pt, Eigen::Vector4f max_pt);
	void parse_point_data(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud);
	void array_segmentation(Eigen::Vector4f pt_min,Eigen::Vector4f pt_max);
	void ProcessSingleSlice(pcl::PointCloud<pcl::PointXYZRGB> slice, pcl::PointCloud<pcl::PointXYZRGB>& minx, pcl::PointCloud<pcl::PointXYZRGB>& maxx,
			pcl::PointCloud<pcl::PointXYZRGB>& miny, pcl::PointCloud<pcl::PointXYZRGB>& maxy);
	Eigen::VectorXf LineEsitmationRANSAC(pcl::PointCloud<pcl::PointXYZRGB> input_cloud, pcl::PointCloud<pcl::PointNormal>::Ptr& ptcld_norm);
	void SegmentXYCloud(pcl::PointCloud<pcl::PointXYZRGB> xy_cloud,std::vector<Eigen::Vector4f>& Z_normals);
	bool ParallelZPlaneRANSAC(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud, Eigen::Vector4f& model_coeff, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& filtered_cloud);
	void ClusteringEucledian(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud);
	float StdCalc(std::vector<float> depth_field, float sum);
private:
	ros::NodeHandle nh;
	//Topics to subscribe
	ros::Subscriber sub_pc;
	ros::Subscriber sub_imu;
	ros::Subscriber sub_img;
	ros::Publisher pub_z_slices;
	ros::Publisher pub_y_slices;
	ros::Publisher pub_x_slices;
	ros::Publisher pub_corrected_pc;
	ros::Publisher pub_edge_img;
	ros::Publisher pub_detected_img;

#ifdef _DISP_ALL_POINTS
	ros::Publisher pub_S0_xmin;
	ros::Publisher pub_S0_xmax;
	ros::Publisher pub_S0_ymin;
	ros::Publisher pub_S0_ymax;
	ros::Publisher pub_xy_Main;
	ros::Publisher pub_xy_seg1;
	ros::Publisher pub_xy_seg2;
	ros::Publisher pub_xy_seg3;
	ros::Publisher pub_xy_seg4;
	ros::Publisher pub_xy_seg5;
	ros::Publisher pub_xy_seg6;
	ros::Publisher pub_xy_seg7;
	ros::Publisher pub_xy_seg8;
#endif

#ifndef _SAVE_EXEC_TIME
	ros::Publisher pub_xmin_plane;
	ros::Publisher pub_xmax_plane;
	ros::Publisher pub_ymin_plane;
	ros::Publisher pub_ymax_plane;
	ros::Publisher pub_Z_plane;
#endif



	//ros::Publisher pub_xminp,pub_yminp,pub_xmaxp,pub_ymaxp;
	ros::Publisher pub_ext_planes, pub_plane_extracted;
	pcl::PointCloud<pcl::PointXYZI> all_planes;
	pcl::PointCloud<pcl::PointXYZRGB> all_planes_RGB;
	pcl::PointCloud<pcl::PointXYZRGB> xmin_pc, ymin_pc,xmax_pc,ymax_pc,slices_z_pc;

	bool coeff_valid;
	std::vector <pcl::PointXYZRGB> xmin_points, xmax_points,ymin_points,ymax_points;
	bool first_imu_msg,first_pc_msg;
	geometry_msgs::Vector3 rpy;
	int imu_loop_itr;
	std::deque<xsens_slim::imuX> imu_message;

	float points[IMAGE_WIDTH][IMAGE_HEIGHT][IMAGE_DEPTH];

	pcl::PointCloud<pcl::PointXYZRGB> previous_cloud;
	pcl::ModelCoefficients first_xmin_coeff, first_xmax_coeff, first_ymin_coeff, first_ymax_coeff;
	pcl::ModelCoefficients prev_xmin_coeff, prev_xmax_coeff, prev_ymin_coeff, prev_ymax_coeff;

	//FILE i/O operations
	std::ofstream total_exe_time;
	std::ofstream seg_pcl_time;
	std::ofstream seg_array_time;
};



#endif /* TUNNEL_PLANAR_POS_INCLUDE_TUNNEL_PLANAR_POS_UTILS_H_ */
