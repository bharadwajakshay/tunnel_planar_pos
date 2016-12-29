/*
 * tunnel_planar_pos_utils.cpp
 *
 *  Created on: Sep 30, 2016
 *      Author: nuc
 */

#include "tunnel_planar_pos_utils.h"


tunnel_planar_pos::tunnel_planar_pos()
{
	ROS_INFO("Topics Subscribed are:\n1. %s\n2. %s\n",topic_pc.c_str(),topic_imu.c_str());
	sub_pc=nh.subscribe(topic_pc.c_str(),1000,&tunnel_planar_pos::callbackpointclouds,this);
	sub_img=nh.subscribe(topic_img.c_str(),1000,&tunnel_planar_pos::callbacklwirimg,this);
	sub_imu=nh.subscribe(topic_imu.c_str(),1000,&tunnel_planar_pos::callbackpointimu,this);
	pub_x_plane = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("RANSAC_x_plane",1000);
	pub_y_plane = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("RANSAC_y_plane",1000);
	pub_z_plane = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("RANSAC_z_plane",1000);
	pub_x_line = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("x_line",1000);
	pub_y_line = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("y_line",1000);
	pub_slices = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("Slice_PC",1000);
	pub_xminp = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("X_min_plane",1000);
	pub_xmaxp = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("X_max_plane",1000);
	pub_yminp = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("Y_min_plane",1000);
	pub_ymaxp = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("Y_max_plane",1000);
	pub_corrected_pc = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("corrected_pt_cloud",1000);
	xmin=false,xmax=false;ymin=false;ymax=false;
	xmin_coeff=false, xmax_coeff=false, ymin_coeff=false, ymax_coeff =false;
	coeff_valid =false;
	first_imu_msg = true;
	imu_loop_itr = 0;
}

void tunnel_planar_pos::callbackpointclouds(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
	pcl::PCLPointCloud2 pcl_pc2;
	double z1=NAN,z2=NAN,z3=NAN;

	// Convert to PCP PointCloud2 structure
	pcl_conversions::toPCL(*msg,pcl_pc2);

	pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr slice_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud_sliced(new pcl::PointCloud<pcl::PointXYZRGB>);
	Eigen::Vector4f min_pt, max_pt;

	// Convert to PCL PointCloud structure
	pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

	// Copy to the converted point cloud
	pcl::copyPointCloud(*temp_cloud, *slice_cloud);

	// Convert to PCL PointCloud structure with RGB data
	pcl::fromPCLPointCloud2(pcl_pc2,*rgb_cloud);

	pcl::getMinMax3D(*rgb_cloud,min_pt,max_pt);
#ifdef _DEBUG
	std::cout<<"Minimum pt=\n\tx min = "<< min_pt[0]<< "\ty min = "<<min_pt[1]<<"\tz min ="<<min_pt[2]<<"\n";
	std::cout<<"Maximum pt=\n\tx max = "<< max_pt[0]<< "\ty max = "<<max_pt[1]<<"\tz max ="<<max_pt[2]<< "\n";
#endif
	/***********************************
	 * Extract image from point cloud
	 ***********************************/
	//setting up the point clouds
	xmin_pc.clear();
	ymin_pc.clear();
	xmax_pc.clear();
	ymax_pc.clear();
	std::vector<int> indices_nan;
	pcl::removeNaNFromPointCloud(*rgb_cloud,*rgb_cloud,indices_nan);

	this->pc_segmentation(rgb_cloud,min_pt,max_pt);
	/*******************************************************************
	 * publish Segmented POint clouds
	 *******************************************************************/
	//extract the Thermal image from the point cloud
	this->image_segmentation(slices_pc);
	/*************************************
	 *  Disable IMU correction
	 *************************************/
	//pub_corrected_pc.publish(tunnel_planar_pos::imu_correction(rgb_cloud));

	/*stacking the coeffitients
	 * xmin_coeff => Modelcoefficients[0]
	 * xmax_coeff => Modelcoefficients[1]
	 * ymin_coeff => Modelcoefficients[2]
	 * ymax_coeff => Modelcoefficients[3]
	 */
	pcl::ModelCoefficients xmin_coeff, xmax_coeff, ymin_coeff, ymax_coeff;	//Defining the coefficients
	/*********************************************************************************************
	 * Estimate the planes using Singular value Decomposition
	 ********************************************************************************************/
	if(xmin)
	{
		xmin_coeff = this->plane_est_svd(xmin_pc);
		std::cout<<"The xmin_coeff coefficents calculated using svd are\t"<<xmin_coeff.values[0]<<"\t"
				<<xmin_coeff.values[1]<<"\t"<<xmin_coeff.values[2]<<"\t"<<xmin_coeff.values[3]<<std::endl;
	}
	if(xmax)
	{
		xmax_coeff = this->plane_est_svd(xmax_pc);
		std::cout<<"The xmax_coeff coefficents calculated using svd are\t"<<xmax_coeff.values[0]<<"\t"
				<<xmax_coeff.values[1]<<"\t"<<xmax_coeff.values[2]<<"\t"<<xmax_coeff.values[3]<<std::endl;
	}
	if(ymin)
	{
		ymin_coeff = this->plane_est_svd(ymin_pc);
		std::cout<<"The ymin_coeff coefficents calculated using svd are\t"<<ymin_coeff.values[0]<<"\t"
				<<ymin_coeff.values[1]<<"\t"<<ymin_coeff.values[2]<<"\t"<<ymin_coeff.values[3]<<std::endl;
	}
	if(ymax)
	{
		ymax_coeff = this->plane_est_svd(ymax_pc);
		std::cout<<"The ymax_coeff coefficents calculated using svd are\t"<<ymax_coeff.values[0]<<"\t"
				<<ymax_coeff.values[1]<<"\t"<<ymax_coeff.values[2]<<"\t"<<ymax_coeff.values[3]<<std::endl;
	}
	/***********************************************************************************************
	 *  Check for perpendicular planes between the X-plane and the Y-plane
	 ***********************************************************************************************/

	if(xmin)
	{
		if(ymin)
			std::cout<<"The angle between xmin plane and ymin plane = "<<this->angle_btw_planes(xmin_coeff,ymin_coeff)<<std::endl;
		if(ymax)
			std::cout<<"The angle between xmin plane and ymax plane = "<<this->angle_btw_planes(xmin_coeff,ymax_coeff)<<std::endl;
	}
	else if(xmax)
	{
		if(ymin)
			std::cout<<"The angle between xmax plane and ymin plane = "<<this->angle_btw_planes(xmax_coeff,ymin_coeff)<<std::endl;
		if(ymax)
			std::cout<<"The angle between xmax plane and ymax plane = "<<this->angle_btw_planes(xmax_coeff,ymax_coeff)<<std::endl;
	}

	/***********************************************************************************************
	 *  Check for parallel planes between the X-plane and the Y-plane
	 ***********************************************************************************************/

	if(xmin && xmax)
		std::cout<<"The angle between xmin plane and xmax plane = "<<this->angle_btw_planes(xmin_coeff,xmax_coeff)<<std::endl;
	else if(ymin && ymax)
		if(ymax)
			std::cout<<"The angle between ymin plane and ymax plane = "<<this->angle_btw_planes(ymin_coeff,ymax_coeff)<<std::endl;

}

/****************************************************************************************
 * Extraction of planar equations from the segmented point clouds
 ***************************************************************************************/
pcl::ModelCoefficients tunnel_planar_pos::plane_est_svd(pcl::PointCloud<pcl::PointXYZRGB> point_cloud)
{
	Eigen::MatrixXd points_3D(3,point_cloud.width);
	//assigning the points from point cloud to matrix
	for (int i=0;i<point_cloud.width;i++)
	{
		points_3D(0,i) = point_cloud.at(i).x;
		points_3D(1,i) = point_cloud.at(i).y;
		points_3D(2,i) = point_cloud.at(i).z;
	}

	// calcaulating the centroid of the pointcloud
	Eigen::MatrixXd centroid = points_3D.rowwise().mean();
	//std::cout<<"The centroid of the pointclouds is given by:\t"<<centroid<<std::endl;

	//subtract the centroid from points
	points_3D.row(0).array() -= centroid(0);
	points_3D.row(1).array() -= centroid(1);
	points_3D.row(2).array() -= centroid(2);

	//calculate the SVD of points_3D matrix
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(points_3D,Eigen::ComputeFullU);
	Eigen::MatrixXd U_MAT = svd.matrixU();
	//std::cout<<"U matrix transpose is:"<<U_MAT<<std::endl<<std::endl<<"U matrix is:"<<svd.matrixU()<<std::endl;

	/*********************************************************************************************
	 * caculating d by sybstituting the centroid back in the quation
	 * 		aCx+bCy+cCz = -d
	 ********************************************************************************************/
	//double d = -((U_MAT(0,2)*points_3D(0,1))+ (U_MAT(1,2)*points_3D(1,1)) + (U_MAT(1,2)*points_3D(1,2)));
	double d = -((U_MAT(0,2)*centroid(0))+ (U_MAT(1,2)*centroid(1)) + (U_MAT(2,2)*centroid(2)));

	pcl::ModelCoefficients normals;
	normals.values.push_back(U_MAT(0,2));
	normals.values.push_back(U_MAT(1,2));
	normals.values.push_back(U_MAT(2,2));
	normals.values.push_back(d);
	return(normals);

}

/*
 *
 */
void tunnel_planar_pos::image_segmentation(pcl::PointCloud<pcl::PointXYZRGB> input_cloud)
{
	/**************************************************************************
	 * Create an Image extraction object to extract the image
	 **************************************************************************/
	//check if the pc is organised
	/*if(!input_cloud->isOrganized())
		return;
	//Define an OPENCV Image
	cv::Mat rgb_image(input_cloud->height,input_cloud->width,CV_8UC3);
	for(int i=0;i<input_cloud->height;i++)
		for(int j=0;j<input_cloud->width;j++)
		{
			uint32_t rgb = *reinterpret_cast<int*>(&input_cloud->at(j,i).rgb);
			cv::Vec3b color;
			color[0] = (rgb >> 16) & 0x0000ff;
			color[2] = (rgb >> 18) & 0x0000ff;
			color[1] = (rgb)	   & 0x0000ff;
			rgb_image.at<cv::Vec3b>(cv::Point(j,i)) = color;
		}*/
	//cv::imshow("recovered IR image",rgb_image);
	//cv::waitKey(0);
	pcl::PCLImage image;
	pcl::io::PointCloudImageExtractorFromRGBField<pcl::PointXYZRGB> img_ext;
	img_ext.extract(input_cloud,image);
	std::cout<<image.width<<std::endl;

}

void tunnel_planar_pos::callbacklwirimg(const sensor_msgs::Image::ConstPtr& msg)
{
	cv_bridge::CvImagePtr ir_img_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
	cv::Mat lwir_image = ir_img_ptr->image;
	//cv::cvtColor(lwir_image,lwir_image,CV_BGR2GRAY);
	/*cv::inRange(lwir_image, cv::Scalar(0, 0, 175), cv::Scalar(255, 255, 235), lwir_image);
	cv::imshow("Thresholded image", lwir_image);
	cv::waitKey(0);
	cv::*/

	//apply SURF object detection
	int min_hessian = 400;

}

void tunnel_planar_pos::pc_segmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud, Eigen::Vector4f min_pt, Eigen::Vector4f max_pt)
{
	std::vector <pcl::PointCloud<pcl::PointXYZRGB> > segmented_pc;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr xmin_plane (new pcl::PointCloud<pcl::PointXYZRGB>),ymin_plane (new pcl::PointCloud<pcl::PointXYZRGB>),
			xmax_plane (new pcl::PointCloud<pcl::PointXYZRGB>),ymax_plane(new pcl::PointCloud<pcl::PointXYZRGB>),slices (new pcl::PointCloud<pcl::PointXYZRGB>);
	xmin_plane->header=input_cloud->header;
	ymin_plane->header=input_cloud->header;
	xmax_plane->header=input_cloud->header;
	ymax_plane->header=input_cloud->header;
	slices->header=input_cloud->header;
	xmin_plane->clear();
	ymin_plane->clear();
	xmax_plane->clear();
	ymax_plane->clear();
	slices->clear();
	for(int i=0;i<input_cloud->width;i++)
	{
		if((input_cloud->at(i).x!=0) && (input_cloud->at(i).y!=0)&&((input_cloud->at(i).z!=0)&&(input_cloud->at(i).z<7)))
			if((min_pt[0]-input_cloud->at(i).x<0)&&(min_pt[0]-input_cloud->at(i).x>-0.2))
				xmin_plane->push_back(input_cloud->at(i));
			else if((min_pt[1]-input_cloud->at(i).y<0)&&((min_pt[1]-input_cloud->at(i).y)>-0.2))
				ymin_plane->push_back(input_cloud->at(i));
			else if((max_pt[0]-input_cloud->at(i).x)<0.20)
				xmax_plane->push_back(input_cloud->at(i));
			else if((max_pt[1]-input_cloud->at(i).y)<0.20)
				ymax_plane->push_back(input_cloud->at(i));
			else
				if(((input_cloud->at(i).z >1.9) && (input_cloud->at(i).z < 1.95))
						||((input_cloud->at(i).z >1.4) && (input_cloud->at(i).z < 1.45))
						||((input_cloud->at(i).z >2.4) && (input_cloud->at(i).z < 2.45))
						||((input_cloud->at(i).z >2.9) && (input_cloud->at(i).z < 2.95))
						||((input_cloud->at(i).z >3.4) && (input_cloud->at(i).z < 3.45)))
				{
					if ((input_cloud->at(i).z >1.9) && (input_cloud->at(i).z < 1.95))
						input_cloud->at(i).z=1.9;
					else if ((input_cloud->at(i).z >2.4) && (input_cloud->at(i).z < 2.45))
						input_cloud->at(i).z=2.4;
					else if ((input_cloud->at(i).z >1.4) && (input_cloud->at(i).z < 1.45))
						input_cloud->at(i).z=1.4;
					else if ((input_cloud->at(i).z >2.9) && (input_cloud->at(i).z < 2.95))
						input_cloud->at(i).z=2.9;
					else
						input_cloud->at(i).z=3.4;

					slices->push_back(input_cloud->at(i));
				}
	}
	if(xmin_plane->width>5000)
	{
		pcl::copyPointCloud(*xmin_plane,xmin_pc);
		xmin=true;
	}
	if(ymin_plane->width>5000)
	{
		pcl::copyPointCloud(*ymin_plane,ymin_pc);
		ymin=true;
	}
	if(xmax_plane->width>5000)
	{
		pcl::copyPointCloud(*xmax_plane,xmax_pc);
		xmax=true;
	}
	if(ymax_plane->width>5000)
	{
		pcl::copyPointCloud(*ymax_plane,ymax_pc);
		ymax=true;
	}
	pcl::copyPointCloud(*slices,slices_pc);

}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr tunnel_planar_pos::imu_correction(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr corrected_pc (new pcl::PointCloud<pcl::PointXYZRGB>);
	corrected_pc->header=input_cloud->header;
	corrected_pc->clear();
	double phi=0,theta=0,psi=0;
	// Stamp is expressed in micro seconds, convert to seconds, Time delay is in milliseconds, convert to seconds
	double pc_time = (input_cloud->header.stamp * 1e-6) - (IMU_TIME_DELAY * 1e-3);
	int itiration_no = 0;
	double residual,imu_time,min=100;
	for(int i=0;i<imu_message.size();i++)
	{
		imu_time = imu_message.at(i).header.stamp.toSec();
		residual = fabs(imu_time - pc_time);
		if(residual< min)
		{
			min = residual;
			itiration_no = i;
		}
	}
	// Copy roll pitch and yaw and convert angle from degrees to radians
	phi = ((imu_message.at(itiration_no).rpy.x)*M_PI)/180.0 ;
	theta = ((imu_message.at(itiration_no).rpy.y)*M_PI)/180.0 ;
	psi = ((imu_message.at(itiration_no).rpy.z)*M_PI)/180.0;
	Eigen::Matrix3d conv_mat;
	conv_mat(0,0) = cos(theta)*cos(psi);
	conv_mat(0,1) = (-cos(phi)*sin(psi))+(sin(phi)*sin(theta)*cos(psi));
	conv_mat(0,2) = (sin(phi)*sin(psi))+(cos(phi)*sin(theta)*cos(phi));
	conv_mat(1,0) = cos(theta)*sin(phi);
	conv_mat(1,1) = (cos(phi)*cos(psi))+(sin(phi)*sin(theta)*sin(psi));
	conv_mat(1,2) = (-sin(phi)*cos(psi))+(cos(phi)*sin(theta)*sin(psi));
	conv_mat(2,0) = -sin(theta);
	conv_mat(2,1) = sin(phi)*cos(theta);
	conv_mat(2,2) = cos(phi)*cos(theta);
	Eigen::Matrix<double,3,1> orig_point,conv_point;
	pcl::PointXYZRGB out_point;
	for(int i=0;i<input_cloud->size();i++)
	{
		out_point = input_cloud->at(i);
		orig_point(0,0) = input_cloud->at(i).x;
		orig_point(1,0) = input_cloud->at(i).y;
		orig_point(2,0) = input_cloud->at(i).z;
		conv_point = conv_mat * orig_point;
		out_point.x = conv_point(0,0);
		out_point.y =  conv_point(1,0);
		out_point.z = conv_point(2,0);
		corrected_pc->push_back(out_point);
	}

	return(corrected_pc);
}

double tunnel_planar_pos::angle_btw_planes(pcl::ModelCoefficients plane1, pcl::ModelCoefficients plane2)
{
	double dot_prod = (plane1.values.at(0) * plane2.values.at(0))
			+ (plane1.values.at(1) * plane2.values.at(1))
			+ (plane1.values.at(2) * plane2.values.at(2));
	double mag_p1 = std::sqrt((plane1.values.at(0)*plane1.values.at(0))
			+ (plane1.values.at(1) * plane1.values.at(1))
			+ (plane1.values.at(2) * plane1.values.at(2)));
	double mag_p2 = std::sqrt((plane2.values.at(0)*plane2.values.at(0))
			+ (plane2.values.at(1) * plane2.values.at(1))
			+ (plane2.values.at(2) * plane2.values.at(2)));
	double angle = std::acos(dot_prod/(mag_p1 + mag_p2));
	return((angle*180)/M_PI);
}
void tunnel_planar_pos::callbackpointimu(const xsens_slim::imuX::ConstPtr& msg)
{

	geometry_msgs::Vector3 imu_rpy;
	xsens_slim::imuX local_msg = *msg;
	if(first_imu_msg)
	{
		imu_rpy.x=msg->rpy.x;
		imu_rpy.y=msg->rpy.y;
		imu_rpy.z=msg->rpy.z;
		first_imu_msg = false;
	}
	local_msg.rpy.x = local_msg.rpy.x - imu_rpy.x;
	local_msg.rpy.y = local_msg.rpy.y - imu_rpy.y;
	local_msg.rpy.z = local_msg.rpy.z - imu_rpy.z;
	if(imu_loop_itr < SIZE_OF_VECTOR)
		imu_message.push_back(local_msg);
	else
	{
		imu_message.pop_front();
		imu_message.push_back(local_msg);
	}

}

