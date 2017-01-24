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
	sub_imu=nh.subscribe(topic_imu.c_str(),1000,&tunnel_planar_pos::callbackpointimu,this);

	pub_slices = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/tunnel_planar_pos/Slice_PC",1000);
	pub_ext_planes = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/tunnel_planar_pos/extracted_planes",1000);
	pub_plane_extracted = nh.advertise<pcl::PointCloud<pcl::PointXYZI> >("/tunnel_planar_pos/pc_planes_removed",1000);
	pub_corrected_pc = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/tunnel_planar_pos/corrected_pt_cloud",1000);
	pub_edge_img = nh.advertise<sensor_msgs::Image>("/tunnel_planar_pos/EdgeImage",1000);

	xmin=false,xmax=false;ymin=false;ymax=false;
	coeff_valid =false;
	first_imu_msg = true;
	imu_loop_itr = 0;
	first_pc_msg = true;
}

void tunnel_planar_pos::callbackpointclouds(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
	ros::Time start_time = ros::Time::now();
	pcl::PCLPointCloud2 pcl_pc2;
	double z1=NAN,z2=NAN,z3=NAN;

	//set the origin of the sensors to x=0, y=0, z=0
	pcl::PointXYZ origin(0,0,0);

	// Convert to PCP PointCloud2 structure
	pcl_conversions::toPCL(*msg,pcl_pc2);

	pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr slice_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud_sliced(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB> back_up_pc;
	Eigen::Vector4f min_pt, max_pt;

	// Convert to PCL PointCloud structure
	pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

	// Copy to the converted point cloud
	pcl::copyPointCloud(*temp_cloud, *slice_cloud);

	// Convert to PCL PointCloud structure with RGB data
	pcl::fromPCLPointCloud2(pcl_pc2,*rgb_cloud);
	pcl::fromPCLPointCloud2(pcl_pc2,back_up_pc);

	pcl::getMinMax3D(*rgb_cloud,min_pt,max_pt);
#ifdef _DEBUG
	std::cout<<"Minimum pt=\n\tx min = "<< min_pt[0]<< "\ty min = "<<min_pt[1]<<"\tz min ="<<min_pt[2]<<"\n";
	std::cout<<"Maximum pt=\n\tx max = "<< max_pt[0]<< "\ty max = "<<max_pt[1]<<"\tz max ="<<max_pt[2]<< "\n";
#endif

	/*************************************************************************************
	 * calculate normlas for all the points of organised pointclouds
	 *************************************************************************************/
	//this->normal_extrct_org_pc(rgb_cloud);

	/*************************************************************************************
	 * Extract the Edge image based on Depth discontinuty
	 *************************************************************************************/
	cv::Mat EdgeImg;
	EdgeImg.release();
	EdgeImg= this->ExtractEdgeImg(slice_cloud);
	cv_bridge::CvImage out_msg;
	out_msg.header   = msg->header; // Same timestamp and tf frame as input image
	out_msg.encoding = sensor_msgs::image_encodings::MONO8; // Or whatever
	out_msg.image    = EdgeImg;
	pub_edge_img.publish(out_msg);

	this->ApplyHoughLineTranform(EdgeImg);

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

	pcl::ModelCoefficients xmin_coeff, xmax_coeff, ymin_coeff, ymax_coeff;	//Defining the coefficients
	/*********************************************************************************************
	 * Estimate the planes using Singular value Decomposition
	 ********************************************************************************************/
	if(xmin)
	{
		xmin_coeff = this->plane_est_svd(xmin_pc);
		std::cout<<"The xmin_coeff coefficents calculated using svd are\t"<<xmin_coeff.values[0]<<"\t"
				<<xmin_coeff.values[1]<<"\t"<<xmin_coeff.values[2]<<"\t"<<xmin_coeff.values[3]<<std::endl;
		std::cout<<"The distance between Origin and xmin plane is "<<xmin_coeff.values[3]<<std::endl;
	}
	if(xmax)
	{
		xmax_coeff = this->plane_est_svd(xmax_pc);
		std::cout<<"The xmax_coeff coefficents calculated using svd are\t"<<xmax_coeff.values[0]<<"\t"
				<<xmax_coeff.values[1]<<"\t"<<xmax_coeff.values[2]<<"\t"<<xmax_coeff.values[3]<<std::endl;
		std::cout<<"The distance between Origin and xmax plane is "<<xmax_coeff.values[3]<<std::endl;
	}
	if(ymin)
	{
		ymin_coeff = this->plane_est_svd(ymin_pc);
		std::cout<<"The ymin_coeff coefficents calculated using svd are\t"<<ymin_coeff.values[0]<<"\t"
				<<ymin_coeff.values[1]<<"\t"<<ymin_coeff.values[2]<<"\t"<<ymin_coeff.values[3]<<std::endl;
		std::cout<<"The distance between Origin and ymin plane is "<<ymin_coeff.values[3]<<std::endl;
	}
	if(ymax)
	{
		ymax_coeff = this->plane_est_svd(ymax_pc);
		std::cout<<"The ymax_coeff coefficents calculated using svd are\t"<<ymax_coeff.values[0]<<"\t"
				<<ymax_coeff.values[1]<<"\t"<<ymax_coeff.values[2]<<"\t"<<ymax_coeff.values[3]<<std::endl;
		std::cout<<"The distance between Origin and ymax plane is "<<ymax_coeff.values[3]<<std::endl;
	}
	if(!((xmin||xmax)&&(ymin||ymax)))
	{
		std::cout<< "No measurement possible"<<std::endl;
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

	/*****************************************************************************************************
	 * Manually extract the inliers to the pointcloud by caluclating the eucledean distance
	 *****************************************************************************************************/
	this->extract_inliers(xmin_coeff,xmax_coeff,ymin_coeff,ymax_coeff);

	/******************************************************************************************************
	 * Publish the individual pointclouds
	 ******************************************************************************************************/
	all_planes.header = rgb_cloud->header;
	/*pcl::PointCloud<pcl::PointXYZRGB> pc_planes_sub;
	pc_planes_sub = back_up_pc - all_planes_RGB;*/
	//pc_planes_sub.header = rgb_cloud->header;
	pub_ext_planes.publish(all_planes);
	//pub_plane_extracted.publish(pc_planes_sub);

	/********************************************************************************************************
	 * Calculate the change in orientation from 1st msg and change in angle between successive measurements
	 * the angle stack up in the vector is
	 * 1. change in angle from previous msg
	 * 2. Change in angle btw 1st msg and current msg
	 ********************************************************************************************************/
	//std::vector<double> angles;
/*	double delta_roll,roll, delta_pitch, delta_yaw, pitch, yaw;
	if(!first_pc_msg)
	{
		//calculating the roll angles, give xmax 1st priority
		if(xmax)
		{
			//angles = this->chng_angl_orient(xmax_coeff,prev_xmax_coeff,first_xmax_coeff);
			delta_roll = this->angle_btw_planes(xmax_coeff,prev_xmax_coeff);
			std::cout<<"The previous xmax coefficients are\t"<<prev_xmax_coeff.values[0]<<"\t"
							<<prev_xmax_coeff.values[1]<<"\t"<<prev_xmax_coeff.values[2]<<"\t"<<prev_xmax_coeff.values[3]<<std::endl;
		}
		else if(xmin)
			delta_roll = this->angle_btw_planes(xmin_coeff,prev_xmin_coeff);

		//calculating the roll angles, give xmax 1st priority
		if(ymax)
		{
			delta_pitch = this->angle_btw_planes(ymax_coeff,prev_ymax_coeff);
			std::cout<<"The previous ymax coefficients calculated using svd are\t"<<prev_ymax_coeff.values[0]<<"\t"
										<<prev_ymax_coeff.values[1]<<"\t"<<prev_ymax_coeff.values[2]<<"\t"<<prev_ymax_coeff.values[3]<<std::endl;
		}
		else if(ymin)
			delta_pitch = this->angle_btw_planes(xmin_coeff,prev_ymin_coeff);

		//add yaw here

		std::cout<<"The Delta roll is " << delta_roll<<" delta pitch is " << delta_pitch<<std::endl;
	}

*/


	/************************************************************************************************************
	 * Note: DO NOT COMMENT OR REMOVE!!! Other wise causes error during SVD calculation
	 * Setting all the segmented point clouds bool flags to zero
	 ************************************************************************************************************/
	xmin = false; xmax = false; ymin = false; ymax = false;

	ros::Time end_time = ros::Time::now();
	std::cout<<"Total time taken for execution is "<<end_time.toSec()- start_time.toSec() <<"Seconds"<<std::endl;
	pcl::fromPCLPointCloud2(pcl_pc2,previous_cloud);
	prev_xmax_coeff = xmax_coeff;
	prev_xmin_coeff = xmin_coeff;
	prev_ymax_coeff = ymax_coeff;
	prev_ymin_coeff = ymin_coeff;
	if(first_pc_msg)
	{
		first_pc_msg = false;
		first_xmax_coeff = xmax_coeff;
		first_xmin_coeff = xmin_coeff;
		first_ymax_coeff = ymax_coeff;
		first_ymin_coeff = ymin_coeff;
	}
	std::cout<<std::endl<<std::endl<<std::endl;
}

/****************************************************************************************
 * Extraction of planar equations from the segmented point clouds
 ***************************************************************************************/
pcl::ModelCoefficients tunnel_planar_pos::plane_est_svd(pcl::PointCloud<pcl::PointXYZRGB> point_cloud)
{
	ros::Time SVD_start_time = ros::Time::now();
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

	/* check if the distance calculated from the origin is -ve if yes,
	   change the direction of the normal*/
	if(d<0)
	{
		U_MAT.col(2) = -U_MAT.col(2);
		d = -d;
	}

	pcl::ModelCoefficients normals;
	normals.values.push_back(U_MAT(0,2));
	normals.values.push_back(U_MAT(1,2));
	normals.values.push_back(U_MAT(2,2));
	normals.values.push_back(d);
	ros::Time SVD_end_time = ros::Time::now();
	std::cout<<"Total time taken for Planar segmentation using SVD of point cloud is "<<SVD_end_time.toSec()- SVD_start_time.toSec() <<"Seconds"<<std::endl;
	return(normals);

}

void tunnel_planar_pos::pc_segmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud, Eigen::Vector4f min_pt, Eigen::Vector4f max_pt)
{
	ros::Time seg_start_time = ros::Time::now();
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
	if(xmin_plane->width>3000)
	{
		pcl::copyPointCloud(*xmin_plane,xmin_pc);
		xmin=true;
	}
	if(ymin_plane->width>3000)
	{
		pcl::copyPointCloud(*ymin_plane,ymin_pc);
		ymin=true;
	}
	if(xmax_plane->width>3000)
	{
		pcl::copyPointCloud(*xmax_plane,xmax_pc);
		xmax=true;
	}
	if(ymax_plane->width>3000)
	{
		pcl::copyPointCloud(*ymax_plane,ymax_pc);
		ymax=true;
	}
	pcl::copyPointCloud(*slices,slices_pc);
	ros::Time seg_end_time = ros::Time::now();
	std::cout<<"Total time taken for Segmentation of point cloud is "<<seg_end_time.toSec()- seg_start_time.toSec() <<"Seconds"<<std::endl;
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
/***************************************************************************************************
 * Calculates the angles between 2 planes (i.e. angles between 2 normal vectors
 * return value: angle in degrees
 ***************************************************************************************************/

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

/****************************************************************************************************
 * Calculates the perpendicular or shortest distance parallel to the normal vector between
 * a given point in 3D coordinates and a given plane equation
 * formula used is
 * 		d=|Ax+By+Cz+d|/ sqrt(A^2+B^2+C^2)
 * return value: Distance in meters
 ****************************************************************************************************/
double tunnel_planar_pos::distance_frm_point_2_plane(pcl::PointXYZI point, pcl::ModelCoefficients plane)
{
	double num = plane.values.at(0)*point.x + plane.values.at(1)*point.y + plane.values.at(2)*point.z
			+ plane.values.at(3);
	double denum = std::sqrt(std::pow(plane.values.at(0),2) + std::pow(plane.values.at(1),2)
	+ std::pow(plane.values.at(2),2));
	double distance = std::fabs(num)/denum;
	return (distance);
}


void tunnel_planar_pos::extract_inliers(pcl::ModelCoefficients xmin_coef, pcl::ModelCoefficients xmax_coef, pcl::ModelCoefficients ymin_coef, pcl::ModelCoefficients ymax_coef )
{
	pcl::PointXYZI point;

	if(xmin)
		for(int i=0;i<xmin_pc.width;i++)
		{
			point.x=xmin_pc.at(i).x;
			point.y=xmin_pc.at(i).y;
			point.z=xmin_pc.at(i).z;
			point.intensity = 20;

			if(this->distance_frm_point_2_plane(point,xmin_coef)<0.05)
			{
				all_planes.push_back(point);
				all_planes_RGB.push_back(xmin_pc.at(i));
			}
		}
	if(xmax)
		for(int i=0;i<xmax_pc.width;i++)
		{
			point.x=xmax_pc.at(i).x;
			point.y=xmax_pc.at(i).y;
			point.z=xmax_pc.at(i).z;
			point.intensity = 70;
			if(this->distance_frm_point_2_plane(point,xmax_coef)<0.05)
			{
				all_planes.push_back(point);
				all_planes_RGB.push_back(xmax_pc.at(i));
			}
		}
	if(ymin)
		for(int i=0;i<ymin_pc.width;i++)
		{
			point.x=ymin_pc.at(i).x;
			point.y=ymin_pc.at(i).y;
			point.z=ymin_pc.at(i).z;
			point.intensity = 130;
			if(this->distance_frm_point_2_plane(point,ymin_coef)<0.05)
			{
				all_planes_RGB.push_back(ymin_pc.at(i));
				all_planes.push_back(point);
			}

		}
	if(ymax)
		for(int i=0;i<ymax_pc.width;i++)
		{
			point.x=ymax_pc.at(i).x;
			point.y=ymax_pc.at(i).y;
			point.z=ymax_pc.at(i).z;
			point.intensity = 180;
			if(this->distance_frm_point_2_plane(point,ymax_coef)<0.05)
			{
				all_planes_RGB.push_back(ymax_pc.at(i));
				all_planes.push_back(point);
			}
		}

}

void tunnel_planar_pos::normal_extrct_org_pc(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud)
{
	ros::Time start_time = ros::Time::now();
	// estimate normals
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
	ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
	ne.setMaxDepthChangeFactor(0.02f);
	ne.setNormalSmoothingSize(10.0f);
	ne.setInputCloud(input_cloud);
	ne.compute(*normals);
	// visualize normals
	pcl::visualization::PCLVisualizer viewer("PCL Viewer");
	viewer.setBackgroundColor (0.0, 0.0, 0.5);
	viewer.addPointCloudNormals<pcl::PointXYZRGB,pcl::Normal>(input_cloud, normals);
	ros::Time end_time = ros::Time::now();
	std::cout<<"Time taken to calculate the normals from organised pointclouds is "<< end_time.toSec()-start_time.toSec()<<std::endl;
	while (!viewer.wasStopped ())
	{
		viewer.spinOnce ();
	}
}

/*********************************************************************************************
 * Calculates the change in orientation and change in angle
 * internally used this->angle_btw_planes() function to calculate the angle
 * the angle stack up in the vector is
 * 1. change in angle from previous msg
 * 2. Change in angle btw 1st msg and current msg
 *********************************************************************************************/
std::vector<double> tunnel_planar_pos::chng_angl_orient(pcl::ModelCoefficients current_coeff, pcl::ModelCoefficients prev_coeff, pcl::ModelCoefficients first_coeff)
{
	std::vector<double> angle_orentation;
	angle_orentation.push_back(this->angle_btw_planes(current_coeff,prev_coeff));
	angle_orentation.push_back(this->angle_btw_planes(current_coeff,first_coeff));
	return(angle_orentation);
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

