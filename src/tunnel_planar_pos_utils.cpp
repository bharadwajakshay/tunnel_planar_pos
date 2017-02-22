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

	pub_z_slices = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/tunnel_planar_pos/Slice_PC_Z",1000);
	pub_y_slices = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/tunnel_planar_pos/Slice_PC_Y",1000);
	pub_x_slices = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/tunnel_planar_pos/Slice_PC_X",1000);
	pub_ext_planes = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/tunnel_planar_pos/extracted_planes",1000);
	pub_plane_extracted = nh.advertise<pcl::PointCloud<pcl::PointXYZI> >("/tunnel_planar_pos/pc_planes_removed",1000);
	pub_corrected_pc = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/tunnel_planar_pos/corrected_pt_cloud",1000);
	pub_edge_img = nh.advertise<sensor_msgs::Image>("/tunnel_planar_pos/EdgeImage",1000);
	pub_detected_img = nh.advertise<sensor_msgs::Image>("/tunnel_planar_pos/DetectedEdges",1000);

#ifdef _DISP_ALL_POINTS
	pub_S0_xmin = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/tunnel_planar_pos/Xmin_cloud",1000);
	pub_S0_xmax = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/tunnel_planar_pos/Xmax_cloud",1000);
	pub_S0_ymin = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/tunnel_planar_pos/Ymin_cloud",1000);
	pub_S0_ymax = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/tunnel_planar_pos/Ymax_cloud",1000);
	pub_xy_Main = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/tunnel_planar_pos/XYmain_cloud",1000);
	//
	pub_xy_seg1 = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/tunnel_planar_pos/XYmain_seg1",1000);
	pub_xy_seg2 = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/tunnel_planar_pos/XYmain_seg2",1000);
	pub_xy_seg3 = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/tunnel_planar_pos/XYmain_seg3",1000);
	pub_xy_seg4 = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/tunnel_planar_pos/XYmain_seg4",1000);
	pub_xy_seg5 = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/tunnel_planar_pos/XYmain_seg5",1000);
	pub_xy_seg6 = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/tunnel_planar_pos/XYmain_seg6",1000);
	pub_xy_seg7 = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/tunnel_planar_pos/XYmain_seg7",1000);
	pub_xy_seg8 = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/tunnel_planar_pos/XYmain_seg8",1000);
#endif

#ifndef _SAVE_EXEC_TIME
	pub_xmin_plane = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/tunnel_planar_pos/Xmin_plane",1000);
	pub_xmax_plane = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/tunnel_planar_pos/Xmax_plane",1000);
	pub_ymin_plane = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/tunnel_planar_pos/Ymin_plane",1000);
	pub_ymax_plane = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/tunnel_planar_pos/Ymax_plane",1000);
	pub_Z_plane = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/tunnel_planar_pos/Z_plane",1000);
#endif

	coeff_valid =false;
	first_imu_msg = true;
	imu_loop_itr = 0;
	first_pc_msg = true;

	//open file for file IO
	total_exe_time.open("/home/nuc/Documents/total_exe_time.dat");
	if(!total_exe_time.is_open())
		std::cout<<"Error opening total_exe_time.dat file"<<std::endl;
	else
		total_exe_time.clear();
	seg_array_time.open("/home/nuc/Documents/seg_array_time.dat");
	if(!seg_array_time.is_open())
		std::cout<<"Error opening seg_array_time.dat file"<<std::endl;
	else
		seg_array_time.clear();
	seg_pcl_time.open("/home/nuc/Documents/seg_pcl_time.dat");
	if(!seg_pcl_time.is_open())
		std::cout<<"Error opening seg_pcl_time.dat file"<<std::endl;
	else
		seg_pcl_time.clear();
}

tunnel_planar_pos::~tunnel_planar_pos()
{
	if(total_exe_time.is_open())
	{
		total_exe_time.close();
		std::cout<<"File total_exe_time.dat closed"<<std::endl;
	}
	if(seg_array_time.is_open())
	{
		std::cout<<"File seg_array_time.dat closed"<<std::endl;
		seg_array_time.close();
	}
	if(seg_pcl_time.is_open())
	{
		std::cout<<"File seg_pcl_time.dat closed"<<std::endl;
		seg_pcl_time.close();
	}


}

void tunnel_planar_pos::callbackpointclouds(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
	ros::Time start_time_main = ros::Time::now();
	pcl::PCLPointCloud2 pcl_pc2;

	// Convert to PCP PointCloud2 structure
	pcl_conversions::toPCL(*msg,pcl_pc2);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	// Convert to PCL PointCloud structure with RGB data
	pcl::fromPCLPointCloud2(pcl_pc2,*rgb_cloud);
	// Calculate the minimum and the maximum points in the Point Cloud (Takes about 2-3ms
	Eigen::Vector4f min_pt,max_pt;
	pcl::getMinMax3D(*rgb_cloud,min_pt,max_pt);
	bool xmin_coeff = false , xmax_coeff = false, ymin_coeff = false, ymax_coeff = false, z_coeff = false;

	ros::Time end_time_init = ros::Time::now();
	std::cout<<"Total time taken for initialization is "<<end_time_init.toSec()- start_time_main.toSec()<<"Seconds"<<std::endl;
#ifdef _DEBUG
	std::cout<<"Minimum pt=\n\tx min = "<< min_pt[0]<< "\ty min = "<<min_pt[1]<<"\tz min ="<<min_pt[2]<<"\n";
	std::cout<<"Maximum pt=\n\tx max = "<< max_pt[0]<< "\ty max = "<<max_pt[1]<<"\tz max ="<<max_pt[2]<< "\n";
#endif
//	this->parse_point_data(rgb_cloud);
	/*************************************************************************************
	 * Extract the Edge image based on Depth discontinuty
	 *************************************************************************************/
/*	cv::Mat EdgeImg;
	EdgeImg.release();
	EdgeImg= this->ExtractEdgeImg(slice_cloud);
	cv_bridge::CvImage::Ptr out_msg(new cv_bridge::CvImage);
	out_msg->header   = msg->header; // Same timestamp and tf frame as input image
	out_msg->encoding = sensor_msgs::image_encodings::MONO8; // Or whatever
	out_msg->image    = EdgeImg;
	pub_edge_img.publish(out_msg);

	cv::Mat EdgeDetectedImg;
	EdgeDetectedImg.release();
	EdgeDetectedImg = this->ApplyHoughLineTranform(EdgeImg);pcl::copyPointCloud
	cv_bridge::CvImage::Ptr out_img(new cv_bridge::CvImage);
	out_img->header   = msg->header;
	out_img->encoding = sensor_msgs::image_encodings::MONO8;
	out_img->image    = EdgeDetectedImg;
	pub_detected_img.publish(out_img);*/

	/***********************************
	 * Extract image from point cloud
	 ***********************************/
	//setting up the point clouds
	xmin_pc.clear();
	ymin_pc.clear();
	xmax_pc.clear();
	ymax_pc.clear();
	//std::vector<int> indices_nan;
	//pcl::removeNaNFromPointCloud(*rgb_cloud,*rgb_cloud,indices_nan);
	//this->array_segmentation(min_pt,max_pt);

	/***********************************************************************************************************
	 * Ceofficeints for plane
	 * Xmin
	 * Xmax
	 * Ymin
	 * Ymax
	 * Zplane
	 ***********************************************************************************************************/
	Eigen::Vector4f normXmin = Eigen::Vector4f::Zero(), normXmax = Eigen::Vector4f::Zero(),
			normYmin = Eigen::Vector4f::Zero(), normYmax = Eigen::Vector4f::Zero();
	std::vector<Eigen::Vector4f> normZ;
	this->pc_segmentation(rgb_cloud,min_pt,max_pt,normXmin,normXmax,normYmin,normYmax,normZ);
	//this->pc_segmentation_y(back_up_pc,min_pt,max_pt);
	/************************************************************************************************************
	 * Note: DO NOT COMMENT OR REMOVE!!! Other wise causes error during SVD calculation
	 * Setting all the segmented point clouds bool flags to zero
	 ************************************************************************************************************/
	ros::Time end_time_main = ros::Time::now();

	if(total_exe_time.is_open())
		total_exe_time<<end_time_main.toSec() - start_time_main.toSec()<<std::endl;
	else
		std::cout<<"Total time taken for execution is "<<end_time_main.toSec() - start_time_main.toSec() <<"Seconds"<<std::endl;
	pcl::fromPCLPointCloud2(pcl_pc2,previous_cloud);
	std::cout<<std::endl<<std::endl<<std::endl;
}

/****************************************************************************************
 * pcl::ModelCoefficients tunnel_planar_pos::plane_est_svd(pcl::PointCloud<pcl::PointXYZRGB> point_cloud)
 * Extraction of planar equations from the segmented point clouds
 ***************************************************************************************/
pcl::ModelCoefficients tunnel_planar_pos::plane_est_svd(pcl::PointCloud<pcl::PointXYZRGB> point_cloud, int func)
{
	ros::Time SVD_start_time = ros::Time::now();
	pcl::ModelCoefficients normals;
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

	if(func == PLANAR_FIT)
	{
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

		normals.values.push_back(U_MAT(0,2));
		normals.values.push_back(U_MAT(1,2));
		normals.values.push_back(U_MAT(2,2));
		normals.values.push_back(d);
		ros::Time SVD_end_time = ros::Time::now();
		std::cout<<"Total time taken for Planar segmentation using SVD of point cloud is "<<SVD_end_time.toSec()- SVD_start_time.toSec() <<"Seconds"<<std::endl;

	}
	else if (func == LINE_FIT)
	{
		normals.values.push_back(U_MAT(0,0));
		normals.values.push_back(U_MAT(1,0));
		normals.values.push_back(U_MAT(2,0));
		ros::Time SVD_end_time = ros::Time::now();
		std::cout<<"Total time taken for Planar segmentation using SVD of point cloud is "<<SVD_end_time.toSec()- SVD_start_time.toSec() <<"Seconds"<<std::endl;
	}
	return(normals);
}

/****************************************************************************************
 * pcl::ModelCoefficients tunnel_planar_pos::plane_est_svd(Eigen::MatrixXd points)
 * Extraction of planar equations from the points
 ***************************************************************************************/
pcl::ModelCoefficients tunnel_planar_pos::plane_est_svd(Eigen::MatrixXf points)
{
	ros::Time SVD_start_time = ros::Time::now();

	// calcaulating the centroid of the pointcloud
	Eigen::MatrixXf centroid = points.rowwise().mean();
	//std::cout<<"The centroid of the pointclouds is given by:\t"<<centroid<<std::endl;

	//subtract the centroid from points
	points.row(0).array() -= centroid(0);
	points.row(1).array() -= centroid(1);
	points.row(2).array() -= centroid(2);

	//calculate the SVD of points matrix
	Eigen::JacobiSVD<Eigen::MatrixXf> svd(points,Eigen::ComputeThinU);
	Eigen::MatrixXf U_MAT = svd.matrixU();
	//std::cout<<"U matrix transpose is:"<<U_MAT<<std::endl<<std::endl<<"U matrix is:"<<svd.matrixU()<<std::endl;

	/*********************************************************************************************
	 * caculating d by sybstituting the centroid back in the quation
	 * 		aCx+bCy+cCz = -d
	 ********************************************************************************************/
	//double d = -((U_MAT(0,2)*points(0,1))+ (U_MAT(1,2)*points(1,1)) + (U_MAT(1,2)*points(1,2)));
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
	//std::cout<<"Total time taken for normal extraction using SVD of point cloud is "<<SVD_end_time.toSec()- SVD_start_time.toSec() <<"Seconds"<<std::endl;
	std::cout<<SVD_end_time.toSec()- SVD_start_time.toSec() <<std::endl;
	return(normals);

}

void tunnel_planar_pos::pc_segmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud,Eigen::Vector4f pt_min,Eigen::Vector4f pt_max,
		Eigen::Vector4f& xmin_coeff,Eigen::Vector4f& xmax_coeff,Eigen::Vector4f& ymin_coeff,Eigen::Vector4f& ymax_coeff, std::vector<Eigen::Vector4f> Z_normals)
{
	pcl::PointCloud<pcl::PointXYZRGB> slice_0;
	pcl::PointCloud<pcl::PointXYZRGB> slice_1;
	pcl::PointCloud<pcl::PointXYZRGB> slice_2;
	pcl::PointCloud<pcl::PointXYZRGB> slice_3;
	pcl::PointCloud<pcl::PointXYZRGB> slice_4;
	pcl::PointCloud<pcl::PointXYZRGB> slice_5;
	pcl::PointCloud<pcl::PointXYZRGB> slice_z;
	pcl::PointCloud<pcl::PointXYZRGB> slice_y(input_cloud->width,input_cloud->height);
	pcl::PointCloud<pcl::PointXYZRGB> slice_x(input_cloud->width,input_cloud->height);
	pcl::PointCloud<pcl::PointXYZRGB> slice_xy(input_cloud->width,input_cloud->height);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr slice_xy_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);

	//clearing pointclouds
	slice_0.clear();
	slice_1.clear();
	slice_2.clear();
	slice_3.clear();
	slice_4.clear();
	slice_5.clear();
	slice_z.clear();

	float xmin0=100, xmin1=100, xmin2=100, xmin3=100, xmin4=100, xmin5=100;
	float ymin0=100, ymin1=100, ymin2=100, ymin3=100, ymin4=100, ymin5=100;
	float xmax0=-100, xmax1=-100, xmax2=-100, xmax3=-100, xmax4=-100, xmax5=-100;
	float ymax0=-100, ymax1=-100, ymax2=-100, ymax3=-100, ymax4=-100, ymax5=-100;


	//Slices for Y direction
	Eigen::VectorXf slice_pts_y(NO_OF_SLICES);
	Eigen::VectorXf diff_y(NO_OF_SLICES);
	Eigen::VectorXf slice_pts_x(NO_OF_SLICES_X);
	Eigen::VectorXf diff_x(NO_OF_SLICES_X);

	//Slice Vector Y
	double dist_btw_slice = (pt_max[1] - pt_min[1])/NO_OF_SLICES;
	for(int i=1; i<=NO_OF_SLICES;i++)
		slice_pts_y[i-1] = pt_min[1] + (i*dist_btw_slice);

	//Slice Vector X
	dist_btw_slice = (pt_max[0] - pt_min[0])/NO_OF_SLICES_X;
	for(int i=1; i<=NO_OF_SLICES_X;i++)
		slice_pts_x[i-1] = pt_min[0] + (i*dist_btw_slice);

	ros::Time start_for_loop_time = ros::Time::now();
	for(int ii=0;ii<input_cloud->width;ii++)
		for(int jj=0;jj<input_cloud->height;jj++)
		{
			pcl::PointXYZRGB point = input_cloud->at(ii,jj);
			if((pcl_isfinite(point.z)))
			{
				if((point.z > 0.9)&&(point.z < 1))
				{
					slice_0.push_back(point);
				}
				else if ((point.z > 1.4)&&(point.z < 1.5))
				{
					slice_1.push_back(point);
				}
				else if((point.z > 1.9)&&(point.z < 2))
				{
					slice_2.push_back(point);
				}
				else if ((point.z > 2.4)&&(point.z < 2.5))
				{
					slice_3.push_back(point);
				}
				else if((point.z > 2.9)&&(point.z < 3))
				{
					slice_4.push_back(point);
				}
				else if ((point.z > 3.4)&&(point.z < 3.5))
				{
					slice_5.push_back(point);
				}
			}
			if((pcl_isfinite(point.y)))
			{
				diff_y = slice_pts_y - (point.y * Eigen::VectorXf::Ones(NO_OF_SLICES));
				diff_y=diff_y.array().abs();
				std::sort(diff_y.data(),diff_y.data()+NO_OF_SLICES,std::less<float>());
				if(diff_y[0]<0.005)
				{
					slice_y.at(ii,jj)=point;
					slice_xy.at(ii,jj)=point;
					slice_xy_ptr->push_back(point);
				}
			}
			if((pcl_isfinite(point.x)))
			{
				diff_x = slice_pts_x - (point.x * Eigen::VectorXf::Ones(NO_OF_SLICES_X));
				diff_x=diff_x.array().abs();
				std::sort(diff_x.data(),diff_x.data()+NO_OF_SLICES_X,std::less<float>());
				if(diff_x[0]<0.005)
				{
					slice_x.at(ii,jj)=point;
					slice_xy.at(ii,jj)=point;
					slice_xy_ptr->push_back(point);
				}
			}
		}
	ros::Time end_for_loop_time = ros::Time::now();
	if(seg_pcl_time.is_open())
		seg_pcl_time<<end_for_loop_time.toSec() - start_for_loop_time.toSec()<<std::endl;
	else
		std::cout<<"Segmented Point Cloud for Z slices "<<end_for_loop_time.toSec() - start_for_loop_time.toSec()<<" seconds"<<std::endl;
	ros::Time start_pc_add_time = ros::Time::now();
	slice_z = slice_0 + slice_1;
	slice_z += slice_2;
	slice_z += slice_3;
	slice_z += slice_4;
	slice_z += slice_5;
	pcl::copyPointCloud(slice_z,slices_z_pc);
	ros::Time end_pc_add_time = ros::Time::now();
	std::cout<<"Adding and Publishing Point Cloud for Z slices "<<end_pc_add_time.toSec() - start_pc_add_time.toSec()<<" seconds"<<std::endl;
	// Publish point clouds
	slices_z_pc.header = input_cloud->header;
	slice_x.header = input_cloud->header;
	slice_y.header = input_cloud->header;
	pub_x_slices.publish(slice_x);
	pub_y_slices.publish(slice_y);
	pub_z_slices.publish(slices_z_pc);


	//Processing Slices
	ros::Time strt_slice_proc_time = ros::Time::now();
	pcl::PointCloud<pcl::PointXYZRGB> xmax;
	xmax.clear();
	xmax.header = input_cloud->header;
	pcl::PointCloud<pcl::PointXYZRGB> xmin;
	xmin.clear();
	xmin.header = input_cloud->header;
	pcl::PointCloud<pcl::PointXYZRGB> ymax;
	ymax.clear();
	ymax.header = input_cloud->header;
	pcl::PointCloud<pcl::PointXYZRGB> ymin;
	ymin.clear();
	ymin.header = input_cloud->header;

	this->ProcessSingleSlice(slice_0,xmin,xmax,ymin,ymax);
	this->ProcessSingleSlice(slice_1,xmin,xmax,ymin,ymax);
	this->ProcessSingleSlice(slice_2,xmin,xmax,ymin,ymax);
	this->ProcessSingleSlice(slice_3,xmin,xmax,ymin,ymax);
	this->ProcessSingleSlice(slice_4,xmin,xmax,ymin,ymax);
	this->ProcessSingleSlice(slice_5,xmin,xmax,ymin,ymax);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr xmax_plane (new pcl::PointCloud<pcl::PointXYZRGB>);
	xmax_plane->clear();
	xmax_plane->header = input_cloud->header;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr xmin_plane (new pcl::PointCloud<pcl::PointXYZRGB>);
	xmin_plane->clear();
	xmin_plane->header = input_cloud->header;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ymax_plane (new pcl::PointCloud<pcl::PointXYZRGB>);
	ymax_plane->clear();
	ymax_plane->header = input_cloud->header;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ymin_plane (new pcl::PointCloud<pcl::PointXYZRGB>);
	ymin_plane->clear();
	ymin_plane->header = input_cloud->header;


	if(xmin.width>500)
		xmin_coeff = this->LineEsitmationRANSAC(xmin,xmin_plane);
	if(xmax.width>500)
		xmax_coeff = this->LineEsitmationRANSAC(xmax,xmax_plane);
	if(ymin.width>500)
		ymin_coeff = this->LineEsitmationRANSAC(ymin,ymin_plane);
	if(ymax.width>500)
		ymax_coeff = this->LineEsitmationRANSAC(ymax,ymax_plane);


	//Publishing each point cloud
	pub_S0_xmax.publish(xmax);
	pub_S0_xmin.publish(xmin);
	pub_S0_ymin.publish(ymin);
	pub_S0_ymin.publish(ymax);

#ifndef _SAVE_EXEC_TIME
	pub_xmin_plane.publish(xmin_plane);
	pub_xmax_plane.publish(xmax_plane);
	pub_xmin_plane.publish(ymin_plane);
	pub_ymax_plane.publish(ymax_plane);
#endif

	//processing X_Y Clouds
	Z_normals.clear();
	this->SegmentXYCloud(slice_xy,Z_normals);

	//this->ClusteringEucledian(slice_xy_ptr);

	ros::Time end_slice_proc_time = ros::Time::now();
	std::cout<<"Processing each Z slice for 1 cloud takes "<<end_slice_proc_time.toSec() - strt_slice_proc_time.toSec()<<" seconds"<<std::endl;
	std::cout<<"No of Planar normals detected are "<<Z_normals.size()<<std::endl;

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
	double angle = pcl::rad2deg(std::acos(dot_prod/(mag_p1 * mag_p2)));

	//Calculation of angles from PCL library
	Eigen::Vector4f norm1(plane1.values.data());
	Eigen::Vector4f norm2(plane2.values.data());
	//norm1<<plane1.values.at(0),plane1.values.at(1),plane1.values.at(2),plane1.values.at(3);
	//norm2<< plane2.values.at(0), plane2.values.at(1), plane2.values.at(2), plane2.values.at(3);
	double angle_2 = pcl::rad2deg(pcl::getAngle3D(norm1,norm2));
	std::cout<<"Angle Dot Prod = "<<angle<<"\tSngle PCL method = "<<angle_2<<std::endl;
	return(angle);
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
/*	pcl::PointXYZI point;

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
*/
}

void tunnel_planar_pos::normal_extrct_org_pc(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud)
{
	ros::Time start_time = ros::Time::now();
	// estimate normals
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
	ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
	ne.useSensorOriginAsViewPoint(); // Selects the sensor origin as view point, All the nomrals are wrt to this viwe point
	ne.setMaxDepthChangeFactor(0.05f);
	ne.setNormalSmoothingSize(30.0f);
	ne.setInputCloud(input_cloud);
	ne.compute(*normals);
	ros::Time end_time = ros::Time::now();
	pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*input_cloud,*point_cloud);

	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);

	pcl::copyPointCloud(*point_cloud, *cloud_with_normals);
	pcl::copyPointCloud(*normals, *cloud_with_normals);
	pcl::io::savePCDFileASCII("~/test.pcd", *cloud_with_normals);
}

void tunnel_planar_pos::PlanarExtraction(pcl::PointCloud<pcl::PointXYZRGB> point_cloud)
{
	ros::Time SVD_start_time = ros::Time::now();
	pcl::ModelCoefficients est_norm_vect;
	if(!point_cloud.isOrganized())
	{
		ROS_INFO_STREAM("Error: The Point cloud is not organised");
		return;
	}
	for(int i=1;i<8;i++)
		for(int j=1;j<8;j++)
		{
			Eigen::MatrixXf sub_points(3,(point_cloud.height/8)*(point_cloud.width/8)); //Initialize the matrix to maximum points
			int count =0; //initialize a point counter
			for(int ii=i-1;ii<point_cloud.height/8;ii++)
				for(int jj=j-1;jj<point_cloud.width/8;jj++)
				{
					if(!(std::isnan(point_cloud.at(jj,ii).x)||std::isnan(point_cloud.at(jj,ii).y)||std::isnan(point_cloud.at(jj,ii).z)))
					{
						sub_points(0,count) = point_cloud.at(jj,ii).x;
						sub_points(1,count) = point_cloud.at(jj,ii).y;
						sub_points(2,count) = point_cloud.at(jj,ii).z;
						count++;
					}
				}
			sub_points = sub_points.block(0,0,3,(count-1));
			est_norm_vect = this->plane_est_svd(sub_points);
			/*std::cout<<"The Normal calculated using svd are\t"<<est_norm_vect.values[0]<<"\t"
							<<est_norm_vect.values[1]<<"\t"<<est_norm_vect.values[2]<<"\t"<<est_norm_vect.values[3]<<std::endl;*/
		}
	ros::Time SVD_end_time = ros::Time::now();
	std::cout<<"Total time taken for normal extraction using SVD of point cloud is "<<SVD_end_time.toSec()- SVD_start_time.toSec() <<"Seconds"<<std::endl;
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


void tunnel_planar_pos::parse_point_data(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud)
{
	ros::Time data_parsing_start = ros::Time::now();
	for(int i =0; i<input_cloud->width;i++)
		for(int j=0;j<input_cloud->height;j++)
		{
			points[i][j][0]=input_cloud->at(i,j).x;
			points[i][j][1]=input_cloud->at(i,j).y;
			points[i][j][2]=input_cloud->at(i,j).z;
		}
	ros::Time data_parsing_end = ros::Time::now();
	std::cout<<"Total time taken for parsing the data is "<<data_parsing_end.toSec()- data_parsing_start.toSec() <<"Seconds"<<std::endl;
}


void tunnel_planar_pos::array_segmentation(Eigen::Vector4f pt_min,Eigen::Vector4f pt_max)
{
	float slice_0[IMAGE_WIDTH][IMAGE_HEIGHT][IMAGE_DEPTH]={0.0f/0.0f};
	float slice_1[IMAGE_WIDTH][IMAGE_HEIGHT][IMAGE_DEPTH]={0.0f/0.0f};
	float slice_2[IMAGE_WIDTH][IMAGE_HEIGHT][IMAGE_DEPTH]={0.0f/0.0f};
	float slice_3[IMAGE_WIDTH][IMAGE_HEIGHT][IMAGE_DEPTH]={0.0f/0.0f};
	float slice_4[IMAGE_WIDTH][IMAGE_HEIGHT][IMAGE_DEPTH]={0.0f/0.0f};
	float slice_5[IMAGE_WIDTH][IMAGE_HEIGHT][IMAGE_DEPTH]={0.0f/0.0f};
	float slice_x[IMAGE_WIDTH][IMAGE_HEIGHT][IMAGE_DEPTH]={0.0f/0.0f};
	float slice_y[IMAGE_WIDTH][IMAGE_HEIGHT][IMAGE_DEPTH]={0.0f/0.0f};

	//Slices for Y direction
	Eigen::VectorXf slice_pts_y(NO_OF_SLICES);
	Eigen::VectorXf diff_y(NO_OF_SLICES);
	Eigen::VectorXf slice_pts_x(NO_OF_SLICES_X);
	Eigen::VectorXf diff_x(NO_OF_SLICES_X);

	//Slice Vector Y
	double dist_btw_slice = (pt_max[1] - pt_min[1])/NO_OF_SLICES;
	for(int i=1; i<=NO_OF_SLICES;i++)
		slice_pts_y[i-1] = pt_min[1] + (i*dist_btw_slice);

	//Slice Vector X
	dist_btw_slice = (pt_max[0] - pt_min[0])/NO_OF_SLICES_X;
	for(int i=1; i<=NO_OF_SLICES_X;i++)
		slice_pts_x[i-1] = pt_min[1] + (i*dist_btw_slice);

	ros::Time start_for_loop_time = ros::Time::now();
	for(int ii=0;ii<IMAGE_WIDTH;ii++)
		for(int jj=0;jj<IMAGE_HEIGHT;jj++)
		{
			if((std::isfinite(points[ii][jj][2])))
			{std::cout<<"Processing Slice 0"<<std::endl;
				if((points[ii][jj][2]> 0.9)&&(points[ii][jj][2] < 1))
				{
					slice_0[ii][jj][0] = points[ii][jj][0];
					slice_0[ii][jj][1] = points[ii][jj][1];
					slice_0[ii][jj][2] = points[ii][jj][2];
				}
				else if ((points[ii][jj][2] > 1.4)&&(points[ii][jj][2] < 1.5))
				{
					slice_1[ii][jj][0] = points[ii][jj][0];
					slice_1[ii][jj][1] = points[ii][jj][1];
					slice_1[ii][jj][2] = points[ii][jj][2];
				}
				else if((points[ii][jj][2] > 1.9)&&(points[ii][jj][2] < 2))
				{
					slice_2[ii][jj][0] = points[ii][jj][0];
					slice_2[ii][jj][1] = points[ii][jj][1];
					slice_2[ii][jj][2] = points[ii][jj][2];
				}
				else if ((points[ii][jj][2] > 2.4)&&(points[ii][jj][2] < 2.5))
				{
					slice_3[ii][jj][0] = points[ii][jj][0];
					slice_3[ii][jj][1] = points[ii][jj][1];
					slice_3[ii][jj][2] = points[ii][jj][2];
				}
				else if((points[ii][jj][2] > 2.9)&&(points[ii][jj][2] < 3))
				{
					slice_4[ii][jj][0] = points[ii][jj][0];
					slice_4[ii][jj][1] = points[ii][jj][1];
					slice_4[ii][jj][2] = points[ii][jj][2];
				}
				else if ((points[ii][jj][2] > 3.4)&&(points[ii][jj][2] < 3.5))
				{
					slice_5[ii][jj][0] = points[ii][jj][0];
					slice_5[ii][jj][1] = points[ii][jj][1];
					slice_5[ii][jj][2] = points[ii][jj][2];
				}
			}
			if(std::isfinite(points[ii][jj][1]))
			{
				diff_y = slice_pts_y - (points[ii][jj][1] * Eigen::VectorXf::Ones(NO_OF_SLICES));
				diff_y=diff_y.array().abs();
				std::sort(diff_y.data(),diff_y.data()+NO_OF_SLICES,std::less<float>());
				if(diff_y[0]<0.005)
				{
					slice_y[ii][jj][0] = points[ii][jj][0];
					slice_y[ii][jj][1] = points[ii][jj][1];
					slice_y[ii][jj][2] = points[ii][jj][2];
				}
			}
			if(std::isfinite(points[ii][jj][1]))
			{
				diff_x = slice_pts_x - (points[ii][jj][0] * Eigen::VectorXf::Ones(NO_OF_SLICES_X));
				diff_x=diff_x.array().abs();
				std::sort(diff_x.data(),diff_x.data()+NO_OF_SLICES_X,std::less<float>());
				if(diff_x[0]<0.005)
				{
					slice_x[ii][jj][0] = points[ii][jj][0];
					slice_x[ii][jj][1] = points[ii][jj][1];
					slice_x[ii][jj][2] = points[ii][jj][2];
				}
			}
		}
	ros::Time end_for_loop_time = ros::Time::now();
	if(seg_array_time.is_open())
		seg_array_time<<end_for_loop_time.toSec() - start_for_loop_time.toSec()<<std::endl;
	else
		std::cout<<"Z slices Segmentation as Arrays "<<end_for_loop_time.toSec() - start_for_loop_time.toSec()<<" nano seconds"<<std::endl;
}

void tunnel_planar_pos::ProcessSingleSlice(pcl::PointCloud<pcl::PointXYZRGB> slice, pcl::PointCloud<pcl::PointXYZRGB>& minx, pcl::PointCloud<pcl::PointXYZRGB>& maxx,
		pcl::PointCloud<pcl::PointXYZRGB>& miny, pcl::PointCloud<pcl::PointXYZRGB>& maxy)
{

	//get min max
	Eigen::Vector4f min, max;
	pcl::getMinMax3D(slice,min,max);

	float xmin = min[0];
	float xmax = max[0];
	float ymin = min[1];
	float ymax = max[1];

	int xmin_count = 0, ymin_count = 0,xmax_count = 0,ymax_count = 0;
	for(int i=0;i<slice.width;i++)
	{
		pcl::PointXYZRGB point = slice.at(i);
		if(std::fabs(xmin-point.x)<0.02)
		{
			minx.push_back(point);
		}
		if(std::fabs(ymin-point.y)<0.02)
		{
			miny.push_back(point);
		}
		if(std::fabs(xmax-point.x)<0.02)
		{
			maxx.push_back(point);
		}
		if(std::fabs(ymax-point.y)<0.02)
		{
			maxy.push_back(point);
		}
	}

}

Eigen::VectorXf tunnel_planar_pos::LineEsitmationRANSAC(pcl::PointCloud<pcl::PointXYZRGB> input_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ptcld)
{
	/*Eigen::VectorXf model_coeff(4);
	std::vector<int> inliers;
	pcl::SampleConsensusModelLine<pcl::PointXYZRGB>::Ptr line_model(new pcl::SampleConsensusModelLine<pcl::PointXYZRGB> (input_cloud));
	pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac_method (line_model);
	ransac_method.setDistanceThreshold(0.01);
	ransac_method.getModelCoefficients(model_coeff);
	std::cout<<"The Line coefficients are "<<model_coeff.transpose()<<std::endl;
	return(model_coeff);*/
	Eigen::VectorXf model_coeff(4);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud(input_cloud,*temp_cloud);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.003);
	seg.setInputCloud(temp_cloud);
	seg.setEpsAngle(0.0523599); //Setting the angle epsilon (delta) threshold

	pcl::ModelCoefficients::Ptr normals (new pcl::ModelCoefficients);
	seg.segment(*inliers,*normals );
	if(inliers->indices.size()==0)
		ROS_INFO("There are no lines in the point cloud\n");

	model_coeff(0) =  normals->values.at(0);
	model_coeff(1) =  normals->values.at(1);
	model_coeff(2) =  normals->values.at(2);
	model_coeff(3) =  normals->values.at(3);

#ifndef _SAVE_EXEC_TIME
	// Extract the planar inliers from the input cloud
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr inliers_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	extract.setInputCloud(temp_cloud);
	extract.setIndices (inliers);
	extract.setNegative (false);
	extract.filter(*ptcld);
	pcl::Normal nor;
	nor.normal_x = normals->values.at(0);
	nor.normal_y = normals->values.at(1);
	nor.normal_z = normals->values.at(2);

	/*pcl::PointCloud<pcl::Normal>::Ptr norm (new pcl::PointCloud<pcl::Normal>);
	norm->push_back(nor);
	norm->header = slices_z_pc.header;
	inliers_cloud->header = slices_z_pc.header;
	pcl::copyPointCloud(*inliers_cloud,*ptcld_norm);
	pcl::copyPointCloud(*norm,*ptcld_norm);*/
#endif

	return(model_coeff);

}

void tunnel_planar_pos::SegmentXYCloud(pcl::PointCloud<pcl::PointXYZRGB> xy_cloud,std::vector<Eigen::Vector4f>& Z_normals)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr seg_main_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr seg1_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr seg2_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr seg3_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr seg4_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr seg5_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr seg6_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr seg7_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr seg8_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointXYZRGB point;
	std::vector<float> depth_pts_mainseg, depth_pts_seg1, depth_pts_seg2, depth_pts_seg3, depth_pts_seg4, depth_pts_seg5, depth_pts_seg6, depth_pts_seg7, depth_pts_seg8;
	float sum_main = 0, sum_1 = 0,sum_2 = 0,sum_3 = 0,sum_4 = 0,sum_5 = 0,sum_6 = 0,sum_7 = 0,sum_8 = 0;
	// Point Cloud segment (Main)
	if(xy_cloud.isOrganized())
	{
		for(int ii=0;ii<xy_cloud.width;ii++)
			for(int jj=0;jj<xy_cloud.height;jj++)
			{
				point = xy_cloud.at(ii,jj);
				if(point.x > 0 || point.y > 0 || point.z > 0)
				{
					if((ii<160)&&(jj<120))
					{
						seg1_cloud->push_back(point);
						depth_pts_seg1.push_back(point.z);
					}
					if((ii>=160) && (ii<480))
					{
						seg_main_cloud->push_back(point);
						depth_pts_mainseg.push_back(point.z);
					}
					if((ii>=480)&&(jj<120))
					{
						seg2_cloud->push_back(point);
						depth_pts_seg2.push_back(point.z);
					}

					if((ii<160)&&((jj>=120)&&(jj<240)))
					{
						seg3_cloud->push_back(point);
						depth_pts_seg3.push_back(point.z);
					}
					if((ii>=480)&&((jj>=120)&&(jj<240)))
					{
						seg4_cloud->push_back(point);
						depth_pts_seg4.push_back(point.z);
					}

					if((ii<160)&&((jj>=240)&&(jj<360)))
					{
						seg5_cloud->push_back(point);
						depth_pts_seg5.push_back(point.z);
					}
					if((ii>=480)&&((jj>=240)&&(jj<360)))
					{
						seg6_cloud->push_back(point);
						depth_pts_seg6.push_back(point.z);
					}
					if((ii<160)&&(jj>=360))
					{
						seg7_cloud->push_back(point);
						depth_pts_seg7.push_back(point.z);
					}
					if((ii>=480)&&(jj>=360))
					{
						seg8_cloud->push_back(point);
						depth_pts_seg8.push_back(point.z);
					}


				}
			}

		/*float stdev_main = this->StdCalc(depth_pts_mainseg,std::accumulate(depth_pts_mainseg.begin(),depth_pts_mainseg.end(),0));
		float stdev_1 = this->StdCalc(depth_pts_seg1,std::accumulate(depth_pts_seg1.begin(),depth_pts_seg1.end(),0));
		float stdev_2 = this->StdCalc(depth_pts_seg2,std::accumulate(depth_pts_seg2.begin(),depth_pts_seg2.end(),0));
		float stdev_3 = this->StdCalc(depth_pts_seg3,std::accumulate(depth_pts_seg3.begin(),depth_pts_seg3.end(),0));
		float stdev_4 = this->StdCalc(depth_pts_seg4,std::accumulate(depth_pts_seg4.begin(),depth_pts_seg4.end(),0));
		float stdev_5 = this->StdCalc(depth_pts_seg5,std::accumulate(depth_pts_seg5.begin(),depth_pts_seg5.end(),0));
		float stdev_6 = this->StdCalc(depth_pts_seg6,std::accumulate(depth_pts_seg6.begin(),depth_pts_seg6.end(),0));
		float stdev_7 = this->StdCalc(depth_pts_seg7,std::accumulate(depth_pts_seg7.begin(),depth_pts_seg7.end(),0));
		float stdev_8 = this->StdCalc(depth_pts_seg8,std::accumulate(depth_pts_seg8.begin(),depth_pts_seg8.end(),0));*/



		Eigen::Vector4f model_coeff(4);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr extract_mainZ (new pcl::PointCloud<pcl::PointXYZRGB>);
		bool valid =false;

		valid = this->ParallelZPlaneRANSAC(seg_main_cloud,model_coeff,extract_mainZ);
		if(valid)
		{
			std::cout<<"The Model coeff of the Z plane is "<<model_coeff.transpose()<<std::endl;
			Z_normals.push_back(model_coeff);
		}

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr extract_seg1 (new pcl::PointCloud<pcl::PointXYZRGB>);
		valid = this->ParallelZPlaneRANSAC(seg1_cloud,model_coeff, extract_seg1);
		if(valid)
		{
			std::cout<<"The Model coeff of the Z plane is "<<model_coeff.transpose()<<std::endl;
			Z_normals.push_back(model_coeff);
		}

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr extract_seg2 (new pcl::PointCloud<pcl::PointXYZRGB>);
		valid =	this->ParallelZPlaneRANSAC(seg2_cloud,model_coeff,extract_seg2);
		if(valid)
		{
			std::cout<<"The Model coeff of the Z plane is "<<model_coeff.transpose()<<std::endl;
			Z_normals.push_back(model_coeff);
		}

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr extract_seg3 (new pcl::PointCloud<pcl::PointXYZRGB>);
		valid =	this->ParallelZPlaneRANSAC(seg3_cloud,model_coeff,extract_seg2);
		if(valid)
		{
			std::cout<<"The Model coeff of the Z plane is "<<model_coeff.transpose()<<std::endl;
			Z_normals.push_back(model_coeff);
		}

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr extract_seg4 (new pcl::PointCloud<pcl::PointXYZRGB>);
		valid =	this->ParallelZPlaneRANSAC(seg4_cloud,model_coeff,extract_seg2);
		if(valid)
		{
			std::cout<<"The Model coeff of the Z plane is "<<model_coeff.transpose()<<std::endl;
			Z_normals.push_back(model_coeff);
		}

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr extract_seg5 (new pcl::PointCloud<pcl::PointXYZRGB>);
		valid =	this->ParallelZPlaneRANSAC(seg5_cloud,model_coeff,extract_seg5);
		if(valid)
		{
			std::cout<<"The Model coeff of the Z plane is "<<model_coeff.transpose()<<std::endl;
			Z_normals.push_back(model_coeff);
		}

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr extract_seg6 (new pcl::PointCloud<pcl::PointXYZRGB>);
		valid =	this->ParallelZPlaneRANSAC(seg6_cloud,model_coeff,extract_seg5);
		if(valid)
		{
			std::cout<<"The Model coeff of the Z plane is "<<model_coeff.transpose()<<std::endl;
			Z_normals.push_back(model_coeff);
		}

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr extract_seg7 (new pcl::PointCloud<pcl::PointXYZRGB>);
		valid =	this->ParallelZPlaneRANSAC(seg7_cloud,model_coeff,extract_seg7);
		if(valid)
		{
			std::cout<<"The Model coeff of the Z plane is "<<model_coeff.transpose()<<std::endl;
			Z_normals.push_back(model_coeff);
		}

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr extract_seg8 (new pcl::PointCloud<pcl::PointXYZRGB>);
		valid =	this->ParallelZPlaneRANSAC(seg8_cloud,model_coeff,extract_seg8);
		if(valid)
		{
			std::cout<<"The Model coeff of the Z plane is "<<model_coeff.transpose()<<std::endl;
			Z_normals.push_back(model_coeff);
		}


/*		std::cout<<"The size of the Main segment is "<<seg_main_cloud->width<<" Points"<<std::endl;
		std::cout<<"The Standard Deviation of Main segment "<<stdev_main<<std::endl;
		std::cout<<"The Standard Deviation of 1st segment "<<stdev_1<<std::endl;
		std::cout<<"The Standard Deviation of 1st segment "<<stdev_2<<std::endl;
		std::cout<<"The Standard Deviation of 1st segment "<<stdev_3<<std::endl;
		std::cout<<"The Standard Deviation of 1st segment "<<stdev_4<<std::endl;
		std::cout<<"The Standard Deviation of 1st segment "<<stdev_5<<std::endl;
		std::cout<<"The Standard Deviation of 1st segment "<<stdev_6<<std::endl;
		std::cout<<"The Standard Deviation of 1st segment "<<stdev_7<<std::endl;
		std::cout<<"The Standard Deviation of 1st segment "<<stdev_8<<std::endl;*/

		//publish Pointclouds
		seg1_cloud->header = slices_z_pc.header;
		pub_xy_seg1.publish(seg1_cloud);
		seg2_cloud->header = slices_z_pc.header;
		pub_xy_seg2.publish(seg2_cloud);
		seg3_cloud->header = slices_z_pc.header;
		pub_xy_seg3.publish(seg3_cloud);
		seg4_cloud->header = slices_z_pc.header;
		pub_xy_seg4.publish(seg4_cloud);
		seg5_cloud->header = slices_z_pc.header;
		pub_xy_seg5.publish(seg5_cloud);
		seg6_cloud->header = slices_z_pc.header;
		pub_xy_seg6.publish(seg6_cloud);
		seg7_cloud->header = slices_z_pc.header;
		pub_xy_seg7.publish(seg7_cloud);
		seg8_cloud->header = slices_z_pc.header;
		pub_xy_seg8.publish(seg8_cloud);

		seg_main_cloud->header = slices_z_pc.header;
		pub_xy_Main.publish(seg_main_cloud);

		//Add all z planes
		pcl::PointCloud<pcl::PointXYZRGB> Zplane;
		Zplane.clear();
		Zplane = *extract_mainZ + *extract_seg1;
		Zplane += *extract_seg2;
		Zplane += *extract_seg3;
		Zplane += *extract_seg4;
		Zplane += *extract_seg5;
		Zplane += *extract_seg6;
		Zplane += *extract_seg7;
		Zplane += *extract_seg8;
		Zplane.header = slices_z_pc.header;
		pub_Z_plane.publish(Zplane);
	}
	else
		std::cout<<"The Point cloud is not organised for XY segmentation"<<std::endl;
}

bool tunnel_planar_pos::ParallelZPlaneRANSAC(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud, Eigen::Vector4f& model_coeff, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& filtered_cloud)
{
//	/pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	bool isvalid = false;
	//pcl::copyPointCloud(input_cloud,*temp_cloud);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	model_coeff(0) = 0;
	model_coeff(1) = 0;
	model_coeff(2) = 0;
	model_coeff(3) = 0;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.02);
	seg.setInputCloud(input_cloud);
	seg.setEpsAngle(0.0523599); //Setting the angle epsilon (delta) threshold
	//seg.setEpsAngle(0.0872665);
	seg.setAxis(Eigen::Vector3f(0,0,1));


	pcl::ModelCoefficients::Ptr normals (new pcl::ModelCoefficients);
	seg.segment(*inliers,*normals );
	if(inliers->indices.size() < 100)
	{
		ROS_INFO("There are no planes in the point cloud\n");
		isvalid = false;
	}
	else
	{
#ifndef _SAVE_EXEC_TIME
		isvalid = true;
		// Extract the planar inliers from the input cloud
		pcl::ExtractIndices<pcl::PointXYZRGB> extract;
		extract.setInputCloud(input_cloud);
		extract.setIndices (inliers);
		extract.setNegative (false);
		extract.filter(*filtered_cloud);

#endif
	}

	model_coeff(0) =  normals->values.at(0);
	model_coeff(1) =  normals->values.at(1);
	model_coeff(2) =  normals->values.at(2);
	model_coeff(3) =  normals->values.at(3);
	//std::cout<<"The Model coeff of the Z plane is "<<model_coeff.transpose()<<std::endl; Posted to main fuction
	return(isvalid);

}


float tunnel_planar_pos::StdCalc(std::vector<float> depth_field, float sum)
{
	//calculating STD deviation
	float mean = sum/depth_field.size();
	float accum = 0.0;
	std::for_each (depth_field.begin(), depth_field.end(), [&](const float d) {
		accum += (d - mean) * (d - mean);
	});
	float stdev = std::sqrt(accum / (depth_field.size()-1));
	return(stdev);
}
