/*
 * tunnel_planar_pos.cpp
 *
 *  Created on: Sep 30, 2016
 *      Author: nuc
 */

#include "tunnel_planar_pos_utils.h"

std::string topic_pc;
std::string topic_imu;
std::string topic_img;

int main(int argc, char **argv)
{
	ros::init(argc,argv,"tunnel_planar_pos");
	//Read parameters
	ros::NodeHandle pnh("~");
	pnh.param("topic_pc",topic_pc,std::string("/camera/lepton/depth_registered/points"));
	pnh.param("topic_img",topic_img,std::string("/camera/lepton/image_rect_color"));
	pnh.param("topic_imu",topic_imu,std::string("/mti/imuX"));
	tunnel_planar_pos obj;
	ros::spin();
	return(0);
}




