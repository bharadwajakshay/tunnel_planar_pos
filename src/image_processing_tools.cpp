/*
 * image_processing_tools.cpp
 *
 *  Created on: Dec 27, 2016
 *      Author: nuc
 */

#include "tunnel_planar_pos_utils.h"


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

}


void tunnel_planar_pos::comp_hist_col_img(cv::Mat image)
{
	/// Establish the number of bins
		int histSize = 256;

		/// Set the ranges ( for B,G,R) )
		float range[] = { 0, 256 } ;
		const float* histRange = { range };

		bool uniform = true; bool accumulate = false;

		cv::Mat b_hist, g_hist, r_hist;
		std::vector<cv::Mat> indv_channels;
		cv::split(image,indv_channels);
		/// Compute the histograms:
		cv::calcHist( &indv_channels[0], 1, 0, cv::Mat(), b_hist, 1, &histSize, &histRange, uniform, accumulate );
		cv::calcHist( &indv_channels[1], 1, 0, cv::Mat(), g_hist, 1, &histSize, &histRange, uniform, accumulate );
		cv::calcHist( &indv_channels[2], 1, 0, cv::Mat(), r_hist, 1, &histSize, &histRange, uniform, accumulate );
		// Draw the histograms for B, G and R
		int hist_w = 512; int hist_h = 400;
		int bin_w = cvRound( (double) hist_w/histSize );

		cv::Mat histImageb( hist_h, hist_w, CV_8UC3, cv::Scalar( 0,0,0) );
		cv::Mat histImageg( hist_h, hist_w, CV_8UC3, cv::Scalar( 0,0,0) );
		cv::Mat histImager( hist_h, hist_w, CV_8UC3, cv::Scalar( 0,0,0) );

		/// Normalize the result to [ 0, histImage.rows ]
		cv::normalize(b_hist, b_hist, 0, histImageb.rows, cv::NORM_MINMAX, -1, cv::Mat() );
		cv::normalize(g_hist, g_hist, 0, histImageg.rows, cv::NORM_MINMAX, -1, cv::Mat() );
		cv::normalize(r_hist, r_hist, 0, histImager.rows, cv::NORM_MINMAX, -1, cv::Mat() );

		/// Draw for each channel
		for( int i = 1; i < histSize; i++ )
		{
			cv::line( histImageb, cv::Point( bin_w*(i-1), hist_h - cvRound(b_hist.at<float>(i-1)) ) ,
					cv::Point( bin_w*(i), hist_h - cvRound(b_hist.at<float>(i)) ),
					cv::Scalar( 255, 0, 0), 2, 8, 0  );
			cv::line( histImageg, cv::Point( bin_w*(i-1), hist_h - cvRound(g_hist.at<float>(i-1)) ) ,
					cv::Point( bin_w*(i), hist_h - cvRound(g_hist.at<float>(i)) ),
					cv::Scalar( 0, 255, 0), 2, 8, 0  );
			cv::line( histImager, cv::Point( bin_w*(i-1), hist_h - cvRound(r_hist.at<float>(i-1)) ) ,
					cv::Point( bin_w*(i), hist_h - cvRound(r_hist.at<float>(i)) ),
					cv::Scalar( 0, 0, 255), 2, 8, 0  );
		}

		/// Display
		cv::namedWindow("calcHist Demo", CV_WINDOW_AUTOSIZE );
		cv::imshow("Histogram of Blue", histImageb );
		cv::imshow("Histogram of Green", histImageg );
		cv::imshow("Histogram of Red", histImager );
		cv::waitKey(0);
}



/******************************************************************************************
 * void tunnel_planar_pos::ExtractEdgeImg(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud)
 * Extracts the
 ******************************************************************************************/
cv::Mat tunnel_planar_pos::ExtractEdgeImg(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud)
{
	ros::Time time = ros::Time::now();

	// Open an image of the size of input
	cv::Mat image(input_cloud->height, input_cloud->width, CV_8UC1);

	double max_range = 6.0;

	// For each point of
	int window_size = 3;
	for(int ii=1;ii<input_cloud->height-1;ii++)
	{
		for(int jj=1;jj<input_cloud->width-1;jj++)
		{

			//image.at<uchar>(ii,jj)= (unsigned char)(0);
			if(!isnan(input_cloud->at(jj,ii).z) && (input_cloud->at(jj,ii).z < 4.0) )
			{
				if( isnan(input_cloud->at(jj-1,ii-1).z) ||
				 	isnan(input_cloud->at(jj-1,ii).z) 	||
				 	isnan(input_cloud->at(jj-1,ii+1).z) ||
				 	isnan(input_cloud->at(jj,ii-1).z) ||
				 	isnan(input_cloud->at(jj,ii+1).z) ||
				 	isnan(input_cloud->at(jj+1,ii-1).z) ||
					isnan(input_cloud->at(jj+1,ii).z) ||
			 		isnan(input_cloud->at(jj+1,ii+1).z) )
			 	{
					image.at<uchar>(ii,jj)= (unsigned char)(255);
			 	}
			 	else
			 	if( 	((input_cloud->at(jj,ii).z - input_cloud->at(jj-1,ii-1).z) > 0.1) ||
				 			((input_cloud->at(jj,ii).z - input_cloud->at(jj-1,ii).z) > 0.1) ||
				 			((input_cloud->at(jj,ii).z - input_cloud->at(jj-1,ii+1).z) > 0.1) ||
				 			((input_cloud->at(jj,ii).z - input_cloud->at(jj,ii-1).z) > 0.1) ||
				 			((input_cloud->at(jj,ii).z - input_cloud->at(jj,ii+1).z) > 0.1) ||
				 			((input_cloud->at(jj,ii).z - input_cloud->at(jj+1,ii-1).z) > 0.1) ||
				 			((input_cloud->at(jj,ii).z - input_cloud->at(jj+1,ii).z) > 0.1) ||
			 				((input_cloud->at(jj,ii).z - input_cloud->at(jj+1,ii+1).z) > 0.1) )
			 	{
			 		image.at<uchar>(ii,jj)= (unsigned char)(255);
			 	}
			 	else
			 	{
			 		image.at<uchar>(ii,jj)= (unsigned char)(0);
			 	}

			 }

		}
	}

return(image);
}


cv::Mat tunnel_planar_pos::ApplyHoughLineTranform(cv::Mat input_image)
{
	cv::Mat img_line(input_image.rows,input_image.cols, CV_8UC1);
	//cv::imshow("Input image",input_image);
	cv::vector<cv::Vec4i> lines;
	cv::HoughLinesP(input_image, lines, 1, CV_PI/180, 50, 50, 10 );
	for( size_t i = 0; i < lines.size(); i++ )
	{
	  cv::Vec4i l = lines[i];
	  cv::line( img_line, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(255), 3, CV_AA);
	}
/*
	cv::imshow("Input image",input_image);
	if(!img_line.empty())
		cv::imshow("Lines found in the Image", img_line);
	else
		std::cout<<"The line detcted image is empty\n";
	cv::imshow("Lines found in the Image", img_line);
	cv::waitKey(0);*/
	return(img_line);
}
