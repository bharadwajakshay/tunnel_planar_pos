/*
 * image_processing_tools.cpp
 *
 *  Created on: Dec 27, 2016
 *      Author: nuc
 */

#include "tunnel_planar_pos_utils.h"

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



