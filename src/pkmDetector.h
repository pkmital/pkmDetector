/*
 *  pkmDetector.h
 *  opencvExample
 *
 *  Created by Mr. Magoo on 12/28/10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */
#pragma once

#include <cstdio>
#include <cstring>
#include <vector.h>
#include "cv.h"

#define USE_SURF 1
//#define USE_MSER 1

#define USE_FLANN 1

#if(USE_MSER)
	#ifndef NUM_MSERS
		#define NUM_MSERS 256
	#endif
	#ifndef MSER_DIM
		#define MSER_DIM 64
	#endif
#endif

class pkmDetector
{

	public:
	
	pkmDetector(int width = 320, int height = 240)
	{
		bSetImageSearch = false;
		bSetImageTemplate = false;
		
		for (int i = 0; i < 4; i++) {
			dst_corners[i] = cvPoint(0,0);
			prev_dst_corners[i] = dst_corners[i];
			temp_prev_dst_corners[i] = dst_corners[i];
		}
		
		use_flann = false;
		use_linear_interpolation = true;
		
		hsv_img = NULL; 
		hsv_template = NULL;
		gray_img = NULL;
		gray_template = NULL;
		
		distance_threshold = 10;
		
		_h = cvMat(3, 3, CV_64F, h);
	}
	
	void setImageTemplate(IplImage *img_t)
	{
		if(bSetImageTemplate) {	
			cvReleaseMemStorage(&storage1);
			bSetImageTemplate = false;
		}
		storage1 = cvCreateMemStorage(0);
				
		img_template = img_t;
		
		printf("[DEBUG] image template: %d, %d\n", img_t->width, img_t->height);
		
		int n = getSURFDescriptors(img_t, &keypts1, &feat1, &storage1);

		if (n) {
			bSetImageTemplate = true;
		}
		else {
			cvReleaseMemStorage(&storage1);
			printf("[ERROR]: Not enough features were computed.");
		}

		
	}
	
	void setImageSearch(IplImage *img_s)
	{		
		if (bSetImageSearch) {
			cvReleaseMemStorage(&storage2);
			bSetImageSearch = false;
		}
		storage2 = cvCreateMemStorage(0);
			
		img_search = img_s;
		
		int n = getSURFDescriptors(img_s, &keypts2, &feat2, &storage2);
		
		if (n) {	
			bSetImageSearch = true;
		}
		else {
			cvReleaseMemStorage(&storage2);
			printf("[ERROR]: Not enough features were computed.");
		}
	}
	
	int getSURFDescriptors(IplImage* img, CvSeq **kypts, CvSeq **desc, CvMemStorage **storage)
	{
		
		// SURF
		CvSURFParams par;
		par.extended = 0;
		par.hessianThreshold = 500;
		par.nOctaves = 1;
		par.nOctaveLayers = 3;
		/*
		Mat matimg(*img);
		vector<KeyPoint> keypoints;
		FAST( matimg, keypoints, 3, true );
		
		CvSeqWriter writer;
		cvStartAppendToSeq( *kypts, &writer ); 

		for (vector<KeyPoint>::iterator it = keypoints.begin(); it != keypoints.end(); it++) {
//			CV_WRITE_SEQ_ELEM(
		}
		
		cvExtractSURF(*img, NULL, kypts, NULL, *storage, par);	
		 */
		cvExtractSURF(img, NULL, kypts, desc, *storage, par);		
		
		if (*kypts) {
			printf("[OK] Extracted %i keypoints\n", (*kypts)->total);
			return (*kypts)->total;
		}
		else {
			printf("[ERROR] No keypoints found\n");
			return 0;
		}

		
	}
	
	/*
	int getFASTDescriptors(IplImage** img, CvSeq **kypts, CvSeq **desc, CvMemStorage **storage)
	{
		
		// SURF
		CvSURFParams par;
		par.extended = 0;
		par.hessianThreshold = 800;
		par.nOctaves = 1;
		par.nOctaveLayers = 3;
		
		Mat matimg(*img);
		vector<KeyPoint> keypoints;
		FAST( matimg, keypoints, 3, true );
		
		CvSeqWriter writer;
		cvStartAppendToSeq( *kypts, &writer ); 
		
		for (vector<KeyPoint>::iterator it = keypoints.begin(); it != keypoints.end(); it++) {
			//			CV_WRITE_SEQ_ELEM(
		}
		
		cvExtractSURF(*img, NULL, kypts, NULL, *storage, par);	
		//cvExtractSURF(*img, NULL, kypts, desc, *storage, par);		
		
		if (*kypts) {
			printf("[OK] Extracted %i keypoints\n", (*kypts)->total);
			return (*kypts)->total;
		}
		else {
			printf("[ERROR] No keypoints found\n");
			return 0;
		}
		
		
	}
	*/
	
	float
	compareSURFDescriptors( const float* d1, const float* d2, float best, int length )
	{
		float total_cost = 0;
		assert( length % 4 == 0 );
		for( int i = 0; i < length; i += 4 )
		{
			float t0 = d1[i] - d2[i];
			float t1 = d1[i+1] - d2[i+1];
			float t2 = d1[i+2] - d2[i+2];
			float t3 = d1[i+3] - d2[i+3];
			total_cost += t0*t0 + t1*t1 + t2*t2 + t3*t3;
			if( total_cost > best )
				break;
		}
		return total_cost;
	}
	
	
	int
	naiveNearestNeighbor( const float* vec, int laplacian,
						 const CvSeq* model_keypoints,
						 const CvSeq* model_descriptors )
	{
		int length = (int)(model_descriptors->elem_size/sizeof(float));
		int i, neighbor = -1;
		float d, dist1 = 1e6, dist2 = 1e6;
		CvSeqReader reader, kreader;
		cvStartReadSeq( model_keypoints, &kreader, 0 );
		cvStartReadSeq( model_descriptors, &reader, 0 );
		
		for( i = 0; i < model_descriptors->total; i++ )
		{
			const CvSURFPoint* kp = (const CvSURFPoint*)kreader.ptr;
			const float* mvec = (const float*)reader.ptr;
			CV_NEXT_SEQ_ELEM( kreader.seq->elem_size, kreader );
			CV_NEXT_SEQ_ELEM( reader.seq->elem_size, reader );
			if( laplacian != kp->laplacian )
				continue;
			d = compareSURFDescriptors( vec, mvec, dist2, length );
			if( d < dist1 )
			{
				dist2 = dist1;
				dist1 = d;
				neighbor = i;
			}
			else if ( d < dist2 )
				dist2 = d;
		}
		if ( dist1 < 0.6*dist2 )
			return neighbor;
		return -1;
	}
	
	void
	findPairs( const CvSeq* objectKeypoints, const CvSeq* objectDescriptors,
			  const CvSeq* imageKeypoints, const CvSeq* imageDescriptors, vector<int>& ptpairs )
	{
		int i;
		CvSeqReader reader, kreader;
		cvStartReadSeq( objectKeypoints, &kreader );
		cvStartReadSeq( objectDescriptors, &reader );
		ptpairs.clear();
		
		for( i = 0; i < objectDescriptors->total; i++ )
		{
			const CvSURFPoint* kp = (const CvSURFPoint*)kreader.ptr;
			const float* descriptor = (const float*)reader.ptr;
			CV_NEXT_SEQ_ELEM( kreader.seq->elem_size, kreader );
			CV_NEXT_SEQ_ELEM( reader.seq->elem_size, reader );
			int nearest_neighbor = naiveNearestNeighbor( descriptor, kp->laplacian, imageKeypoints, imageDescriptors );
			if( nearest_neighbor >= 0 )
			{
				ptpairs.push_back(i);
				ptpairs.push_back(nearest_neighbor);
			}
		}
	}
	
	/*
	void
	flannFindPairs( const CvSeq*, const CvSeq* objectDescriptors,
				   const CvSeq*, const CvSeq* imageDescriptors, vector<int>& ptpairs )
	{
		int length = (int)(objectDescriptors->elem_size/sizeof(float));
		
		cv::Mat m_object(objectDescriptors->total, length, CV_32F);
		cv::Mat m_image(imageDescriptors->total, length, CV_32F);
		
		
		// copy descriptors
		CvSeqReader obj_reader;
		float* obj_ptr = m_object.ptr<float>(0);
		cvStartReadSeq( objectDescriptors, &obj_reader );
		for(int i = 0; i < objectDescriptors->total; i++ )
		{
			const float* descriptor = (const float*)obj_reader.ptr;
			CV_NEXT_SEQ_ELEM( obj_reader.seq->elem_size, obj_reader );
			memcpy(obj_ptr, descriptor, length*sizeof(float));
			obj_ptr += length;
		}
		CvSeqReader img_reader;
		float* img_ptr = m_image.ptr<float>(0);
		cvStartReadSeq( imageDescriptors, &img_reader );
		for(int i = 0; i < imageDescriptors->total; i++ )
		{
			const float* descriptor = (const float*)img_reader.ptr;
			CV_NEXT_SEQ_ELEM( img_reader.seq->elem_size, img_reader );
			memcpy(img_ptr, descriptor, length*sizeof(float));
			img_ptr += length;
		}
		
		// find nearest neighbors using FLANN
		cv::Mat m_indices(objectDescriptors->total, 2, CV_32S);
		cv::Mat m_dists(objectDescriptors->total, 2, CV_32F);
		//cv::flann::KMeansIndexParams params(32,							// branching
		//									8,							// iterations
		//									cvflann::CENTERS_GONZALES,	// centers
		//									0.2 );
		//cv::flann::Index_<float> flann_index(m_image, params); //cv::flann::KDTreeIndexParams(4));  // using 4 randomized kdtrees
		
		cv::flann::Index flann_index(m_image, cv::flann::KDTreeIndexParams(4));  // using 4 randomized kdtrees
		flann_index.knnSearch(m_object, m_indices, m_dists, 2, cv::flann::SearchParams(64) ); // maximum number of leafs checked
		
		int* indices_ptr = m_indices.ptr<int>(0);
		float* dists_ptr = m_dists.ptr<float>(0);
		for (int i=0;i<m_indices.rows;++i) {
			if (dists_ptr[2*i]<0.6*dists_ptr[2*i+1]) {
				ptpairs.push_back(i);
				ptpairs.push_back(indices_ptr[2*i]);
			}
		}
	}
	 */
	
	int getHomography()
	{
		if(bSetImageSearch && bSetImageTemplate)
			return locatePlanarObject(keypts1, feat1, keypts2, feat2, dst_corners);
		else {
			return 0;
		}

	}
	
	
	// a rough implementation for object location using homography
	int locatePlanarObject( const CvSeq* objectKeypoints, const CvSeq* objectDescriptors,
							const CvSeq* imageKeypoints, const CvSeq* imageDescriptors,
							CvPoint dst_corners[4] )
	{
		
		vector<int> ptpairs;
		vector<CvPoint2D32f> pt1, pt2;
		CvMat _pt1, _pt2;
		int i, n;
		
		//if(use_flann)
		//	flannFindPairs( objectKeypoints, objectDescriptors, imageKeypoints, imageDescriptors, ptpairs );
		//else
		findPairs( objectKeypoints, objectDescriptors, imageKeypoints, imageDescriptors, ptpairs );

		
		n = ptpairs.size()/2;
		if( n < 4 )
			return 0;
		
		pt1.resize(n);
		pt2.resize(n);
		
		// TODO:
		// can use some LK like optimizations here in checking only a certain set of pts
		// then LMeDs could be used (need >50% inliers)...
		// checking for keypoints would require computing a model based on rotation/transformations...
		
		
		for( i = 0; i < n; i++ )
		{
			pt1[i] = ((CvSURFPoint*)cvGetSeqElem(objectKeypoints,ptpairs[i*2]))->pt;
			pt2[i] = ((CvSURFPoint*)cvGetSeqElem(imageKeypoints,ptpairs[i*2+1]))->pt;
		}
		
		_pt1 = cvMat(1, n, CV_32FC2, &pt1[0] );
		_pt2 = cvMat(1, n, CV_32FC2, &pt2[0] );
		if( !cvFindHomography( &_pt1, &_pt2, &_h, CV_RANSAC, 4 ))
			return 0;
		
		/*
		 src_corners[0] = {0,0};
		 src_corners[1] = {img_t->width,0};
		 src_corners[2] = {img_t->width, img_t->height};
		 src_corners[3] = {0, img_t->height};
		 */

		CvPoint src_corners[4] = {	cvPoint(0,0), 
									cvPoint(img_template->width,0), 
									cvPoint(img_template->width, img_template->height), 
									cvPoint(0, img_template->height)	};
		
		for( i = 0; i < 4; i++ )
		{
			float x = src_corners[i].x, y = src_corners[i].y;
			float Z = 1./(h[6]*x + h[7]*y + h[8]);
			float X = (h[0]*x + h[1]*y + h[2])*Z;
			float Y = (h[3]*x + h[4]*y + h[5])*Z;
			
			/*
			if ( abs(cvRound(X) - dst_corners[i].x) > distance_threshold &&
				 abs(cvRound(Y) - dst_corners[i].y) > distance_threshold) {
				
				for (int c = 0; c < 4; c++) {
					temp_prev_dst_corners[c] = dst_corners[c];					
				}
				
				// use a projected warp
				for (int c = 0; c < 4; c++) {
					dst_corners[c] = cvPoint(dst_corners[c].x + (prev_dst_corners[c].x - dst_corners[c].x),
											 dst_corners[c].y + (prev_dst_corners[c].y - dst_corners[c].y));
				}
				for (int c = 0; c < 4; c++) {
					prev_dst_corners[c] = temp_prev_dst_corners[c];					
				}
			}
			else {
			 */
			/*
				for (int c = 0; c < 4; c++) {
					prev_dst_corners[c] = dst_corners[c];					
				}
			 */

				// use calculated warp
				if (use_linear_interpolation) {
					dst_corners[i] = cvPoint(dst_corners[i].x * 0.1 + 0.9 * cvRound(X), 
											 dst_corners[i].y * 0.1 + 0.9 * cvRound(Y));
				}
				else {
					dst_corners[i] = cvPoint(cvRound(X), cvRound(Y));
				}

			//}
		}
		
		return n;
	}
	
	
	float											distance_threshold;

	bool											use_flann, use_linear_interpolation;

	CvSeq											*keypts1, *keypts2;
	CvSeq											*feat1, *feat2;
	CvMemStorage									*storage1, *storage2;
	
	double											h[9];
	CvMat											_h;// = cvMat(3, 3, CV_64F, h);
    CvPoint											dst_corners[4];
    CvPoint											prev_dst_corners[4];
    CvPoint											temp_prev_dst_corners[4];
		
	bool											bSetImageTemplate, bSetImageSearch;
	
	IplImage										*img_search, *img_template;
	IplImage										*hsv_img, *hsv_template, 
													*gray_img, *gray_template;
	
};