/*
 *  pkmKeyframe.h
 *  opencvExample
 *
 *  Created by Mr. Magoo on 1/8/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#pragma once

#include <cstdio>
#include <cstring>
#include <vector.h>

#include "cv.h"

class pkmKeyframe
{
public:
	
	pkmKeyframe()
	{
		bCreated = false;
	}
	~pkmKeyframe()
	{
		if (bCreated) {
			cvReleaseMemStorage(&storage);
		}
	}
	
	int create(IplImage *img)
	{
		if (bCreated) {
			cvReleaseMemStorage(&storage);
		}
		storage = cvCreateMemStorage(0);
		
		int n = getSURFDescriptors(img, &keypoints, &descriptors, &storage);		
	}
	
	int getSURFDescriptors(const IplImage* img, CvSeq **kypts, CvSeq **desc, CvMemStorage **storage)
	{
		
		// Using SURF's default param. Not computing descriptors
		CvSURFParams par;
		par.extended = 1;
		par.hessianThreshold = 500;
		par.nOctaves = 3;
		par.nOctaveLayers = 4;
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
	
	

private:
	CvSeq *									keypoints;
	CvSeq *									descriptors;
	CvMemStorage *							storage;
	
	
	bool									bCreated;

};