/*
 *  pkmObject.h
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

#include "pkmKeyframe.h"

const int minfeaturesperkeyframe = 30;

class pkmObject
{

public:
	pkmObject();	
	~pkmObject()
	{
		while (keyframes.size()) {
			pkmKeyframe *ky = keyframes.front();
			keyframes.pop_back();
			delete ky;
		}
	}

	int addKeyframe(IplImage *img)
	{
		pkmKeyframe *ky = new pkmKeyframe();
		if (ky->create(img) > minfeaturesperkeyframe) {
			printf("[OK]: Added keyframe");
			keyframes.push_back(ky);
		}
		else {
			delete ky;
		}
	}

	static int			numberOfObjects;
	int					objectNumber;	
private:
	
	vector<pkmKeyframe *>		keyframes;

};