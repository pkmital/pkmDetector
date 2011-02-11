#include "testApp.h"
//--------------------------------------------------------------
testApp::~testApp(){
}
void testApp::setup(){
	
	
	// init video input
	vidInput.initGrabber(CAM_WIDTH,CAM_HEIGHT);
	
	// -------- INITIALIZE IPHONE DISPLAY/EVENTS -------
	//ofSetWindowShape(960, 640);
	ofxiPhoneSetOrientation(OFXIPHONE_ORIENTATION_LANDSCAPE_RIGHT);
	//ofxiPhoneSetOrientation(OFXIPHONE_ORIENTATION_PORTRAIT);
	
	// register touch events
	ofRegisterTouchEvents(this);
	
	// iPhoneAlerts will be sent to this.
	ofxiPhoneAlerts.addListener(this);
	
	// black
	ofBackground(0,0,0);

	//vidInput.setUseTexture(true);
	
	
	// allocate stuff
	colorImg.allocate(CAM_WIDTH, CAM_HEIGHT);
	grayImg.allocate(CAM_WIDTH, CAM_HEIGHT);
	
	drawEllipses = true;
	drawColors = true;
	choosing_img = false;
	chosen_img = false;
	
}

//--------------------------------------------------------------
void testApp::update(){
	
		
	if (chosen_img) {
		colorImg.setFromPixels(vidInput.getPixels(), CAM_WIDTH, CAM_HEIGHT);
		
		grayImg = colorImg;
		
		detector.setImageSearch(grayImg.getCvImage());
		
		detection = detector.getHomography();
	}
		 
				
	//} // end if(isFrameNew())
}

//--------------------------------------------------------------
void testApp::draw(){
	ofBackground(0,0,0);
	
	ofSetColor(255, 255, 255);
	
	// camera image
	vidInput.draw(0, 0);
	
	// draw a rectanlge around the current selection
	if (choosing_img) {
		int x = mouseX;
		int y = mouseY;
		
		ofNoFill();
		ofRect(x_start < x ? x_start : x, 
			   y_start < y ? y_start : y, 
			   abs(x_start - x), 
			   abs(y_start - y));
		
	}
	
	if (chosen_img) {
		
		CvSeqReader kreader;
		cvStartReadSeq( detector.keypts2, &kreader, 0 );
		
		for(int i = 0; i < detector.keypts2->total; i++ )
		{
			ofPushMatrix();
			const CvSURFPoint* kp = (const CvSURFPoint*)kreader.ptr;
			ofTranslate(kp->pt.x, kp->pt.y);
			ofRotate(kp->dir, 0, 0, 1);
			ofCircle(0, 0, kp->size);
			ofLine(0, 0, kp->size, kp->size);
			CV_NEXT_SEQ_ELEM( kreader.seq->elem_size, kreader );
			ofPopMatrix();
			
		}
			
		
		/*
		ofNoFill();
		for (vector<MyKeypoint >::iterator it = detector.keypts2.begin(); it != detector.keypts2.end(); it++) {
			ofPushMatrix();
			ofTranslate(it->x, it->y, 0);
			ofRotate(it->ori, 0, 0, 1);
			ofRect(0, 0, it->scale/5.0f, it->scale/5.0f);
			ofPopMatrix();
		}
		*/
		/*ofPushMatrix();
		ofTranslate(CAM_WIDTH, 0, 0);
		testImg.draw(0, 0);
		ofPopMatrix();
		 */
		/*
		for (vector<MyKeypoint >::iterator it = detector.keypts1.begin(); it != detector.keypts1.end(); it++) {
			ofPushMatrix();
			ofTranslate(it->x, it->y, 0);
			ofRotate(it->ori, 0, 0, 1);
			ofRect(0, 0, it->scale/5.0f, it->scale/5.0f);
			ofPopMatrix();
		}
		ofPopMatrix();
		
		ofFill();
		
		if (detection > 4) {
			
			ofSetColor(200, 20, 80);
			int n = detector.match_left.size();
			for( int i = 0; i < n; i++ )
			{
				ofLine(CAM_WIDTH*2 + detector.match_left[i].x, 
					   detector.match_left[i].y, 
					   detector.match_right[i].x, 
					   detector.match_right[i].y);	
			}
		}		
		*/
		ofSetColor(200, 20, 50);
		ofLine(detector.dst_corners[0].x, detector.dst_corners[0].y,
			   detector.dst_corners[1].x, detector.dst_corners[1].y);
		
		ofLine(detector.dst_corners[2].x, detector.dst_corners[2].y,
			   detector.dst_corners[1].x, detector.dst_corners[1].y);
		
		ofLine(detector.dst_corners[2].x, detector.dst_corners[2].y,
			   detector.dst_corners[3].x, detector.dst_corners[3].y);
		
		ofLine(detector.dst_corners[0].x, detector.dst_corners[0].y,
			   detector.dst_corners[3].x, detector.dst_corners[3].y);
		
		//char buf[256];
		//sprintf(buf, "detection: %d\nflann: %d\n", detection, detector.use_flann);
		//ofDrawBitmapString(buf, 20, CAM_HEIGHT+10);
	}
	
	// mser image
	//colorImg.draw(CAM_WIDTH, 0);
	
	
	/*
	if(drawEllipses)
	{
		ellipseTick = cvGetTickCount();
		ofNoFill();
		ofPushMatrix();
		ofTranslate(CAM_WIDTH, 0, 0);
		for (vector<cv::KeyPoint>::iterator it = keypts.begin(); it != keypts.end(); it++) 
		{
			ofPushMatrix();
			ofTranslate(it->pt.x + it->size/2, it->pt.y - it->size, 0);
			ofRotate(it->angle, 0, 0, 1);
			ofRect(0, 0, it->size, it->size);
			ofPopMatrix();	
		}
		ofPopMatrix();
		ofFill();
		ellipseTick = cvGetTickCount() - ellipseTick;
	}	
	// individual msers:
	for (int i = 0; i < NUM_MSERS; i++)
	{
		mserSquares[i].draw(CAM_WIDTH*2+10 + MSER_DIM*floor(i/((CAM_HEIGHT+50)/MSER_DIM)), MSER_DIM*(i%((CAM_HEIGHT+50)/MSER_DIM)));
	}
	
	ofSetColor(10, 120, 0);
	
	char sbuf[256];
	sprintf(sbuf, "FPS: %f", ofGetFrameRate());
	ofDrawBitmapString(sbuf, 10, CAM_HEIGHT + 12);
	
	sprintf(sbuf, "MSER: %g ms (found: %d)\n", mserTick/((float)cvGetTickFrequency()*1000.), num_mser_found );
	ofDrawBitmapString(sbuf, 10, CAM_HEIGHT + 24);
	
	sprintf(sbuf, "Keypoints: %g ms\n", keyptsTick/((float)cvGetTickFrequency()*1000.) );
	ofDrawBitmapString(sbuf, 10, CAM_HEIGHT + 36);
	
	sprintf(sbuf, "Descriptors: %g ms\n", descTick/((float)cvGetTickFrequency()*1000.) );
	ofDrawBitmapString(sbuf, 10, CAM_HEIGHT + 48);
	
	if (drawEllipses) {
		sprintf(sbuf, "Ellipses: %g ms\n", ellipseTick/((float)cvGetTickFrequency()*1000.) );
		ofDrawBitmapString(sbuf, 10, CAM_HEIGHT + 60);
	}
	 */
	
	
}


//--------------------------------------------------------------
void testApp::touchDown(ofTouchEventArgs &touch){
	// start a rectangle selection
	if(!choosing_img)
	{
		choosing_img = true;
		x_start = touch.x;// * touch_scale_x;
		y_start = touch.y;// * touch_scale_y;
		printf("starting x:%d, y:%d\n", x_start, y_start);
	}
}

//--------------------------------------------------------------
void testApp::touchMoved(ofTouchEventArgs &touch){

}

//--------------------------------------------------------------
void testApp::touchUp(ofTouchEventArgs &touch){
	// end the rectangle selection
	if (choosing_img) {
		choosing_img = false;
		x_end = touch.x;// * touch_scale_x;
		y_end = touch.y;// * touch_scale_y;
		printf("ending x:%d, y:%d\n", x_end, y_end);
		int w = abs(x_start - x_end);
		int h = abs(y_start - y_end);
		printf("allocating w:%d, h:%d\n", w, h);
		printf("original w:%d, h:%d\n", colorImg.getCvImage()->width, colorImg.getCvImage()->height);
		
		cvSetImageROI(colorImg.getCvImage(), 
					  cvRect(x_start < x_end ? x_start : x_end, 
							 y_start < y_end ? y_start : y_end, 
							 w, h));
		
		if (testImg.bAllocated) {
			testImg.clear();
			testGrayImg.clear();
		}
		testImg.allocate(w, h);
		testGrayImg.allocate(w, h);

		cvCopy(colorImg.getCvImage(), testImg.getCvImage(), NULL);
		cvCvtColor(testImg.getCvImage(), testGrayImg.getCvImage(), CV_RGB2GRAY);
		
		cvResetImageROI(colorImg.getCvImage());
		
		detector.setImageTemplate(testGrayImg.getCvImage());
		
		chosen_img = true;
	}
	
}

//--------------------------------------------------------------
void testApp::touchDoubleTap(ofTouchEventArgs &touch){

}

