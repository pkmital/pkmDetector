#ifndef _TEST_APP
#define _TEST_APP

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxiPhone.h"
#include "ofxiPhoneExtras.h"


#include "pkmDetector.h"

const int CAM_WIDTH = 480;
const int CAM_HEIGHT = 360;
const float touch_scale_x = CAM_WIDTH/480.;
const float touch_scale_y = CAM_HEIGHT/360.;

class testApp : public ofxiPhoneApp {

	public:

	~testApp();
	void setup();

	void update();
	void draw();
	
	void touchDown			(ofTouchEventArgs &touch);
	void touchMoved			(ofTouchEventArgs &touch);
	void touchUp			(ofTouchEventArgs &touch);
	void touchDoubleTap		(ofTouchEventArgs &touch);
	
	ofxiPhoneVideoGrabber	vidInput;
	
	ofxCvColorImage			colorImg;
	ofxCvColorImage			testImg;
	ofxCvGrayscaleImage		grayImg;
	ofxCvGrayscaleImage		testGrayImg;
	
	int						x_start, x_end, y_start, y_end;
	bool					choosing_img, chosen_img;
	pkmDetector				detector;
	int						detection;
	
	bool					drawEllipses, drawColors;
};

#endif
