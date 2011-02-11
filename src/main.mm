#include "ofMain.h"
#include "testApp.h"

int main(){
	ofSetupOpenGL(480,320, OF_FULLSCREEN);			// <-------- setup the GL context

	ofRunApp(new testApp);
}
