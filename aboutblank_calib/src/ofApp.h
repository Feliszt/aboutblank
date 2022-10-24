#pragma once

#include "ofMain.h"
#include "ofxKinect.h"
#include "ofxGui.h"
#include "ofxOpenCv.h"
#include "ofxXmlSettings.h"
#include "ofxUtils.h"

#include "GLFW/glfw3.h"

class ofApp : public ofBaseApp {

public:
	void setup();
	void setupGui();
	void update();
	void draw();
	void drawGui(ofEventArgs& args);

	void keyPressed(int key);
	void keyReleased(int key);
	void mouseMoved(int x, int y);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void mouseEntered(int x, int y);
	void mouseExited(int x, int y);
	void windowResized(int w, int h);
	void dragEvent(ofDragInfo dragInfo);
	void gotMessage(ofMessage msg);

	// global
	void saveCalibFunc();
	void gaussian_elimination(float* input, int n);
	void findHomography(ofPoint src[4], ofPoint dst[4], float homography[16]);
	ofMatrix4x4 findHomography(ofPoint src[4], ofPoint dst[4]);

	// windows size
	float mainWindowW;
	float mainWindowH;
	float guiWindowW;
	float guiWindowH;

	// kinect
	ofxKinect				kinect;
	ofxCvColorImage			kinectColor;
	ofxCvGrayscaleImage		kinectGray, kinectDepthRaw, kinectDepth;
	ofRectangle				tableLimits;

	// calibrations
	// general
	ofxCvContourFinder	contourFinder;
	// gray detection shader
	ofShader			gd_shader;
	ofFbo				gd_fboRes;
	ofxCvColorImage		gd_resImColor;
	ofxCvGrayscaleImage gd_resImGray;
	// color detection shader
	ofShader			cd_shader;
	ofFbo				cd_fboRes;
	ofxCvColorImage		cd_resImColor;
	ofxCvGrayscaleImage cd_resImGray;
	// calib 0 : kinect <-> projector
	DraggablePoint		k2p_draggablePoint;
	ofVec2f				k2p_calibPoint, k2p_resPoint;
	vector<ofVec4f>		k2p_storePoints;
	ofMatrix4x4			k2p_matrix, p2k_matrix;
	ofVec3f				k2p_testPoint_k;
	bool				k2p_matrixComputed;
	// calib 1 : table <-> kinect
	vector<ofVec4f>		t2k_storePoints;
	ofMatrix4x4			t2k_matrix, k2t_matrix;
	bool				t2k_matrixComputed;
	int					t2k_calibOrder = 0;
	ofVec3f				t2k_testPoint_k;
	// calib 2 : pattern detection
	ofColor				pd_colorToDetect;

	// gui
	ofxPanel			gui;
	ofxFloatSlider		tableMinH, tableMaxH, tableMinW, tableMaxW;
	ofxButton			saveCalib;
	ofxIntSlider		grayThresholdDetect, minDet, maxDet;
	// calib 2 : pattern detection
	ofxFloatSlider		pd_colorThresh;

	//
	int					calibStateInd = 0;
	ofxXmlSettings		dataSettings;
};
