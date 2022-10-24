#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup() {
	// gui
	gui.setup();
	gui.setPosition(2 * kinect.width + 10, 500);
	// global
	gui.add(saveCalib.setup("SAVE"));
	gui.add(tableMinH.setup("TABLEMIN_H", 0.0, 0.0, 1.0));
	gui.add(tableMaxH.setup("TABLEMAX_H", 1.0, 0.0, 1.0));
	gui.add(tableMinW.setup("TABLEMIN_W", 0.0, 0.0, 1.0));
	gui.add(tableMaxW.setup("TABLEMAX_W", 1.0, 0.0, 1.0));
	// gray threshold detect
	gui.add(grayThresholdDetect.setup("GRAYTHRESHOLDDETECT", 11, 0, 255));
	gui.add(minDet.setup("MINDET", 5, 1, 5000));
	gui.add(maxDet.setup("MAXDET", 400, 1, 5000));
	// calib state 2
	gui.add(pd_colorThresh.setup("PD_COLORTHRESH", 0.1, 0.0, 1.0));

	// functions
	saveCalib.addListener(this, &ofApp::saveCalibFunc);


	// init stuff for k2p calibration
	k2p_draggablePoint.setup(kinect.width * 0.5, kinect.height * 1.5, 8);
	k2p_matrixComputed = false;

	// init stuff for t2k calibration
	ofVec4f p;
	//
	p.x = 5;
	p.y = 5;
	t2k_storePoints.push_back(p);
	//
	p.x = 25;
	p.y = 5;
	t2k_storePoints.push_back(p);
	//
	p.x = 5;
	p.y = 25;
	t2k_storePoints.push_back(p);
	//
	p.x = 25;
	p.y = 25;
	t2k_storePoints.push_back(p);

	// load data
	gui.loadFromFile("aboutblank_calib.json");

	if (dataSettings.loadFile("aboutblank_data.xml")) {
		// load k2p_storePoints
		dataSettings.pushTag("k2p_storePoints");
		int k2p_storePointsN = dataSettings.getNumTags("k2p_storePoint");
		for (int i = 0; i < k2p_storePointsN; i++) {
			dataSettings.pushTag("k2p_storePoint", i);

			ofVec4f p;
			p.x = dataSettings.getValue("x", 0);
			p.y = dataSettings.getValue("y", 0);
			p.z = dataSettings.getValue("z", 0);
			p.w = dataSettings.getValue("w", 0);

			k2p_storePoints.push_back(p);
			dataSettings.popTag();
		}
		dataSettings.popTag();

		// load t2k_storePoints
		dataSettings.pushTag("t2k_storePoints");
		int t2k_storePointsN = dataSettings.getNumTags("t2k_storePoint");
		for (int i = 0; i < t2k_storePointsN; i++) {

			ofLog() << "load";
			dataSettings.pushTag("t2k_storePoint", i);

			ofVec4f p;
			p.x = dataSettings.getValue("x", 0);
			p.y = dataSettings.getValue("y", 0);
			p.z = dataSettings.getValue("z", 0);
			p.w = dataSettings.getValue("w", 0);

			t2k_storePoints[i] = p;
			dataSettings.popTag();
		}
		dataSettings.popTag();

		// load pd_colorToDetect
		dataSettings.pushTag("pd_colorToDetect");
		ofColor c;
		c.r = dataSettings.getValue("r", 0.0);
		c.g = dataSettings.getValue("g", 0.0);
		c.b = dataSettings.getValue("b", 0.0);
		c.a = dataSettings.getValue("a", 0.0);
		pd_colorToDetect = ofColor(c);
		dataSettings.popTag();
	}

	// init gray detection shader
	gd_shader.load("shadersGL4/shader_grayThreshold");
	gd_fboRes.allocate(kinect.width, kinect.height, GL_RGB);
	gd_resImColor.allocate(kinect.width, kinect.height);
	gd_resImGray.allocate(kinect.width, kinect.height);
	// init color detection shader
	cd_shader.load("shadersGL4/shader_colorDetection");
	cd_fboRes.allocate(kinect.width, kinect.height, GL_RGB);
	cd_resImColor.allocate(kinect.width, kinect.height);
	cd_resImGray.allocate(kinect.width, kinect.height);
}

//--------------------------------------------------------------
void ofApp::setupGui() {
	// display monitors info
	int count;
	GLFWmonitor** monitors = glfwGetMonitors(&count);
	for (int i = 0; i < count; i++) {
		int xM; int yM;
		glfwGetMonitorPos(monitors[i], &xM, &yM);
		ofLog() << "monitor #" << i << " @ (" << xM << ", " << yM << ")";
	}

	// init kinect
	kinect.setRegistration(true);
	kinect.init();
	kinect.open();

	// init kinect textures
	kinectColor.allocate(kinect.width, kinect.height);
	kinectDepth.allocate(kinect.width, kinect.height);
	kinectGray.allocate(kinect.width, kinect.height);
}

//--------------------------------------------------------------
void ofApp::update() {
	// update kinect
	kinect.update();
	tableLimits = ofRectangle(tableMinW * kinect.width, tableMinH * kinect.height, (tableMaxW - tableMinW) * kinect.width, (tableMaxH - tableMinH) * kinect.height);

	// store data
	if (kinect.isFrameNew()) {
		// get RGB captor of kinect
		ofPixels& pixim = kinect.getPixels();
		kinectColor.setFromPixels(pixim);
		kinectGray = kinectColor;

		// k2p calibration
		if (calibStateInd == 0) {
			// update calibration point from draggable point
			k2p_calibPoint.set(ofMap(k2p_draggablePoint.x, 0, kinect.width, 0, guiWindowW), ofMap(k2p_draggablePoint.y, kinect.height, 2 * kinect.height, 0, guiWindowH));

			// perform detection with shader
			gd_fboRes.begin();
			gd_shader.begin();
			gd_shader.setUniform1f("minX", tableMinW * kinect.width);
			gd_shader.setUniform1f("maxX", tableMaxW * kinect.width);
			gd_shader.setUniform1f("minY", tableMinH * kinect.height);
			gd_shader.setUniform1f("maxY", tableMaxH * kinect.height);
			gd_shader.setUniform1f("grayThresh", grayThresholdDetect);
			kinectGray.draw(0, 0);
			gd_shader.end();
			gd_fboRes.end();

			// perform detection
			ofPixels pix;
			gd_fboRes.readToPixels(pix);
			gd_resImColor.setFromPixels(pix);
			gd_resImGray = gd_resImColor;
			gd_resImGray.erode();
			gd_resImGray.dilate();

			contourFinder.findContours(gd_resImGray, minDet, maxDet, 1, true);
			if (contourFinder.nBlobs == 1) {
				k2p_resPoint.set(contourFinder.blobs[0].centroid);
			}
		}

		// t2k calibration
		if (calibStateInd == 1) {
			// perform detection with shader
			gd_fboRes.begin();
			gd_shader.begin();
			gd_shader.setUniform1f("minX", tableMinW * kinect.width);
			gd_shader.setUniform1f("maxX", tableMaxW * kinect.width);
			gd_shader.setUniform1f("minY", tableMinH * kinect.height);
			gd_shader.setUniform1f("maxY", tableMaxH * kinect.height);
			gd_shader.setUniform1f("grayThresh", grayThresholdDetect);
			kinectGray.draw(0, 0);
			gd_shader.end();
			gd_fboRes.end();

			// perform detection
			ofPixels pix;
			gd_fboRes.readToPixels(pix);
			gd_resImColor.setFromPixels(pix);
			gd_resImGray = gd_resImColor;
			gd_resImGray.erode();
			gd_resImGray.dilate();

			contourFinder.findContours(gd_resImGray, minDet, maxDet, 4, true);
		}

		// pattern detection
		if (calibStateInd == 2) {
			// perform detection with shader
			cd_fboRes.begin();
			cd_shader.begin();
			cd_shader.setUniform1f("minX", tableMinW * kinect.width);
			cd_shader.setUniform1f("maxX", tableMaxW * kinect.width);
			cd_shader.setUniform1f("minY", tableMinH * kinect.height);
			cd_shader.setUniform1f("maxY", tableMaxH * kinect.height);
			cd_shader.setUniform4f("colorToDetect", pd_colorToDetect.r, pd_colorToDetect.g, pd_colorToDetect.b, pd_colorToDetect.a);
			cd_shader.setUniform1f("colorThresh", pd_colorThresh);
			kinect.draw(0, 0);
			cd_shader.end();
			cd_fboRes.end();

			// perform detection
			ofPixels pix;
			cd_fboRes.readToPixels(pix);
			cd_resImColor.setFromPixels(pix);
			cd_resImGray = cd_resImColor;
			cd_resImGray.erode();
			cd_resImGray.dilate();

			contourFinder.findContours(cd_resImGray, minDet, maxDet, 2, true);
		}
	}

	// update state
	if (calibStateInd > 3) {
		calibStateInd = 3;
	}
	if (calibStateInd < 0) {
		calibStateInd = 0;
	}
}

//--------------------------------------------------------------
void ofApp::draw() {
	//
	ofBackground(0);

	//
	ofSetColor(ofColor::white);
	kinectColor.draw(0, 0);

	// draw table limits
	ofSetColor(ofColor::red);
	ofNoFill();
	ofDrawRectangle(tableLimits);

	// display info
	ofDrawBitmapStringHighlight("about:blank -- calibration", kinect.width * 2 + 10, 20);
	switch (calibStateInd) {
	case 0:
	{
		ofDrawBitmapStringHighlight("calib 1\tkinect <-> projector calibration", kinect.width * 2 + 10, 40);
		ofDrawBitmapStringHighlight("k2p calib point", kinect.width * 2 + 10, 80);

		ofDrawBitmapStringHighlight("actions :", kinect.width * 2 + 10, 80);
		ofDrawBitmapStringHighlight("- drag point in projection window to move white dot on table", kinect.width * 2 + 10, 100);
		ofDrawBitmapStringHighlight("- set detection sensitivity with 'GRAYTHRESHOLDDETECT'", kinect.width * 2 + 10, 120);
		ofDrawBitmapStringHighlight("- press SPACE when there is a match", kinect.width * 2 + 10, 140);
		ofDrawBitmapStringHighlight("- press 'c' to clear array", kinect.width * 2 + 10, 140);
		ofDrawBitmapStringHighlight("- when k2p_matrix is computed, click on kinect texture to check calibration", kinect.width * 2 + 10, 160);

		ofDrawBitmapStringHighlight("k2p proj point :", kinect.width * 2 + 10, 200);
		ofDrawBitmapStringHighlight("x = " + to_string(k2p_calibPoint.x), kinect.width * 2 + 10, 220);
		ofDrawBitmapStringHighlight("y = " + to_string(k2p_calibPoint.y), kinect.width * 2 + 10, 240);

		if (contourFinder.nBlobs == 1) {
			ofDrawBitmapStringHighlight("k2p kinect point", kinect.width * 2 + 210, 200, ofColor::black, ofColor::red);
			ofDrawBitmapStringHighlight("x = " + to_string(k2p_resPoint.x), kinect.width * 2 + 210, 220, ofColor::black, ofColor::red);
			ofDrawBitmapStringHighlight("y = " + to_string(k2p_resPoint.y), kinect.width * 2 + 210, 240, ofColor::black, ofColor::red);
		}

		ofDrawBitmapStringHighlight("kinect points", kinect.width * 2 + 30, 280);
		ofDrawBitmapStringHighlight("projector points", kinect.width * 2 + 330, 280);
		float k2p_debugPosY = 300;
		for (int i = 0; i < k2p_storePoints.size(); i++) {
			ofDrawBitmapStringHighlight("#" + to_string(i + 1), kinect.width * 2 + 10, k2p_debugPosY);
			ofDrawBitmapStringHighlight("(" + to_string((int)k2p_storePoints[i].x) + ", " + to_string((int)k2p_storePoints[i].y) + ")", kinect.width * 2 + 30, k2p_debugPosY);
			ofDrawBitmapStringHighlight("(" + to_string((int)k2p_storePoints[i].z) + ", " + to_string((int)k2p_storePoints[i].w) + ")", kinect.width * 2 + 330, k2p_debugPosY);

			k2p_debugPosY += 20;
		}

		// draw gray detection result
		ofSetColor(ofColor::white);
		gd_resImGray.draw(kinect.width, 0);
		contourFinder.draw(0, 0);
		contourFinder.draw(kinect.width, 0);

		// draw window to move calibration point accross the table
		ofNoFill();
		ofSetColor(ofColor::white);
		ofDrawRectangle(0, kinect.height, kinect.width, kinect.height);
		ofFill();
		ofSetColor(ofColor::purple);
		k2p_draggablePoint.draw(true);

		break;
	}
	case 1:
	{
		ofDrawBitmapStringHighlight("calib 2\ttable <-> kinect calibration", kinect.width * 2 + 10, 40);

		ofDrawBitmapStringHighlight("actions :", kinect.width * 2 + 10, 80);
		ofDrawBitmapStringHighlight("- place table <-> kinect calibration pattern on top left of table", kinect.width * 2 + 10, 100);
		ofDrawBitmapStringHighlight("- set detection sensitivity with 'GRAYTHRESHOLDDETECT'", kinect.width * 2 + 10, 120);

		ofDrawBitmapStringHighlight("table points", kinect.width * 2 + 30, 160);
		ofDrawBitmapStringHighlight("kinect points", kinect.width * 2 + 330, 160);
		float t2k_debugPosY = 180;
		for (int i = 0; i < t2k_storePoints.size(); i++) {
			ofDrawBitmapStringHighlight("#" + to_string(i + 1), kinect.width * 2 + 10, t2k_debugPosY);
			ofDrawBitmapStringHighlight("(" + to_string((int)t2k_storePoints[i].x) + ", " + to_string((int)t2k_storePoints[i].y) + ")", kinect.width * 2 + 30, t2k_debugPosY);
			ofDrawBitmapStringHighlight("(" + to_string((int)t2k_storePoints[i].z) + ", " + to_string((int)t2k_storePoints[i].w) + ")", kinect.width * 2 + 330, t2k_debugPosY);
			t2k_debugPosY += 20;
		}

		ofSetColor(ofColor::white);
		gd_resImGray.draw(kinect.width, 0);
		contourFinder.draw(0, 0);
		contourFinder.draw(kinect.width, 0);

		break;
	}
	case 2:
	{
		ofDrawBitmapStringHighlight("calib 3\tpatterns color calibration", kinect.width * 2 + 10, 40);

		ofDrawBitmapStringHighlight("actions :", kinect.width * 2 + 10, 80);
		ofDrawBitmapStringHighlight("- click on kinect image to select color to detect", kinect.width * 2 + 10, 100);
		ofDrawBitmapStringHighlight("- set detection sensitivity with 'pd_colorthresh'", kinect.width * 2 + 10, 120);

		ofDrawBitmapStringHighlight("detected color : ", kinect.width * 2 + 10, 160);
		ofSetColor(pd_colorToDetect);
		ofFill();
		ofDrawRectangle(kinect.width * 2 + 10, 180, 50, 50);

		ofSetColor(ofColor::white);
		cd_resImGray.draw(kinect.width, 0);
		contourFinder.draw(0, 0);
		contourFinder.draw(kinect.width, 0);


		break;
	}

	case 3:
		ofDrawBitmapStringHighlight("calib 4\tpage flipping calibration", kinect.width * 2 + 10, 40);
		break;
	}

	// draw gui
	gui.draw();
}

//--------------------------------------------------------------
void ofApp::drawGui(ofEventArgs& args) {
	//
	ofBackground(0);


	switch (calibStateInd) {
		// state 0 : kinect <-> projector calibration
	case 0:
	{
		// draw calib point from draggable point
		ofSetColor(ofColor::white);
		ofFill();
		ofDrawCircle(k2p_calibPoint, 20);

		if (k2p_matrixComputed) {
			ofSetColor(ofColor::white);
			ofFill();
			ofDrawCircle(k2p_matrix * k2p_testPoint_k, 10);
		}
		break;
	}

	case 1:
	{
		if (t2k_matrixComputed) {

			// get center of table in table space
			ofVec3f t2k_testPoint_t = k2t_matrix * t2k_testPoint_k;
			ofVec3f t2k_testPoint_p = k2p_matrix * t2k_testPoint_k;

			// get 10cm by 10cm quad in table space
			vector<ofVec3f> tableQuad_t;
			tableQuad_t.push_back(t2k_testPoint_t + ofVec3f(-5, -5));
			tableQuad_t.push_back(t2k_testPoint_t + ofVec3f(5, -5));
			tableQuad_t.push_back(t2k_testPoint_t + ofVec3f(5, 5));
			tableQuad_t.push_back(t2k_testPoint_t + ofVec3f(-5, 5));

			// get quad in projector space
			vector<ofVec3f> tablerQuad_p;
			for (int i = 0; i < tableQuad_t.size(); i++) {
				tablerQuad_p.push_back((k2p_matrix * (t2k_matrix * tableQuad_t[i])));
			}

			ofSetColor(ofColor::white);
			ofFill();
			ofDrawCircle(t2k_testPoint_p, 10);

			ofNoFill();
			ofBeginShape();
			for (int i = 0; i < tablerQuad_p.size(); i++) {
				ofVertex(tablerQuad_p[i]);
			}
			ofEndShape(true);

		}
		break;
	}
	}
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key) {
	if (key == '1') {
		calibStateInd = 0;
	}
	if (key == '2') {
		calibStateInd = 1;
	}
	if (key == '3') {
		calibStateInd = 2;
	}
	if (key == '4') {
		calibStateInd = 3;
	}

	if (key == ' ') {
		// save calib point in state 0
		if (k2p_storePoints.size() < 4) {
			if (calibStateInd == 0) {
				k2p_storePoints.push_back(ofVec4f(k2p_resPoint.x, k2p_resPoint.y, k2p_calibPoint.x, k2p_calibPoint.y));
			}
		}
	}

	if (key == 'c') {
		// clear points
		if (calibStateInd == 0) {
			k2p_storePoints.clear();
		}
	}

	if (key == 'm') {
		// k2p -> compute homography matrix
		if (calibStateInd == 0 && k2p_storePoints.size() == 4) {
			ofPoint kPoints[4];
			ofPoint pPoints[4];

			for (int i = 0; i < k2p_storePoints.size(); i++) {
				kPoints[i] = ofPoint(k2p_storePoints[i].x, k2p_storePoints[i].y);
				pPoints[i] = ofPoint(k2p_storePoints[i].z, k2p_storePoints[i].w);
			}
			k2p_matrix = findHomography(kPoints, pPoints);
			p2k_matrix = findHomography(pPoints, kPoints);
			k2p_matrixComputed = true;
		}

		// t2k -> compute homography matrix
		if (calibStateInd == 1) {
			ofPoint tPoints[4];
			ofPoint kPoints[4];

			for (int i = 0; i < t2k_storePoints.size(); i++) {
				tPoints[i] = ofPoint(t2k_storePoints[i].x, t2k_storePoints[i].y);
				kPoints[i] = ofPoint(t2k_storePoints[i].z, t2k_storePoints[i].w);
			}
			t2k_matrix = findHomography(tPoints, kPoints);
			k2t_matrix = findHomography(kPoints, tPoints);
			t2k_matrixComputed = true;
		}
	}
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key) {

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button) {
	// k2p_calib move test point 
	if (calibStateInd == 0) {
		k2p_testPoint_k.set(x, y, 0);
	}

	// table kinect calibration
	if (calibStateInd == 1) {
		t2k_testPoint_k.set(x, y, 0);

		if (x < kinect.width && y < kinect.height) {
			for (int i = 0; i < contourFinder.nBlobs; i++) {
				ofPolyline polyL = ofPolyline(contourFinder.blobs[i].pts);
				if (pointIsInPolygon(polyL, ofPoint(x, y))) {
					t2k_storePoints[t2k_calibOrder].z = contourFinder.blobs[i].centroid.x;
					t2k_storePoints[t2k_calibOrder].w = contourFinder.blobs[i].centroid.y;
					t2k_calibOrder++;
				}
			}
		}
	}

	// pattern color detection
	if (calibStateInd == 2) {
		if (x < kinect.width && y < kinect.height) {
			ofImage temp;
			temp.allocate(kinect.width, kinect.height, OF_IMAGE_COLOR);
			temp.setFromPixels(kinectColor.getPixels());
			pd_colorToDetect = temp.getColor(x, y);
		}
	}
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h) {

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg) {

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo) {

}

//--------------------------------------------------------------
void ofApp::saveCalibFunc() {
	gui.saveToFile("aboutblank_calib.json");

	if (k2p_storePoints.size() > 0 && !dataSettings.tagExists("k2p_storePoints")) {
		dataSettings.addTag("k2p_storePoints");
		dataSettings.pushTag("k2p_storePoints");
		for (int i = 0; i < k2p_storePoints.size(); i++) {
			dataSettings.addTag("k2p_storePoint");
			dataSettings.pushTag("k2p_storePoint", i);

			dataSettings.addValue("x", k2p_storePoints[i].x);
			dataSettings.addValue("y", k2p_storePoints[i].y);
			dataSettings.addValue("z", k2p_storePoints[i].z);
			dataSettings.addValue("w", k2p_storePoints[i].w);

			dataSettings.popTag();
		}
		dataSettings.popTag();
	}

	if (!dataSettings.tagExists("t2k_storePoints")) {
		dataSettings.addTag("t2k_storePoints");
		dataSettings.pushTag("t2k_storePoints");
		for (int i = 0; i < t2k_storePoints.size(); i++) {
			dataSettings.addTag("t2k_storePoint");
			dataSettings.pushTag("t2k_storePoint", i);

			dataSettings.addValue("x", t2k_storePoints[i].x);
			dataSettings.addValue("y", t2k_storePoints[i].y);
			dataSettings.addValue("z", t2k_storePoints[i].z);
			dataSettings.addValue("w", t2k_storePoints[i].w);

			dataSettings.popTag();
		}
		dataSettings.popTag();
	}

	if (!dataSettings.tagExists("pd_colorToDetect")) {
		dataSettings.addTag("pd_colorToDetect");
	}
	else {
		dataSettings.pushTag("pd_colorToDetect");
		if (dataSettings.getValue("r", -1) == -1) {
			dataSettings.addValue("r", pd_colorToDetect.r);
			dataSettings.addValue("g", pd_colorToDetect.g);
			dataSettings.addValue("b", pd_colorToDetect.b);
			dataSettings.addValue("a", pd_colorToDetect.a);
		}
		else {
			dataSettings.setValue("r", pd_colorToDetect.r);
			dataSettings.setValue("g", pd_colorToDetect.g);
			dataSettings.setValue("b", pd_colorToDetect.b);
			dataSettings.setValue("a", pd_colorToDetect.a);
		}
		dataSettings.popTag();
	}

	dataSettings.saveFile("aboutblank_data.xml");
}


void ofApp::gaussian_elimination(float* input, int n) {
	// ported to c from pseudocode in
	// http://en.wikipedia.org/wiki/Gaussian_elimination

	float* A = input;
	int i = 0;
	int j = 0;
	int m = n - 1;
	while (i < m && j < n) {
		// Find pivot in column j, starting in row i:
		int maxi = i;
		for (int k = i + 1; k < m; k++) {
			if (fabs(A[k * n + j]) > fabs(A[maxi * n + j])) {
				maxi = k;
			}
		}
		if (A[maxi * n + j] != 0) {
			//swap rows i and maxi, but do not change the value of i
			if (i != maxi)
				for (int k = 0; k < n; k++) {
					float aux = A[i * n + k];
					A[i * n + k] = A[maxi * n + k];
					A[maxi * n + k] = aux;
				}
			//Now A[i,j] will contain the old value of A[maxi,j].
			//divide each entry in row i by A[i,j]
			float A_ij = A[i * n + j];
			for (int k = 0; k < n; k++) {
				A[i * n + k] /= A_ij;
			}
			//Now A[i,j] will have the value 1.
			for (int u = i + 1; u < m; u++) {
				//subtract A[u,j] * row i from row u
				float A_uj = A[u * n + j];
				for (int k = 0; k < n; k++) {
					A[u * n + k] -= A_uj * A[i * n + k];
				}
				//Now A[u,j] will be 0, since A[u,j] - A[i,j] * A[u,j] = A[u,j] - 1 * A[u,j] = 0.
			}

			i++;
		}
		j++;
	}

	//back substitution
	for (int i = m - 2; i >= 0; i--) {
		for (int j = i + 1; j < n - 1; j++) {
			A[i * n + m] -= A[i * n + j] * A[j * n + m];
			//A[i*n+j]=0;
		}
	}
}

void ofApp::findHomography(ofPoint src[4], ofPoint dst[4], float homography[16]) {

	// create the equation system to be solved
	//
	// from: Multiple View Geometry in Computer Vision 2ed
	//       Hartley R. and Zisserman A.
	//
	// x' = xH
	// where H is the homography: a 3 by 3 matrix
	// that transformed to inhomogeneous coordinates for each point
	// gives the following equations for each point:
	//
	// x' * (h31*x + h32*y + h33) = h11*x + h12*y + h13
	// y' * (h31*x + h32*y + h33) = h21*x + h22*y + h23
	//
	// as the homography is scale independent we can let h33 be 1 (indeed any of the terms)
	// so for 4 points we have 8 equations for 8 terms to solve: h11 - h32
	// after ordering the terms it gives the following matrix
	// that can be solved with gaussian elimination:

	float P[8][9] = {
			{-src[0].x, -src[0].y, -1,   0,   0,  0, src[0].x * dst[0].x, src[0].y * dst[0].x, -dst[0].x }, // h11
			{  0,   0,  0, -src[0].x, -src[0].y, -1, src[0].x * dst[0].y, src[0].y * dst[0].y, -dst[0].y }, // h12

			{-src[1].x, -src[1].y, -1,   0,   0,  0, src[1].x * dst[1].x, src[1].y * dst[1].x, -dst[1].x }, // h13
			{  0,   0,  0, -src[1].x, -src[1].y, -1, src[1].x * dst[1].y, src[1].y * dst[1].y, -dst[1].y }, // h21

			{-src[2].x, -src[2].y, -1,   0,   0,  0, src[2].x * dst[2].x, src[2].y * dst[2].x, -dst[2].x }, // h22
			{  0,   0,  0, -src[2].x, -src[2].y, -1, src[2].x * dst[2].y, src[2].y * dst[2].y, -dst[2].y }, // h23

			{-src[3].x, -src[3].y, -1,   0,   0,  0, src[3].x * dst[3].x, src[3].y * dst[3].x, -dst[3].x }, // h31
			{  0,   0,  0, -src[3].x, -src[3].y, -1, src[3].x * dst[3].y, src[3].y * dst[3].y, -dst[3].y }, // h32
	};

	gaussian_elimination(&P[0][0], 9);

	// gaussian elimination gives the results of the equation system
	// in the last column of the original matrix.
	// opengl needs the transposed 4x4 matrix:
	float aux_H[] = { P[0][8],P[3][8],0,P[6][8], // h11  h21 0 h31
					P[1][8],P[4][8],0,P[7][8], // h12  h22 0 h32
					0      ,      0,0,0,       // 0    0   0 0
					P[2][8],P[5][8],0,1 };      // h13  h23 0 h33

	for (int i = 0; i < 16; i++) homography[i] = aux_H[i];
}

ofMatrix4x4 ofApp::findHomography(ofPoint src[4], ofPoint dst[4]) {
	ofMatrix4x4 matrix;

	// create the equation system to be solved
	//
	// from: Multiple View Geometry in Computer Vision 2ed
	//       Hartley R. and Zisserman A.
	//
	// x' = xH
	// where H is the homography: a 3 by 3 matrix
	// that transformed to inhomogeneous coordinates for each point
	// gives the following equations for each point:
	//
	// x' * (h31*x + h32*y + h33) = h11*x + h12*y + h13
	// y' * (h31*x + h32*y + h33) = h21*x + h22*y + h23
	//
	// as the homography is scale independent we can let h33 be 1 (indeed any of the terms)
	// so for 4 points we have 8 equations for 8 terms to solve: h11 - h32
	// after ordering the terms it gives the following matrix
	// that can be solved with gaussian elimination:

	float P[8][9] = {
			{-src[0].x, -src[0].y, -1,   0,   0,  0, src[0].x * dst[0].x, src[0].y * dst[0].x, -dst[0].x }, // h11
			{  0,   0,  0, -src[0].x, -src[0].y, -1, src[0].x * dst[0].y, src[0].y * dst[0].y, -dst[0].y }, // h12

			{-src[1].x, -src[1].y, -1,   0,   0,  0, src[1].x * dst[1].x, src[1].y * dst[1].x, -dst[1].x }, // h13
			{  0,   0,  0, -src[1].x, -src[1].y, -1, src[1].x * dst[1].y, src[1].y * dst[1].y, -dst[1].y }, // h21

			{-src[2].x, -src[2].y, -1,   0,   0,  0, src[2].x * dst[2].x, src[2].y * dst[2].x, -dst[2].x }, // h22
			{  0,   0,  0, -src[2].x, -src[2].y, -1, src[2].x * dst[2].y, src[2].y * dst[2].y, -dst[2].y }, // h23

			{-src[3].x, -src[3].y, -1,   0,   0,  0, src[3].x * dst[3].x, src[3].y * dst[3].x, -dst[3].x }, // h31
			{  0,   0,  0, -src[3].x, -src[3].y, -1, src[3].x * dst[3].y, src[3].y * dst[3].y, -dst[3].y }, // h32
	};

	gaussian_elimination(&P[0][0], 9);

	matrix(0, 0) = P[0][8];
	matrix(0, 1) = P[1][8];
	matrix(0, 2) = 0;
	matrix(0, 3) = P[2][8];

	matrix(1, 0) = P[3][8];
	matrix(1, 1) = P[4][8];
	matrix(1, 2) = 0;
	matrix(1, 3) = P[5][8];

	matrix(2, 0) = 0;
	matrix(2, 1) = 0;
	matrix(2, 2) = 0;
	matrix(2, 3) = 0;

	matrix(3, 0) = P[6][8];
	matrix(3, 1) = P[7][8];
	matrix(3, 2) = 0;
	matrix(3, 3) = 1;

	return matrix;
}





