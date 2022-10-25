#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup() {
	// general
	ofSetFrameRate(30);

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
	gui.add(minDet.setup("MINDET", 5, 1, 500));
	gui.add(maxDet.setup("MAXDET", 400, 1, 500));
	// calib state 2
	gui.add(pd_colorThresh.setup("PD_COLORTHRESH", 0.1, 0.0, 1.0));
	// calib state 3
	gui.add(speedThreshold.setup("SPEEDTHRESHOLD", 0.0, 0.0, 5.0));
	gui.add(pageMinDet.setup("PAGEMINDET", 5, 1, 500));
	gui.add(pageMaxDet.setup("PAGEMAXDET", 250, 1, 1000));
	gui.add(pageAgeDetect.setup("PAGEAGEDETECT", 10, 2, 20));

	// calib state 3
	gui.add(bookInteriorOffsetX.setup("BOOKINTERIOROFFSETX", 0, 0, 10));
	gui.add(bookInteriorOffsetY.setup("BOOKINTERIOROFFSETY", 0, 0, 10));
	gui.add(bookLength.setup("BOOKLENGTH", 20, 5, 50));

	// functions
	saveCalib.addListener(this, &ofApp::saveCalibFunc);

	// init stuff for page flipping detection
	pf_pageContourFinder.setThreshold(15);
	pf_pageContourFinder.getTracker().setMaximumDistance(50);
	pf_pageContourFinder.getTracker().setPersistence(20);

	// load videos
	ofBuffer videoListbuffer = ofBufferFromFile("videoList.txt");
	for (auto line : videoListbuffer.getLines()) {
		videoPaths.push_back(line);
	}

	// load first video
	video.load(videoPaths[videoCurrInd]);
	video.play();
	videoSrcPoints[0] = ofPoint(0, 0);
	videoSrcPoints[1] = ofPoint(video.getWidth(), 0);
	videoSrcPoints[2] = ofPoint(video.getWidth(), video.getHeight());
	videoSrcPoints[3] = ofPoint(0, video.getHeight());

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
			dataSettings.pushTag("t2k_storePoint", i);

			ofVec4f p;
			p.x = dataSettings.getValue("x", 0);
			p.y = dataSettings.getValue("y", 0);
			p.z = dataSettings.getValue("z", 0);
			p.w = dataSettings.getValue("w", 0);

			t2k_storePoints.push_back(p);
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

	// compute kinect <-> projector matrix
	ofPoint kPoints[4];
	ofPoint pPoints[4];
	for (int i = 0; i < k2p_storePoints.size(); i++) {
		kPoints[i] = ofPoint(k2p_storePoints[i].x, k2p_storePoints[i].y);
		pPoints[i] = ofPoint(k2p_storePoints[i].z, k2p_storePoints[i].w);
	}
	k2p_matrix = findHomography(kPoints, pPoints);
	p2k_matrix = findHomography(pPoints, kPoints);

	// compute table <-> kinectmatrix
	ofPoint tPoints[4];
	//ofPoint kPoints[4];

	for (int i = 0; i < t2k_storePoints.size(); i++) {
		tPoints[i] = ofPoint(t2k_storePoints[i].x, t2k_storePoints[i].y);
		kPoints[i] = ofPoint(t2k_storePoints[i].z, t2k_storePoints[i].w);
	}
	t2k_matrix = findHomography(tPoints, kPoints);
	k2t_matrix = findHomography(kPoints, tPoints);

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
	// init fbo mask
	pf_fboMask.allocate(kinect.width, kinect.height, GL_RED);
	pf_bg.allocate(kinect.width, kinect.height);
	pf_diff.allocate(kinect.width, kinect.height);
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

	// update video
	video.update();
	if (videoCurrInd < 0) videoCurrInd = 0;
	if (videoCurrInd >= videoPaths.size()) videoCurrInd = videoPaths.size() - 1;

	// store data
	if (kinect.isFrameNew()) {
		// get RGB sensor of kinect
		ofPixels& pixim = kinect.getPixels();
		kinectColor.setFromPixels(pixim);
		kinectGray = kinectColor;

		// get depth sensor of kinect
		ofPixels& pixDepth = kinect.getDepthPixels();
		kinectDepth.setFromPixels(pixDepth);

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
		pd_contourFinder.findContours(cd_resImGray, minDet, maxDet, 2, true);

		// set left and right patterns
		if (pd_contourFinder.nBlobs == 2) {
			// vidoe volume to 1
			video.setVolume(1.0);

			if (pd_contourFinder.blobs[0].centroid.x < pd_contourFinder.blobs[1].centroid.x) {
				patternLeft_k.set(pd_contourFinder.blobs[1].centroid);
				patternRight_k.set(pd_contourFinder.blobs[0].centroid);
			}
			else {
				patternLeft_k.set(pd_contourFinder.blobs[0].centroid);
				patternRight_k.set(pd_contourFinder.blobs[1].centroid);
			}
			ofVec3f patternLeft_t = k2t_matrix * patternLeft_k;
			ofVec3f patternRight_t = k2t_matrix * patternRight_k;
			float minPattern_y = min(patternLeft_k.y, patternRight_k.y);
			float maxPattern_y = max(patternLeft_k.y, patternRight_k.y);

			// detect movement
			float patternLeftSpeed = patternLeft_k.distanceSquared(patternLeft_k_prev);
			float patternRightSpeed = patternRight_k.distanceSquared(patternRight_k_prev);
			bookIsMoving = (patternLeftSpeed > speedThreshold && patternRightSpeed > speedThreshold);

			// store background for page flipping detection
			if (!bookIsMoving and bookIsMoving_prev) {
				pf_bg.setFromPixels(kinectDepth.getPixels());
			}

			// book is not moving
			if (!bookIsMoving) {
				pf_diff.absDiff(pf_bg, kinectDepth);
				pf_diff.threshold(10);

				// perform detection with shader
				gd_fboRes.begin();
				gd_shader.begin();
				gd_shader.setUniform1f("minX", patternRight_k.x - 10);
				gd_shader.setUniform1f("maxX", patternLeft_k.x + 10);
				gd_shader.setUniform1f("minY", minPattern_y - 50);
				gd_shader.setUniform1f("maxY", maxPattern_y + 10);
				gd_shader.setUniform1f("grayThresh", grayThresholdDetect);
				pf_diff.draw(0, 0);
				gd_shader.end();
				gd_fboRes.end();

				// perform detection
				ofPixels pix;
				gd_fboRes.readToPixels(pix);
				gd_resImColor.setFromPixels(pix);
				gd_resImGray = gd_resImColor;
				gd_resImGray.erode();
				gd_resImGray.erode();
				gd_resImGray.dilate();
				gd_resImGray.dilate();

				pf_pageContourFinder.setMinAreaRadius(pageMinDet);
				pf_pageContourFinder.setMaxAreaRadius(pageMaxDet);
				pf_pageContourFinder.findContours(gd_resImGray);

				// if a contour exists, compare position at age 1 and age 10 to get direction
				pf_pageBackward = false;
				pf_pageForward = false;
				if (pf_pageContourFinder.size() == 1) {
					if (pf_pageContourFinder.getTracker().getAge(pf_pageContourFinder.getLabel(0)) == 1) {
						pf_contourStartX = pf_pageContourFinder.getCentroid(0).x;
					}
					if (pf_pageContourFinder.getTracker().getAge(pf_pageContourFinder.getLabel(0)) == pageAgeDetect) {
						float pf_contourDiffX = pf_contourStartX - pf_pageContourFinder.getCentroid(0).x;
						if (pf_contourDiffX < 0) {
							pf_pageForward = true;
							changeVideo(videoCurrInd + 1);
						}
						else {
							pf_pageBackward = true;
							changeVideo(videoCurrInd - 1);
						}
					}
				}
			}
			// book is moving
			else {
				// set detection quad in table space
				bookAngle = angleBetweenPoints(ofVec2f(patternLeft_t), ofVec2f(patternRight_t));
				bookQuad_t.clear();
				bookQuad_t.push_back(patternLeft_t + ofVec3f(bookInteriorOffsetX * cos(bookAngle) - bookInteriorOffsetY * sin(bookAngle), bookInteriorOffsetX * sin(bookAngle) + bookInteriorOffsetY * cos(bookAngle)));
				bookQuad_t.push_back(patternRight_t + ofVec3f(-bookInteriorOffsetX * cos(bookAngle) - bookInteriorOffsetY * sin(bookAngle), -bookInteriorOffsetX * sin(bookAngle) + bookInteriorOffsetY * cos(bookAngle)));
				bookQuad_t.push_back(patternRight_t + ofVec3f(-bookInteriorOffsetX * cos(bookAngle) - (bookInteriorOffsetY + bookLength) * sin(bookAngle), -bookInteriorOffsetX * sin(bookAngle) + (bookInteriorOffsetY + bookLength) * cos(bookAngle)));
				bookQuad_t.push_back(patternLeft_t + ofVec3f(bookInteriorOffsetX * cos(bookAngle) - (bookInteriorOffsetY + bookLength) * sin(bookAngle), bookInteriorOffsetX * sin(bookAngle) + (bookInteriorOffsetY + bookLength) * cos(bookAngle)));

				// set detection quad in kinect space
				bookQuad_k.clear();
				bookQuad_k.push_back(t2k_matrix * bookQuad_t[0]);
				bookQuad_k.push_back(t2k_matrix * bookQuad_t[1]);
				bookQuad_k.push_back(t2k_matrix * bookQuad_t[2]);
				bookQuad_k.push_back(t2k_matrix * bookQuad_t[3]);

				// set detection quad in projector space
				bookQuad_p.clear();
				bookQuad_p.push_back(k2p_matrix * bookQuad_k[0]);
				bookQuad_p.push_back(k2p_matrix * bookQuad_k[1]);
				bookQuad_p.push_back(k2p_matrix * bookQuad_k[2]);
				bookQuad_p.push_back(k2p_matrix * bookQuad_k[3]);

				// compute projection matrix
				ofPoint _bookQuad_p[4];
				_bookQuad_p[0] = bookQuad_p[0];
				_bookQuad_p[1] = bookQuad_p[1];
				_bookQuad_p[2] = bookQuad_p[2];
				_bookQuad_p[3] = bookQuad_p[3];
				findHomography(videoSrcPoints, _bookQuad_p, projectionMatrix);
			}

			// update pattern positions
			patternLeft_k_prev = patternLeft_k;
			patternRight_k_prev = patternRight_k;
			bookIsMoving_prev = bookIsMoving;
			bookIsGoneCountdown = 0;
		}

		bookIsGone = (pd_contourFinder.nBlobs == 0);

		if (bookIsGone) {
			video.setVolume(0.0);
			bookIsGoneCountdown++;

			if (bookIsGoneCountdown == 5 * 30) {
				// reset to first video
				changeVideo(0);
			}
		}

		// update book presence
		bookIsGone_prev = bookIsGone;

		/*
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

			pd_contourFinder.findContours(cd_resImGray, minDet, maxDet, 2, true);
		}

		// page flipping detection
		if (calibStateInd == 3) {
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
			pd_contourFinder.findContours(cd_resImGray, minDet, maxDet, 2, true);

			// set left and right patterns
			if (pd_contourFinder.nBlobs == 2) {
				if (pd_contourFinder.blobs[0].centroid.x < pd_contourFinder.blobs[1].centroid.x) {
					patternLeft_k.set(pd_contourFinder.blobs[1].centroid);
					patternRight_k.set(pd_contourFinder.blobs[0].centroid);
				}
				else {
					patternLeft_k.set(pd_contourFinder.blobs[0].centroid);
					patternRight_k.set(pd_contourFinder.blobs[1].centroid);
				}
				float minPattern_y = min(patternLeft_k.y, patternRight_k.y);
				float maxPattern_y = max(patternLeft_k.y, patternRight_k.y);

				// detect movement
				float patternLeftSpeed = patternLeft_k.distanceSquared(patternLeft_k_prev);
				float patternRightSpeed = patternRight_k.distanceSquared(patternRight_k_prev);
				bookIsMoving = (patternLeftSpeed > speedThreshold && patternRightSpeed > speedThreshold);

				// store background for page flipping detection
				if (!bookIsMoving and bookIsMoving_prev) {
					pf_bg.setFromPixels(kinectDepth.getPixels());
				}

				//
				if (!bookIsMoving) {
					pf_diff.absDiff(pf_bg, kinectDepth);
					pf_diff.threshold(10);

					// perform detection with shader
					gd_fboRes.begin();
					gd_shader.begin();
					gd_shader.setUniform1f("minX", patternRight_k.x - 10);
					gd_shader.setUniform1f("maxX", patternLeft_k.x + 10);
					gd_shader.setUniform1f("minY", minPattern_y - 50);
					gd_shader.setUniform1f("maxY", maxPattern_y + 10);
					gd_shader.setUniform1f("grayThresh", grayThresholdDetect);
					pf_diff.draw(0, 0);
					gd_shader.end();
					gd_fboRes.end();

					// perform detection
					ofPixels pix;
					gd_fboRes.readToPixels(pix);
					gd_resImColor.setFromPixels(pix);
					gd_resImGray = gd_resImColor;
					gd_resImGray.erode();
					gd_resImGray.erode();
					gd_resImGray.dilate();
					gd_resImGray.dilate();

					pf_pageContourFinder.setMinAreaRadius(pageMinDet);
					pf_pageContourFinder.setMaxAreaRadius(pageMaxDet);
					pf_pageContourFinder.findContours(gd_resImGray);

					// if a contour exists, compare position at age 1 and age 10 to get direction
					pf_pageBackward = false;
					pf_pageForward = false;
					if (pf_pageContourFinder.size() == 1) {
						if (pf_pageContourFinder.getTracker().getAge(pf_pageContourFinder.getLabel(0)) == 1) {
							pf_contourStartX = pf_pageContourFinder.getCentroid(0).x;
						}
						if (pf_pageContourFinder.getTracker().getAge(pf_pageContourFinder.getLabel(0)) == 10) {
							float pf_contourDiffX = pf_contourStartX - pf_pageContourFinder.getCentroid(0).x;
							if (pf_contourDiffX < 0) {
								pf_pageForward = true;
							}
							else {
								pf_pageBackward = true;
							}
						}
					}
				}

				// update pattern positions
				patternLeft_k_prev = patternLeft_k;
				patternRight_k_prev = patternRight_k;
				bookIsMoving_prev = bookIsMoving;
			}
		}

		// book size and mapping calibration
		if (calibStateInd == 4) {
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
			pd_contourFinder.findContours(cd_resImGray, minDet, maxDet, 2, true);

			// set left and right patterns
			if (pd_contourFinder.nBlobs == 2) {
				if (pd_contourFinder.blobs[0].centroid.x < pd_contourFinder.blobs[1].centroid.x) {
					patternLeft_k.set(pd_contourFinder.blobs[1].centroid);
					patternRight_k.set(pd_contourFinder.blobs[0].centroid);
				}
				else {
					patternLeft_k.set(pd_contourFinder.blobs[0].centroid);
					patternRight_k.set(pd_contourFinder.blobs[1].centroid);
				}
				ofVec3f patternLeft_t = k2t_matrix * patternLeft_k;
				ofVec3f patternRight_t = k2t_matrix * patternRight_k;

				// set detection quad in table space
				bookAngle = angleBetweenPoints(ofVec2f(patternLeft_t), ofVec2f(patternRight_t));
				bookQuad_t.clear();
				bookQuad_t.push_back(patternLeft_t + ofVec3f(bookInteriorOffsetX * cos(bookAngle) - bookInteriorOffsetY * sin(bookAngle), bookInteriorOffsetX * sin(bookAngle) + bookInteriorOffsetY * cos(bookAngle)));
				bookQuad_t.push_back(patternRight_t + ofVec3f(-bookInteriorOffsetX * cos(bookAngle) - bookInteriorOffsetY * sin(bookAngle), -bookInteriorOffsetX * sin(bookAngle) + bookInteriorOffsetY * cos(bookAngle)));
				bookQuad_t.push_back(patternRight_t + ofVec3f(-bookInteriorOffsetX * cos(bookAngle) - (bookInteriorOffsetY + bookLength) * sin(bookAngle), -bookInteriorOffsetX * sin(bookAngle) + (bookInteriorOffsetY + bookLength) * cos(bookAngle)));
				bookQuad_t.push_back(patternLeft_t + ofVec3f(bookInteriorOffsetX * cos(bookAngle) - (bookInteriorOffsetY + bookLength) * sin(bookAngle), bookInteriorOffsetX * sin(bookAngle) + (bookInteriorOffsetY + bookLength) * cos(bookAngle)));

				// set detection quad in kinect space
				bookQuad_k.clear();
				bookQuad_k.push_back(t2k_matrix * bookQuad_t[0]);
				bookQuad_k.push_back(t2k_matrix * bookQuad_t[1]);
				bookQuad_k.push_back(t2k_matrix * bookQuad_t[2]);
				bookQuad_k.push_back(t2k_matrix * bookQuad_t[3]);

				// set detection quad in projector space
				bookQuad_p.clear();
				bookQuad_p.push_back(k2p_matrix * bookQuad_k[0]);
				bookQuad_p.push_back(k2p_matrix * bookQuad_k[1]);
				bookQuad_p.push_back(k2p_matrix * bookQuad_k[2]);
				bookQuad_p.push_back(k2p_matrix * bookQuad_k[3]);

				// compute projection matrix
				ofPoint _bookQuad_p[4];
				_bookQuad_p[0] = bookQuad_p[0];
				_bookQuad_p[1] = bookQuad_p[1];
				_bookQuad_p[2] = bookQuad_p[2];
				_bookQuad_p[3] = bookQuad_p[3];
				findHomography(videoSrcPoints, _bookQuad_p, projectionMatrix);
			}
		}
		*/
	}
}

//--------------------------------------------------------------
void ofApp::draw() {
	//
	ofBackground(0);

	//
	if (showDebug) {
		ofSetColor(ofColor::white);
		kinectColor.draw(0, 0);


		// draw table limits
		ofSetColor(ofColor::red);
		ofNoFill();
		ofDrawRectangle(tableLimits);
	}

	// display info
	ofDrawBitmapStringHighlight("about:blank -- run", kinect.width * 2 + 10, 20);

	if (bookIsMoving) {
		ofDrawBitmapStringHighlight("book is moving.", kinect.width * 2 + 10, 80);
	}

	ofDrawBitmapStringHighlight("page backward", kinect.width * 2 + 10, 100);
	ofDrawBitmapStringHighlight("page forward", kinect.width * 2 + 310, 100);
	if (pf_pageBackward) {
		ofSetColor(ofColor::white);
		ofFill();
		ofDrawCircle(kinect.width * 2 + 20, 120, 5);
	}
	if (pf_pageForward) {
		ofSetColor(ofColor::white);
		ofFill();
		ofDrawCircle(kinect.width * 2 + 320, 120, 5);
	}

	//
	ofDrawBitmapStringHighlight(to_string(videoPaths.size()) + " videos", kinect.width * 2 + 10, 160);
	float videoListPosY = 180;
	for (int i = 0; i < videoPaths.size(); i++) {
		ofDrawBitmapStringHighlight("#" + to_string(i+1), kinect.width * 2 + 10, videoListPosY, ofColor::black, i == videoCurrInd ? ofColor::red : ofColor::white);
		ofDrawBitmapStringHighlight(videoPaths[i], kinect.width * 2 + 50, videoListPosY, ofColor::black, i == videoCurrInd ? ofColor::red : ofColor::white);
		videoListPosY += 20;
	}

	if (showDebug) {
		ofSetColor(ofColor::white);
		cd_resImGray.draw(kinect.width, 0);
		pd_contourFinder.draw(kinect.width, 0);
		pf_bg.draw(0, kinect.height);
		pf_diff.draw(kinect.width, kinect.height);
		gd_resImGray.draw(kinect.width, kinect.height);
		ofPushMatrix();
		ofTranslate(kinect.width, kinect.height);
		ofSetColor(ofColor::blue);
		pf_pageContourFinder.draw();
		ofPopMatrix();

		ofDrawBitmapStringHighlight("L", patternLeft_k);
		ofDrawBitmapStringHighlight("R", patternRight_k);

		// draw book boundaries on kinect space
		if (bookQuad_k.size() == 4) {
			ofSetColor(ofColor::black);
			ofBeginShape();
			ofVertex(bookQuad_k[0]);
			ofVertex(bookQuad_k[1]);
			ofVertex(bookQuad_k[2]);
			ofVertex(bookQuad_k[3]);
			ofEndShape(true);
			ofDrawLine(bookQuad_k[0], bookQuad_k[2]);
			ofDrawLine(bookQuad_k[1], bookQuad_k[3]);
		}
	}

	// draw gui
	gui.draw();
	ofDrawBitmapStringHighlight("fps : " + to_string(ofGetFrameRate()), 10, ofGetHeight() - 10);
}

//--------------------------------------------------------------
void ofApp::drawGui(ofEventArgs& args) {
	//
	ofBackground(0);

	// draw video
	if (!bookIsGone) {
		ofPushMatrix();
		ofMultMatrix(projectionMatrix);
		ofSetColor(255);
		video.draw(0, 0);
		ofPopMatrix();
	}
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key) {
	if (key == ' ') {
		showDebug = !showDebug;
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
	if (x < kinect.width && y < kinect.height) {
		ofImage temp;
		temp.allocate(kinect.width, kinect.height, OF_IMAGE_COLOR);
		temp.setFromPixels(kinectColor.getPixels());
		pd_colorToDetect = temp.getColor(x, y);
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
void ofApp::changeVideo(int _targetInd) {
	videoCurrInd = _targetInd;
	if (videoCurrInd < 0) {
		videoCurrInd = 0;
		return;
	}

	if (videoCurrInd >= videoPaths.size()) {
		videoCurrInd = 0;
	}

	// close current video then load and display the next
	video.close();
	video.load(videoPaths[videoCurrInd]);
	video.play();

	// get info about video
	videoSrcPoints[0] = ofPoint(0, 0);
	videoSrcPoints[1] = ofPoint(video.getWidth(), 0);
	videoSrcPoints[2] = ofPoint(video.getWidth(), video.getHeight());
	videoSrcPoints[3] = ofPoint(0, video.getHeight());

	// find homography
	ofPoint _bookQuad_p[4];
	_bookQuad_p[0] = bookQuad_p[0];
	_bookQuad_p[1] = bookQuad_p[1];
	_bookQuad_p[2] = bookQuad_p[2];
	_bookQuad_p[3] = bookQuad_p[3];
	findHomography(videoSrcPoints, _bookQuad_p, projectionMatrix);
}

//--------------------------------------------------------------
void ofApp::saveCalibFunc() {
	gui.saveToFile("aboutblank_calib.json");

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