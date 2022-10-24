#include "ofMain.h"
#include "ofApp.h"
#include "ofAppGLFWWindow.h"

//========================================================================
int main() {
	ofGLFWWindowSettings settings;

	//
	float mainWindowW = 1920;
	float mainWindowH = 1080;
	float guiWindowW = 1920;
	float guiWindowH = 1080;

	// main window
	settings.setGLVersion(4, 5);
	settings.setPosition(ofVec2f(0, 0));
	settings.setSize(mainWindowW, mainWindowH);
	settings.decorated = false;
	settings.resizable = false;
	shared_ptr<ofAppBaseWindow> mainWindow = ofCreateWindow(settings);

	settings.setSize(guiWindowW, guiWindowH);
	settings.setPosition(ofVec2f(1920, -853));
	settings.decorated = true;
	settings.resizable = true;
	settings.shareContextWith = mainWindow;
	shared_ptr<ofAppBaseWindow> guiWindow = ofCreateWindow(settings);
	guiWindow->setVerticalSync(false);

	shared_ptr<ofApp> mainApp(new ofApp);
	mainApp->mainWindowW = mainWindowW;
	mainApp->mainWindowH = mainWindowH;
	mainApp->guiWindowW = guiWindowW;
	mainApp->guiWindowH = guiWindowH;
	mainApp->setupGui();
	ofAddListener(guiWindow->events().draw, mainApp.get(), &ofApp::drawGui);

	ofRunApp(mainWindow, mainApp);
	ofRunMainLoop();
}
