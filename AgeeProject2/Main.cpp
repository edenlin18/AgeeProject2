#include "Main.h"

int main(int argc, char *argv []) {
	LeapCallbackListener listener;
	Controller controller;
	Utility::registerInputModeCallBack(drawWorld::inputHandle);

	controller.addListener(listener);
	
	osgViewer::Viewer viewer;
	viewer.setUpViewInWindow(50, 50, 800, 800);
	Vec3d eye(0.0, 50.0, 50.0);
	Vec3d center(0.0, 0.0, 0.0);
	Vec3d up(0.0, 0.0, 1.0);
	viewer.getCamera()->setViewMatrixAsLookAt(eye, center, up);
	drawWorld::init();
	//osg::MatrixTransform* rootnode = new osg::MatrixTransform;
	osg::Node* rootnode = drawWorld::getRoot();
	//rootnode->setMatrix(osg::Matrix::rotate(osg::inDegrees(90.0f), 1.0f, 0.0f, 0.0f));
	//rootnode->addChild(drawWorld::getRoot());

	// run optimization over the scene graph
	osgUtil::Optimizer optimzer;
	optimzer.optimize(rootnode);

	// set the scene to render
	viewer.setSceneData(rootnode);

	//viewer.setCameraManipulator(new osgGA::TrackballManipulator());
	viewer.run();

	// Keep this process running until Enter is pressed
	/*std::cout << "Press Enter to quit..." << std::endl;
	std::cin.get();*/

	// Remove the sample listener when done
	controller.removeListener(listener);

	return 0;
}