#include "Main.h"

int main(int argc, char *argv []) {
	LeapCallbackListener listener;
	Controller controller;
	Utility::registerInputModeCallBack(drawWorld::inputHandle);

	controller.addListener(listener);
	
	osgViewer::Viewer viewer;
	drawWorld::init();
	osg::MatrixTransform* rootnode = new osg::MatrixTransform;
	rootnode->setMatrix(osg::Matrix::rotate(osg::inDegrees(30.0f), 1.0f, 0.0f, 0.0f));
	rootnode->addChild(drawWorld::getRoot());

	// run optimization over the scene graph
	osgUtil::Optimizer optimzer;
	optimzer.optimize(rootnode);

	// set the scene to render
	viewer.setSceneData(rootnode);

	viewer.setCameraManipulator(new osgGA::TrackballManipulator());
	viewer.run();

	// Remove the sample listener when done
	controller.removeListener(listener);

	return 0;
}