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
	Vec3d up(0.0, 1.0, 0.0);
	// viewer.getCamera()->setViewMatrixAsLookAt(eye, center, up);
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

	osg::ref_ptr<osgGA::TrackballManipulator> trackBall = new osgGA::TrackballManipulator;
	viewer.setCameraManipulator(trackBall);
	viewer.getCameraManipulator()->setHomePosition(eye, center, up, false);
	viewer.home();
	// viewer.run();
	viewer.realize();
	osg::Timer_t frame_tick = osg::Timer::instance()->tick();
	while (!viewer.done()) {
		// drawWorld::draw();
		if (drawWorld::readyToSimulate()){
			osg::Timer_t now_tick = osg::Timer::instance()->tick();
			float dt = osg::Timer::instance()->delta_s(frame_tick, now_tick);
			frame_tick = now_tick;
			drawWorld::simulate(dt);
			// std::cout << "simulating" << std::endl;
		}
		
		EventHandler current_event;
		while (Utility::events.try_pop(current_event)) {
			drawWorld::inputHandle(current_event.eventType, current_event.finger_x, 
				current_event.finger_y, current_event.finger_z, current_event.palm_x, 
				current_event.palm_y, current_event.palm_z);
			
			/*switch (current_event.eventType) {
				case 1:
					drawWorld::draw();
					break;
				default:
					break;
			}*/
		}

		viewer.frame();
	}

	// Keep this process running until Enter is pressed
	/*std::cout << "Press Enter to quit..." << std::endl;
	std::cin.get();*/

	// Remove the sample listener when done
	controller.removeListener(listener);

	return 0;
}