#ifndef DRAWWORLD_H_
#define DRAWWORLD_H_

#include <Windows.h>

#include <osg/Node>
#include <osg/Group>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/ShapeDrawable>
#include <osg/Material>
#include <osg/Matrixd>
#include <osg/MatrixTransform>
#include <osg/Texture2D>
#include <osgDB/ReadFile> 
#include <osgViewer/Viewer>
#include <osg/PositionAttitudeTransform>
#include <osgGA/TrackballManipulator>

#include <vector>

#define RADIUS 0.1
#define THRESHOLD 1

using namespace osg;

struct NodeInfo{
	ref_ptr<MatrixTransform> node;
	Vec3 position;
	NodeInfo(ref_ptr<MatrixTransform> _node, Vec3 _position){
		node = _node;
		position = _position;
	}
};

class drawWorld{

public :
	drawWorld(){};
	static void init();
	static Node* getRoot();
	static void inputHandle(unsigned int mode, float finger_x, float finger_y, float finger_z,
		float palm_x, float palm_y, float palm_z);
	
private :
	static void draw(Vec3 start, Vec3 end);
	static void erase(Vec3 start, Vec3 end);
	static void move(Vec3);
	static Node* drawWorld::createBase(const osg::Vec3& center, float radius);
	static void addSphere(Vec3 center, float radius, Vec4 CylinderColor, Group *pAddToThisGroup);
	static void addCylinderBetweenPoints(Vec3 StartPoint, Vec3 EndPoint, float radius, Vec4 CylinderColor, Group *pAddToThisGroup);
	static NodeInfo * searchNearest(Vec3 point);
	static ref_ptr<Group> scene;
	static ref_ptr<MatrixTransform> object;
	static ref_ptr<MatrixTransform> lastOne;
	static ref_ptr<MatrixTransform> cursor_mt;
	static Vec3 lastPosition;
	static Vec3 start;	// drawing starting position
	static Vec3 end;	// drawing ending position
	static bool ready;	// false----no action, true----draw/erase
	static Vec3 cursor;
	static Vec4 currentColor;
	static std::vector<NodeInfo *> nodes;
	
};

#endif