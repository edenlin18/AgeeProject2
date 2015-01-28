#ifndef DRAWWORLD_H_
#define DRAWWORLD_H_

#include <Windows.h>

#include <limits>
#include <ppl.h>
#include <concurrent_queue.h>

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
#include <osg/TexMat>
#include <osg/BoundingBox>
#include <osg/TextureRectangle>
#include <osg/PositionAttitudeTransform>
#include <osgGA/TrackballManipulator>
#include <osgShadow/ShadowedScene>
#include <osgShadow\ViewDependentShadowMap>

#include <btBulletDynamicsCommon.h>
#include <LinearMath/btTransform.h>
#include <BulletCollision\CollisionShapes\btCylinderShape.h>
#include <BulletCollision\CollisionShapes\btCompoundShape.h>
#include <BulletSoftBody/btSoftRigidDynamicsWorld.h>
#include <BulletSoftBody/btDefaultSoftBodySolver.h>
#include <BulletSoftBody/btSoftBodyHelpers.h>
#include <BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h>

#include <vector>

#include <thread>         // std::thread
#include <mutex>          // std::mutex

#define RADIUS 0.1
#define THRESHOLD 0.1
#define SENSITIVITY 0.1

using namespace osg;

struct NodeInfo{
	ref_ptr<MatrixTransform> node;
	Vec3 position;
	btTransform bt;
	NodeInfo(ref_ptr<MatrixTransform> _node, Vec3 _position, btTransform _bt){
		node = _node;
		position = _position;
		bt = _bt;
	}
};

/* struct DrawCue {
	Vec3 start;
	Vec3 end;
	DrawCue() { }
	DrawCue(Vec3 _start, Vec3 _end) {
		start = _start;
		end = _end;
	}
};*/

class drawWorld{

public :
	drawWorld(){};
	static void init();
	static Node* getRoot();
	static void inputHandle(unsigned int mode, float finger_x, float finger_y, float finger_z,
		float palm_x, float palm_y, float palm_z);
	static void draw(Vec3 _start, Vec3 _end);

private :
	// static void draw(Vec3 _start, Vec3 _end);
	static void erase(Vec3 _start, Vec3 _end);
	static void move(Vec3);
	static void startFalling();
	static Node* createBase(const osg::Vec3& center, float radius);
	static Node* createPlane(const Vec3& center, const Vec4& color, float radius);
	static void addSphere(Vec3 center, float radius, Vec4 CylinderColor, Group *pAddToThisGroup);
	static btTransform addCylinderBetweenPoints(Vec3 StartPoint, Vec3 EndPoint, float radius, Vec4 CylinderColor, Group *pAddToThisGroup);
	static NodeInfo * searchNearest(Vec3 point);
	
	//bullet functions
	static btCylinderShape * addCylinder(float d, float h, btTransform& t);
	

	static ref_ptr<osgShadow::ShadowedScene> shadowScene;
	static ref_ptr<Group> scene;
	static ref_ptr<MatrixTransform> object;
	static NodeInfo* lastOne;
	static ref_ptr<MatrixTransform> cursor_mt;
	static Vec3 start;	// drawing starting position
	static Vec3 end;	// drawing ending position
	static bool ready;	// false----no action, true----draw/erase
	static Vec3 cursor;
	static Vec4 currentColor;
	static std::vector<NodeInfo *> nodes;
	static bool setup;
	static Vec3 offset;

	//bullet variables
	static btCompoundShape* compound;
	static std::vector<btRigidBody*> bodies;
};

#endif