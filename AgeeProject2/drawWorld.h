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
#include <osg/TexMat>
#include <osg/BoundingBox>
#include <osg/TextureRectangle>
#include <osg/PositionAttitudeTransform>
#include <osgGA/TrackballManipulator>
#include <osgShadow/ShadowedScene>
#include <osgShadow/ShadowMap>

#include <vector>

#define RADIUS 0.1
#define THRESHOLD 0.1
#define SENSITIVITY 0.05

using namespace osg;

struct NodeInfo{
	ref_ptr<MatrixTransform> node;
	Vec3 position;
	NodeInfo(ref_ptr<MatrixTransform> _node, Vec3 _position){
		node = _node;
		position = _position;
	}
};

class TexturePanCallback : public osg::NodeCallback
{
public:
	TexturePanCallback(osg::TexMat* texmat,
		double delay = 0.05) :
		_texmat(texmat),
		_phaseS(35.0f),
		_phaseT(18.0f),
		_phaseScale(5.0f),
		_delay(delay),
		_prevTime(0.0)
	{
	}

	virtual void operator()(osg::Node*, osg::NodeVisitor* nv)
	{
		if (!_texmat)
			return;

		if (nv->getFrameStamp()) {
			double currTime = nv->getFrameStamp()->getSimulationTime();
			if (currTime - _prevTime > _delay) {

				float rad = osg::DegreesToRadians(currTime);

				// zoom scale (0.2 - 1.0)
				float scale = sin(rad * _phaseScale) * 0.4f + 0.6f;
				float scaleR = 1.0f - scale;

				// calculate new texture coordinates
				float s, t;
				s = ((sin(rad * _phaseS) + 1) * 0.5f) * (scaleR);
				t = ((sin(rad * _phaseT) + 1) * 0.5f) * (scaleR);


				_texmat->setMatrix(osg::Matrix::translate(s, t, 1.0)*osg::Matrix::scale(scale, scale, 1.0));

				// record time
				_prevTime = currTime;
			}
		}
	}

private:
	osg::TexMat* _texmat;

	float _phaseS, _phaseT, _phaseScale;

	double _delay;
	double _prevTime;
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
	static Node* createBase(const osg::Vec3& center, float radius);
	static Node* createBase2(const osg::Vec3& center, float radius);
	static Node* createPlane(const Vec3& center, const Vec4& color, float radius);
	static Node* createRectangle(BoundingBox& bb,
		const std::string& filename);
	static void addSphere(Vec3 center, float radius, Vec4 CylinderColor, Group *pAddToThisGroup);
	static void addCylinderBetweenPoints(Vec3 StartPoint, Vec3 EndPoint, float radius, Vec4 CylinderColor, Group *pAddToThisGroup);
	static NodeInfo * searchNearest(Vec3 point);
	static ref_ptr<osgShadow::ShadowedScene> shadowScene;
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
	static bool setup;
	static Vec3 offset;
	
};

#endif