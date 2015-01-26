#include "drawWorld.h"
#include <limits>
using namespace osg;

ref_ptr<Group> drawWorld::scene = new Group();
ref_ptr<MatrixTransform> drawWorld::object = NULL;
ref_ptr<MatrixTransform> drawWorld::lastOne = NULL;
ref_ptr<MatrixTransform> drawWorld::cursor_mt = NULL;
Vec3 drawWorld::lastPosition;
Vec3 drawWorld::cursor;
Vec3 drawWorld::start;
Vec3 drawWorld::end;
bool drawWorld::ready = false;
Vec4 drawWorld::currentColor(0.8, 0.1, 0.1, 1.0);
std::vector<NodeInfo *> drawWorld::nodes;

void drawWorld::init(){
	// set up scene 
	// light, shadow, floor, etc
	scene->addChild(createBase(Vec3(0, 0, 0), 20.0));
	cursor_mt = new MatrixTransform;
	Matrixf m;
	m.makeTranslate(0, 10, 0);
	cursor_mt->setMatrix(m);
	scene->addChild(cursor_mt);
	addSphere(Vec3(0, 0, 0), 0.5, Vec4(1.0, 0, 0, 0.5), cursor_mt);
	// addCylinderBetweenPoints(Vec3(0, 0, 5), Vec3(0, 5, 5), 1, Vec4(0, 1, 0, 1.0), scene);
}

Node* drawWorld::getRoot(){
	return scene;
}

void drawWorld::draw(Vec3 start, Vec3 end){
	if (lastOne == NULL){	// first time to draw
		Matrixf m;
		m.makeTranslate(start);
		object = new MatrixTransform();
		scene->addChild(object);
		object->setMatrix(m);
		addCylinderBetweenPoints(start, end, RADIUS, currentColor, object);
		ref_ptr<MatrixTransform> mt = new MatrixTransform();
		m.makeTranslate(end - start);
		mt->setMatrix(m);
		object->addChild(mt);
		lastOne = mt;
		lastPosition = end;
		nodes.push_back(new NodeInfo(mt, end));
	}
	else{
		Vec3 d = start - lastPosition;
		if (d.length() <= THRESHOLD){	// contineous drawing
			addCylinderBetweenPoints(lastPosition, end, RADIUS, currentColor, lastOne);
			Matrixf m;
			ref_ptr<MatrixTransform> mt = new MatrixTransform();
			m.makeTranslate(end - lastPosition);
			mt->setMatrix(m);
			lastOne->addChild(mt);
			lastOne = mt;
			lastPosition = end;
			nodes.push_back(new NodeInfo(mt, end));
		}
		else{	// need to search the nearest point
			NodeInfo * ni = searchNearest(start);
			move(ni->position);
			lastOne = ni->node;
		}
	}
}

void drawWorld::erase(Vec3 start, Vec3 end){

}

void drawWorld::move(Vec3 point){
	cursor = point;
	// update position of cursor
	Matrixf m;
	m.makeTranslate(point);
	cursor_mt->setMatrix(m);
}

void drawWorld::inputHandle(unsigned int mode, float finger_x, float finger_y, float finger_z,
	float palm_x, float palm_y, float palm_z){
	switch (mode){
	case 0:
		move(Vec3(finger_x / 40, finger_y / 30, finger_z / 40));
		break;
	case 1:	//draw
		move(Vec3(finger_x / 40, finger_y / 30, finger_z / 40));
		if (ready){
			end = Vec3(finger_x / 40, finger_y / 30, finger_z / 40);
			Vec3 t = end - start;
			if (t.length() > 1) {
				draw(start, end);
				ready = false;
			}
		}else{
			start = Vec3(finger_x / 40, finger_y / 30, finger_z / 40);
			ready = true;
		}
		break;
	case 2:	// erase

		break;

	case 3:	// trackball

		break;

	case 4:	// invalid
		break;

	}
}

NodeInfo * drawWorld::searchNearest(Vec3 point){
	double smallest = 10000;
	int index = 0;
	for (int i = 0; i < nodes.size(); i++){
		Vec3 d = point - nodes[i]->position;
		double distance = d.length();
		if (distance <= THRESHOLD)
			return nodes[i];
		if (distance < smallest){
			smallest = distance;
			index = i;
		}
	}
	return nodes[index];
}

void drawWorld::addCylinderBetweenPoints(Vec3 StartPoint,Vec3 EndPoint, float radius, Vec4 CylinderColor,Group *pAddToThisGroup)
{
	ref_ptr<Geode> geode = new osg::Geode;
	osg::Vec3	center;
	float	height;
	ref_ptr<Cylinder> cylinder;
	ref_ptr<ShapeDrawable> cylinderDrawable;
	ref_ptr<Material> pMaterial;

	height = (StartPoint - EndPoint).length();
	center = Vec3((StartPoint.x() + EndPoint.x()) / 2, (StartPoint.y() + EndPoint.y()) / 2, (StartPoint.z() + EndPoint.z()) / 2);

	// This is the default direction for the cylinders to face in OpenGL
	Vec3 z = Vec3(0, 0, 1);

	// Get diff between two points you want cylinder along
	Vec3 p = (StartPoint - EndPoint);

	// Get CROSS product (the axis of rotation)
	Vec3 t = z ^ p;

	// Get angle. length is magnitude of the vector
	double angle = acos((z * p) / p.length());

	//	Create a cylinder between the two points with the given radius
	cylinder = new Cylinder(center, radius, height);
	cylinder->setRotation(Quat(angle, Vec3(t.x(), t.y(), t.z())));

	cylinderDrawable = new ShapeDrawable(cylinder);
	geode->addDrawable(cylinderDrawable);

	//	Set the color of the cylinder that extends between the two points.
	pMaterial = new Material;
	pMaterial->setDiffuse(Material::FRONT, CylinderColor);
	geode->getOrCreateStateSet()->setAttribute(pMaterial, StateAttribute::OVERRIDE);

	//	Add the cylinder between the two points to an existing group
	pAddToThisGroup->addChild(geode);
}

void drawWorld::addSphere(Vec3 center, float radius, Vec4 CylinderColor, Group *pAddToThisGroup)
{
	ref_ptr<Geode> geode = new osg::Geode;
	ref_ptr<Sphere> sphere;
	ref_ptr<ShapeDrawable> sphereDrawable;
	ref_ptr<Material> pMaterial;

	//	Create a cylinder between the two points with the given radius
	sphere = new Sphere(center, radius);

	sphereDrawable = new ShapeDrawable(sphere);
	geode->addDrawable(sphereDrawable);

	//	Set the color of the cylinder that extends between the two points.
	pMaterial = new Material;
	pMaterial->setDiffuse(Material::FRONT, CylinderColor);
	geode->getOrCreateStateSet()->setAttribute(pMaterial, StateAttribute::OVERRIDE);

	//	Add the cylinder between the two points to an existing group
	pAddToThisGroup->addChild(geode);
}

Node* drawWorld::createBase(const osg::Vec3& center, float radius)
{

	int numTilesX = 10;
	int numTilesY = 10;

	float width = 2 * radius;
	float height = 2 * radius;

	osg::Vec3 v000(center - osg::Vec3(width*0.5f, height*0.5f, 0.0f));
	osg::Vec3 dx(osg::Vec3(width / ((float)numTilesX), 0.0, 0.0f));
	osg::Vec3 dy(osg::Vec3(0.0f, height / ((float)numTilesY), 0.0f));

	// fill in vertices for grid, note numTilesX+1 * numTilesY+1...
	osg::Vec3Array* coords = new osg::Vec3Array;
	int iy;
	for (iy = 0; iy <= numTilesY; ++iy)
	{
		for (int ix = 0; ix <= numTilesX; ++ix)
		{
			coords->push_back(v000 + dx*(float)ix + dy*(float)iy);
		}
	}

	//Just two colours - black and white.
	osg::Vec4Array* colors = new osg::Vec4Array;
	colors->push_back(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f)); // white
	colors->push_back(osg::Vec4(0.0f, 0.0f, 0.0f, 1.0f)); // black

	osg::ref_ptr<osg::DrawElementsUShort> whitePrimitives = new osg::DrawElementsUShort(GL_QUADS);
	osg::ref_ptr<osg::DrawElementsUShort> blackPrimitives = new osg::DrawElementsUShort(GL_QUADS);

	int numIndicesPerRow = numTilesX + 1;
	for (iy = 0; iy<numTilesY; ++iy)
	{
		for (int ix = 0; ix<numTilesX; ++ix)
		{
			osg::DrawElementsUShort* primitives = ((iy + ix) % 2 == 0) ? whitePrimitives.get() : blackPrimitives.get();
			primitives->push_back(ix + (iy + 1)*numIndicesPerRow);
			primitives->push_back(ix + iy*numIndicesPerRow);
			primitives->push_back((ix + 1) + iy*numIndicesPerRow);
			primitives->push_back((ix + 1) + (iy + 1)*numIndicesPerRow);
		}
	}

	// set up a single normal
	osg::Vec3Array* normals = new osg::Vec3Array;
	normals->push_back(osg::Vec3(0.0f, 0.0f, 1.0f));

	osg::Geometry* geom = new osg::Geometry;
	geom->setVertexArray(coords);

	geom->setColorArray(colors, osg::Array::BIND_PER_PRIMITIVE_SET);

	geom->setNormalArray(normals, osg::Array::BIND_OVERALL);

	geom->addPrimitiveSet(whitePrimitives.get());
	geom->addPrimitiveSet(blackPrimitives.get());

	osg::Geode* geode = new osg::Geode;
	geode->addDrawable(geom);

	return geode;
}

