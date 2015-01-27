#include "drawWorld.h"
#include <limits>
using namespace osg;

ref_ptr<osgShadow::ShadowedScene> drawWorld::shadowScene = new osgShadow::ShadowedScene();
ref_ptr<Group> drawWorld::scene = new Group();
ref_ptr<MatrixTransform> drawWorld::object = NULL;
ref_ptr<MatrixTransform> drawWorld::lastOne = NULL;
ref_ptr<MatrixTransform> drawWorld::cursor_mt = NULL;
Vec3 drawWorld::lastPosition;
Vec3 drawWorld::cursor;
Vec3 drawWorld::offset = Vec3(0, 0, 0);
Vec3 drawWorld::start;
Vec3 drawWorld::end;
bool drawWorld::ready = false;
Vec4 drawWorld::currentColor(0.2, 0.8, 0.1, 1.0);
std::vector<NodeInfo *> drawWorld::nodes;

void drawWorld::init(){
	// set up scene 
	
	// light, shadow, floor, etc
	osg::ref_ptr<osg::LightSource> ls = new osg::LightSource;
	ls->getLight()->setPosition(osg::Vec4(0, 50, 0, 1.0)); // make 4th coord 1 for point
	ls->getLight()->setAmbient(osg::Vec4(0.8, 0.8, 0.8, 1.0));
	ls->getLight()->setDiffuse(osg::Vec4(0.7, 0.7, 0.7, 1.0));
	ls->getLight()->setSpecular(osg::Vec4(1.0, 1.0, 1.0, 1.0));

	// shadow
	osg::ref_ptr<osgShadow::ShadowMap> sm = new osgShadow::ShadowMap;
	shadowScene->setShadowTechnique(sm.get());
	shadowScene->addChild(ls.get());	shadowScene->addChild(scene);

	
	// floor
	Matrixf m;
	ref_ptr<MatrixTransform> mt = new MatrixTransform;
	m.makeRotate(osg::inDegrees(90.0f), 1.0f, 0.0f, 0.0f);
	mt->setMatrix(m);
	//mt->addChild(createPlane(Vec3(0,0,0), Vec4(0.8,0.8,0.8,1.0), 20));
	mt->addChild(createBase(Vec3(0, 0, 0), 20));
	scene->addChild(mt);

	// cursor
	
	cursor_mt = new MatrixTransform;
	m.makeTranslate(0, 10, 0);
	cursor_mt->setMatrix(m);
	scene->addChild(cursor_mt);
	addSphere(Vec3(0, 0, 0), 0.5, Vec4(1.0, 0, 0, 0.5), cursor_mt);
	// addCylinderBetweenPoints(Vec3(0, 0, 5), Vec3(0, 5, 5), 1, Vec4(0, 1, 0, 1.0), scene);
}

Node* drawWorld::getRoot(){
	return shadowScene;
}

void drawWorld::draw(Vec3 _start, Vec3 _end){
	if (lastOne == NULL){	// first time to draw
		Matrixf m;
		m.makeTranslate(_start);
		object = new MatrixTransform();
		object->setDataVariance(osg::Object::DYNAMIC);
		scene->addChild(object);
		object->setMatrix(m);
		addCylinderBetweenPoints(Vec3(0,0,0), _end - _start, RADIUS, currentColor, object);
		ref_ptr<MatrixTransform> mt = new MatrixTransform();
		m.makeTranslate(_end - _start);
		mt->setMatrix(m);
		object->addChild(mt);
		lastOne = mt;
		lastPosition = _end;
		nodes.push_back(new NodeInfo(mt, _end));
	}
	else{
		Vec3 d = start - lastPosition;
		if (d.length() <= THRESHOLD){	// continous drawing
			move(_end);
			addCylinderBetweenPoints(Vec3(0,0,0), _end - lastPosition, RADIUS, currentColor, lastOne);
			Matrixf m;
			ref_ptr<MatrixTransform> mt = new MatrixTransform();
			mt->setDataVariance(osg::Object::DYNAMIC);
			m.makeTranslate(_end - lastPosition);
			mt->setMatrix(m);
			lastOne->addChild(mt);
			lastOne = mt;
			lastPosition = _end;
			nodes.push_back(new NodeInfo(mt, _end));
		}
		else{	// need to search the nearest point
			NodeInfo * ni = searchNearest(_start);
			Vec3 t = cursor - ni->position;
			offset = t / SENSITIVITY;
			move(ni->position);
			lastOne = ni->node;
			lastPosition = ni->position;
			ready = false;
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
	{
		Vec3 t = Vec3(finger_x, finger_y, finger_z);
		t -= offset;
		t *= SENSITIVITY;
		move(t);
	}
		break;
	case 1:	//draw
	{
		Vec3 t = Vec3(finger_x, finger_y, finger_z);
		t -= offset;
		t *= SENSITIVITY;
		move(t);
		if (ready){
			end = t;
			Vec3 d = end - start;
			double dis = d.length();
			if (dis > 1) {
				draw(start, end);
				ready = false;
			}
		}
		else{
			start = t;
			ready = true;
		}
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
	int nodes_size = nodes.size();
	std::cout << "node size: " << nodes_size << std::endl;
	for (int i = 0; i < nodes_size; i++) {
		Vec3 d = point - nodes[i]->position;
		double distance = d.length();
		if (distance <= THRESHOLD)
			return nodes[i];
		if (distance < smallest){
			smallest = distance;
			index = i;
		}
	}
	std::cerr << index << std::endl;
	return nodes[index];
}

void drawWorld::addCylinderBetweenPoints(Vec3 StartPoint,Vec3 EndPoint, float radius, Vec4 CylinderColor,Group *pAddToThisGroup)
{
	ref_ptr<Geode> geode = new osg::Geode;
	osg::Vec3	center;
	float	height;
	ref_ptr<Capsule> cylinder;
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
	cylinder = new Capsule(center, radius, height);
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
	colors->push_back(osg::Vec4(0.5f, 0.5f, 0.5f, 1.0f)); // black

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

Node* drawWorld::createPlane(const Vec3& center, const Vec4& color, float radius){
	ref_ptr<Geode> geode = new Geode;
	ref_ptr<Geometry> plane = new Geometry;
	ref_ptr<Vec3Array> vertice = new Vec3Array;
	vertice->push_back(center - Vec3(radius, 0, radius));
	vertice->push_back(center - Vec3(radius, 0, -radius));
	vertice->push_back(center - Vec3(-radius, 0, -radius));
	vertice->push_back(center - Vec3(-radius, 0, radius));
	plane->setVertexArray(vertice);

	ref_ptr<DrawElementsUInt> de = new DrawElementsUInt(PrimitiveSet::QUADS, 0);
	de->push_back(3);
	de->push_back(2);
	de->push_back(1);
	de->push_back(0);
	plane->addPrimitiveSet(de);

	ref_ptr<Vec4Array> colors = new Vec4Array;
	colors->push_back(color);
	colors->push_back(color);
	colors->push_back(color);
	colors->push_back(color);
	plane->setColorArray(colors);
	plane->setColorBinding(Geometry::BIND_PER_VERTEX);

	geode->addDrawable(plane);
	return geode;

}

Node* drawWorld::createBase2(const osg::Vec3& center, float radius)
{

	int numTilesX = 1;
	int numTilesY = 1;

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
	colors->push_back(osg::Vec4(0.5f, 0.5f, 0.5f, 1.0f)); // white
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
	//geom->addPrimitiveSet(blackPrimitives.get());

	osg::Geode* geode = new osg::Geode;
	geode->addDrawable(geom);

	return geode;
}
Node* drawWorld::createRectangle(osg::BoundingBox& bb,
	const std::string& filename)
{
	osg::Vec3 top_left(bb.xMin(), bb.yMax(), bb.zMax());
	osg::Vec3 bottom_left(bb.xMin(), bb.yMax(), bb.zMin());
	osg::Vec3 bottom_right(bb.xMax(), bb.yMax(), bb.zMin());
	osg::Vec3 top_right(bb.xMax(), bb.yMax(), bb.zMax());

	// create geometry
	osg::Geometry* geom = new osg::Geometry;

	osg::Vec3Array* vertices = new osg::Vec3Array(4);
	(*vertices)[0] = top_left;
	(*vertices)[1] = bottom_left;
	(*vertices)[2] = bottom_right;
	(*vertices)[3] = top_right;
	geom->setVertexArray(vertices);

	osg::Vec2Array* texcoords = new osg::Vec2Array(4);
	(*texcoords)[0].set(0.0f, 0.0f);
	(*texcoords)[1].set(1.0f, 0.0f);
	(*texcoords)[2].set(1.0f, 1.0f);
	(*texcoords)[3].set(0.0f, 1.0f);
	geom->setTexCoordArray(0, texcoords);

	osg::Vec3Array* normals = new osg::Vec3Array(1);
	(*normals)[0].set(0.0f, -1.0f, 0.0f);
	geom->setNormalArray(normals, osg::Array::BIND_OVERALL);

	osg::Vec4Array* colors = new osg::Vec4Array(1);
	(*colors)[0].set(1.0f, 1.0f, 1.0f, 1.0f);
	geom->setColorArray(colors, osg::Array::BIND_OVERALL);

	geom->addPrimitiveSet(new osg::DrawArrays(GL_QUADS, 0, 4));

	// disable display list so our modified tex coordinates show up
	geom->setUseDisplayList(false);

	// load image
	osg::Image* img = osgDB::readImageFile(filename);

	// setup texture
	osg::TextureRectangle* texture = new osg::TextureRectangle(img);

	osg::TexMat* texmat = new osg::TexMat;
	texmat->setScaleByTextureRectangleSize(true);

	// setup state
	osg::StateSet* state = geom->getOrCreateStateSet();
	state->setTextureAttributeAndModes(0, texture, osg::StateAttribute::ON);
	state->setTextureAttributeAndModes(0, texmat, osg::StateAttribute::ON);

	// turn off lighting
	state->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

	// install 'update' callback
	osg::Geode* geode = new osg::Geode;
	geode->addDrawable(geom);
	geode->setUpdateCallback(new TexturePanCallback(texmat));

	return geode;
}
