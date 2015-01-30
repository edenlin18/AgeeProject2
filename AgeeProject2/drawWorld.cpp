#include "drawWorld.h"

using namespace osg;

ref_ptr<osgShadow::ShadowedScene> drawWorld::shadowScene = new osgShadow::ShadowedScene();
ref_ptr<Group> drawWorld::scene = new Group();
ref_ptr<MatrixTransform> drawWorld::object = NULL;
NodeInfo* drawWorld::lastOne = NULL;
ref_ptr<MatrixTransform> drawWorld::cursor_mt = NULL;
Vec3 drawWorld::cursor;
Vec3 drawWorld::offset = Vec3(0, 0, 0);
Vec3 drawWorld::start;
Vec3 drawWorld::end;
bool drawWorld::ready = false;
Vec4 drawWorld::currentColor(0.2, 0.8, 0.1, 1.0);
std::vector<NodeInfo *> drawWorld::nodes;

btCompoundShape* drawWorld::compound = new btCompoundShape();
std::vector<btRigidBody*> drawWorld::bodies;
btDiscreteDynamicsWorld* drawWorld::world;
btDispatcher* drawWorld::dispatcher;
btCollisionConfiguration* drawWorld::collisionConfig;
btBroadphaseInterface* drawWorld::broadphase;
btConstraintSolver* drawWorld::solver;
bool drawWorld::rts = false;

btTransform translate;
btTransform rotate;

// concurrency::concurrent_queue<DrawCue> drawQ;
// std::mutex mtx;           // mutex for critical section

void drawWorld::init(){
	// set up scene 
	const int ReceivesShadowTraversalMask = 0x1;
	const int CastsShadowTraversalMask = 0x2;

	// light, shadow, floor, etc
	osg::ref_ptr<osg::LightSource> ls = new osg::LightSource;
	ls->getLight()->setPosition(osg::Vec4(0, 1, 0, 0.0)); // make 4th coord 1 for point
	ls->getLight()->setAmbient(osg::Vec4(0.8, 0.8, 0.8, 1.0));
	ls->getLight()->setDiffuse(osg::Vec4(0.7, 0.7, 0.7, 1.0));
	ls->getLight()->setSpecular(osg::Vec4(1.0, 1.0, 1.0, 1.0));

	// shadow
	shadowScene = new osgShadow::ShadowedScene;
	shadowScene->setReceivesShadowTraversalMask(ReceivesShadowTraversalMask);
	shadowScene->setCastsShadowTraversalMask(CastsShadowTraversalMask);
	osg::ref_ptr<osgShadow::ViewDependentShadowMap> sm = new osgShadow::ViewDependentShadowMap;
	shadowScene->setShadowTechnique(sm.get());
	shadowScene->addChild(ls.get());
	shadowScene->addChild(scene.get());

	// floor
	Matrixf m;
	ref_ptr<MatrixTransform> mt = new MatrixTransform;
	m.makeRotate(osg::inDegrees(90.0f), 1.0f, 0.0f, 0.0f);
	mt->setMatrix(m);
	//scene->addChild(createPlane(Vec3(0,0,0), Vec4(1.0,1.0,1.0,1.0), 20));
	mt->addChild(createBase(Vec3(0.0, 0.0, -0.45), 20));
	scene->addChild(mt);

	// cursor
	cursor_mt = new MatrixTransform;
	cursor_mt->setDataVariance(osg::Object::DYNAMIC);
	m.makeTranslate(0, 10, 0);
	cursor_mt->setMatrix(m);
	scene->addChild(cursor_mt);
	addSphere(Vec3(0, 0, 0), 0.5, Vec4(1.0, 0, 0, 0.5), cursor_mt);
	// addCylinderBetweenPoints(Vec3(0, 0, 5), Vec3(0, 5, 5), 1, Vec4(0, 1, 0, 1.0), scene);
}

Node* drawWorld::getRoot(){
	return shadowScene;
}

void drawWorld::draw() {
	if (lastOne == NULL) {	// first time to draw
		Matrixf m;
		m.makeIdentity();
		object = new MatrixTransform();
		scene->addChild(object);
		object->setMatrix(m);

		Vec3 center = (end + start) / 2;
		m.makeTranslate(center);
		ref_ptr<MatrixTransform> mt = new MatrixTransform;
		
		object->addChild(mt);
		AngleAndAxis aaa = addCylinderBetweenPoints(start - center, end - center, RADIUS, currentColor, mt);	
		Matrixf mq;
		mq.setRotate(Quat(aaa.angle, aaa.axis));
		mt->setMatrix(mq * m);

		// calculating rotation for bt
		btVector3 bt_t(aaa.axis[0], aaa.axis[1], aaa.axis[2]);
		rotate.setIdentity();
		rotate.setRotation(btQuaternion(bt_t, aaa.angle));

		translate.setIdentity();
		translate.setOrigin(btVector3(center[0], center[1], center[2]));
		float height = (end - start).length();
		lastOne = new NodeInfo(mt, center, end);
		addCapsule(RADIUS, height - 2 * RADIUS, translate*rotate, lastOne);
		// TODO add cylinder to compound
		
		nodes.push_back(lastOne);
	}
	else{
		Vec3 d = start - lastOne->end;

		if (d.length() <= 0.3){	// continous drawing
			move(end);
			Matrixf m;
			Vec3 center = (end + lastOne->end) / 2;
			m.makeTranslate(center);
			ref_ptr<MatrixTransform> mt = new MatrixTransform;
			object->addChild(mt);

			AngleAndAxis aaa = addCylinderBetweenPoints(lastOne->end - center, end - center, RADIUS, currentColor, mt);
			Matrixf mq;
			mq.setRotate(Quat(aaa.angle, aaa.axis));
			mt->setMatrix(mq * m);
			std::cout << "aaa angle: " << aaa.angle << "aaa axis: (" << aaa.axis[0] << ", " << aaa.axis[1] << ", " << aaa.axis[2] << ")" << std::endl;
			// mmm.

			// calculating rotation for bt
			btVector3 bt_t(aaa.axis[0], aaa.axis[1], aaa.axis[2]);
			rotate.setIdentity();
			rotate.setRotation(btQuaternion(bt_t, aaa.angle));
			//mt->setMatrix(m);
			translate.setIdentity();
			translate.setOrigin(btVector3(center[0], center[1], center[2]));
			float height = (end - lastOne->end).length();
			lastOne = new NodeInfo(mt, center, end);
			addCapsule(RADIUS, height - 2 * RADIUS, translate * rotate, lastOne);
			// TODO add cylinder to compound
			
			nodes.push_back(lastOne);
		}
		else{	// need to search the nearest point
			NodeInfo * ni = searchNearest(start);
			Vec3 t = cursor - ni->end;
			offset = t / SENSITIVITY;
			move(ni->end);
			lastOne = ni;
			start = ni->end;
			ready = true;
		}
	}
}

void drawWorld::erase(Vec3 start, Vec3 end){

}

void drawWorld::move(Vec3 point){
	// std::cout << "cursor position: (" << point[0] << ", " << point[1] << ", " << point[2] << ")" << std::endl;
	cursor = point;
	// update position of cursor
	Matrixf m;
	m.makeTranslate(point);
	cursor_mt->setMatrix(m);
	// start = point;
}

void drawWorld::inputHandle(unsigned int mode, float finger_x, float finger_y, float finger_z,
	float palm_x, float palm_y, float palm_z){
	switch (mode){
	case 0: // cursor
	{
		ready = false;
		Vec3 t = Vec3(palm_x, palm_y, palm_z);
		t -= offset;
		t *= SENSITIVITY;
		move(t);
	}
		break;
	case 1:	// draw
	{
		Vec3 t = Vec3(palm_x, palm_y, palm_z);
		t -= offset;
		t *= SENSITIVITY;
		// start = cursor;
		move(t);
		if (ready){
			end = t;
			Vec3 d = end - start;
			float dis = d.length();
			// std::cout << "end - start: " << dis << std::endl;
			if (dis > 1.0) {
				ready = false;
				draw();
				// drawQ.push(DrawCue(start, end));

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
		std::cout << "yeah" << std::endl;
		if (rts == false) {
			drawWorld::initPhysics();
			rts = true;
		}
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
		Vec3 d = point - nodes[i]->end;
		double distance = d.length();
		if (distance <= THRESHOLD)
			return nodes[i];
		if (distance < smallest){
			smallest = distance;
			index = i;
		}
	}

	std::cout << "search check: " << index << std::endl;
	return nodes[index];
}


AngleAndAxis drawWorld::addCylinderBetweenPoints(Vec3 StartPoint, Vec3 EndPoint, float radius, Vec4 CylinderColor, Group *pAddToThisGroup)
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
	std::cout << "aaaaaaa: " << angle << std::endl;

	//	Create a cylinder between the two points with the given radius
	cylinder = new Capsule(center, radius, height);

	cylinderDrawable = new ShapeDrawable(cylinder);
	geode->addDrawable(cylinderDrawable);

	//	Set the color of the cylinder that extends between the two points.
	pMaterial = new Material;
	pMaterial->setDiffuse(Material::FRONT, CylinderColor);
	geode->getOrCreateStateSet()->setAttribute(pMaterial, StateAttribute::OVERRIDE);

	//	Add the cylinder between the two points to an existing group
	pAddToThisGroup->addChild(geode);

	return AngleAndAxis(angle, t);
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
	Geode * geode = new Geode;
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

	osg::Vec3Array* normals = new osg::Vec3Array;
	normals->push_back(osg::Vec3(0.0f, 1.0f, 0.0f));
	plane->setNormalArray(normals);

	geode->addDrawable(plane);

	return geode;
}

btCapsuleShape* drawWorld::addCapsule(btScalar radius, btScalar height, btTransform t, NodeInfo * info)
{
	btCapsuleShape* capsule = new btCapsuleShape(radius, height);
	capsule->setUserPointer(info);
	compound->addChildShape(t, capsule);

	return capsule;
}

void drawWorld::drawCompound(btRigidBody* shape){

	//std::cout << shape->getShapeType() << std::endl;
	
	//if (shape->getShapeType() == COMPOUND_SHAPE_PROXYTYPE)
	{
		btCompoundShape * compoundshape = static_cast<btCompoundShape*>(shape->getCollisionShape());

		// std::cout << "drawCompound" << std::endl;
		btTransform child_trans;
		btTransform child_trans2;
		for (int j = compoundshape->getNumChildShapes() - 1; j >= 0; j--)
		{
			const btCollisionShape* col_shape = compoundshape->getChildShape(j);
			child_trans = shape->getWorldTransform() * compoundshape->getChildTransform(j);
			child_trans2 = compoundshape->getChildTransform(j);
			// child_trans = compoundshape->getChildTransform(j);
			 btQuaternion rotation = child_trans2.getRotation();
			// btMatrix3x3 rotation = child_trans.getBasis();
			
			float angle = rotation.getAngle();
			btVector3 axis = rotation.getAxis();
			btVector3 translate = child_trans.getOrigin();

			NodeInfo * info = (NodeInfo *)col_shape->getUserPointer();
			btScalar * m = new btScalar[16];
			
			Matrixf osgM_translation;
			osgM_translation.makeTranslate(Vec3(translate.getX(), translate.getY(), translate.getZ()));

			// btVector3 x = rotation.getColumn(0);
			// btVector3 y = rotation.getColumn(1);
			// btVector3 z = rotation.getColumn(2);
			// Matrixf osgM_rotation(x.getX(), y.getX(), z.getX(), 0, x.getY(), y.getY(), z.getY(), 0, x.getZ(), y.getZ(), z.getZ(), 0, 0, 0, 0, 1);
			// Matrixf osgM_rotation(x.getX(), x.getY(), x.getZ(), 0, y.getX(), y.getY(), y.getZ(), 0, z.getX(), z.getY(), z.getZ(), 0, 0, 0, 0, 1);
			// std::cout << "ROTATION MATRIX: " << x.getX() << ", " << x.getY() << ", " << x.getZ() << ", " << y.getX() << ", " << y.getY() << ", " << y.getZ() << ", " << z.getX() << ", " << z.getY() << ", " << z.getZ() << std::endl;
			Matrixf osgM_rotation;
			osgM_rotation.makeRotate(Quat(angle, Vec3(axis.getX(), axis.getY(), axis.getZ())));
			//std::cout << "angle: " << angle << std::endl;
			
			// std::cout << "pass1" << std::endl;
			// system("PAUSE");
			// child_trans.getOpenGLMatrix(m);
			// std::cout << "pass2" << std::endl;
			// system("PAUSE");
			// Matrixf osgM(m);
			//std::cout << "pass3" << std::endl;
			//system("PAUSE");
			info->mt->setMatrix(osgM_rotation * osgM_translation);
			std::cout << j << std::endl;
			//info->mt->setMatrix
		}
	}
}

void drawWorld::initPhysics(){
	// Setting up physics world
	// Build the broadphase
	broadphase = new btDbvtBroadphase();

	// Set up the collision configuration and dispatcher
	collisionConfig = new btSoftBodyRigidBodyCollisionConfiguration();
	dispatcher = new btCollisionDispatcher(collisionConfig);

	// The actual physics solver

	solver = new btSequentialImpulseConstraintSolver;
	

	// The world
	world = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfig);
	world->setGravity(btVector3(0, -10, 0));

	// Setting up ground
	btTransform t;
	t.setIdentity();
	t.setOrigin(btVector3(0, 0, 0));
	btCollisionShape* plane = new btStaticPlaneShape(btVector3(0, 1, 0), 1);
	btDefaultMotionState* motion = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, -1, 0)));
	btRigidBody::btRigidBodyConstructionInfo info(0.0, motion, plane, btVector3(0, 0, 0));
	//info.m_linearDamping = 0.2f;
	info.m_restitution = 1.0f;
	info.m_friction = 0.5f;
	btRigidBody* body = new btRigidBody(info);
	world->addRigidBody(body);
	bodies.push_back(body);
	body->setUserPointer(bodies[bodies.size() - 1]);


	//add compound into our world
	//btCollisionShape * compoundShape = new 
	//bodies.push_back(compound);
	btMotionState* motion2 = new btDefaultMotionState(t);
	btRigidBody::btRigidBodyConstructionInfo info2(10, motion2, compound, btVector3(0, 0, 0));
	info2.m_restitution = 0.1f;
	info2.m_friction = 1.5f;
	btRigidBody* b = new btRigidBody(info2);
	world->addRigidBody(b);
	bodies.push_back(b);
	body->setUserPointer(bodies[bodies.size() - 1]);


}

void drawWorld::simulate(float dt){
	world->stepSimulation(dt);

	for (int i = 0; i < bodies.size(); i++){
		if (bodies[i]->getCollisionShape()->getShapeType() == COMPOUND_SHAPE_PROXYTYPE){
			drawCompound(bodies[i]);
		}
		//std::cout << bodies[i]->getShapeType() << std::endl;
		//std::cout << i << std::endl;
	}
}

bool drawWorld::readyToSimulate(){
	return rts;
}