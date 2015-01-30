#include "drawWorld.h"

using namespace osg;

ref_ptr<osgShadow::ShadowedScene> drawWorld::shadowScene = new osgShadow::ShadowedScene();
ref_ptr<Group> drawWorld::scene = new Group();
ref_ptr<MatrixTransform> drawWorld::object = NULL;
NodeInfo* drawWorld::lastOne = NULL;
ref_ptr<MatrixTransform> drawWorld::cursor_mt = NULL;
Vec3 drawWorld::cursor;
Vec3 drawWorld::cursor_color(0.1, 0.1, 0.8);
Vec3 drawWorld::offset = Vec3(0, 0, 0);
Vec3 drawWorld::start;
Vec3 drawWorld::initStart;
Vec3 drawWorld::end;
bool drawWorld::ready = false;
Vec4 drawWorld::currentColor(0.2, 0.8, 0.1, 1.0);
std::vector<NodeInfo *> drawWorld::nodes;
Geode * drawWorld::cursor_geode = NULL;
Material * drawWorld::pMaterial = new Material();

btCompoundShape* drawWorld::compound = new btCompoundShape();
std::vector<btRigidBody*> drawWorld::bodies;

btCollisionConfiguration* drawWorld::collisionConfig = new btSoftBodyRigidBodyCollisionConfiguration();
btDispatcher* drawWorld::dispatcher = new btCollisionDispatcher(collisionConfig);
btBroadphaseInterface* drawWorld::broadphase= new btDbvtBroadphase();
btConstraintSolver* drawWorld::solver = new btSequentialImpulseConstraintSolver;
btDiscreteDynamicsWorld* drawWorld::world = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfig);

bool drawWorld::rts = false;




// btRigidBody* drawWorld::compound_body;


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
	mt->addChild(createBase(Vec3(0.0, 0.0, 0), 20));
	scene->addChild(mt);

	// cursor
	cursor_mt = new MatrixTransform;
	cursor_mt->setDataVariance(osg::Object::DYNAMIC);
	m.makeTranslate(0, 10, 0);
	cursor_mt->setMatrix(m);
	scene->addChild(cursor_mt);
	cursor_geode = addSphere(Vec3(0, 0, 0), 0.5, Vec4(0.1, 0.1, 0.8, 1.0), cursor_mt);
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
		//initStart = Vec3(0, 10, 0);
		initStart = start;
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
		double height = (end - start).length();
		lastOne = new NodeInfo(mt, center, end, NULL);
		//btRigidBody * bodyNow = addCapsule(RADIUS, (height - 2 * RADIUS) * 0.99,height * MASS_PER_HEIGHT, translate*rotate, lastOne);
		//lastOne->body = bodyNow;
		addCapsule(RADIUS, (height - 2 * RADIUS) * 0.99, translate*rotate, lastOne);
		/*
		btVector3 localPivot = btVector3(end[0], end[1], end[2]);
		btPoint2PointConstraint * p2p = new btPoint2PointConstraint(*bodyNow, localPivot);
		world->addConstraint(p2p);
		*/
		
		
		

		/*
		btVector3 localPivot = btVector3(end[0], end[1], end[2]);
		btTransform tr;
		tr.setIdentity();
		tr.setOrigin(localPivot);
		btGeneric6DofConstraint* dof6 = new btGeneric6DofConstraint(*bodyNow, tr, true);
		dof6->setLinearLowerLimit(btVector3(1, 1, 1));
		dof6->setLinearUpperLimit(btVector3(0.9, 0.9, 0.9));
		dof6->setAngularLowerLimit(btVector3(0, 0, 0));
		dof6->setAngularUpperLimit(btVector3(0, 0, 0));
		dof6->setParam(BT_CONSTRAINT_STOP_CFM, 0.0f, 0);
		dof6->setParam(BT_CONSTRAINT_STOP_CFM, 0.0f, 1);
		dof6->setParam(BT_CONSTRAINT_STOP_CFM, 0.0f, 2);
		dof6->setParam(BT_CONSTRAINT_STOP_CFM, 0.1f, 3);
		dof6->setParam(BT_CONSTRAINT_STOP_CFM, 0.1f, 4);
		dof6->setParam(BT_CONSTRAINT_STOP_CFM, 0.1f, 5);

		dof6->setParam(BT_CONSTRAINT_STOP_ERP, 1.0f, 0);
		dof6->setParam(BT_CONSTRAINT_STOP_ERP, 1.0f, 1);
		dof6->setParam(BT_CONSTRAINT_STOP_ERP, 1.0f, 2);
		dof6->setParam(BT_CONSTRAINT_STOP_ERP, 1.0f, 3);
		dof6->setParam(BT_CONSTRAINT_STOP_ERP, 1.0f, 4);
		dof6->setParam(BT_CONSTRAINT_STOP_ERP, 1.0f, 5);
		world->addConstraint(dof6);
		*/

		
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

			NodeInfo * nodeCurrent = new NodeInfo(mt, center, end, NULL);
			
			//btRigidBody * bodyNow = addCapsule(RADIUS, (height - 2 * RADIUS) * 0.99, height * MASS_PER_HEIGHT, translate*rotate, nodeCurrent);
			addCapsule(RADIUS, (height - 2 * RADIUS) * 0.99, translate*rotate, nodeCurrent);
			//nodeCurrent->body = bodyNow;
			btRigidBody * bodyLast = lastOne->body;
			
			

			/*
			btVector3 localPivot = btVector3(end[0], end[1], end[2]);
			btPoint2PointConstraint * p2p = new btPoint2PointConstraint(*bodyNow, localPivot);
			world->addConstraint(p2p);
			*/

			/*
			btVector3 localPivot = btVector3(end[0], end[1], end[2]);
			btTransform tr;
			tr.setIdentity();
			tr.setOrigin(localPivot);
			btGeneric6DofConstraint* dof6 = new btGeneric6DofConstraint(*bodyNow, tr, true);
			dof6->setLinearLowerLimit(btVector3(1, 1, 1));
			dof6->setLinearUpperLimit(btVector3(0.9, 0.9, 0.9));
			dof6->setAngularLowerLimit(btVector3(0, 0, 0));
			dof6->setAngularUpperLimit(btVector3(0, 0, 0));
			dof6->setParam(BT_CONSTRAINT_STOP_CFM, 0.0f, 0);
			dof6->setParam(BT_CONSTRAINT_STOP_CFM, 0.0f, 1);
			dof6->setParam(BT_CONSTRAINT_STOP_CFM, 0.0f, 2);
			dof6->setParam(BT_CONSTRAINT_STOP_CFM, 0.1f, 3);
			dof6->setParam(BT_CONSTRAINT_STOP_CFM, 0.1f, 4);
			dof6->setParam(BT_CONSTRAINT_STOP_CFM, 0.1f, 5);

			dof6->setParam(BT_CONSTRAINT_STOP_ERP, 1.0f, 0);
			dof6->setParam(BT_CONSTRAINT_STOP_ERP, 1.0f, 1);
			dof6->setParam(BT_CONSTRAINT_STOP_ERP, 1.0f, 2);
			dof6->setParam(BT_CONSTRAINT_STOP_ERP, 1.0f, 3);
			dof6->setParam(BT_CONSTRAINT_STOP_ERP, 1.0f, 4);
			dof6->setParam(BT_CONSTRAINT_STOP_ERP, 1.0f, 5);
			world->addConstraint(dof6);
			*/
			/*
			btVector3 pivotA((lastOne->end)[0], (lastOne->end)[1], (lastOne->end)[2]);
			btVector3 pivotB = pivotA;
			Vec3 dirA = lastOne->end - lastOne->center;
			Vec3 dirB = end - lastOne->end;
			Vec3 axis = dirA ^ dirB;
			btVector3 btAxis(axis[0], axis[1], axis[2]);
			addHingeConstraint(*bodyLast, *bodyNow, pivotA, btAxis, btAxis);
			*/
			lastOne = nodeCurrent;
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
		pMaterial->setDiffuse(Material::FRONT, Vec4(0.1, 0.1, 0.8, 1.0));
		cursor_geode->getOrCreateStateSet()->setAttribute(pMaterial, StateAttribute::OVERRIDE);
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
				pMaterial->setDiffuse(Material::FRONT, currentColor);
				cursor_geode->getOrCreateStateSet()->setAttribute(pMaterial, StateAttribute::OVERRIDE);
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

		pMaterial->setDiffuse(Material::FRONT, Vec4(0.1, 0.1, 0.8, 1.0));
		cursor_geode->getOrCreateStateSet()->setAttribute(pMaterial, StateAttribute::OVERRIDE);
		if (lastOne != NULL) {
			std::cout << "yeah" << std::endl;
			//drawWorld::initPhysics();
			addCompoundToWorld();
			rts = true;
		}
		break;

	case 4:	// invalid
		break;
	case 100:
	{	double r = abs(palm_x) * 3;
		double g = abs(palm_y) * 3;
		double b = abs(palm_z) * 3;
		Vec4 color = Vec4(r / 256.0, g / 256.0, b / 256.0, 1.0);
		pMaterial->setDiffuse(Material::FRONT, color);
		cursor_geode->getOrCreateStateSet()->setAttribute(pMaterial, StateAttribute::OVERRIDE);
		currentColor = color;
	}		break;
	default:
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


Geode* drawWorld::addSphere(Vec3 center, float radius, Vec4 CylinderColor, Group *pAddToThisGroup)
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
	return geode;
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

btRigidBody* drawWorld::addCapsule(btScalar radius, btScalar height, float mass, btTransform t, NodeInfo* info) {
	btCapsuleShape* capsule = new btCapsuleShape(radius, height);
	capsule->setUserPointer(info);
	btVector3 inertia(0, 0, 0);
	if (mass != 0.0)
		capsule->calculateLocalInertia(mass, inertia);
	btMotionState* motion = new btDefaultMotionState(t);
	btRigidBody::btRigidBodyConstructionInfo btinfo(mass, motion, capsule, inertia);
	btRigidBody* body = new btRigidBody(btinfo);
	world->addRigidBody(body);
	bodies.push_back(body);
	return body;
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
			child_trans2 = shape->getWorldTransform() *compoundshape->getChildTransform(j);
			// child_trans = compoundshape->getChildTransform(j);
			 btQuaternion rotation = child_trans2.getRotation();
			// btMatrix3x3 rotation = child_trans.getBasis();
			
			float angle = rotation.getAngle();
			btVector3 axis = rotation.getAxis();
			btVector3 translate = child_trans.getOrigin();


			NodeInfo * info = (NodeInfo *)col_shape->getUserPointer();
			// btScalar * m = new btScalar[16];
			
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
			//info->mt->setMatrix
		}
	}
}

void drawWorld::drawRigidBody() {
	for (int i = 0; i < bodies.size(); i++) {
		btRigidBody * body = (btRigidBody *) bodies[i];
		if (body->getCollisionShape()->getShapeType() != STATIC_PLANE_PROXYTYPE) {
			const btCollisionShape* col_shape = body->getCollisionShape();
			btTransform child_trans;
			body->getMotionState()->getWorldTransform(child_trans);

			// child_trans = compoundshape->getChildTransform(j);
			btQuaternion rotation = child_trans.getRotation();
			// btMatrix3x3 rotation = child_trans.getBasis();

			float angle = rotation.getAngle();
			btVector3 axis = rotation.getAxis();
			btVector3 translate = child_trans.getOrigin();

			NodeInfo * info = (NodeInfo *) col_shape->getUserPointer();
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
			std::cout << i << std::endl;
		}

	}
}

void drawWorld::addCompoundToWorld(){
	std::cerr << "fire";
#ifdef SHIFT_TRANSFORM
	btVector3 localInertia(0, 0, 0);
	btTransform startTransform;
	startTransform.setIdentity();
	//startTransform.setIdentity();
	//startTransform.setOrigin(btVector3(initStart[0], initStart[1], initStart[2]));
	Vec3 v = lastOne->end;
	startTransform.setOrigin(btVector3(v[0], v[1], v[2]));
	btTransform shift;
	shift.setIdentity();
	btCompoundShape* newBoxCompound = shiftTransform(compound, 30, shift);
	newBoxCompound->calculateLocalInertia(30, localInertia);
	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform*shift);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(30, myMotionState, newBoxCompound, localInertia);
#else
	boxCompound->calculateLocalInertia(mass, localInertia);
	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, boxCompound, localInertia);
#endif

	//add compound into our world
	//btMotionState* motion2 = new btDefaultMotionState(t);
	//btRigidBody::btRigidBodyConstructionInfo info2(10, motion2, compound, btVector3(0, 0, 0));
	//info2.m_restitution = 0.1f;
	//info2.m_friction = 1.5f;
	btRigidBody * compound_body = new btRigidBody(rbInfo);
	

	compound_body->setCenterOfMassTransform(startTransform);
	compound_body->setAngularFactor(btVector3(1, 1, 1));
	compound_body->setLinearFactor(btVector3(1, 1, 0.5f));
	compound_body->setDamping(0.9, 0.3);
	compound_body->setRestitution(0.3f);
	compound_body->setFriction(0.4);
	compound_body->forceActivationState(DISABLE_DEACTIVATION);
	world->addRigidBody(compound_body);
	bodies.push_back(compound_body);
	compound = new btCompoundShape();
	lastOne = NULL;
}

void drawWorld::initPhysics(){
	
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
	
	/*
#ifdef SHIFT_TRANSFORM
	btVector3 localInertia(0, 0, 0);
	btTransform startTransform;
	startTransform.setIdentity();
	btTransform shift;
	shift.setIdentity();
	btCompoundShape* newBoxCompound = shiftTransform(compound, 30, shift);
	newBoxCompound->calculateLocalInertia(30, localInertia);
	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform*shift);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(30, myMotionState, newBoxCompound, localInertia);
#else
	boxCompound->calculateLocalInertia(mass, localInertia);
	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, boxCompound, localInertia);
#endif

	//add compound into our world
	//btMotionState* motion2 = new btDefaultMotionState(t);
	//btRigidBody::btRigidBodyConstructionInfo info2(10, motion2, compound, btVector3(0, 0, 0));
	//info2.m_restitution = 0.1f;
	//info2.m_friction = 1.5f;
	Vec3 center = (end + start) / 2;
	btRigidBody * compound_body = new btRigidBody(rbInfo);
	startTransform.setIdentity();
	startTransform.setOrigin(btVector3(initStart[0], initStart[1], initStart[2]));

	compound_body->setCenterOfMassTransform(startTransform);
	compound_body->setAngularFactor(btVector3(1, 1, 1));
	compound_body->setLinearFactor(btVector3(1, 1, 0.5f));
	compound_body->setDamping(0.9, 0.3);
	compound_body->setRestitution(0.3f);
	compound_body->setFriction(0.4);
	compound_body->forceActivationState(DISABLE_DEACTIVATION);
	world->addRigidBody(compound_body);
	bodies.push_back(compound_body);
	//compound = new btCompoundShape();
	*/
}

void drawWorld::simulate(float dt){
	world->stepSimulation(dt);
	///*
	for (int i = 0; i < bodies.size(); i++){
		if (bodies[i]->getCollisionShape()->getShapeType() == COMPOUND_SHAPE_PROXYTYPE){
			drawCompound(bodies[i]);
		}
		//std::cout << bodies[i]->getShapeType() << std::endl;
		//std::cout << i << std::endl;
	}
	//*/
	//drawRigidBody();
}

bool drawWorld::readyToSimulate(){
	return rts;
}

void drawWorld::initCompound(btVector3 start) {

#ifdef SHIFT_TRANSFORM
	btVector3 localInertia(0, 0, 0);
	btTransform startTransform;
	startTransform.setIdentity();
	btTransform shift;
	shift.setIdentity();
	btCompoundShape* newBoxCompound = shiftTransform(compound, 30, shift);
	newBoxCompound->calculateLocalInertia(30, localInertia);
	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform*shift);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(30, myMotionState, newBoxCompound, localInertia);
#else
	boxCompound->calculateLocalInertia(mass, localInertia);
	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, boxCompound, localInertia);
#endif

	//add compound into our world
	//btMotionState* motion2 = new btDefaultMotionState(t);
	//btRigidBody::btRigidBodyConstructionInfo info2(10, motion2, compound, btVector3(0, 0, 0));
	//info2.m_restitution = 0.1f;
	//info2.m_friction = 1.5f;
	btRigidBody* compound_body = new btRigidBody(rbInfo);
	startTransform.setIdentity();
	startTransform.setOrigin(start);

	compound_body->setCenterOfMassTransform(startTransform);
	compound_body->setAngularFactor(btVector3(1, 1, 1));
	compound_body->setLinearFactor(btVector3(1, 1, 0.5f));
	compound_body->setDamping(0.9, 0.3);
	compound_body->setRestitution(0.3f);
	compound_body->setFriction(0.4);
	compound_body->forceActivationState(DISABLE_DEACTIVATION);
	world->addRigidBody(compound_body);
	bodies.push_back(compound_body);
	//compound = new btCompoundShape();
}

void drawWorld::addHingeConstraint(btRigidBody &rbA, btRigidBody &rbB, btVector3 &anchor, btVector3 &axis1, btVector3 &axis2) {
	btHinge2Constraint * constraint = new btHinge2Constraint(rbA, rbB, anchor, axis1, axis2);
	//btHingeConstraint * constraint = new btHingeConstraint(*a, pivotInA, axisInA, useReferenceFrameA);
	//constraint->setLimit(0.0f, 0.0f);
	/*constraint->setLimit(0.1,
		0.2,
		1.0f,
		0.3f,
		0.0f);*/
	world->addConstraint(constraint);
	//constraint->setDbgDrawSize(btScalar(5.f));
}

btCompoundShape* drawWorld::shiftTransform(btCompoundShape* boxCompound, btScalar mass, btTransform& shift) {
	btTransform principal;
	btVector3 principalInertia;
	btScalar* masses = new btScalar[boxCompound->getNumChildShapes()];
	for (int j = 0; j<boxCompound->getNumChildShapes(); j++) {
		//evenly distribute mass
		masses[j] = mass / boxCompound->getNumChildShapes();
	}


	boxCompound->calculatePrincipalAxisTransform(masses, principal, principalInertia);


	///create a new compound with world transform/center of mass properly aligned with the principal axis

	///non-recursive compound shapes perform better

#ifdef USE_RECURSIVE_COMPOUND

	btCompoundShape* newCompound = new btCompoundShape();
	newCompound->addChildShape(principal.inverse(), boxCompound);
	newBoxCompound = newCompound;
	//m_collisionShapes.push_back(newCompound);

	//btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
	//btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,newCompound,principalInertia);

#else
#ifdef CHANGE_COMPOUND_INPLACE
	newBoxCompound = boxCompound;
	for (int i = 0; i<boxCompound->getNumChildShapes(); i++) {
		btTransform newChildTransform = principal.inverse()*boxCompound->getChildTransform(i);
		///updateChildTransform is really slow, because it re-calculates the AABB each time. todo: add option to disable this update
		boxCompound->updateChildTransform(i, newChildTransform);
	}
	bool isDynamic = (mass != 0.f);
	btVector3 localInertia(0, 0, 0);
	if (isDynamic)
		boxCompound->calculateLocalInertia(mass, localInertia);

#else
	///creation is faster using a new compound to store the shifted children
	btCompoundShape * newBoxCompound = new btCompoundShape();
	for (int i = 0; i<boxCompound->getNumChildShapes(); i++) {
		btTransform newChildTransform = principal.inverse()*boxCompound->getChildTransform(i);
		///updateChildTransform is really slow, because it re-calculates the AABB each time. todo: add option to disable this update
		newBoxCompound->addChildShape(newChildTransform, boxCompound->getChildShape(i));
	}



#endif

#endif//USE_RECURSIVE_COMPOUND

	shift = principal;
	return newBoxCompound;
}