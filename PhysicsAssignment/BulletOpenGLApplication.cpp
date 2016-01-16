#include "BulletOpenGLApplication.h"

#include <fstream>  
#include <iostream>  
#include <string>  

// Some constants for 3D math and the camera speed
#define RADIANS_PER_DEGREE 0.01745329f
#define CAMERA_STEP_SIZE 5.0f

GLfloat h;

BulletOpenGLApplication::BulletOpenGLApplication() 
:
m_cameraPosition(0.0f, 130.0f, 0.0f),
m_cameraTarget(0.0f, 0.0f, 0.0f),
m_cameraDistance(45.0f),
m_cameraPitch(50.0f),
m_cameraYaw(130.0f),
m_upVector(0.0f, 1.0f, 0.0f),
m_nearPlane(1.0f),
m_farPlane(1000.0f),
m_pBroadphase(0),
m_pCollisionConfiguration(0),
m_pDispatcher(0),
m_pSolver(0),
m_pWorld(0)
{
}

BulletOpenGLApplication::~BulletOpenGLApplication() {

	// shutdown the physics system
	ShutdownPhysics();
}


void BulletOpenGLApplication::ShutdownPhysics() {
	delete m_pWorld;
	delete m_pSolver;
	delete m_pBroadphase;
	delete m_pDispatcher;
	delete m_pCollisionConfiguration;
}



void BulletOpenGLApplication::RotateCamera(float &angle, float value) {
	// change the value (it is passed by reference, so we
	// can edit it here)
	angle -= value;
	// keep the value within bounds
	if (angle < 0) angle += 360;
	if (angle >= 360) angle -= 360;
	// update the camera since we changed the angular value
	UpdateCamera();
}


void BulletOpenGLApplication::Keyboard(unsigned char key, int x, int y) {

	
	
	char k [2];
	memset(k, 0, sizeof(k));
	k[0] = key;
	k[1] = '\0';
	//DebugFile(k);

	switch (key) {
		// 'z' zooms in
	case 'z': ZoomCamera(+CAMERA_STEP_SIZE); break;
		// 'x' zoom out
	case 'x': ZoomCamera(-CAMERA_STEP_SIZE); break;
	case 'd':
	{
		
		// create a temp object to store the raycast result
		RayResult result;
		// perform the raycast
		if (!Raycast(m_cameraPosition, GetPickingRay(x, y), result))
			return; // return if the test failed
					// destroy the corresponding game object
		DebugFile("Found Something attempting to destory");
		DestroyGameObject(result.pBody);
		break;
	}
	}

	/*
	switch(key) {
	// if r is pressed
	case 'r':
		if(reset == 0)
		{
			// reset = 1; so the physics world doesnt try update the domino bodies while deleteing them
			reset = 1;

			// remove domino body from world and remove domino from list
			for(int i = 0; i < dominos.size(); i++)
			{
				m_pWorld->removeRigidBody(dominos.at(i)->GetRigidBody());
				dominos.at(i)->~Domino();
			}
		}
		break;
	}*/
}



void BulletOpenGLApplication::Special(int key, int x, int y) {
	// This function is called by FreeGLUT whenever special keys
	// are pressed down, like the arrow keys, or Insert, Delete etc.

	//DebugFile("Writing a special character");


	switch (key) {
		// the arrow keys rotate the camera up/down/left/right
	case GLUT_KEY_LEFT:
		RotateCamera(m_cameraYaw, +CAMERA_STEP_SIZE); break;
	case GLUT_KEY_RIGHT:
		RotateCamera(m_cameraYaw, -CAMERA_STEP_SIZE); break;
	case GLUT_KEY_UP:
		RotateCamera(m_cameraPitch, +CAMERA_STEP_SIZE); break;
	case GLUT_KEY_DOWN:
		RotateCamera(m_cameraPitch, -CAMERA_STEP_SIZE); break;
	}
}

void BulletOpenGLApplication::SpecialUp(int key, int x, int y) {
	// DebugFile("Writing a special character"); 
}









void BulletOpenGLApplication::DestroyGameObject(btRigidBody* pBody) {
	// we need to search through the objects in order to 
	// find the corresponding iterator (can only erase from 
	// an std::vector by passing an iterator)
	for (GameObjects::iterator iter = m_objects.begin(); iter != m_objects.end(); ++iter) {
		if ((*iter)->GetRigidBody() == pBody) {
			GameObject* pObject = *iter;
			// remove the rigid body from the world
			m_pWorld->removeRigidBody(pObject->GetRigidBody());
			// erase the object from the list
			m_objects.erase(iter);
			// delete the object from memory
			delete pObject;
			// done
			return;
		}
	}
}

void BulletOpenGLApplication::Idle() {

	// this function is called frequently, whenever FreeGlut
	// isn't busy processing its own events. It should be used
	// to perform any updating and rendering tasks

	// clear the backbuffer
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	// get the time since the last iteration
	float dt = m_clock.getTimeMilliseconds();
	// reset the clock to 0
	m_clock.reset();
	// update the scene (convert ms to s)
	UpdateScene(dt / 1000.0f);

	// update the camera
	UpdateCamera();

	// render the scene
	RenderScene();

	// swap the front and back buffers
	glutSwapBuffers();
}

void BulletOpenGLApplication::UpdateCamera() {
	// exit in erroneous situations
	if (m_screenWidth == 0 && m_screenHeight == 0)
		return;
	
	// select the projection matrix
	glMatrixMode(GL_PROJECTION);
	// set it to the matrix-equivalent of 1
	glLoadIdentity();
	// determine the aspect ratio of the screen
	float aspectRatio = m_screenWidth / (float)m_screenHeight;
	// create a viewing frustum based on the aspect ratio and the
	// boundaries of the camera
	glFrustum (-aspectRatio * m_nearPlane, aspectRatio * m_nearPlane, -m_nearPlane, m_nearPlane, m_nearPlane, m_farPlane);
	// the projection matrix is now set

	// select the view matrix
	glMatrixMode(GL_MODELVIEW);
	// set it to '1'
	glLoadIdentity();

	// our values represent the angles in degrees, but 3D 
	// math typically demands angular values are in radians.
	float pitch = m_cameraPitch * RADIANS_PER_DEGREE;
	float yaw = m_cameraYaw * RADIANS_PER_DEGREE;

	// create a quaternion defining the angular rotation 
	// around the up vector
	btQuaternion rotation(m_upVector, yaw);

	// set the camera's position to 0,0,0, then move the 'z' 
	// position to the current value of m_cameraDistance.
	btVector3 cameraPosition(0,0,0);
	cameraPosition[2] = -m_cameraDistance;

	// create a Bullet Vector3 to represent the camera 
	// position and scale it up if its value is too small.
	btVector3 forward(cameraPosition[0], cameraPosition[1], cameraPosition[2]);
	if (forward.length2() < SIMD_EPSILON) {
		forward.setValue(1.f,0.f,0.f);
	}

	// figure out the 'right' vector by using the cross 
	// product on the 'forward' and 'up' vectors
	btVector3 right = m_upVector.cross(forward);

	// create a quaternion that represents the camera's roll
	btQuaternion roll(right, - pitch);

	// turn the rotation (around the Y-axis) and roll (around 
	// the forward axis) into transformation matrices and 
	// apply them to the camera position. This gives us the 
	// final position
	cameraPosition = btMatrix3x3(rotation) * btMatrix3x3(roll) * cameraPosition;

	// save our new position in the member variable, and 
	// shift it relative to the target position (so that we 
	// orbit it)
	m_cameraPosition[0] = cameraPosition.getX();
	m_cameraPosition[1] = cameraPosition.getY();
	m_cameraPosition[2] = cameraPosition.getZ();
	m_cameraPosition += m_cameraTarget;

	// create a view matrix based on the camera's position and where it's
	// looking
	gluLookAt(m_cameraPosition[0], m_cameraPosition[1], m_cameraPosition[2], m_cameraTarget[0], m_cameraTarget[1], m_cameraTarget[2], m_upVector.getX(), m_upVector.getY(), m_upVector.getZ());
	// the view matrix is now set
}



void BulletOpenGLApplication::RenderScene() {
	// create an array of 16 floats (representing a 4x4 matrix)
	btScalar transform[16];

	for(int i = 0; i < m_objects.size(); i++)
	{
		m_objects.at(i)->GetTransform(transform);
		DrawShape(transform, m_objects.at(i)->GetShape(), m_objects.at(i)->GetColor(), 0.0f);
	}

}

void BulletOpenGLApplication::UpdateScene(float dt) {

	// as long as we arent trying to delete the objects
	//if(reset == 0)
	//{
		// check if the world object exists
		if (m_pWorld) {
			// step the simulation through time. This is called
			// every update and the amount of elasped time was 
			// determined back in ::Idle() by our clock object.
			m_pWorld->stepSimulation(dt);
		}

		// if the first domino hasnt already tipped over and started the chain reaction
		CheckForCollisionEvents();
/*
		if(start == 0)
		{
			// apply a force to the first domino, starting the chain reaction
			//dominos.at(0)->GetRigidBody()->applyCentralForce(btVector3(0, 0, 20));
		}
	}

	// if we have deleted all the dominos
	if(reset == 1)
	{
		//re create them (but this doesnt work, it just breaks, cant figure out why)
		CreateObjects();
		reset = 0;
	}*/
}



void BulletOpenGLApplication::CreateGameObject(btCollisionShape* pShape, const float &mass, const btVector3 &color, const btVector3 &initialPosition, const btVector3 &LinearConstraint, const btQuaternion &initialRotation) {
	// create a new game object
	GameObject* pObject = new GameObject(pShape, mass, color, initialPosition, initialRotation);

	pObject->GetRigidBody()->setLinearFactor(LinearConstraint);

	// push it to the back of the list
	m_objects.push_back(pObject);

	// check if the world object is valid
	if (m_pWorld) {
		// add the object's rigid body to the world
		m_pWorld->addRigidBody(pObject->GetRigidBody());
	}
}

// ---------------------------------------- INITIALIZE THE SCENE --------------------------------------- //

void BulletOpenGLApplication::Initialize() {
	// this function is called inside glutmain() after
	// creating the window, but before handing control
	// to FreeGLUT

	// create some floats for our ambient, diffuse, specular and position
	GLfloat ambient[] = { 0.2f, 0.2f, 0.2f, 1.0f }; // dark grey
	GLfloat diffuse[] = { 1.0f, 1.0f, 1.0f, 1.0f }; // white
	GLfloat specular[] = { 1.0f, 1.0f, 1.0f, 1.0f }; // white
	GLfloat position[] = { 5.0f, 10.0f, 1.0f, 0.0f };

	// set the ambient, diffuse, specular and position for LIGHT0
	glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, specular);
	glLightfv(GL_LIGHT0, GL_POSITION, position);

	glEnable(GL_LIGHTING); // enables lighting
	glEnable(GL_LIGHT0); // enables the 0th light
	glEnable(GL_COLOR_MATERIAL); // colors materials when lighting is enabled

								 // enable specular lighting via materials
	glMaterialfv(GL_FRONT, GL_SPECULAR, specular);
	glMateriali(GL_FRONT, GL_SHININESS, 15);

	// enable smooth shading
	glShadeModel(GL_SMOOTH);

	// enable depth testing to be 'less than'
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);

	// set the backbuffer clearing color to a lightish blue
	glClearColor(0.6, 0.65, 0.85, 0);

	InitializePhysics();
}

void BulletOpenGLApplication::InitializePhysics() {
	// create the collision configuration
	m_pCollisionConfiguration = new btDefaultCollisionConfiguration();
	// create the dispatcher
	m_pDispatcher = new btCollisionDispatcher(m_pCollisionConfiguration);
	// create the broadphase
	m_pBroadphase = new btDbvtBroadphase();
	// create the constraint solver
	m_pSolver = new btSequentialImpulseConstraintSolver();
	// create the world
	m_pWorld = new btDiscreteDynamicsWorld(m_pDispatcher, m_pBroadphase, m_pSolver, m_pCollisionConfiguration);
	// create our scene's physics objects
	CreateObjects();

	reset = 0;
	start = 0;
}


// ---------------------------------------- CREATE THE OBJECTS --------------------------------------------- //

void BulletOpenGLApplication::CreateObjects() {

	SetDominoProperties();

	// create a ground plane
	CreateGameObject(new btBoxShape(btVector3(1, 200, 200)), 0, btVector3(0.2f, 0.2f, 0.2f), btVector3(00.0f, 0.0f, 0.0f));


	// first level ramp level ground
	CreateGameObject(new btBoxShape(btVector3(3, 4, 4)), 0, btVector3(0.0f, 0.1f, 0.7f), btVector3(0.0f, 04.0f, -62));

	// first level ramp level ground
	CreateGameObject(new btBoxShape(btVector3(3, 10, 4)), 0, btVector3(0.0f, 0.1f, 0.7f), btVector3(14.0f, 04.0f, -62));


	// ----------- convex hull
	// create a vertex cloud defining a square-based pyramid
	btVector3 points[10] = {
		btVector3(-0.5,4,10),
		btVector3(-0.5,4,-10),
		btVector3(5, 4,-10),
		btVector3(5, 3,-10),
		btVector3(2, 3,-10),
		btVector3(2,-3,-10),
		btVector3(5,-3,-10),
		btVector3(5,-4,-10),
		btVector3(-0.5,-4,-10),
		btVector3(-0.5,-4,10) };


	btQuaternion RampRotation;
	RampRotation.setEuler(-1.561f, 0.0f, 1.561f);

	// create our convex hull
	btConvexHullShape* pShape = new btConvexHullShape(&points[0].getX(), sizeof(points) / sizeof(points[0]));
	// initialize the object as a polyhedron
	pShape->initializePolyhedralFeatures();

	// create the ground level ramp
	CreateGameObject(pShape, 100.0, btVector3(0.7f, 0.1f, 0.7f), btVector3(0, 1.5, -48), btVector3(1.0f, 1.0f, 1.0f), btQuaternion(0, 0, 1, 1));

	// create the second level ramp
	CreateGameObject(pShape, 100.0, btVector3(0.7f, 0.1f, 0.1f), btVector3(14, 8, -62), btVector3(1.0f, 1.0f, 1.0f), RampRotation);  //btQuaternion(0.5, 0.5, -0.5, 0.5))
																														   //btQuaternion(0.7071067811865476,0, 0 ,-0.7071067811865476))
																															
	
	// --------------------------------------------------------------------------- //

	// create a yellow sphere
	CreateGameObject(new btSphereShape(1.5f), .5, btVector3(0.7f, 0.7f, 0.0f), btVector3(0, 10, -58.56), btVector3(0.0f, 1.0f, 1.0f));

	// create a pink sphere
	CreateGameObject(new btSphereShape(1.5f), 10.0, btVector3(0.7f, 0.1f, 0.7f), btVector3(20, 15, -61.65), btVector3(1.0f, 1.0f, 1.0f));



	// --------------------------------------------------------------------------- //

	float spacing = 1.2;
	float x, z, y;


	z = -37.0f;
	x = 0.0f;
	y = 0.0f;

	// the initial idea was to have a cool looking domino setup, spirals, staircases ect
	// this relies on being able to rotate the dominos so you can have them in more than just a straight line, you need to
	// rotate them so they can curve and go in circles among other things
	// however, we couldnt get the dominos to rotate correctly, they would rotate, but act and fall as if not rotated
	// becuase of this we were very limited in what we could do in terms of 'domino setup' so just showed a few dominos falling over

	GLfloat rotation = 0.0f;

	// set up first 6 dominos
	for (int i = 0; i < 12; i++)
	{
		CreateGameObject(DominoCollisionShape, 3.0, DominoColor, btVector3(x, y, z), LinearConstraint, Rotation);
		z += spacing;
	}

	x = -1.0;
	float x2 = 1.2;

	// set up next 12 dominos in 2 lines
	for (int i = 0; i < 24; i++)
	{
		CreateGameObject(DominoCollisionShape, 3.0, DominoColor, btVector3(x, y, z), LinearConstraint, Rotation);
		CreateGameObject(DominoCollisionShape, 3.0, DominoColor, btVector3(x2, y, z), LinearConstraint, Rotation);
		z += spacing;
	}

	x = -2.0;
	x2 = 2.0;
	float x3 = 0.0;

	// set up next 12 dominos in 3 lines
	for (int i = 0; i < 12; i++)
	{
		CreateGameObject(DominoCollisionShape, 3.0, DominoColor, btVector3(x, y, z), LinearConstraint, Rotation);
		CreateGameObject(DominoCollisionShape, 3.0, DominoColor, btVector3(x2, y, z), LinearConstraint, Rotation);
		CreateGameObject(DominoCollisionShape, 3.0, DominoColor, btVector3(x3, y, z), LinearConstraint, Rotation);
		z += spacing;
	}


	// ------------------------------------ Create The Domino Skyscraper ----------------------------------------- //

	float yTotal = y + 2;

	for (int i = 0; i < 4; i++) {

		//Rotation = btQuaternion(0, 1, 0, 1);

		Rotation.setEulerZYX(0,1.5,1.5);

		yTotal += 2;
		CreateGameObject(DominoCollisionShape, 2.0, DominoColor, btVector3(x, yTotal, z - 2.5f), LinearConstraint, Rotation);
		CreateGameObject(DominoCollisionShape, 2.0, DominoColor, btVector3(x2, yTotal, z - 2.5f), LinearConstraint, Rotation);
		CreateGameObject(DominoCollisionShape, 2.0, DominoColor, btVector3(x3, yTotal, z - 2.5f), LinearConstraint, Rotation);

		Rotation = btQuaternion(0, 0, 1, 1);

		z -= 3.6f;

		yTotal += 1.7f;

		for (int i = 0; i < 3; i++)
		{
			CreateGameObject(DominoCollisionShape, 2.0, DominoColor, btVector3(x, yTotal, z), LinearConstraint, Rotation);
			CreateGameObject(DominoCollisionShape, 2.0, DominoColor, btVector3(x2, yTotal, z), LinearConstraint, Rotation);
			CreateGameObject(DominoCollisionShape, 2.0, DominoColor, btVector3(x3, yTotal, z), LinearConstraint, Rotation);
			z += 1.2f;
		}

		Rotation.setEulerZYX(0, 1.5, 1.5);


	}

}

void BulletOpenGLApplication::CheckForCollisionEvents() {

	// iterate through all of the manifolds in the dispatcher
	for (int i = 0; i < m_pDispatcher->getNumManifolds(); ++i) {
		
		// get the manifold
		btPersistentManifold* pManifold = m_pDispatcher->getManifoldByIndexInternal(i);
		
		// ignore manifolds that have no contact points.
		if (pManifold->getNumContacts() > 0) {

			// get the two rigid bodies involved in the collision
			const btRigidBody* pBody0 = static_cast<const btRigidBody*>(pManifold->getBody0());
			const btRigidBody* pBody1 = static_cast<const btRigidBody*>(pManifold->getBody1());

			CollisionEvent((btRigidBody*)pBody0, (btRigidBody*)pBody1);
		}
	}
}

void BulletOpenGLApplication::CollisionEvent(btRigidBody * pBody0, btRigidBody * pBody1) 
{
	// if one of the collided dominos is the first
/*	if(pBody0 == dominos.at(0)->GetRigidBody() || pBody1 == dominos.at(0)->GetRigidBody())
	{
		//if one of the collided dominos id the second
		if(pBody0 == dominos.at(1)->GetRigidBody() || pBody1 == dominos.at(1)->GetRigidBody())
		{
			// stop tipping over the first (applying the force to it)
			start = 1;
		}
	}*/
}

// --------------------------------------------- DRAW SHAPE ------------------------------------------------------ //



void BulletOpenGLApplication::SetDominoProperties() {
	// ----------------------- Donimo properties ------------------------- //

	Rotation = btQuaternion(0, 0, 1, 1);
	LinearConstraint = btVector3(1.0f, 1.0f, 1.0f);
	DominoColor = btVector3(1.0f, 0.2f, 0.2f);
	DominoCollisionShape = new btBoxShape(btVector3(1.6f, 0.8f, 0.19f));

	// ------------------------------------------------------------------- //

}



void BulletOpenGLApplication::DrawShape(btScalar* transform, const btCollisionShape* pShape, const btVector3 &color, GLfloat rotation) {
	// set the color
	glColor3f(color.x(), color.y(), color.z());

	// push the matrix stack
	glPushMatrix();
	glMultMatrixf(transform);

	// make a different draw call based on the object type
	switch (pShape->getShapeType()) {
		// an internal enum used by Bullet for boxes
	case BOX_SHAPE_PROXYTYPE:
	{
		// assume the shape is a box, and typecast it
		const btBoxShape* box = static_cast<const btBoxShape*>(pShape);
		// get the 'halfSize' of the box
		btVector3 halfSize = box->getHalfExtentsWithMargin();

		// rotate the dominos based on their rotation
		glRotatef(rotation, 1.0f, 0.0f, 0.0f);

		DrawBox(halfSize);
		break;
	}
	case SPHERE_SHAPE_PROXYTYPE:
	{
		// assume the shape is a sphere and typecast it
		const btSphereShape* sphere = static_cast<const btSphereShape*>(pShape);
		// get the sphere's size from the shape
		float radius = sphere->getMargin();
		// draw the sphere
		DrawSphere(radius);
		break;
	}

	case CYLINDER_SHAPE_PROXYTYPE:
	{
		// assume the object is a cylinder
		const btCylinderShape* pCylinder = static_cast<const btCylinderShape*>(pShape);
		// get the relevant data
		float radius = pCylinder->getRadius();
		float halfHeight = pCylinder->getHalfExtentsWithMargin()[1];
		// draw the cylinder
		DrawCylinder(radius, halfHeight);

		break;
	}
	case CONVEX_HULL_SHAPE_PROXYTYPE:
	{
		// draw the convex hull shape...whatever it is
		DrawConvexHull(pShape);
		break;
	}
	default:
		// unsupported type
		break;
	}

	// pop the stack
	glPopMatrix();
}


// --------------------------------------------- SPECIFIC SHAPES ------------------------------------------------------ //

/*ADD*/	void BulletOpenGLApplication::DrawConvexHull(const btCollisionShape* shape) {
	// get the polyhedral data from the convex hull
	const btConvexPolyhedron* pPoly = shape->isPolyhedral() ? ((btPolyhedralConvexShape*)shape)->getConvexPolyhedron() : 0;
	if (!pPoly) return;
	/*ADD*/
	// begin drawing triangles
	glBegin(GL_TRIANGLES);
	/*ADD*/
	// iterate through all faces
	for (int i = 0; i < pPoly->m_faces.size(); i++) {
			// get the indices for the face
			int numVerts = pPoly->m_faces[i].m_indices.size();
			if (numVerts>2) {
					// iterate through all index triplets
					for (int v = 0; v <pPoly->m_faces[i].m_indices.size() - 2; v++) {
							// grab the three vertices
							btVector3 v1 = pPoly->m_vertices[pPoly->m_faces[i].m_indices[0]];
							btVector3 v2 = pPoly->m_vertices[pPoly->m_faces[i].m_indices[v + 1]];
							btVector3 v3 = pPoly->m_vertices[pPoly->m_faces[i].m_indices[v + 2]];
							// calculate the normal
							btVector3 normal = (v3 - v1).cross(v2 - v1);
							normal.normalize();
							// draw the triangle
							glNormal3f(normal.getX(), normal.getY(), normal.getZ());
							glVertex3f(v1.x(), v1.y(), v1.z());
							glVertex3f(v2.x(), v2.y(), v2.z());
							glVertex3f(v3.x(), v3.y(), v3.z());
				/*ADD*/
			}
			/*ADD*/
		}
		/*ADD*/
	}
	// done drawing
	glEnd();
	/*ADD*/
}


void BulletOpenGLApplication::DrawCylinder(const btScalar &radius, const btScalar &halfHeight) {
static int slices = 15;
static int stacks = 10;
// tweak the starting position of the
// cylinder to match the physics object
glRotatef(-90.0, 1.0, 0.0, 0.0);
glTranslatef(0.0, 0.0, -halfHeight);
// create a quadric object to render with
GLUquadricObj *quadObj = gluNewQuadric();
// set the draw style of the quadric
gluQuadricDrawStyle(quadObj, (GLenum)GLU_FILL);
gluQuadricNormals(quadObj, (GLenum)GLU_SMOOTH);
// create a disk to cap the cylinder
gluDisk(quadObj, 0, radius, slices, stacks);
// create the main hull of the cylinder (no caps)
gluCylinder(quadObj, radius, radius, 2.f*halfHeight, slices, stacks);
// shift the position and rotation again
glTranslatef(0.0f, 0.0f, 2.f*halfHeight);
glRotatef(-180.0f, 0.0f, 1.0f, 0.0f);
// draw the cap on the other end of the cylinder
gluDisk(quadObj, 0, radius, slices, stacks);
// don't need the quadric anymore, so remove it
// to save memory
gluDeleteQuadric(quadObj);
/*ADD*/	}

void BulletOpenGLApplication::DrawSphere(const btScalar &radius) {
	// some constant values for more many segments to build the sphere from
	static int lateralSegments = 25;
	static int longitudinalSegments = 25;

	// iterate laterally
	for (int i = 0; i <= lateralSegments; i++) {
		// do a little math to find the angles of this segment
		btScalar lat0 = SIMD_PI * (-btScalar(0.5) + (btScalar)(i - 1) / lateralSegments);
		btScalar z0 = radius*sin(lat0);
		btScalar zr0 = radius*cos(lat0);

		btScalar lat1 = SIMD_PI * (-btScalar(0.5) + (btScalar)i / lateralSegments);
		btScalar z1 = radius*sin(lat1);
		btScalar zr1 = radius*cos(lat1);

		// start rendering strips of quads (polygons with 4 poins)
		glBegin(GL_QUAD_STRIP);
		// iterate longitudinally
		for (int j = 0; j <= longitudinalSegments; j++) {
			// determine the points of the quad from the lateral angles
			btScalar lng = 2 * SIMD_PI * (btScalar)(j - 1) / longitudinalSegments;
			btScalar x = cos(lng);
			btScalar y = sin(lng);
			// draw the normals and vertices for each point in the quad
			// since it is a STRIP of quads, we only need to add two points
			// each time to create a whole new quad

			// calculate the normal
			btVector3 normal = btVector3(x*zr0, y*zr0, z0);
			normal.normalize();
			glNormal3f(normal.x(), normal.y(), normal.z());
			// create the first vertex
			glVertex3f(x * zr0, y * zr0, z0);

			// calculate the next normal
			normal = btVector3(x*zr1, y*zr1, z1);
			normal.normalize();
			glNormal3f(normal.x(), normal.y(), normal.z());
			// create the second vertex
			glVertex3f(x * zr1, y * zr1, z1);

			// and repeat...
		}
		glEnd();
	}
}


void BulletOpenGLApplication::DrawBox(const btVector3 &halfSize) {

	float halfWidth = halfSize.x();
	float halfHeight = halfSize.y();
	float halfDepth = halfSize.z();

	// create the vertex positions
	btVector3 vertices[8] = {
		btVector3(halfWidth,halfHeight,halfDepth),
		btVector3(-halfWidth,halfHeight,halfDepth),
		btVector3(halfWidth,-halfHeight,halfDepth),
		btVector3(-halfWidth,-halfHeight,halfDepth),
		btVector3(halfWidth,halfHeight,-halfDepth),
		btVector3(-halfWidth,halfHeight,-halfDepth),
		btVector3(halfWidth,-halfHeight,-halfDepth),
		btVector3(-halfWidth,-halfHeight,-halfDepth) };

	// create the indexes for each triangle, using the 
	// vertices above. Make it static so we don't waste 
	// processing time recreating it over and over again
	static int indices[36] = {
		0,1,2,
		3,2,1,
		4,0,6,
		6,0,2,
		5,1,4,
		4,1,0,
		7,3,1,
		7,1,5,
		5,4,7,
		7,4,6,
		7,2,3,
		7,6,2 };

	// start processing vertices as triangles
	glBegin(GL_TRIANGLES);

	// increment the loop by 3 each time since we create a 
	// triangle with 3 vertices at a time.

	for (int i = 0; i < 36; i += 3) {
		// get the three vertices for the triangle based
		// on the index values set above
		// use const references so we don't copy the object
		// (a good rule of thumb is to never allocate/deallocate
		// memory during *every* render/update call. This should 
		// only happen sporadically)
		const btVector3 &vert1 = vertices[indices[i]];
		const btVector3 &vert2 = vertices[indices[i + 1]];
		const btVector3 &vert3 = vertices[indices[i + 2]];

		// create a normal that is perpendicular to the 
		// face (use the cross product)
		btVector3 normal = (vert3 - vert1).cross(vert2 - vert1);
		normal.normalize();

		// set the normal for the subsequent vertices
		glNormal3f(normal.getX(), normal.getY(), normal.getZ());

		// create the vertices
		glVertex3f(vert1.x(), vert1.y(), vert1.z());
		glVertex3f(vert2.x(), vert2.y(), vert2.z());
		glVertex3f(vert3.x(), vert3.y(), vert3.z());
	}

	// stop processing vertices
	glEnd();
}

void BulletOpenGLApplication::DebugFile(char* Message) {

	MessageBox(0, Message, "MessageBox caption", MB_OK);

	std::string s = Message;
	std::ofstream os("filename.txt");
	if (!os) { std::cerr << "Error writing to ..." << std::endl; }
	else {
		os << s;
	}
}

void BulletOpenGLApplication::ZoomCamera(float distance) {
	// change the distance value
	m_cameraDistance -= distance;
	// prevent it from zooming in too far
	if (m_cameraDistance < 0.1f) m_cameraDistance = 0.1f;
	// update the camera since we changed the zoom distance
	UpdateCamera();
}


btVector3 BulletOpenGLApplication::GetPickingRay(int x, int y) {

	//--
	std::string s = std::to_string(x);
	char * p = (char*)(x);

	DebugFile(p);
	//--


	// calculate the field-of-view
	float tanFov = 1.0f / m_nearPlane;
	float fov = btScalar(2.0) * btAtan(tanFov);

	// get a ray pointing forward from the 
	// camera and extend it to the far plane
	btVector3 rayFrom = m_cameraPosition;
	btVector3 rayForward = (m_cameraTarget - m_cameraPosition);
	rayForward.normalize();
	rayForward *= m_farPlane;

	// find the horizontal and vertical vectors 
	// relative to the current camera view
	btVector3 ver = m_upVector;
	btVector3 hor = rayForward.cross(ver);
	hor.normalize();
	ver = hor.cross(rayForward);
	ver.normalize();
	hor *= 2.f * m_farPlane * tanFov;
	ver *= 2.f * m_farPlane * tanFov;

	// calculate the aspect ratio
	btScalar aspect = m_screenWidth / (btScalar)m_screenHeight;

	// adjust the forward-ray based on
	// the X/Y coordinates that were clicked
	hor *= aspect;
	btVector3 rayToCenter = rayFrom + rayForward;
	btVector3 dHor = hor * 1.f / float(m_screenWidth);
	btVector3 dVert = ver * 1.f / float(m_screenHeight);
	btVector3 rayTo = rayToCenter - 0.5f * hor + 0.5f * ver;
	rayTo += btScalar(x) * dHor;
	rayTo -= btScalar(y) * dVert;

	// return the final result
	return rayTo;
}


bool BulletOpenGLApplication::Raycast(const btVector3 &startPosition, const btVector3 &direction, RayResult &output, bool includeStatic) {

	DebugFile("inside raycast");

	if (!m_pWorld)
		return false;

	// get the picking ray from where we clicked
	btVector3 rayTo = direction;
	btVector3 rayFrom = m_cameraPosition;

	// create our raycast callback object
	btCollisionWorld::ClosestRayResultCallback rayCallback(rayFrom, rayTo);

	// perform the raycast
	m_pWorld->rayTest(rayFrom, rayTo, rayCallback);

	// did we hit something?
	if (rayCallback.hasHit())
	{
		// if so, get the rigid body we hit
		btRigidBody* pBody = (btRigidBody*)btRigidBody::upcast(rayCallback.m_collisionObject);
		if (!pBody)
			return false;

		// prevent us from picking objects 
		// like the ground plane
		if (!includeStatic) // skip this check if we want it to hit static objects
			if (pBody->isStaticObject() || pBody->isKinematicObject())
				return false;

		// set the result data
		output.pBody = pBody;
		output.hitPoint = rayCallback.m_hitPointWorld;
		return true;
	}

	// we didn't hit anything
	return false;
}

void BulletOpenGLApplication::LoadTextures() {




}

