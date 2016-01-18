#ifndef _BULLETOPENGLAPP_H_
#define _BULLETOPENGLAPP_H_

#include <Windows.h>
#include <GL/GL.h>
#include <GL/freeglut.h>

#include "BulletDynamics/Dynamics/btDynamicsWorld.h"
// includes for convex hulls
#include "BulletCollision/CollisionShapes/btConvexPolyhedron.h"
#include "BulletCollision/CollisionShapes/btShapeHull.h"



// include our custom Motion State object
#include "OpenGLMotionState.h"

// Our custom debug renderer
#include "DebugDrawer.h"

#include "GameObject.h"
#include "PhysicsDemo.h"
#include <set>
#include <iterator>
#include <algorithm>
#include <vector>

/*ADD*/	// includes required for soft body simulation
/*ADD*/	#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"
/*ADD*/	#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"
/*ADD*/	#include "BulletSoftBody/btSoftBodyHelpers.h"

// a convenient typedef to reference an STL vector of GameObjects
typedef std::vector<GameObject*> GameObjects;


// struct to store our raycasting results
struct RayResult {
 	btRigidBody* pBody;
 	btVector3 hitPoint;
};

class BulletOpenGLApplication {
public:
	BulletOpenGLApplication();
	~BulletOpenGLApplication();
	void Initialize();
	// FreeGLUT callbacks //
	virtual void Keyboard(unsigned char key, int x, int y);
	virtual void Special(int key, int x, int y);
	virtual void SpecialUp(int key, int x, int y);
	virtual void Reshape(int w, int h);
	virtual void Idle();

	// rendering. Can be overrideen by derived classes
	virtual void RenderScene();

	// scene updating. Can be overridden by derived classes
	virtual void UpdateScene(float dt);

	// drawing functions
	void DrawBox(const btVector3 &halfSize);
	void DrawShape(btScalar* transform, const btCollisionShape* pShape, const btVector3 &color, GLfloat rotation);

	void CreateGameObject(btCollisionShape* pShape,
		const float &mass,
		const btVector3 &color = btVector3(1.0f, 1.0f, 1.0f),
		const btVector3 &initialPosition = btVector3(0.0f, 0.0f, 0.0f),
		const btVector3 &LinearConstraint = btVector3(1.0f, 1.0f, 1.0f),
		const btQuaternion &initialRotation = btQuaternion(0, 0, 1, 1));

	void InitializePhysics();

	// camera functions
	void UpdateCamera();

	void SpawnSoftBody();

	void ShutdownPhysics();

	void RotateCamera(float &angle, float value);

	void ZoomCamera(float distance);

	void DestroyGameObject(btRigidBody* pBody);

	bool Raycast(const btVector3 &startPosition, const btVector3 &direction, RayResult &output, bool includeStatic = false);

    void DrawCylinder(const btScalar &radius, const btScalar &halfHeight);

	void DrawConvexHull(const btCollisionShape* shape);

	void DrawSphere(const btScalar &radius);

	void CreateObjects();

	void SetDominoProperties();

	void CheckForCollisionEvents();

	void CollisionEvent(btRigidBody * pBody0, btRigidBody * pBody1);

	void DebugFile(char* Message);

	void LoadTextures();

	void CreateSoftBodyObject();

	void SetupTopLineDominoes();

	btVector3 ShiftDominoColor(int NumberOfTransitions, const btVector3 &CurrentColor);

	btVector3 GetPickingRay(int x, int y);


protected:
	// camera control
	btVector3 m_cameraPosition; // the camera's current position
	btVector3 m_cameraTarget;	 // the camera's lookAt target
	float m_nearPlane; // minimum distance the camera will render
	float m_farPlane; // farthest distance the camera will render
	btVector3 m_upVector; // keeps the camera rotated correctly
	float m_cameraDistance; // distance from the camera to its target
	float m_cameraPitch; // pitch of the camera 
	float m_cameraYaw; // yaw of the camera

	int m_screenWidth;
	int m_screenHeight;

	int reset;
	int start;

	// core Bullet components
	btBroadphaseInterface* m_pBroadphase;
	btCollisionConfiguration* m_pCollisionConfiguration;
	btCollisionDispatcher* m_pDispatcher;
	btConstraintSolver* m_pSolver;
	btDynamicsWorld* m_pWorld;

	// a simple clock for counting time
	btClock m_clock;

	// an array of our game objects
	GameObjects m_objects;

	// debug renderer
	DebugDrawer* m_pDebugDrawer;

	// ----------------------- Donimo properties ------------------------- //

	btQuaternion Rotation;
	btVector3 Position;
	btVector3 LinearConstraint;
	btVector3 DominoColor;
	btCollisionShape* DominoCollisionShape;

	// ------------------------------------------------------------------- //

	// --------------------- Sogt body properties ------------------------ //

	// a pointer to our world, typecast into its soft body type
	btSoftRigidDynamicsWorld*  m_pSoftBodyWorld;
		
	// the soft body world info. Needed for proper contact generation
	btSoftBodyWorldInfo  m_softBodyWorldInfo;

	// ------------------------------------------------------------------- //

	btVector3 BallPosition;
	
};
#endif
