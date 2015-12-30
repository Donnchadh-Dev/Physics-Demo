#ifndef _BULLETOPENGLAPP_H_
#define _BULLETOPENGLAPP_H_

#include <Windows.h>
#include <GL/GL.h>
#include <GL/freeglut.h>

#include "BulletDynamics/Dynamics/btDynamicsWorld.h"

// include our custom Motion State object
#include "OpenGLMotionState.h"

#include "GameObject.h"
#include "Domino.h"
#include <vector>

// a convenient typedef to reference an STL vector of GameObjects
typedef std::vector<GameObject*> GameObjects;

typedef std::vector<Domino*> Dominos;


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
	virtual void Idle();

	// rendering. Can be overrideen by derived classes
	virtual void RenderScene();

	// scene updating. Can be overridden by derived classes
	virtual void UpdateScene(float dt);

	// camera functions
	void UpdateCamera();

	// drawing functions
	void DrawBox(const btVector3 &halfSize);
	void DrawShape(btScalar* transform, const btCollisionShape* pShape, const btVector3 &color, GLfloat rotation);

	void CreateGameObject(btCollisionShape* pShape, 
			const float &mass, 
			const btVector3 &color = btVector3(1.0f,1.0f,1.0f), 
			const btVector3 &initialPosition = btVector3(0.0f,0.0f,0.0f), 
			const btQuaternion &initialRotation = btQuaternion(0,0,1,1));

	void CreateDomino(const btVector3 &initialPosition, GLfloat rotation, btQuaternion &Rotation);

    void DrawCylinder(const btScalar &radius, const btScalar &halfHeight);

	void CreateObjects();

	void CheckForCollisionEvents();

	void CollisionEvent(btRigidBody * pBody0, btRigidBody * pBody1);

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

	Dominos dominos;
};
#endif
