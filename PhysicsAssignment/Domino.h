#ifndef _DOMINO_H_
#define _DOMINO_H_

#include <Windows.h>
#include <GL/GL.h>

#include "btBulletDynamicsCommon.h"
#include "OpenGLMotionState.h"

class Domino {
public:
	Domino(const btVector3 &initialPosition, GLfloat rotation);
	~Domino();

	// accessors
	btCollisionShape* GetShape() { return m_pShape; }

	btRigidBody* GetRigidBody() { return m_pBody; }

	btMotionState* GetMotionState() { return m_pMotionState; }

	void GetTransform(btScalar* transform) { 
		if (m_pMotionState) m_pMotionState->GetWorldTransform(transform); 
	}

	btVector3 initialosition;
	btVector3 GetColor() { return m_color; }
	GLfloat rotation;
protected:
	btCollisionShape*  m_pShape;
	btRigidBody*    m_pBody;
	OpenGLMotionState*  m_pMotionState;
	btVector3      m_color;
	float mass;
};
#endif