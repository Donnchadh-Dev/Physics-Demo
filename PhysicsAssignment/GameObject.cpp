#include "GameObject.h"
GameObject::GameObject(btCollisionShape* pShape, float mass, const btVector3 &color, const btVector3 &initialPosition, const btQuaternion &initialRotation, const bool isDomino) {

	if (isDomino) {

		m_pShape = new btBoxShape(btVector3(1.6f, 0.8f, 0.19f));
		// store the color
		m_color = btVector3(1.0f, 0.2f, 0.2f);
		mass = 3;

	} else {

		// store the shape for later usage
		m_pShape = pShape;

		// store the color
		m_color = color;
	}

	// create the initial transform
	btTransform transform;
	transform.setIdentity();
	transform.setOrigin(initialPosition);
	transform.setRotation(initialRotation);

	// create the motion state from the
	// initial transform
	m_pMotionState = new OpenGLMotionState(transform);

	// calculate the local inertia
	btVector3 localInertia(0,0,0);

	// objects of infinite mass can't
	// move or rotate
	if (mass != 0.0f)
		pShape->calculateLocalInertia(mass, localInertia);

	// create the rigid body construction
	// info using the mass, motion state
	// and shape
	btRigidBody::btRigidBodyConstructionInfo cInfo(mass, m_pMotionState, pShape, localInertia);
	
	// create the rigid body
	m_pBody = new btRigidBody(cInfo);

}

GameObject::~GameObject() {
	delete m_pBody;
	delete m_pMotionState;
	delete m_pShape;
}
