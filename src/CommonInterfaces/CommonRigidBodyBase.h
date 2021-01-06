
#ifndef COMMON_RIGID_BODY_BASE_H
#define COMMON_RIGID_BODY_BASE_H

#include "btBulletDynamicsCommon.h"
#include "CommonExampleInterface.h"
#include "CommonGUIHelperInterface.h"
#include "CommonRenderInterface.h"
#include "CommonCameraInterface.h"
#include "CommonGraphicsAppInterface.h"
#include "CommonWindowInterface.h"
#include "BulletCollision/NarrowPhaseCollision/btRaycastCallback.h"

struct CommonRigidBodyBase
{
	//keep the collision shapes, for deletion/cleanup
	btAlignedObjectArray<btCollisionShape*> m_collisionShapes;
  CommonRigidBodyBase* m_broadphase;
	btCollisionDispatcher* m_dispatcher;
	btConstraintSolver* m_solver;
	btDefaultCollisionConfiguration* m_collisionConfiguration;
	btDiscreteDynamicsWorld* m_dynamicsWorld;

	//data for picking objects
	class btRigidBody* m_pickedBody;
	class btTypedConstraint* m_pickedConstraint;
	int m_savedState;
	btVector3 m_oldPickingPos;
	btVector3 m_hitPos;
	btScalar m_oldPickingDist;

	CommonRigidBodyBase()
		: m_broadphase(0),
		  m_dispatcher(0),
		  m_solver(0),
		  m_collisionConfiguration(0),
		  m_dynamicsWorld(0),
		  m_pickedBody(0),
		  m_pickedConstraint(0)
	{
	}
	virtual ~CommonRigidBodyBase()
	{
	}

	btDiscreteDynamicsWorld* getDynamicsWorld()
	{
		return m_dynamicsWorld;
	}

	virtual void createEmptyDynamicsWorld()
	{
//		///collision configuration contains default setup for memory, collision setup
//		m_collisionConfiguration = new btDefaultCollisionConfiguration();
//		//m_collisionConfiguration->setConvexConvexMultipointIterations();
//
//		///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
//		m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
//
//		// m_broadphase = new btDbvtBroadphase();
//
//		///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
//		btSequentialImpulseConstraintSolver* sol = new btSequentialImpulseConstraintSolver;
//		m_solver = sol;

		// m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration);

		// m_dynamicsWorld->setGravity(btVector3(0, -10, 0));
	}

	virtual void stepSimulation(float deltaTime)
	{
		if (m_dynamicsWorld)
		{
			m_dynamicsWorld->stepSimulation(deltaTime);
		}
	}

	virtual void physicsDebugDraw(int debugFlags)
	{
		if (m_dynamicsWorld && m_dynamicsWorld->getDebugDrawer())
		{
			m_dynamicsWorld->getDebugDrawer()->setDebugMode(debugFlags);
			m_dynamicsWorld->debugDrawWorld();
		}
	}

	virtual void exitPhysics()
	{
		//cleanup in the reverse order of creation/initialization

		//remove the rigidbodies from the dynamics world and delete them

		if (m_dynamicsWorld)
		{
			int i;
			for (i = m_dynamicsWorld->getNumConstraints() - 1; i >= 0; i--)
			{
				m_dynamicsWorld->removeConstraint(m_dynamicsWorld->getConstraint(i));
			}
			for (i = m_dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--)
			{
				btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
				btRigidBody* body = btRigidBody::upcast(obj);
				if (body && body->getMotionState())
				{
					delete body->getMotionState();
				}
				m_dynamicsWorld->removeCollisionObject(obj);
				delete obj;
			}
		}
		//delete collision shapes
		for (int j = 0; j < m_collisionShapes.size(); j++)
		{
			btCollisionShape* shape = m_collisionShapes[j];
			delete shape;
		}
		m_collisionShapes.clear();

		delete m_dynamicsWorld;
		m_dynamicsWorld = 0;

		delete m_solver;
		m_solver = 0;

		delete m_broadphase;
		m_broadphase = 0;

		delete m_dispatcher;
		m_dispatcher = 0;

		delete m_collisionConfiguration;
		m_collisionConfiguration = 0;
	}

	virtual void debugDraw(int debugDrawFlags)
	{
		if (m_dynamicsWorld)
		{
			if (m_dynamicsWorld->getDebugDrawer())
			{
				m_dynamicsWorld->getDebugDrawer()->setDebugMode(debugDrawFlags);
			}
			m_dynamicsWorld->debugDrawWorld();
		}
	}

	virtual bool keyboardCallback(int key, int state)
	{
		if ((key == B3G_F3) && state && m_dynamicsWorld)
		{
			btDefaultSerializer* serializer = new btDefaultSerializer();
			m_dynamicsWorld->serialize(serializer);

			FILE* file = fopen("testFile.bullet", "wb");
			fwrite(serializer->getBufferPointer(), serializer->getCurrentBufferSize(), 1, file);
			fclose(file);
			//b3Printf("btDefaultSerializer wrote testFile.bullet");
			delete serializer;
			return true;
		}
		return false;  //don't handle this key
	}

	btRigidBody* createRigidBody(float mass, const btTransform& startTransform, btCollisionShape* shape, const btVector4& color = btVector4(1, 0, 0, 1))
	{
		btAssert((!shape || shape->getShapeType() != INVALID_SHAPE_PROXYTYPE));

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0, 0, 0);
		if (isDynamic)
			shape->calculateLocalInertia(mass, localInertia);

			//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects

#define USE_MOTIONSTATE 1
#ifdef USE_MOTIONSTATE
		btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);

		btRigidBody::btRigidBodyConstructionInfo cInfo(mass, myMotionState, shape, localInertia);

		btRigidBody* body = new btRigidBody(cInfo);
		//body->setContactProcessingThreshold(m_defaultContactProcessingThreshold);

#else
		btRigidBody* body = new btRigidBody(mass, 0, shape, localInertia);
		body->setWorldTransform(startTransform);
#endif  //

		body->setUserIndex(-1);
		m_dynamicsWorld->addRigidBody(body);
		return body;
	}
};

#endif  //COMMON_RIGID_BODY_SETUP_H
