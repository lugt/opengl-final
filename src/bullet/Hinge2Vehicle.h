/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2015 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef HINGE2_VEHICLE_H
#define HINGE2_VEHICLE_H

#include "Hinge2Vehicle.h"

#include "btBulletDynamicsCommon.h"
#include "BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h"

#include "BulletDynamics/MLCPSolvers/btDantzigSolver.h"
#include "BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h"
#include "BulletDynamics/MLCPSolvers/btMLCPSolver.h"

class btVehicleTuning;

class btCollisionShape;

#include "BulletDynamics/ConstraintSolver/btHingeConstraint.h"
#include "BulletDynamics/ConstraintSolver/btSliderConstraint.h"

#include "LinearMath/btAlignedObjectArray.h"
#include "btBulletCollisionCommon.h"
#include "../CommonInterfaces/CommonRigidBodyBase.h"

class Hinge2Vehicle : public CommonRigidBodyBase
{
public:
  /* extra stuff*/
  btVector3 m_cameraPosition;

  btRigidBody* m_carChassis;
  btRigidBody* localCreateRigidBody(btScalar mass, const btTransform& worldTransform, btCollisionShape* colSape);

  int m_wheelInstances[4];

  bool m_useDefaultCamera;
  //----------------------------

  class btTriangleIndexVertexArray* m_indexVertexArrays;

  btVector3* m_vertices;

  btCollisionShape* m_wheelShape;

  float m_cameraHeight;

  float m_minCameraDistance;
  float m_maxCameraDistance;

  Hinge2Vehicle();

  virtual ~Hinge2Vehicle();

  virtual void stepSimulation(float deltaTime);

  virtual void resetForklift();

  virtual void clientResetScene();

  virtual void displayCallback();

  virtual void specialKeyboard(int key, int x, int y);

  virtual void specialKeyboardUp(int key, int x, int y);

  virtual bool keyboardCallback(int key, int state);

  virtual void renderScene();

  virtual void physicsDebugDraw(int debugFlags);

  void initPhysics();
  void exitPhysics();

  virtual void resetCamera()
  {
    float dist = 8;
    float pitch = -32;
    float yaw = -45;
    float targetPos[3] = {0,0,2};
//		m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
  }

  /*static DemoApplication* Create()
  {
    Hinge2Vehicle* demo = new Hinge2Vehicle();
    demo->myinit();
    demo->initPhysics();
    return demo;
  }
  */
};

#endif  // HINGE2_VEHICLE_H
