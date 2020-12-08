/* Copyright (c) <2003-2019> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#ifndef __PHYSICS_UTIL__
#define __PHYSICS_UTIL__

#include <BallGame.h>

#define DEMO_GRAVITY  dFloat(-10.0f)
//#define DEMO_GRAVITY  dFloat(0.0f)

#ifdef DEMO_CHECK_ASYN_UPDATE
extern int g_checkAsyncUpdate;
#endif

enum PrimitiveType
{
	_NULL_PRIMITIVE,
	_SPHERE_PRIMITIVE,
	_BOX_PRIMITIVE,
	_CAPSULE_PRIMITIVE,
	_CYLINDER_PRIMITIVE,
	_CONE_PRIMITIVE,
	_CHAMFER_CYLINDER_PRIMITIVE,
	_RANDOM_CONVEX_HULL_PRIMITIVE,
	_REGULAR_CONVEX_HULL_PRIMITIVE,
	_COMPOUND_CONVEX_CRUZ_PRIMITIVE,
};

class MakeViualMesh : public dScene::dSceneExportCallback
{
	public:
	MakeViualMesh(NewtonWorld* const world);
	NewtonMesh* CreateVisualMesh(NewtonBody* const body, char* const name, int maxNameSize) const;
	NewtonWorld* m_world;
};

void ExportScene (NewtonWorld* const world, const char* const fileName);

//class DemoMesh;
//class DemoEntity;
//class DemoEntityManager;

/*
void InitEyePoint (const dVector& dir, const dVector& origin);
void SetShowIslands (SceneManager& me, int mode);
void SetShowContacts (SceneManager& me, int mode);
void SetShowMeshCollision (SceneManager& me, int mode);
void ShowBodyContacts (const NewtonBody* body);
void ShowJointInfo(const NewtonCustomJoint* joint);
void ConvexCastPlacement (NewtonBody* body);
void Keyboard(SceneManager& me);
void PhysicsSetTransform (const NewtonBody* body, const dFloat* matrix, int threadIndex);
int  PhysicsIslandUpdate (const NewtonWorld* world, const void* islandHandle, int bodyCount);
NewtonBody* CreateGenericSolid (NewtonWorld* world, SceneManager* scene, const char* meshName, dFloat mass, const dMatrix& matrix, const dVector& size, PrimitiveType type, int materialID);
int GenericContactProcess (const NewtonMaterial* material, const NewtonBody* const body0, const NewtonBody* const body1, dFloat timestep, int threadIndex);
int OnAABBOverlap (const NewtonMaterial* material, const NewtonBody* const body0, const NewtonBody* const body1, int threadIndex);
NewtonMesh* CreateCollisionTreeDoubleFaces (NewtonWorld* world, NewtonCollision* optimizedDoubelFacesTree);
*/



void GetContactOnBody (NewtonBody* const body);
void HandlecollisionPoints (NewtonJoint* const contactjoint);
NewtonJoint* CheckIfBodiesCollide (NewtonBody* const body0, NewtonBody* const body1);

dCustomJoint* FindJoint(const NewtonBody* const body0, const NewtonBody* const body1);
dVector FindFloor (const NewtonWorld* world, const dVector& origin, dFloat dist, dVector* const normal = NULL);

void PhysicsBodyDestructor (const NewtonBody* body);
void PhysicsApplyGravityForce (const NewtonBody* body, dFloat timestep, int threadIndex);

void SetAutoSleepMode (NewtonWorld* const world, int mode);
void CalculateAABB (const NewtonCollision* const collision, const dMatrix& matrix, dVector& minP, dVector& maxP);

void GenericContactProcess (const NewtonJoint* contactJoint, dFloat timestep, int threadIndex);

bool GetLastHit (dVector& posit, dVector& normal);
//NewtonCollision* CreateCollisionTree (NewtonWorld* const world, DemoEntity* const entity, int materialID, bool optimize);
NewtonCollision* CreateConvexCollision (NewtonWorld* const world, const dMatrix& offsetMatrix, const dVector& size, PrimitiveType type, int materialID);

//NewtonBody* CreatePLYMesh (DemoEntityManager* const scene, const char* const name, bool optimized);
//NewtonBody* CreateLevelMeshBody (NewtonWorld* const world, DemoEntity* const ent, bool optimize);
NewtonBody* CreateSimpleBody(NewtonWorld* const world, void* const userData, dFloat mass, const dMatrix& matrix, NewtonCollision* const collision, int materialId, bool generalInertia = false);
NewtonBody* CreateSimpleSolid (NewtonWorld* const world, dFloat mass, const dMatrix& matrix, NewtonCollision* const collision, int materialId, bool generalInertia = false);
//NewtonBody* CreateSimpleSolid (DemoEntityManager* const scene, DemoMesh* const mesh, dFloat mass, const dMatrix& matrix, NewtonCollision* const collision, int materialId, bool generalInertia = false);
//void AddPrimitiveArray (DemoEntityManager* const scene, dFloat mass, const dVector& origin, const dVector& size, int xCount, int zCount, dFloat spacing, PrimitiveType type, int materialID, const dMatrix& shapeOffsetMatrix, dFloat findFloorElevation = 1000.0f, dFloat offsetHigh = 5.0f);


NewtonCollision *ParseEntity (NewtonWorld *world, const MeshPtr mesh, const Matrix4& matrix);
bool DoBodiesCollide(NewtonWorld* const world, NewtonBody *BodyA, NewtonBody *BodyB);
//NewtonBody* AddCase(DemoEntityManager* const scene, const dVector& origin, const dVector& size, int materialID, const dMatrix& shapeOffsetMatrix);
NewtonBody* WorldAddCase(NewtonWorld* const world, const dVector& size, int materialID, const dMatrix& shapeOffsetMatrix, NewtonCollision *tree);
//NewtonBody* AddBall(DemoEntityManager* const scene, dFloat mass, const dVector& origin, const dVector& size, int materialID, const dMatrix& shapeOffsetMatrix, dFloat startElevation = 1000.0f, dFloat offsetHigh = 5.0f);
NewtonBody* WorldAddBall(NewtonWorld* const world, dFloat mass, const dVector& size, int materialID, const dMatrix& shapeOffsetMatrix);

//NewtonBody* CreateInstancedSolid(DemoEntityManager* const scene, DemoEntity* const parent, dFloat mass, const dMatrix& matrix, NewtonCollision* const collision, int materialId, bool generalInertia = false);

//NewtonBody* AddFloorBox(DemoEntityManager* const scene, const dVector& origin, const dVector& size);
NewtonBody* MousePickBody (NewtonWorld* const nWorld, const dVector& origin, const dVector& end, dFloat& paramter, dVector& positionOut, dVector& normalOut);

//void LoadLumberYardMesh(DemoEntityManager* const scene, const dVector& location, int shapeid);
//void SerializationWorld (const char* const name, NewtonWorld* const world);

#endif
