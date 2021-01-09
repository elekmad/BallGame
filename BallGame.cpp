/*
 * BallGame.cpp
 *
 *  Created on: 15 nov. 2020
 *      Author: damien
 */

#include <PhysicsUtils.h>
#include <vector>
#include <map>
#include <iostream>
#include <fstream>
#include <dNewtonScopeBuffer.h>
#include <Newton.h>
#include <OgreRay.h>
#include <CEGUI/PropertyHelper.h>
//#include <sys/types.h>
#include <glob.h>
#include <string.h>
//Put BallGame.h in last because of Xlib defines (True False Bool None) which must be undef
#include "BallGame.h"


#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)
#define LOG std::cout << __func__ <<  " (" << __FILENAME__ << "@" << __LINE__ << ") : "

inline CEGUI::String toCEGUIString(double val)
{
	char buff[64];
	snprintf(buff, sizeof(buff), "%.4f", val);

	return CEGUI::String(buff);
}

inline CEGUI::String toCEGUIString(float val)
{
	char buff[64];
	snprintf(buff, sizeof(buff), "%.4f", val);

	return CEGUI::String(buff);
}

void inline BuildLevelFilename(String &levelname, String &LevelFilename)
{
	LevelFilename = LEVELS_FOLDER;
	LevelFilename += levelname;
	LevelFilename += "." LEVELS_EXTENSION;
}

void inline BuildLevelStatsFilename(String &levelname, String &LevelFilename)
{
	LevelFilename = LEVELS_FOLDER;
	LevelFilename += levelname;
	LevelFilename += "." STATES_EXTENSION;
}

inline float Normalize(float v1, float v2, float v3)
{
	return sqrtf(v1 * v1 + v2 * v2 + v3 * v3);
}

inline double Normalize(double v1, double v2, double v3)
{
	return sqrt(v1 * v1 + v2 * v2 + v3 * v3);
}

static int OnBodyAABBOverlap(const NewtonJoint* const contactJoint, dFloat timestep, int threadIndex)
{
	return 1;
}

static int OnSubShapeAABBOverlapTest (const NewtonJoint* const contact, dFloat timestep, const NewtonBody* const body0, const void* const collsionNode0, const NewtonBody* const body1, const void* const collsionNode1, int threadIndex)
{
	return 1;
}

void EquilibrateAABBAroundOrigin(Node *node)
{
	Vector3 min(NAN, NAN, NAN), max(NAN, NAN, NAN);
	Node::ChildNodeIterator ite(node->getChildIterator());
	while ( ite.hasMoreElements() )
	{
	       SceneNode* child = static_cast<SceneNode*>(ite.getNext());
	       Vector3 childmin = child->_getWorldAABB().getMinimum();
	       Vector3 childmax = child->_getWorldAABB().getMaximum();

	       if(min.isNaN())
	    	   min = childmin;
	       if(max.isNaN())
	    	   max = childmax;

	       min.x = std::min<Ogre::Real>(min.x, childmin.x);
	       min.y = std::min<Ogre::Real>(min.y, childmin.y);
	       min.z = std::min<Ogre::Real>(min.z, childmin.z);

	       max.x = std::max<Ogre::Real>(max.x, childmax.x);
	       max.y = std::max<Ogre::Real>(max.y, childmax.y);
	       max.z = std::max<Ogre::Real>(max.z, childmax.z);
//	       LOG << "child min " << childmin.x << ", " << childmin.y << ", " << childmin.z << std::endl;
//	       LOG << "child max " << childmax.x << ", " << childmax.y << ", " << childmax.z << std::endl;
//	       LOG << "=> min " << min.x << ", " << min.y << ", " << min.z << std::endl;
//	       LOG << "=> max " << max.x << ", " << max.y << ", " << max.z << std::endl;
	}

//	Vector3 pos = node->getPosition();
//	LOG << "Node " << pos.x << ", " << pos.y << ", " << pos.z << std::endl;
	node->translate((min + max) / 2);
//	pos = node->getPosition();
//	LOG << "After Compute Node " << pos.x << ", " << pos.y << ", " << pos.z << std::endl;

	LOG << "After Equilibrate" << std::endl;
	Node::ChildNodeIterator ite2(node->getChildIterator());
	while ( ite2.hasMoreElements() )
	{
		   SceneNode* child = static_cast<SceneNode*>(ite2.getNext());
		   child->translate(-1 * (min + max) / 2);
//	       Vector3 childmin = child->_getWorldAABB().getMinimum();
//	       Vector3 childmax = child->_getWorldAABB().getMaximum();
//	       LOG << "child min " << childmin.x << ", " << childmin.y << ", " << childmin.z << std::endl;
//	       LOG << "child max " << childmax.x << ", " << childmax.y << ", " << childmax.z << std::endl;
	}
}



void BallGame::BodySerialization (NewtonBody* const body, void* const bodyUserData, NewtonSerializeCallback serializeCallback, void* const serializeHandle)
{
	BallGameEntity *Entity = (BallGameEntity*)NewtonBodyGetUserData(body);

	const char* const bodyIndentification = Entity->getName().c_str();
	LOG << "Serialize Entity (" << Entity << "/" << body << ") name :" << bodyIndentification << std::endl;
	int size = (strlen (bodyIndentification) + 3) & -4;
	serializeCallback (serializeHandle, &size, sizeof (size));
	serializeCallback (serializeHandle, bodyIndentification, size);
}

void BallGame::BodyDeserialization (NewtonBody* const body, void* const bodyUserData, NewtonDeserializeCallback deserializecallback, void* const serializeHandle)
{
	int size;
	char bodyIndentification[256];
	BallGame *Game = (BallGame*)bodyUserData;

	deserializecallback (serializeHandle, &size, sizeof (size));
	deserializecallback (serializeHandle, bodyIndentification, size);
	bodyIndentification[size] = 0;

	BallGameEntity *Entity = Game->GetEntity(bodyIndentification);
	LOG << "Deserialize Entity (" << Entity << "/" << body << ") name :" << bodyIndentification << std::endl;

	NewtonBodySetUserData (body, Entity);
	Entity->setNewtonBody(body);
	NewtonBodySetTransformCallback(body, BallGameEntity::TransformCallback);
	if(Entity->getType() == Ball)
		NewtonBodySetForceAndTorqueCallback(body, PhysicsAddForceAndGravity);
	else
		NewtonBodySetForceAndTorqueCallback(body, PhysicsApplyGravityForce);
	NewtonCollision* const collision = NewtonBodyGetCollision(body);

	#ifdef USE_STATIC_MESHES_DEBUG_COLLISION
	if (NewtonCollisionGetType(collision) == SERIALIZE_ID_TREE) {
		NewtonStaticCollisionSetDebugCallback (collision, ShowMeshCollidingFaces);
	}
	#endif
}

BallGameEntity *BallGame::GetEntity(char *name_c)
{
	String name(name_c);
	std::list<CaseEntity*>::iterator Citer(Cases.begin());
	while(Citer != Cases.end())
	{
		CaseEntity *Entity = *(Citer++);
		if(Entity != NULL && Entity->getName() == name)
			return (BallGameEntity*)Entity;
	}
	std::list<BallEntity*>::iterator Biter(Balls.begin());
	while(Biter != Balls.end())
	{
		BallEntity *Entity = *(Biter++);
		if(Entity != NULL && Entity->getName() == name)
			return (BallGameEntity*)Entity;
	}
	return NULL;
}

void BallGame::SerializedPhysicScene(const String* const name)
{
//	NewtonSerializeToFile(m_world, name, NULL, NULL);
	_StopPhysic();
	NewtonSerializeToFile(m_world, name->c_str(), BodySerialization, NULL);
	_StartPhysic();
}

void BallGame::DeserializedPhysicScene(const String* const name)
{
	LOG << "Load '" << (*name) << "'" << std::endl;
	_StopPhysic();
	std::list<CaseEntity*>::iterator Citer(Cases.begin());
	while(Citer != Cases.end())
	{
		CaseEntity *Entity = *(Citer++);
		if(Entity != NULL)
			Entity->setNewtonBody(NULL);
	}
	std::list<BallEntity*>::iterator Biter(Balls.begin());
	while(Biter != Balls.end())
	{
		BallEntity *Entity = *(Biter++);
		if(Entity == NULL)
			continue;
		Entity->setNewtonBody(NULL);
		Entity->CleanupForces();
	}
	NewtonDestroyAllBodies(m_world);
	NewtonDeserializeFromFile(m_world, name->c_str(), BodyDeserialization, this);
	_StartPhysic();
}


bool BallGame::SaveStatePushBCallback(const CEGUI::EventArgs &e)
{
	String *state_filename, state_name = Level;
	state_name += "-";
	state_name += std::to_string(ChooseStateToLoadB->getItemCount());
	state_filename = new String;
	BuildLevelStatsFilename(state_name, *state_filename);
	SerializedPhysicScene(state_filename);
	CEGUI::ListboxTextItem *item = (CEGUI::ListboxTextItem*)new CEGUI::ListboxTextItem(state_name);
	item->setUserData(state_filename);
	ChooseStateToLoadB->addItem(item);
	ChooseStateToLoadB->setSize(CEGUI::USize(CEGUI::UDim(0, 150), CEGUI::UDim(0, 40 * (ChooseStateToLoadB->getItemCount() + 1))));
	return true;
}

bool BallGame::LoadStatePushBCallback(const CEGUI::EventArgs &e)
{
	CEGUI::ListboxItem *item = ChooseStateToLoadB->getSelectedItem();
	if(item == NULL)
		return true;
	String *state_filename = (String*)item->getUserData();
	if(state_filename != NULL && state_filename->empty() == false)
		DeserializedPhysicScene(state_filename);
	return true;
}

bool BallGame::DelStatePushBCallback(const CEGUI::EventArgs &e)
{
	CEGUI::ListboxItem *item = ChooseStateToLoadB->getSelectedItem();
	if(item == NULL)
		return true;
	String *state_filename = (String*)item->getUserData();
	unlink(state_filename->c_str());
	ChooseStateToLoadB->setItemSelectState(item, false);
	ChooseStateToLoadB->removeItem(item);
	return true;
}

bool BallGame::ChooseStateToLoadBCallback(const CEGUI::EventArgs &e)
{
	SetupStatesButtons();
	return true;
}

void BallGame::SetupStatesButtons(void)
{
	LOG << "Change selected state" << std::endl;
	if(ChooseStateToLoadB->getSelectedItem() != NULL)
	{
		LoadStatePushB->setEnabled(true);
		DelStatePushB->setEnabled(true);
	}
	else
	{
		LoadStatePushB->setEnabled(false);
		DelStatePushB->setEnabled(false);
	}
}

BallGameEntity::BallGameEntity(const dMatrix& matrix) :// m_matrix(matrix),
	m_curPosition (matrix.m_posit),
	m_nextPosition (matrix.m_posit),
	m_curRotation (dQuaternion (matrix)),
	m_nextRotation (dQuaternion (matrix))
{
	OgreEntity = NULL;
	Group = NULL;
	Body = NULL;
	type = Case;
}

BallGameEntity::BallGameEntity()
{
	OgreEntity = NULL;
	Body = NULL;
	Group = NULL;
	type = Case;
}

void BallGameEntity::DisplaySelectedBox(bool display)
{
	OgreEntity->showBoundingBox(display);
}

void BallGameEntity::Finalize(void)
{
	NewtonDestroyBody(Body);
	LOG << "Remove Ogre " << OgreEntity->getName() << std::endl;
	SceneNode *parent = (SceneNode*)OgreEntity->getParent();
	parent->removeAndDestroyChild(OgreEntity->getName());
}

void BallGameEntity::setOgreNode(SceneNode *node)
{
	OgreEntity = node;
	if(node != NULL)
	{
		//No need to use derivated functions here, because group will be attached later !
		node->setPosition(InitialPos);
		node->setScale(InitialScale);
		node->setOrientation(InitialOrientation);
		((Ogre::Entity*)node->getAttachedObject(0))->getUserObjectBindings().setUserAny(Ogre::Any(this));
	}
}

void BallGameEntity::setNewtonBody(NewtonBody *body)
{
	if(Body != NULL)
		NewtonDestroyBody(Body);
	Body = body;
	if(body != NULL)
	{
		dMatrix matrix;
		NewtonBodySetUserData(body, this);
		NewtonBodyGetMatrix(body, &matrix[0][0]);
		m_curPosition = matrix.m_posit;
		m_nextPosition = matrix.m_posit;
		m_curRotation = dQuaternion(matrix);
		m_nextRotation = dQuaternion(matrix);
	}
}

void BallGameEntity::SetMatrixUsafe(const dQuaternion& rotation, const dVector& position)
{
	m_curPosition = m_nextPosition;
	m_curRotation = m_nextRotation;

	m_nextPosition = position;
	m_nextRotation = rotation;

	dFloat angle = m_curRotation.DotProduct(m_nextRotation);
	if (angle < 0.0f) {
		m_curRotation.Scale(-1.0f);
		LOG << "angle < 0 : " << angle << std::endl;
	}
}

void BallGameEntity::TransformCallback(const NewtonBody* body, const dFloat* matrix, int threadIndex)
{
//	LOG << "TransformCallback" << std::endl;
	BallGameEntity* const Entity = (BallGameEntity*) NewtonBodyGetUserData(body);
	if (Entity)
	{
		BallGame* const scene = (BallGame*)NewtonWorldGetUserData(NewtonBodyGetWorld(body));
		dMatrix transform(matrix);
		dQuaternion rot;
		NewtonBodyGetRotation(body, &rot.m_x);
		Entity->SetMatrixUsafe(rot, transform.m_posit);


		Vector3 NewPosition(Entity->m_curPosition.m_x, Entity->m_curPosition.m_y, Entity->m_curPosition.m_z);
		Quaternion NewOrientation(Entity->m_curRotation.m_w, Entity->m_curRotation.m_x, Entity->m_curRotation.m_y, Entity->m_curRotation.m_z);

		//scene->Lock(Entity->m_lock);
		//scene->Unlock(Entity->m_lock);
//		LOG << "Entity transform " << "position {" << NewPosition.x << ", " << NewPosition.y << ", " << NewPosition.z << "}";
//		LOG << " Orientation {" << NewOrientation.w << ", " << NewOrientation.x << ", " << NewOrientation.y << ", " << NewOrientation.z << "}" << std::endl;
		Entity->OgreEntity->_setDerivedPosition(NewPosition);
		Entity->OgreEntity->_setDerivedOrientation(NewOrientation);
	}
}

dMatrix * BallGameEntity::PrepareNewtonBody(dVector &NewtonBodyLocation, dVector &NewtonBodySize)
{
	InitialPos = OgreEntity->_getDerivedPosition();
	InitialScale = OgreEntity->_getDerivedScale();
	InitialOrientation = OgreEntity->_getDerivedOrientation();
	NewtonBodyLocation.m_x = InitialPos.x;
	NewtonBodyLocation.m_y = InitialPos.y;
	NewtonBodyLocation.m_z = InitialPos.z;
	NewtonBodyLocation.m_w = 1;
	Entity *ogreEntity = (Ogre::Entity*)OgreEntity->getAttachedObject(0);
	Vector3 AABB(ogreEntity->getBoundingBox().getSize());
	NewtonBodySize.m_x = AABB.x * InitialScale.x;
	NewtonBodySize.m_y = AABB.y * InitialScale.y;
	NewtonBodySize.m_z = AABB.z * InitialScale.z;
	NewtonBodySize.m_w = 0.0f;

	return new dMatrix(InitialOrientation.getPitch(false).valueRadians(), InitialOrientation.getYaw(false).valueRadians(), InitialOrientation.getRoll(false).valueRadians(), NewtonBodyLocation);
}

BallEntity::BallEntity(const dMatrix& matrix):BallGameEntity(matrix)
{
	InitialMass = 1;
	type = Ball;
}

BallEntity::BallEntity()
{
	InitialMass = 1;
	type = Ball;
}

void BallEntity::AddForceVector(dVector *force)
{
//	LOG << "Add Force {" << (*force)[0] << ", " << (*force)[1] << ", " << (*force)[2] << "} On ball" << std::endl;
	Forces.Append(force);
}

void BallEntity::CleanupForces(void)
{
	dVector *f;
	do
	{
		f = GetForceVector();
		if(f != NULL)
			delete f;
	}
	while(f != NULL);
}

dVector *BallEntity::GetForceVector()
{
	dVector *ret = NULL;
	dList<dVector*>::dListNode *node = Forces.GetFirst();
	if(node != NULL)
	{
		ret = node->GetInfo();
		Forces.Remove(node);
	}
	return ret;
}

void BallEntity::CreateNewtonBody(NewtonWorld *m_world)
{
	dVector NewtonBodyLocation;
	dVector NewtonBodySize;
	NewtonBody *newtonBody;
	dMatrix *bodymatrix = PrepareNewtonBody(NewtonBodyLocation, NewtonBodySize);

	LOG << "Place a Ball" << std::endl;

	int defaultMaterialID = NewtonMaterialGetDefaultGroupID (m_world);
	newtonBody = WorldAddBall(m_world, ((BallEntity*)this)->InitialMass, NewtonBodySize, defaultMaterialID, *bodymatrix);
	setNewtonBody(newtonBody);

	delete bodymatrix;
}

dFloat BallEntity::getMass(void)
{
	dFloat inertx, inerty, inertz, ret;
	NewtonBodyGetMass(Body, &ret, &inertx, &inerty, &inertz);
	return ret;
}

void BallEntity::setMass(dFloat newMass)
{
	LOG << "Ball new mass : " << this << std::endl;
	NewtonCollision *collision = NewtonBodyGetCollision(Body);
	NewtonBodySetMassProperties(Body, newMass, collision);
}

CaseEntity::CaseEntity(const dMatrix& matrix, enum CaseType _type):BallGameEntity(matrix)
{
	type = _type;
	this->BallGameEntity::type = Case;
	force_to_apply = NAN;
	force_direction = NULL;
}

CaseEntity::CaseEntity(enum CaseType _type)
{
	type = _type;
	this->BallGameEntity::type = Case;
	force_to_apply = NAN;
	force_direction = NULL;
}

void CaseEntity::SetForceToApply(float force, dVector *direction)
{
	force_to_apply = force;
	if(force_direction != NULL)
		delete force_direction;
	force_direction = direction;
}


void CaseEntity::AddBallColliding(BallEntity *ball)
{
	if(ball == NULL)
		return;
	if(CheckIfAlreadyColliding(ball) == false)
		BallsUnderCollide.push_back(ball);
}

bool CaseEntity::CheckIfAlreadyColliding(BallEntity *ToCheck)
{
	bool ret = false;
	if(ToCheck == NULL)
		return ret;
	std::list<BallEntity*>::iterator iter(BallsUnderCollide.begin());
	while(iter != BallsUnderCollide.end())
	{
		BallEntity *Ball = *(iter++);
		if(Ball == ToCheck)
			return true;
	}
	return ret;
}

void CaseEntity::ApplyForceOnCollidingBalls(void)
{
	std::list<BallEntity*>::iterator iter(BallsUnderCollide.begin());
	while(iter != BallsUnderCollide.end())
	{
		BallEntity *Ball = *iter;
		if(Ball != NULL)
			ApplyForceOnBall(Ball);
		iter = BallsUnderCollide.erase(iter);
	}
}

void CaseEntity::ApplyForceOnBall(BallEntity *ball)
{
//	LOG << "Case " << this << " apply force on ball " << ball << std::endl;
	if(!isnanf(force_to_apply))
	{
		if(force_direction != NULL)
		{
			dVector *force = new dVector();
			*force = force_direction->Scale(force_to_apply);
			ball->AddForceVector(force);
		}
		else
		{
			dFloat velocityf[3];
			double Velocity[3], sum;
			ball->getVelocity(velocityf);
			Velocity[0] = (double)velocityf[0];
			Velocity[1] = (double)velocityf[1];
			Velocity[2] = (double)velocityf[2];
			sum = Normalize(Velocity[0], Velocity[1], Velocity[2]);
			Velocity[0] /= sum;
			Velocity[1] /= sum;
			Velocity[2] /= sum;//Like that we have normalization of velocity into percents, we can use it to scale force.
			dVector *force = new dVector(Velocity[0], Velocity[1], Velocity[2]);
			*force = force->Scale(force_to_apply);
			ball->AddForceVector(force);
		}
	}
	//AddBallColliding(ball);
}

void CaseEntity::CreateNewtonBody(NewtonWorld *m_world)
{
	dVector NewtonBodyLocation;
	dVector NewtonBodySize;
	NewtonBody *newtonBody;

	Entity *ogreEntity = (Ogre::Entity*)OgreEntity->getAttachedObject(0);
	dMatrix *bodymatrix = PrepareNewtonBody(NewtonBodyLocation, NewtonBodySize);

	NewtonCollision *collision_tree = NULL;
	Matrix4 ogre_matrix;
	if(type == CaseEntity::CaseType::typeRamp)
		LOG << "Place a Ramp" << std::endl;
	else
		LOG << "Place a Box" << std::endl;

	ogre_matrix.makeTransform(Vector3::ZERO, InitialScale, Quaternion::IDENTITY);
	const MeshPtr ptr = ogreEntity->getMesh();
	collision_tree = ParseEntity(m_world, ptr, ogre_matrix);

	int defaultMaterialID = NewtonMaterialGetDefaultGroupID (m_world);
	newtonBody = WorldAddCase(m_world, NewtonBodySize, defaultMaterialID, *bodymatrix, collision_tree);
	setNewtonBody(newtonBody);
}


NewtonWorld* BallGame::GetNewton(void)
{
	return m_world;
}

void BallGame::SetCam(float x, float y, float z)
{
    //camNode->translate(camx, camx, camz, Ogre::Node::TransformSpace::TS_LOCAL);
    mCamera->setPosition(x, y, z);
}

void BallGame::MoveCam(float x, float y, float z)
{
	mCamera->moveRelative(Ogre::Vector3(x, y, z));
}


BallGame::BallGame() :
		m_asynchronousPhysicsUpdate(false)
		,m_suspendPhysicsUpdate(false)
		,m_physicsFramesCount(0)
		,m_microsecunds(0)
		,m_mainThreadPhysicsTime(0.0f)
		,m_mainThreadPhysicsTimeAcc(0.0f)
{
	mRenderer = NULL;

	m_world = NULL;
	mWindow = NULL;
	LastHighligted = NULL;
	UnderEditCase = NULL;
	ForcesArrows = NULL;
	UnderEditBall = NULL;
	ToBePlacedEntity = NULL;
	LastPlacedEntity = NULL;
	ToBePlacedEntityType = NULL;
	ToBeDeletedEntity = NULL;
	ogreThumbnailNode = NULL;
	PlacementMode = PlaceMove;
	MultiSelectionMode = false;
	mode = Running;
	MouseOverButton = false;
	// create the newton world
	SetupNewton();
	nb_entities = 0;
}

void BallGame::SetupNewton(void)
{
	m_world = NewtonCreate();

	// link the work with this user data
	NewtonWorldSetUserData(m_world, this);

	// set a post update callback which is call after all simulation and all listeners updates
	NewtonSetPostUpdateCallback (m_world, PostUpdateCallback);
	NewtonSetNumberOfSubsteps (m_world, MAX_PHYSICS_SUB_STEPS);

	// register contact creation destruction callbacks
	//NewtonWorldSetCreateDestroyContactCallback(m_world, OnCreateContact, OnDestroyContact);
	NewtonLoadPlugins(m_world, "newtonPlugins");


	int defaultMaterialID = NewtonMaterialGetDefaultGroupID (m_world);
	NewtonMaterialSetCollisionCallback (m_world, defaultMaterialID, defaultMaterialID, OnBodyAABBOverlap, OnContactCollision);
	NewtonMaterialSetCompoundCollisionCallback(m_world, defaultMaterialID, defaultMaterialID, OnSubShapeAABBOverlapTest);
}

GroupEntity::GroupEntity(String &name, Ogre::SceneManager* mSceneMgr)
{
	OgreEntity = (SceneNode*)mSceneMgr->getRootSceneNode()->createChild(name);
}

void GroupEntity::Finalize(void)
{
	std::list<BallGameEntity*>::iterator iter(childs.begin());
	while(iter != childs.end())
	{
		BallGameEntity *Entity = *iter;
		if(Entity != NULL)
		{
			Entity->OgreEntity->getParent()->removeChild(Entity->OgreEntity);
			OgreEntity->getParent()->addChild(Entity->OgreEntity);
		}
		iter = childs.erase(iter);
	}
	SceneNode *parent = (SceneNode*)OgreEntity->getParent();
	parent->removeAndDestroyChild(OgreEntity->getName());
}

void GroupEntity::AddChild(BallGameEntity* child)
{
	childs.push_back(child);
	child->Group = this;
}

bool GroupEntity::DelChild(BallGameEntity* child)
{
	LOG << "Child " << child << " Removed from Group" << std::endl;
	child->Group = NULL;
	Quaternion ChildOrientation = child->getAbsoluteOrientation();
	child->OgreEntity->setOrientation(ChildOrientation);
	Vector3 ChildPosition = child->getAbsolutePosition();
	child->OgreEntity->setPosition(ChildPosition);
	Vector3 ChildScale = child->getAbsoluteScale();
	child->OgreEntity->setScale(ChildScale);
	std::list<BallGameEntity*>::iterator iter(childs.begin());
	while(iter != childs.end())
	{
		BallGameEntity *Entity = *iter;
		if(Entity == child)
		{
			child->OgreEntity->getParent()->removeChild(child->OgreEntity);
			OgreEntity->getParent()->addChild(child->OgreEntity);
			childs.erase(iter);
			break;
		}
		iter++;
	}
	return childs.empty();
}

void GroupEntity::FillListWithChilds(std::list<BallGameEntity*> &list)
{
	std::list<BallGameEntity*>::iterator iter(childs.begin());
	while(iter != childs.end())
	{
		BallGameEntity *Entity = *(iter++);
		if(Entity == NULL)
			continue;
		list.push_back(Entity);
	}
}

void GroupEntity::ComputeChilds(void)
{
	std::list<BallGameEntity*>::iterator iter(childs.begin());
	while(iter != childs.end())
	{
		BallGameEntity *Entity = *(iter++);
		if(Entity == NULL)
			continue;
		LOG << "Child " << Entity << " Added to Group" << std::endl;
		Entity->OgreEntity->getParentSceneNode()->removeChild(Entity->OgreEntity);
		OgreEntity->addChild(Entity->OgreEntity);
	}
}

void GroupEntity::ComputeAndEquilibrateChilds(void)
{
	ComputeChilds();
	EquilibrateAABBAroundOrigin((Node*)OgreEntity);
}

void GroupEntity::ExportToJson(rapidjson::Value &v, rapidjson::Document::AllocatorType& allocator)
{
	rapidjson::Value name;
	const char *ogrename = (const char*)OgreEntity->getName().c_str();
	name.SetString(ogrename, allocator);
	v.AddMember("NodeName", name, allocator);
	const Vector3 &InitialPos = OgreEntity->getPosition();
	const Vector3 &InitialScale = OgreEntity->getScale();
	const Quaternion &InitialOrientation = OgreEntity->getOrientation();

	v.AddMember("PosX", InitialPos.x, allocator);
	v.AddMember("PosY", InitialPos.y, allocator);
	v.AddMember("PosZ", InitialPos.z, allocator);
	v.AddMember("ScaleX", InitialScale.x, allocator);
	v.AddMember("ScaleY", InitialScale.y, allocator);
	v.AddMember("ScaleZ", InitialScale.z, allocator);
	v.AddMember("OrientationX", InitialOrientation.x, allocator);
	v.AddMember("OrientationY", InitialOrientation.y, allocator);
	v.AddMember("OrientationZ", InitialOrientation.z, allocator);
	v.AddMember("OrientationW", InitialOrientation.w, allocator);
}

void GroupEntity::ImportFromJson(rapidjson::Value &v, BallGame *Game)
{
	Ogre::SceneManager *mSceneMgr = Game->getSceneManager();
	Vector3 InitialPos, InitialScale;
	Quaternion InitialOrientation;
	InitialPos.x = v["PosX"].GetFloat();
	InitialPos.y = v["PosY"].GetFloat();
	InitialPos.z = v["PosZ"].GetFloat();
	InitialScale.x = v["ScaleX"].GetFloat();
	InitialScale.y = v["ScaleY"].GetFloat();
	InitialScale.z = v["ScaleZ"].GetFloat();
	InitialOrientation.x = v["OrientationX"].GetFloat();
	InitialOrientation.y = v["OrientationY"].GetFloat();
	InitialOrientation.z = v["OrientationZ"].GetFloat();
	InitialOrientation.w = v["OrientationW"].GetFloat();

	const char *nodename = v["NodeName"].GetString();
	OgreEntity = mSceneMgr->getRootSceneNode()->createChildSceneNode(nodename, InitialPos);
	OgreEntity->setPosition(InitialPos);
	OgreEntity->setScale(InitialScale);
	OgreEntity->setOrientation(InitialOrientation);
}

BallGame::~BallGame()
{
	EmptyLevel();
	EmptyLevelsList();
	if(m_world != NULL)
		NewtonDestroy(m_world);
	if(mRenderer != NULL)
		CEGUI::OgreRenderer::destroySystem();
	std::list<class EntityType*>::iterator iter(EntityTypes.begin());
	while(iter != EntityTypes.end())
	{
		EntityType *type = *iter;
		if(type != NULL)
			delete type;
		iter = EntityTypes.erase(iter);
	}
}

void BallGame::PostUpdateCallback(const NewtonWorld* const world, dFloat timestep)
{
//	LOG << "Post Update Time " << timestep << std::endl;
/*	BallGame* const scene = (BallGame*) NewtonWorldGetUserData(world);
	scene->m_cameraManager->FixUpdate(scene->GetNewton(), timestep);
	if (scene->m_updateCamera) {
		scene->m_updateCamera(scene, scene->m_updateCameraContext, timestep);
	}*/
}

void BallGame::UpdatePhysics(dFloat timestep)
{
	// update the physics
//	LOG << " Update Time " << timestep << std::endl;
	if (m_world && !m_suspendPhysicsUpdate) {
		D_TRACKTIME();

		dFloat timestepInSecunds = 1.0f / MAX_PHYSICS_FPS;
		unsigned64 timestepMicrosecunds = unsigned64 (timestepInSecunds * 1000000.0f);

		unsigned64 currentTime = dGetTimeInMicrosenconds ();
		unsigned64 nextTime = currentTime - m_microsecunds;
		if (nextTime > timestepMicrosecunds * 2) {
			m_microsecunds = currentTime - timestepMicrosecunds * 2;
			nextTime = currentTime - m_microsecunds;
		}

		bool newUpdate = false;
		dFloat physicsTime = 0.0f;
		//while (nextTime >= timestepMicrosecunds)
		if (nextTime >= timestepMicrosecunds)
		{
			newUpdate = true;
			ClearDebugDisplay(m_world);

			CheckforCollides();

#ifdef DEMO_CHECK_ASYN_UPDATE
			g_checkAsyncUpdate = 1;
#endif
			if (m_asynchronousPhysicsUpdate) {
				NewtonUpdateAsync(m_world, timestepInSecunds);
#ifdef DEMO_CHECK_ASYN_UPDATE
				NewtonWaitForUpdateToFinish(m_world);
				g_checkAsyncUpdate = 0;
#endif
			} else {
				NewtonUpdate(m_world, timestepInSecunds);
			}

			physicsTime += NewtonGetLastUpdateTime(m_world);

			nextTime -= timestepMicrosecunds;
			m_microsecunds += timestepMicrosecunds;
		}

		if (newUpdate) {
			m_physicsFramesCount ++;
			m_mainThreadPhysicsTimeAcc += physicsTime;
			if (m_physicsFramesCount >= 16) {
				m_mainThreadPhysicsTime = m_mainThreadPhysicsTimeAcc / m_physicsFramesCount;
				m_physicsFramesCount = 0;
				m_mainThreadPhysicsTimeAcc = 0.0f;
			}

		}

//dTrace (("%f\n", m_mainThreadPhysicsTime));
	}
}

bool BallGame::EnteringArea(const CEGUI::EventArgs &event)
{
//	LOG << "Enter Button Area" << std::endl;
	MouseOverButton = true;
	return true;
}

bool BallGame::LeavingArea(const CEGUI::EventArgs &event)
{
//	LOG << "Leave Button Area" << std::endl;
	MouseOverButton = false;
	return true;
}

void BallGame::LoadBallGameEntityTypes(void)
{
    glob_t glob_result;
	memset(&glob_result, 0, sizeof(glob_result));
	glob("Elements/*.json", 0, NULL, &glob_result);
	for(size_t i = 0; i < glob_result.gl_pathc; ++i)
	{
		String globname(glob_result.gl_pathv[i]), name;
		size_t slashpos = globname.find_last_of('/'), dotpos = globname.find_last_of('.');
		name = globname.substr(slashpos + 1, dotpos - (slashpos + 1));

		std::ifstream myfile;
		std::stringstream buffer;
		myfile.open (globname.c_str());
		buffer << myfile.rdbuf();
		myfile.close();
		rapidjson::Document in;
		in.Parse(buffer.str().c_str());

		EntityType *type = new EntityType;
		type->Name = name;
		type->Type = strcmp(in["Type"].GetString(), "Case") == 0 ? Case : Ball;
		type->MeshName = in["Mesh"].GetString();
		type->InitialPos.x = in["InitialPosX"].GetFloat();
		type->InitialPos.y = in["InitialPosY"].GetFloat();
		type->InitialPos.z = in["InitialPosZ"].GetFloat();
		type->InitialScale.x = in["InitialScaleX"].GetFloat();
		type->InitialScale.y = in["InitialScaleY"].GetFloat();
		type->InitialScale.z = in["InitialScaleZ"].GetFloat();
		type->InitialOrientation.x = in["InitialOrientationX"].GetFloat();
		type->InitialOrientation.y = in["InitialOrientationY"].GetFloat();
		type->InitialOrientation.z = in["InitialOrientationZ"].GetFloat();
		type->InitialOrientation.w = in["InitialOrientationW"].GetFloat();
		type->InitialMass = in["InitialMass"].GetFloat();

		EntityTypes.push_back(type);
	}
}

void BallGame::createScene(void)
{
	LoadBallGameEntityTypes();

	SetupGUI();

	SetupGame();
}

bool BallGame::CaseForceValueEditBCallback(const CEGUI::EventArgs &event)
{
	UnderEditCaseForce = CEGUI::PropertyHelper<float>::fromString(CaseForceValueEditB->getText());
	return true;
}

bool BallGame::CaseForceDirectionXValueEditBMouseWheelCallback(const CEGUI::EventArgs &e)
{
	CEGUI::MouseEventArgs &event = (CEGUI::MouseEventArgs &)e;

	LOG << "Mouse Wheel " << event.wheelChange << std::endl;
	double force = CEGUI::PropertyHelper<double>::fromString(CaseForceDirectionXValueEditB->getText());
	if(isnan(force) == false)
	{
		force += 0.1 * event.wheelChange / 120.0;
		LOG << " = force " << force << std::endl;
		CaseForceDirectionXValueEditB->setText(toCEGUIString(force));
		NormalizeForceDirection();
	}
	return true;
}

bool BallGame::CaseForceDirectionYValueEditBMouseWheelCallback(const CEGUI::EventArgs &e)
{
	CEGUI::MouseEventArgs &event = (CEGUI::MouseEventArgs &)e;

	LOG << "Mouse Wheel " << event.wheelChange << std::endl;
	double force = CEGUI::PropertyHelper<double>::fromString(CaseForceDirectionYValueEditB->getText());
	if(isnan(force) == false)
	{
		force += 0.1 * event.wheelChange / 120.0;
		LOG << " = force " << force << std::endl;
		CaseForceDirectionYValueEditB->setText(toCEGUIString(force));
		NormalizeForceDirection();
	}
	return true;
}

bool BallGame::CaseForceDirectionZValueEditBMouseWheelCallback(const CEGUI::EventArgs &e)
{
	CEGUI::MouseEventArgs &event = (CEGUI::MouseEventArgs &)e;

	LOG << "Mouse Wheel " << event.wheelChange << std::endl;
	double force = CEGUI::PropertyHelper<double>::fromString(CaseForceDirectionZValueEditB->getText());
	if(isnan(force) == false)
	{
		force += 0.1 * event.wheelChange / 120.0;
		LOG << " = force " << force << std::endl;
		CaseForceDirectionZValueEditB->setText(toCEGUIString(force));
		NormalizeForceDirection();
	}
	return true;
}

void BallGame::NormalizeForceDirection(void)
{
	double direction_x, direction_y, direction_z, direction;
	direction_x = CEGUI::PropertyHelper<double>::fromString(CaseForceDirectionXValueEditB->getText());
	direction_y = CEGUI::PropertyHelper<double>::fromString(CaseForceDirectionYValueEditB->getText());
	direction_z = CEGUI::PropertyHelper<double>::fromString(CaseForceDirectionZValueEditB->getText());

	direction = Normalize(direction_x, direction_y, direction_z);
	LOG << "Before Normalization : " << direction << ", " << direction_x << ", " << direction_y << ", " << direction_z << std::endl;
	direction_x /= direction;
	direction_y /= direction;
	direction_z /= direction;
	LOG << "After Normalization : " << direction << ", " << direction_x << ", " << direction_y << ", " << direction_z << std::endl;

	CaseForceDirectionXValueEditB->setText(toCEGUIString(direction_x));
	CaseForceDirectionYValueEditB->setText(toCEGUIString(direction_y));
	CaseForceDirectionZValueEditB->setText(toCEGUIString(direction_z));

	//Careful, musn't use directly force_diretion vector, has it is float not double !
	force_direction.m_x = direction_x;
	force_direction.m_y = direction_y;
	force_direction.m_z = direction_z;
	UpdateForceArrows();
}

bool BallGame::NormalizeCaseForceDirectionPushBCallback(const CEGUI::EventArgs &e)
{
	NormalizeForceDirection();
	return true;
}

bool BallGame::QuitPushBCallback(const CEGUI::EventArgs &e)
{
	mRoot->queueEndRendering();
    return true;
}

void BallGame::_StartPhysic(void)
{
	StopPhysicPushB->setText("Stop Physic");
	m_suspendPhysicsUpdate = false;
}

void BallGame::_StopPhysic(void)
{
	StopPhysicPushB->setText("Start Physic");
	m_suspendPhysicsUpdate = true;
	NewtonWaitForUpdateToFinish(m_world);
}

bool BallGame::StopPhysicPushBCallback(const CEGUI::EventArgs &e)
{
	if(m_suspendPhysicsUpdate)
		_StartPhysic();
	else
		_StopPhysic();
    return true;
}

void BallGame::SwitchEditMode(void)
{
	if(mode == Running)
	{
		LOG << "Edit Mode" << std::endl;
		mode = Editing;
		_StopPhysic();
		EditingModeTitleBanner->setVisible(true);
		AddElementTitleBanner->setVisible(true);
		ChooseTypeOfElementToAddB->setVisible(true);
		PlaceNewElementB->setVisible(true);
		EditElementB->setVisible(true);
		DeleteElementB->setVisible(true);
		MoveElementB->setVisible(true);
		RotateElementB->setVisible(true);
		ScaleElementB->setVisible(true);
	    ThumbnailWindow->setVisible(true);
	    GroupElementsB->setVisible(false);
		SetMoveNewElement();
	}
	else
	{
		LOG << "Running Mode" << std::endl;
		if(LastHighligted != NULL)
		{
			LastHighligted->DisplaySelectedBox(false);
			LastHighligted = NULL;
		}
		LastPlacedEntity = NULL;
		UnprepareNewElement();
		UnprepareDeleteElement();
		EditBall(NULL);
		EditCase(NULL);
		MultiSelectionSetEmpty();
		mode = Running;
		EditingModeTitleBanner->setVisible(false);
		ApplyForceChangesToCasePushB->setVisible(false);
		CaseHasForceToggleB->setVisible(false);
		CaseForceValueEditB->setVisible(false);
		CaseHasForceDirectionToggleB->setVisible(false);
		CaseForceDirectionXValueEditB->setVisible(false);
		CaseForceDirectionYValueEditB->setVisible(false);
		CaseForceDirectionZValueEditB->setVisible(false);
		NormalizeCaseForceDirectionPushB->setVisible(false);
		AddElementTitleBanner->setVisible(false);
		ChooseTypeOfElementToAddB->setVisible(false);
		PlaceNewElementB->setVisible(false);
		EditElementB->setVisible(false);
		DeleteElementB->setVisible(false);
		MoveElementB->setVisible(false);
		RotateElementB->setVisible(false);
		ScaleElementB->setVisible(false);
	    ThumbnailWindow->setVisible(false);
	    GroupElementsB->setVisible(false);
	}
}

void BallGame::CreateThumbnail(String meshname)
{
	Entity *ogreEntity = mThumbnailSceneMgr->createEntity(meshname);
	mThumbnailSceneMgr->getRootSceneNode()->removeAndDestroyAllChildren();
	ogreThumbnailNode = mThumbnailSceneMgr->getRootSceneNode()->createChildSceneNode();
	ogreThumbnailNode->attachObject(ogreEntity);
	ogreThumbnailNode->scale(130, 130, 130);
	ogreThumbnailNode->setPosition(270, 130, -30);
}

bool BallGame::ChooseTypeOfElementToAddBCallback(const CEGUI::EventArgs &e)
{
	String ElementType;
	ToBePlacedEntityType = NULL;
	ElementType = ChooseTypeOfElementToAddB->getSelectedItem()->getText().c_str();

	std::list<class EntityType*>::iterator iter(EntityTypes.begin());
	while(iter != EntityTypes.end())
	{
		EntityType *type = *iter;
		if(type != NULL && type->Name == ElementType)
		{
			ToBePlacedEntityType = type;
			break;
		}
		iter++;
	}
	if(ToBePlacedEntityType != NULL)
		CreateThumbnail(ToBePlacedEntityType->MeshName);
	UnprepareNewElement();
	LastPlacedEntity = NULL;
	PrepareNewElement();
	return true;
}

void BallGame::DeleteElement(void)
{
	if(PlacementMode != Delete)
		return;
	if(ToBeDeletedEntity != NULL)
	{
		LOG << "Delete Entity : " << ToBeDeletedEntity->getName() << std::endl;
		if(ToBeDeletedEntity == LastHighligted)
			LastHighligted = NULL;
		switch(ToBeDeletedEntity->getType())
		{
		case Ball :
			DeleteBall((BallEntity*)ToBeDeletedEntity);
			break;
		case Case :
			DeleteCase((CaseEntity*)ToBeDeletedEntity);
			break;
		}
		ToBeDeletedEntity = NULL;
	}
}

bool BallGame::DeleteElementBCallback(const CEGUI::EventArgs &e)
{
	switch(PlacementMode)
	{
	case EditMove :
	case EditRotate :
	case EditScale :
		MultiSelectionMode = false;
		MultiSelectionSetEmpty();
		EditBall(NULL);
		EditCase(NULL);
		break;
	case PlaceMove :
	case PlaceRotate :
	case PlaceScale :
		UnprepareNewElement();
		break;
	}
	PlacementMode = Delete;
	DeleteElementB->setDisabled(true);
	PlaceNewElementB->setDisabled(false);
	EditElementB->setDisabled(false);
	MoveElementB->setVisible(false);
	RotateElementB->setVisible(false);
	ScaleElementB->setVisible(false);
	GroupElementsB->setVisible(false);
	return true;
}

bool BallGame::PlaceNewElementBCallback(const CEGUI::EventArgs &e)
{
	DeleteElementB->setDisabled(false);
	PlaceNewElementB->setDisabled(true);
	EditElementB->setDisabled(false);
	MoveElementB->setVisible(true);
	RotateElementB->setVisible(true);
	ScaleElementB->setVisible(true);
	GroupElementsB->setVisible(false);
	SetMoveNewElement();
	return true;
}

bool BallGame::EditElementBCallback(const CEGUI::EventArgs &e)
{
	DeleteElementB->setDisabled(false);
	PlaceNewElementB->setDisabled(false);
	EditElementB->setDisabled(true);
	MoveElementB->setVisible(true);
	RotateElementB->setVisible(true);
	ScaleElementB->setVisible(true);
	GroupElementsB->setVisible(false);
	SetMoveElement();
	return true;
}

void BallGame::SetMoveElement(void)
{
	switch(PlacementMode)
	{
	case PlaceMove :
	case PlaceRotate :
	case PlaceScale :
		UnprepareNewElement();
		break;
	case Delete :
		UnprepareDeleteElement();
		break;
	}
	PlacementMode = EditMove;
	DeleteElementB->setDisabled(false);
	PlaceNewElementB->setDisabled(false);
	EditElementB->setDisabled(true);
	MoveElementB->setDisabled(true);
	RotateElementB->setDisabled(false);
	ScaleElementB->setDisabled(false);
}

void BallGame::SetMoveNewElement(void)
{
	switch(PlacementMode)
	{
	case EditMove :
	case EditRotate :
	case EditScale :
		MultiSelectionMode = false;
		MultiSelectionSetEmpty();
		EditBall(NULL);
		EditCase(NULL);
		break;
	case Delete :
		UnprepareDeleteElement();
		break;
	}
	PlacementMode = PlaceMove;
	PrepareNewElement();
	DeleteElementB->setDisabled(false);
	PlaceNewElementB->setDisabled(true);
	EditElementB->setDisabled(false);
	MoveElementB->setDisabled(true);
	RotateElementB->setDisabled(false);
	ScaleElementB->setDisabled(false);
}

bool BallGame::MoveElementBCallback(const CEGUI::EventArgs &e)
{
	if(PlaceNewElementB->isDisabled() == true) // We are in PlaceNew mode, not Edit mode
		SetMoveNewElement();
	else
		SetMoveElement();
	return true;
}

void BallGame::SetRotateElement(void)
{
	PlacementMode = EditRotate;
	DeleteElementB->setDisabled(false);
	PlaceNewElementB->setDisabled(false);
	EditElementB->setDisabled(true);
	MoveElementB->setDisabled(false);
	RotateElementB->setDisabled(true);
	ScaleElementB->setDisabled(false);
}

void BallGame::SetRotateNewElement(void)
{
	PlacementMode = PlaceRotate;
	DeleteElementB->setDisabled(false);
	PlaceNewElementB->setDisabled(true);
	EditElementB->setDisabled(false);
	MoveElementB->setDisabled(false);
	RotateElementB->setDisabled(true);
	ScaleElementB->setDisabled(false);
}

bool BallGame::RotateElementBCallback(const CEGUI::EventArgs &e)
{
	if(PlaceNewElementB->isDisabled() == true) // We are in PlaceNew mode, not Edit mode
		SetRotateNewElement();
	else
		SetRotateElement();
	return true;
}

void BallGame::SetScaleElement(void)
{
	PlacementMode = EditScale;
	DeleteElementB->setDisabled(false);
	PlaceNewElementB->setDisabled(false);
	EditElementB->setDisabled(true);
	MoveElementB->setDisabled(false);
	RotateElementB->setDisabled(false);
	ScaleElementB->setDisabled(true);
}

void BallGame::SetScaleNewElement(void)
{
	PlacementMode = PlaceScale;
	DeleteElementB->setDisabled(false);
	PlaceNewElementB->setDisabled(true);
	EditElementB->setDisabled(false);
	MoveElementB->setDisabled(false);
	RotateElementB->setDisabled(false);
	ScaleElementB->setDisabled(true);
}

bool BallGame::ScaleElementBCallback(const CEGUI::EventArgs &e)
{
	if(PlaceNewElementB->isDisabled() == true) // We are in PlaceNew mode, not Edit mode
		SetScaleNewElement();
	else
		SetScaleElement();
	return true;
}

inline void BallGame::UnprepareNewElement(void)
{
	if(ToBePlacedEntity != NULL)
	{
		LOG << "Unprepare new element : " << ToBePlacedEntity->getName() << std::endl;
		mSceneMgr->getRootSceneNode()->removeAndDestroyChild(ToBePlacedEntity->getName());
		delete ToBePlacedEntity;
		ToBePlacedEntity = NULL;
	}
}

inline void BallGame::UnprepareDeleteElement(void)
{
	if(ToBeDeletedEntity != NULL)
	{
		if(ToBeDeletedEntity != LastHighligted)
			ToBeDeletedEntity->DisplaySelectedBox(false);
		ToBeDeletedEntity = NULL;
	}
}

inline void BallGame::PrepareDeleteElement(BallGameEntity *Entity)
{
	if(ToBeDeletedEntity != NULL)
		ToBeDeletedEntity->DisplaySelectedBox(false);
	ToBeDeletedEntity = Entity;
	if(ToBeDeletedEntity != NULL)
	{
		LOG << "Prepared to be deleted Entity : " << ToBeDeletedEntity->getName() << std::endl;
		ToBeDeletedEntity->DisplaySelectedBox(true);
	}
}

void BallGame::PrepareNewElement(void)
{
	Entity *ogreEntity;
	SceneNode *ogreNode;
	Vector3 Pos, Scale;
	Quaternion Orient;

	Pos = ToBePlacedEntityType->InitialPos;
	Scale = ToBePlacedEntityType->InitialScale;
	Orient = ToBePlacedEntityType->InitialOrientation;

	LOG << "Placing new element ?" << std::endl;
	switch(ToBePlacedEntityType->Type)
	{
	case Case :
		ToBePlacedEntity = new CaseEntity();
		break;
	case Ball :
		ToBePlacedEntity = new BallEntity();
		((BallEntity*)ToBePlacedEntity)->setInitialMass(ToBePlacedEntityType->InitialMass);
		break;
	}
	LOG << "Pos = " << Pos.x << ", " << Pos.y << ", " << Pos.z << std::endl;
	LOG << "Scale = " << Scale.x << ", " << Scale.y << ", " << Scale.z << std::endl;
	LOG << "Orient = " << Orient.x << ", " << Orient.y << ", " << Orient.z << ", " << Orient.w << std::endl;
	LOG << "Mesh = " << ToBePlacedEntityType->MeshName << std::endl;
	ogreEntity = mSceneMgr->createEntity(ToBePlacedEntityType->MeshName);

	if(LastPlacedEntity != NULL)
	{
		Pos = LastPlacedEntity->getInitialPosition();
		Scale = LastPlacedEntity->getInitialScale();
		Orient = LastPlacedEntity->getInitialOrientation();
	}

	ToBePlacedEntity->setInitialPosition(Pos);
	ToBePlacedEntity->setInitialScale(Scale);
	ToBePlacedEntity->setInitialOrientation(Orient);

	String Name;
	Name = "Entity-";
	Name += std::to_string(nb_entities);
	LOG << "New Entity prepared : " << Name << std::endl;
	ogreNode = mSceneMgr->getRootSceneNode()->createChildSceneNode(Name, Pos);
	ogreNode->attachObject(ogreEntity);
	ToBePlacedEntity->setOgreNode(ogreNode);
	ToBePlacedEntity->DisplaySelectedBox(true);
}

void BallGame::PlaceUnderEditElement(void)
{
	if(UnderEditBall != NULL)
		UnderEditBall->CreateNewtonBody(m_world);

	if(UnderEditCase != NULL)
		UnderEditCase->CreateNewtonBody(m_world);

	if(UnderEditEntites.empty() == false)
	{
		std::list<BallGameEntity*>::iterator iter(UnderEditEntites.begin());
		while(iter != UnderEditEntites.end())
		{
			BallGameEntity *Entity = *(iter++);
			if(Entity == NULL)
				continue;
			switch(Entity->getType())
			{
			case Case :
				((CaseEntity*)Entity)->CreateNewtonBody(m_world);
				break;
			case Ball :
				((BallEntity*)Entity)->CreateNewtonBody(m_world);
				break;
			}
		}
	}
}

void BallGame::PlaceNewElement(void)
{
	PlaceElement(ToBePlacedEntity);
	LastPlacedEntity = ToBePlacedEntity;
	PrepareNewElement();
}

void BallGame::PlaceElement(BallGameEntity *ToBePlaced)
{
	if(ToBePlaced == NULL)
		return;
	LOG << "Placing new element !" << std::endl;

	ToBePlaced->DisplaySelectedBox(false);

	switch(ToBePlaced->getType())
	{
	case Case :
		((CaseEntity*)ToBePlaced)->CreateNewtonBody(m_world);
		AddCase((CaseEntity*)ToBePlaced);
		break;
	case Ball :
		((BallEntity*)ToBePlaced)->CreateNewtonBody(m_world);
		AddBall((BallEntity*)ToBePlaced);
		break;
	}
}

bool BallGame::GroupElementsBCallback(const CEGUI::EventArgs &e)
{
	GroupEntity *Grp = NULL;
	if(GroupElementsB->isSelected())
	{
		LOG << "Toggle is selected" << std::endl;
		String name("Group-");
		name += std::to_string(Groups.size());
		Grp = new GroupEntity(name, mSceneMgr);
	}
	else
		LOG << "Toggle is not selected" << std::endl;
	std:list<BallGameEntity*>::iterator iter(UnderEditEntites.begin());
	while(iter != UnderEditEntites.end())
	{
		BallGameEntity *Entity = *(iter++);
		GroupEntity *old;
		if(Entity == NULL)
			continue;
		if((old = Entity->getGroup()) != NULL)
		{
			bool tobedel = old->DelChild(Entity);
			if(tobedel)
				DeleteGroup(old);
		}
		if(Grp != NULL)
			Grp->AddChild(Entity);
	}
	if(Grp != NULL)
	{
		Grp->ComputeAndEquilibrateChilds();
		AddGroup(Grp);
	}
	return true;
}

bool BallGame::EditModePushBCallback(const CEGUI::EventArgs &e)
{
	SwitchEditMode();
    return true;
}

bool BallGame::StatesModePushBCallback(const CEGUI::EventArgs &e)
{
	if(StatesBanner->isVisible() == false)
	{
		StatesBanner->setVisible(true);
		ChooseStateToLoadB->setVisible(true);
		LoadStatePushB->setVisible(true);
		DelStatePushB->setVisible(true);
		SaveStatePushB->setVisible(true);
	}
	else
	{
		StatesBanner->setVisible(false);
		ChooseStateToLoadB->setVisible(false);
		LoadStatePushB->setVisible(false);
		DelStatePushB->setVisible(false);
		SaveStatePushB->setVisible(false);
	}
	return true;
}

inline void SetWindowsPosNearToOther(CEGUI::Window *self, CEGUI::Window *other, int H_factor, int V_factor)
{
	CEGUI::UVector2 pos(other->getPosition());

	pos.d_x = pos.d_x + H_factor * other->getWidth();
	pos.d_y = pos.d_y + V_factor * other->getHeight();

	self->setPosition(pos);
}

template<typename T> T* BallGame::CreateNewGUIComponent(std::string &TypeName, std::string &Name)
{
    return CreateNewGUIComponent<T>(TypeName.c_str(), Name.c_str());
}

template<typename T> T* BallGame::CreateNewGUIComponent(const char *TypeName, const char *Name)
{
    CEGUI::WindowManager &wmgr = CEGUI::WindowManager::getSingleton();
	T* ret = (T*)wmgr.createWindow(TypeName, Name);
    ret->subscribeEvent(T::EventMouseEntersArea,
			CEGUI::Event::Subscriber(&BallGame::EnteringArea, this));
    ret->subscribeEvent(T::EventMouseLeavesArea,
			CEGUI::Event::Subscriber(&BallGame::LeavingArea, this));
    ret->setVisible(false);
    return ret;
}

void BallGame::SetupGUI(void)
{
	mRenderer = &CEGUI::OgreRenderer::bootstrapSystem();

    CEGUI::ImageManager::setImagesetDefaultResourceGroup("Imagesets");
    CEGUI::Font::setDefaultResourceGroup("Fonts");
    CEGUI::Scheme::setDefaultResourceGroup("Schemes");
    CEGUI::WidgetLookManager::setDefaultResourceGroup("LookNFeel");
    CEGUI::WindowManager::setDefaultResourceGroup("Layouts");

	CEGUI::SchemeManager::getSingleton().createFromFile("OgreTray.scheme");
//	CEGUI::SchemeManager::getSingleton().createFromFile("VanillaSkin.scheme");
//	CEGUI::SchemeManager::getSingleton().createFromFile("VanillaCommonDialogs.scheme");
	CEGUI::System::getSingleton().getDefaultGUIContext().getMouseCursor().setDefaultImage("OgreTrayImages/MouseArrow");

    CEGUI::WindowManager &wmgr = CEGUI::WindowManager::getSingleton();
    CEGUI::Window *sheet = wmgr.createWindow("DefaultWindow", "CEGUIDemo/Sheet");
    CEGUI::System::getSingleton().getDefaultGUIContext().setRootWindow(sheet);

    //Main GUI
    MainLayout = static_cast<CEGUI::LayoutContainer*>(wmgr.createWindow("VerticalLayoutContainer"));
    sheet->addChild(MainLayout);

    StopPhysicPushB = CreateNewGUIComponent<CEGUI::PushButton>("OgreTray/Button");
    StopPhysicPushB->setText("Start/Stop Physic");
    StopPhysicPushB->setSize(CEGUI::USize(CEGUI::UDim(0, 150), CEGUI::UDim(0, 30)));

    MainLayout->addChild(StopPhysicPushB);

    StopPhysicPushB->subscribeEvent(CEGUI::PushButton::EventClicked,
    		CEGUI::Event::Subscriber(&BallGame::StopPhysicPushBCallback, this));

    EditModePushB = CreateNewGUIComponent<CEGUI::PushButton>("OgreTray/Button");
    EditModePushB->setText("Edit");
    EditModePushB->setSize(CEGUI::USize(CEGUI::UDim(0, 75), CEGUI::UDim(0, 30)));

    MainLayout->addChild(EditModePushB);

    EditModePushB->subscribeEvent(CEGUI::PushButton::EventClicked,
    		CEGUI::Event::Subscriber(&BallGame::EditModePushBCallback, this));

    SetWindowsPosNearToOther(EditModePushB, StopPhysicPushB, 0, 1);

    StatesModePushB = CreateNewGUIComponent<CEGUI::PushButton>("OgreTray/Button");
    StatesModePushB->setText("States");
    StatesModePushB->setSize(CEGUI::USize(CEGUI::UDim(0, 75), CEGUI::UDim(0, 30)));

    MainLayout->addChild(StatesModePushB);

    StatesModePushB->subscribeEvent(CEGUI::PushButton::EventClicked,
    		CEGUI::Event::Subscriber(&BallGame::StatesModePushBCallback, this));

    SetWindowsPosNearToOther(StatesModePushB, EditModePushB, 1, 0);

    ChooseLevelComboB = CreateNewGUIComponent<CEGUI::Combobox>("OgreTray/Combobox");
    glob_t glob_result;
    CEGUI::ListboxTextItem *actuallevel = NULL;
	memset(&glob_result, 0, sizeof(glob_result));
	glob(LEVELS_FOLDER"*.json", 0, NULL, &glob_result);
	for(size_t i = 0; i < glob_result.gl_pathc; ++i)
	{
		String *globname, filename;
		globname = new String(glob_result.gl_pathv[i]);
		size_t slashpos = globname->find_last_of('/'), dotpos = globname->find_last_of('.');
		filename = globname->substr(slashpos + 1, dotpos - (slashpos + 1));
		CEGUI::ListboxTextItem *item = new CEGUI::ListboxTextItem((CEGUI::utf8*)filename.c_str());
		item->setUserData(globname);
		ChooseLevelComboB->addItem(item);
        if(ChooseLevelComboB->getText().empty() == true)
        {
        	ChooseLevelComboB->setText(filename);
        	actuallevel = item;
        }
	}
	globfree(&glob_result);
    ChooseLevelComboB->setSize(CEGUI::USize(CEGUI::UDim(0, 150), CEGUI::UDim(0, 40 * (ChooseLevelComboB->getItemCount() + 1))));

    MainLayout->addChild(ChooseLevelComboB);

    ChooseLevelComboB->subscribeEvent(CEGUI::Combobox::EventListSelectionAccepted,
    		CEGUI::Event::Subscriber(&BallGame::ChooseLevelComboBCallback, this));

    SetWindowsPosNearToOther(ChooseLevelComboB, EditModePushB, 0, 1);

    NewLevelEditB = CreateNewGUIComponent<CEGUI::Editbox>("OgreTray/Editbox");
    NewLevelEditB->setText("NewLevel");
    NewLevelEditB->setSize(CEGUI::USize(CEGUI::UDim(0, 90), CEGUI::UDim(0, 30)));

    MainLayout->addChild(NewLevelEditB);

//    NewLevelEditB->subscribeEvent(CEGUI::Editbox::EventClicked,
//    		CEGUI::Event::Subscriber(&BallGame::SaveLevelPushBCallback, this));

    SetWindowsPosNearToOther(NewLevelEditB, EditModePushB, 0, 2);// Be Carefull, Combobox size is size with combo expanded !

    NewLevelCreateB = CreateNewGUIComponent<CEGUI::PushButton>("OgreTray/Button");
    NewLevelCreateB->setText("Create");
    NewLevelCreateB->setSize(CEGUI::USize(CEGUI::UDim(0, 60), CEGUI::UDim(0, 30)));

    MainLayout->addChild(NewLevelCreateB);

    NewLevelCreateB->subscribeEvent(CEGUI::PushButton::EventClicked,
			CEGUI::Event::Subscriber(&BallGame::NewLevelCreateBCallback, this));

    SetWindowsPosNearToOther(NewLevelCreateB, NewLevelEditB, 1, 0);

    SaveLevelPushB = CreateNewGUIComponent<CEGUI::PushButton>("OgreTray/Button");
    SaveLevelPushB->setText("Save");
    SaveLevelPushB->setSize(CEGUI::USize(CEGUI::UDim(0, 150), CEGUI::UDim(0, 30)));

    MainLayout->addChild(SaveLevelPushB);

    SaveLevelPushB->subscribeEvent(CEGUI::PushButton::EventClicked,
    		CEGUI::Event::Subscriber(&BallGame::SaveLevelPushBCallback, this));

    SetWindowsPosNearToOther(SaveLevelPushB, NewLevelEditB, 0, 1);

    QuitPushB = CreateNewGUIComponent<CEGUI::PushButton>("OgreTray/Button");
    QuitPushB->setText("Quit");
    QuitPushB->setTooltipText("Quit");
    QuitPushB->setSize(CEGUI::USize(CEGUI::UDim(0, 150), CEGUI::UDim(0, 30)));

    MainLayout->addChild(QuitPushB);

    QuitPushB->subscribeEvent(CEGUI::PushButton::EventClicked,
    		CEGUI::Event::Subscriber(&BallGame::QuitPushBCallback, this));

    SetWindowsPosNearToOther(QuitPushB, SaveLevelPushB, 0, 1);

    LevelNameBanner = CreateNewGUIComponent<CEGUI::Titlebar>("OgreTray/Titlebar");
    LevelNameBanner->setText("Level");
    LevelNameBanner->setSize(CEGUI::USize(CEGUI::UDim(0, 150), CEGUI::UDim(0, 30)));
    LevelNameBanner->setVerticalAlignment(CEGUI::VA_TOP);
    LevelNameBanner->setHorizontalAlignment(CEGUI::HA_CENTRE);
    LevelNameBanner->setVisible(true);

    MainLayout->addChild(LevelNameBanner);

    StatesBanner = CreateNewGUIComponent<CEGUI::Titlebar>("OgreTray/Titlebar");
    StatesBanner->setText("States");
    StatesBanner->setSize(CEGUI::USize(CEGUI::UDim(0, 150), CEGUI::UDim(0, 30)));
    StatesBanner->setVerticalAlignment(CEGUI::VA_TOP);
    StatesBanner->setHorizontalAlignment(CEGUI::HA_RIGHT);

    MainLayout->addChild(StatesBanner);

    ChooseStateToLoadB = CreateNewGUIComponent<CEGUI::Combobox>("OgreTray/Combobox");
    ChooseStateToLoadB->setSize(CEGUI::USize(CEGUI::UDim(0, 150), CEGUI::UDim(0, 30)));
    ChooseStateToLoadB->setVerticalAlignment(CEGUI::VA_TOP);
    ChooseStateToLoadB->setHorizontalAlignment(CEGUI::HA_RIGHT);

    MainLayout->addChild(ChooseStateToLoadB);
    ChooseStateToLoadB->subscribeEvent(CEGUI::Combobox::EventListSelectionChanged,
    		CEGUI::Event::Subscriber(&BallGame::ChooseStateToLoadBCallback, this));
    ChooseStateToLoadB->subscribeEvent(CEGUI::Combobox::EventListContentsChanged,
    		CEGUI::Event::Subscriber(&BallGame::ChooseStateToLoadBCallback, this));
    SetWindowsPosNearToOther(ChooseStateToLoadB, StatesBanner, 0, 1);

    LoadStatePushB = CreateNewGUIComponent<CEGUI::PushButton>("OgreTray/Button");
    LoadStatePushB->setText("Load");
    LoadStatePushB->setSize(CEGUI::USize(CEGUI::UDim(0, 150), CEGUI::UDim(0, 30)));
    LoadStatePushB->setVerticalAlignment(CEGUI::VA_TOP);
    LoadStatePushB->setHorizontalAlignment(CEGUI::HA_RIGHT);

    MainLayout->addChild(LoadStatePushB);
    LoadStatePushB->subscribeEvent(CEGUI::PushButton::EventClicked,
    		CEGUI::Event::Subscriber(&BallGame::LoadStatePushBCallback, this));
    SetWindowsPosNearToOther(LoadStatePushB, StatesBanner, 0, 2);

    DelStatePushB = CreateNewGUIComponent<CEGUI::PushButton>("OgreTray/Button");
    DelStatePushB->setText("Delete");
    DelStatePushB->setSize(CEGUI::USize(CEGUI::UDim(0, 150), CEGUI::UDim(0, 30)));
    DelStatePushB->setVerticalAlignment(CEGUI::VA_TOP);
    DelStatePushB->setHorizontalAlignment(CEGUI::HA_RIGHT);

    MainLayout->addChild(DelStatePushB);
    DelStatePushB->subscribeEvent(CEGUI::PushButton::EventClicked,
    		CEGUI::Event::Subscriber(&BallGame::DelStatePushBCallback, this));
    SetWindowsPosNearToOther(DelStatePushB, LoadStatePushB, 0, 1);

    SaveStatePushB = CreateNewGUIComponent<CEGUI::PushButton>("OgreTray/Button");
    SaveStatePushB->setText("Save");
    SaveStatePushB->setSize(CEGUI::USize(CEGUI::UDim(0, 150), CEGUI::UDim(0, 30)));
    SaveStatePushB->setVerticalAlignment(CEGUI::VA_TOP);
    SaveStatePushB->setHorizontalAlignment(CEGUI::HA_RIGHT);

    MainLayout->addChild(SaveStatePushB);
    SaveStatePushB->subscribeEvent(CEGUI::PushButton::EventClicked,
    		CEGUI::Event::Subscriber(&BallGame::SaveStatePushBCallback, this));
    SetWindowsPosNearToOther(SaveStatePushB, DelStatePushB, 0, 1);

    //Now LevelNameBanner exist, we can call SetLevel !
    String levelname(actuallevel->getText().c_str());
    String *levelfilename = (String*)actuallevel->getUserData();
    SetLevel(levelname, *levelfilename);


    //Edit GUI

    EditingModeTitleBanner = CreateNewGUIComponent<CEGUI::Titlebar>("OgreTray/Title");
    EditingModeTitleBanner->setText("Edit Mode");
    EditingModeTitleBanner->setSize(CEGUI::USize(CEGUI::UDim(0, 150), CEGUI::UDim(0, 30)));
    EditingModeTitleBanner->setVerticalAlignment(CEGUI::VA_TOP);
    EditingModeTitleBanner->setHorizontalAlignment(CEGUI::HA_CENTRE);

    MainLayout->addChild(EditingModeTitleBanner);

    SetWindowsPosNearToOther(EditingModeTitleBanner, EditingModeTitleBanner, 0, 1);

    // Add new Element GUI
    AddElementTitleBanner = CreateNewGUIComponent<CEGUI::Titlebar>("OgreTray/Titlebar");
    AddElementTitleBanner->setText("Add");
    AddElementTitleBanner->setSize(CEGUI::USize(CEGUI::UDim(0, 150), CEGUI::UDim(0, 30)));
    AddElementTitleBanner->setVerticalAlignment(CEGUI::VA_TOP);
    AddElementTitleBanner->setHorizontalAlignment(CEGUI::HA_RIGHT);
    CEGUI::UVector2 pos = AddElementTitleBanner->getPosition();
    pos.d_y = CEGUI::UDim(0, (mWindow->getHeight() / 2) - 120);
    AddElementTitleBanner->setPosition(pos);

    MainLayout->addChild(AddElementTitleBanner);


    ChooseTypeOfElementToAddB = CreateNewGUIComponent<CEGUI::Combobox>("OgreTray/Combobox");
	std::list<class EntityType*>::iterator iter(EntityTypes.begin());
	while(iter != EntityTypes.end())
	{
		EntityType *type = *iter;
		if(type != NULL)
		{
			ChooseTypeOfElementToAddB->addItem(new CEGUI::ListboxTextItem(type->Name));
			if(ChooseTypeOfElementToAddB->getText().empty() == true)
			{
				ChooseTypeOfElementToAddB->setText(type->Name);
				ToBePlacedEntityType = type;
				CreateThumbnail(ToBePlacedEntityType->MeshName);
			}
		}
		iter++;
	}
    ChooseTypeOfElementToAddB->setSize(CEGUI::USize(CEGUI::UDim(0, 150), CEGUI::UDim(0, 40 * (ChooseLevelComboB->getItemCount() + 1))));
    ChooseTypeOfElementToAddB->setVerticalAlignment(CEGUI::VA_TOP);
    ChooseTypeOfElementToAddB->setHorizontalAlignment(CEGUI::HA_RIGHT);

    ChooseTypeOfElementToAddB->subscribeEvent(CEGUI::Combobox::EventListSelectionAccepted,
    		CEGUI::Event::Subscriber(&BallGame::ChooseTypeOfElementToAddBCallback, this));

    MainLayout->addChild(ChooseTypeOfElementToAddB);

    ThumbnailWindow = CreateNewGUIComponent<CEGUI::Window>("OgreTray/StaticImage", "RTTWindow");
    ThumbnailWindow->setSize(CEGUI::USize(CEGUI::UDim(0, 150),
 						   CEGUI::UDim(0, 150)));
    ThumbnailWindow->setVerticalAlignment(CEGUI::VA_TOP);
    ThumbnailWindow->setHorizontalAlignment(CEGUI::HA_RIGHT);

    sheet->addChild(ThumbnailWindow);

    PlaceNewElementB = CreateNewGUIComponent<CEGUI::PushButton>("OgreTray/Button");
    PlaceNewElementB->setText("Place");
    PlaceNewElementB->setSize(CEGUI::USize(CEGUI::UDim(0, 75), CEGUI::UDim(0, 30)));
    PlaceNewElementB->setVerticalAlignment(CEGUI::VA_TOP);
    PlaceNewElementB->setHorizontalAlignment(CEGUI::HA_RIGHT);

    PlaceNewElementB->subscribeEvent(CEGUI::PushButton::EventClicked,
			CEGUI::Event::Subscriber(&BallGame::PlaceNewElementBCallback, this));

    MainLayout->addChild(PlaceNewElementB);

    EditElementB = CreateNewGUIComponent<CEGUI::PushButton>("OgreTray/Button");
    EditElementB->setText("Edit");
    EditElementB->setSize(CEGUI::USize(CEGUI::UDim(0, 75), CEGUI::UDim(0, 30)));
    EditElementB->setVerticalAlignment(CEGUI::VA_TOP);
    EditElementB->setHorizontalAlignment(CEGUI::HA_RIGHT);

    EditElementB->subscribeEvent(CEGUI::PushButton::EventClicked,
			CEGUI::Event::Subscriber(&BallGame::EditElementBCallback, this));

    MainLayout->addChild(EditElementB);

    DeleteElementB = CreateNewGUIComponent<CEGUI::PushButton>("OgreTray/Button");
    DeleteElementB->setText("Delete");
    DeleteElementB->setSize(CEGUI::USize(CEGUI::UDim(0, 150), CEGUI::UDim(0, 30)));
    DeleteElementB->setVerticalAlignment(CEGUI::VA_TOP);
    DeleteElementB->setHorizontalAlignment(CEGUI::HA_RIGHT);

    DeleteElementB->subscribeEvent(CEGUI::PushButton::EventClicked,
			CEGUI::Event::Subscriber(&BallGame::DeleteElementBCallback, this));

    MainLayout->addChild(DeleteElementB);


    MoveElementB = CreateNewGUIComponent<CEGUI::PushButton>("OgreTray/Button");
    MoveElementB->setText("M");
    MoveElementB->setTooltipText("Move Element. Press M to enter this mode.");
    MoveElementB->setSize(CEGUI::USize(CEGUI::UDim(0, 50), CEGUI::UDim(0, 30)));
    MoveElementB->setVerticalAlignment(CEGUI::VA_TOP);
    MoveElementB->setHorizontalAlignment(CEGUI::HA_RIGHT);

    MoveElementB->subscribeEvent(CEGUI::PushButton::EventClicked,
			CEGUI::Event::Subscriber(&BallGame::MoveElementBCallback, this));

    MainLayout->addChild(MoveElementB);


    RotateElementB = CreateNewGUIComponent<CEGUI::PushButton>("OgreTray/Button");
    RotateElementB->setText("R");
    RotateElementB->setTooltipText("Rotate Element. Press R to enter this mode.");
    RotateElementB->setSize(CEGUI::USize(CEGUI::UDim(0, 50), CEGUI::UDim(0, 30)));
    RotateElementB->setVerticalAlignment(CEGUI::VA_TOP);
    RotateElementB->setHorizontalAlignment(CEGUI::HA_RIGHT);

    RotateElementB->subscribeEvent(CEGUI::PushButton::EventClicked,
			CEGUI::Event::Subscriber(&BallGame::RotateElementBCallback, this));

    MainLayout->addChild(RotateElementB);


    ScaleElementB = CreateNewGUIComponent<CEGUI::PushButton>("OgreTray/Button");
    ScaleElementB->setText("S");
    ScaleElementB->setTooltipText("Scale Element. Press S to enter this mode.");
    ScaleElementB->setSize(CEGUI::USize(CEGUI::UDim(0, 50), CEGUI::UDim(0, 30)));
    ScaleElementB->setVerticalAlignment(CEGUI::VA_TOP);
    ScaleElementB->setHorizontalAlignment(CEGUI::HA_RIGHT);

    ScaleElementB->subscribeEvent(CEGUI::PushButton::EventClicked,
			CEGUI::Event::Subscriber(&BallGame::ScaleElementBCallback, this));

    MainLayout->addChild(ScaleElementB);


    GroupElementsB = CreateNewGUIComponent<CEGUI::ToggleButton>("OgreTray/Checkbox");
    GroupElementsB->setText("Grouped");
    GroupElementsB->setSize(CEGUI::USize(CEGUI::UDim(0, 150), CEGUI::UDim(0, 30)));
    GroupElementsB->setVerticalAlignment(CEGUI::VA_TOP);
    GroupElementsB->setHorizontalAlignment(CEGUI::HA_RIGHT);

    GroupElementsB->subscribeEvent(CEGUI::ToggleButton::EventSelectStateChanged,
			CEGUI::Event::Subscriber(&BallGame::GroupElementsBCallback, this));

    MainLayout->addChild(GroupElementsB);

    SetWindowsPosNearToOther(ChooseTypeOfElementToAddB, AddElementTitleBanner, 0, 1);
    SetWindowsPosNearToOther(ThumbnailWindow, AddElementTitleBanner, 0, 2);
    SetWindowsPosNearToOther(EditElementB, ThumbnailWindow, 0, 1);
    SetWindowsPosNearToOther(PlaceNewElementB, EditElementB, -1, 0);
    SetWindowsPosNearToOther(DeleteElementB, EditElementB, 0, 1);
    SetWindowsPosNearToOther(ScaleElementB, DeleteElementB, 0, 1);
    SetWindowsPosNearToOther(RotateElementB, ScaleElementB, -1, 0);
    SetWindowsPosNearToOther(MoveElementB, RotateElementB, -1, 0);
    SetWindowsPosNearToOther(GroupElementsB, ScaleElementB, 0, 1);


    // Edit Case GUI

    CaseHasForceToggleB = CreateNewGUIComponent<CEGUI::ToggleButton>("OgreTray/Checkbox");
    CaseHasForceToggleB->setText("Has Force");
    CaseHasForceToggleB->setSelected(false);
    CaseHasForceToggleB->setSize(CEGUI::USize(CEGUI::UDim(0, 200), CEGUI::UDim(0, 30)));
    CaseHasForceToggleB->setVerticalAlignment(CEGUI::VA_BOTTOM);
    CaseHasForceToggleB->setHorizontalAlignment(CEGUI::HA_CENTRE);

    CaseHasForceToggleB->subscribeEvent(CEGUI::ToggleButton::EventSelectStateChanged,
			CEGUI::Event::Subscriber(&BallGame::CaseHasForceToggleBCallback, this));

    MainLayout->addChild(CaseHasForceToggleB);

    CaseForceValueEditB = CreateNewGUIComponent<CEGUI::Editbox>("OgreTray/Editbox");
    CaseForceValueEditB->setSize(CEGUI::USize(CEGUI::UDim(0, 50), CEGUI::UDim(0, 30)));
    CaseForceValueEditB->setVerticalAlignment(CEGUI::VA_BOTTOM);
    CaseForceValueEditB->setHorizontalAlignment(CEGUI::HA_CENTRE);
    String numRegex("^(\\-?[0-9]*(\\.[0-9]*)?)?");
    CaseForceValueEditB->setValidationString(numRegex);

    CaseForceValueEditB->subscribeEvent(CEGUI::Editbox::EventTextAccepted,
			CEGUI::Event::Subscriber(&BallGame::CaseForceValueEditBCallback, this));
    CaseForceValueEditB->subscribeEvent(CEGUI::Editbox::EventMouseLeavesArea,
			CEGUI::Event::Subscriber(&BallGame::CaseForceValueEditBCallback, this));

    MainLayout->addChild(CaseForceValueEditB);

    CaseHasForceDirectionToggleB = CreateNewGUIComponent<CEGUI::ToggleButton>("OgreTray/Checkbox");
    CaseHasForceDirectionToggleB->setText("Has Force Directed");
    CaseHasForceDirectionToggleB->setSelected(false);
    CaseHasForceDirectionToggleB->setSize(CEGUI::USize(CEGUI::UDim(0, 200), CEGUI::UDim(0, 30)));
    CaseHasForceDirectionToggleB->setVerticalAlignment(CEGUI::VA_BOTTOM);
    CaseHasForceDirectionToggleB->setHorizontalAlignment(CEGUI::HA_CENTRE);

	CaseHasForceDirectionToggleB->subscribeEvent(CEGUI::ToggleButton::EventSelectStateChanged,
			CEGUI::Event::Subscriber(&BallGame::CaseHasForceDirectionToggleBCallback, this));

    MainLayout->addChild(CaseHasForceDirectionToggleB);

    CaseForceDirectionXValueEditB = CreateNewGUIComponent<CEGUI::Editbox>("OgreTray/Editbox");
    CaseForceDirectionXValueEditB->setSize(CEGUI::USize(CEGUI::UDim(0, 50), CEGUI::UDim(0, 30)));
    CaseForceDirectionXValueEditB->setVerticalAlignment(CEGUI::VA_BOTTOM);
    CaseForceDirectionXValueEditB->setHorizontalAlignment(CEGUI::HA_CENTRE);
    CaseForceDirectionXValueEditB->setValidationString(numRegex);

    CaseForceDirectionXValueEditB->subscribeEvent(CEGUI::Editbox::EventMouseWheel,
			CEGUI::Event::Subscriber(&BallGame::CaseForceDirectionXValueEditBMouseWheelCallback, this));


    MainLayout->addChild(CaseForceDirectionXValueEditB);

    CaseForceDirectionYValueEditB = CreateNewGUIComponent<CEGUI::Editbox>("OgreTray/Editbox");
    CaseForceDirectionYValueEditB->setSize(CEGUI::USize(CEGUI::UDim(0, 50), CEGUI::UDim(0, 30)));
    CaseForceDirectionYValueEditB->setVerticalAlignment(CEGUI::VA_BOTTOM);
    CaseForceDirectionYValueEditB->setHorizontalAlignment(CEGUI::HA_CENTRE);
    CaseForceDirectionYValueEditB->setValidationString(numRegex);

    CaseForceDirectionYValueEditB->subscribeEvent(CEGUI::Editbox::EventMouseWheel,
			CEGUI::Event::Subscriber(&BallGame::CaseForceDirectionYValueEditBMouseWheelCallback, this));


    MainLayout->addChild(CaseForceDirectionYValueEditB);

    CaseForceDirectionZValueEditB = CreateNewGUIComponent<CEGUI::Editbox>("OgreTray/Editbox");
    CaseForceDirectionZValueEditB->setSize(CEGUI::USize(CEGUI::UDim(0, 50), CEGUI::UDim(0, 30)));
    CaseForceDirectionZValueEditB->setVerticalAlignment(CEGUI::VA_BOTTOM);
    CaseForceDirectionZValueEditB->setHorizontalAlignment(CEGUI::HA_CENTRE);
    CaseForceDirectionZValueEditB->setValidationString(numRegex);

    CaseForceDirectionZValueEditB->subscribeEvent(CEGUI::Editbox::EventMouseWheel,
			CEGUI::Event::Subscriber(&BallGame::CaseForceDirectionZValueEditBMouseWheelCallback, this));


    MainLayout->addChild(CaseForceDirectionZValueEditB);

    NormalizeCaseForceDirectionPushB = CreateNewGUIComponent<CEGUI::PushButton>("OgreTray/Button");
    NormalizeCaseForceDirectionPushB->setText("Norm");
    NormalizeCaseForceDirectionPushB->setSize(CEGUI::USize(CEGUI::UDim(0, 50), CEGUI::UDim(0, 30)));
    NormalizeCaseForceDirectionPushB->setVerticalAlignment(CEGUI::VA_BOTTOM);
    NormalizeCaseForceDirectionPushB->setHorizontalAlignment(CEGUI::HA_CENTRE);
    NormalizeCaseForceDirectionPushB->subscribeEvent(CEGUI::PushButton::EventClicked,
    		CEGUI::Event::Subscriber(&BallGame::NormalizeCaseForceDirectionPushBCallback, this));


    MainLayout->addChild(NormalizeCaseForceDirectionPushB);

    ApplyForceChangesToCasePushB = CreateNewGUIComponent<CEGUI::PushButton>("OgreTray/Button");
    ApplyForceChangesToCasePushB->setText("Apply");
    ApplyForceChangesToCasePushB->setSize(CEGUI::USize(CEGUI::UDim(0, 100), CEGUI::UDim(0, 30)));
    ApplyForceChangesToCasePushB->setVerticalAlignment(CEGUI::VA_BOTTOM);
    ApplyForceChangesToCasePushB->setHorizontalAlignment(CEGUI::HA_CENTRE);

    ApplyForceChangesToCasePushB->subscribeEvent(CEGUI::PushButton::EventClicked,
    		CEGUI::Event::Subscriber(&BallGame::ApplyForceChangesToCasePushBCallback, this));

    MainLayout->addChild(ApplyForceChangesToCasePushB);

    SetWindowsPosNearToOther(CaseHasForceDirectionToggleB, ApplyForceChangesToCasePushB, 0, -1);
    SetWindowsPosNearToOther(CaseHasForceToggleB, CaseHasForceDirectionToggleB, 0, -1);
    SetWindowsPosNearToOther(CaseForceValueEditB, CaseHasForceToggleB, 1, 0);
    SetWindowsPosNearToOther(CaseForceDirectionXValueEditB, CaseHasForceDirectionToggleB, 1, 0);
    SetWindowsPosNearToOther(CaseForceDirectionYValueEditB, CaseForceDirectionXValueEditB, 1, 0);
    SetWindowsPosNearToOther(CaseForceDirectionZValueEditB, CaseForceDirectionYValueEditB, 1, 0);
    SetWindowsPosNearToOther(NormalizeCaseForceDirectionPushB, CaseForceDirectionZValueEditB, 1, 0);

    // Edit Ball GUI

    BallMassValueEditB = CreateNewGUIComponent<CEGUI::Editbox>("OgreTray/Editbox");
    BallMassValueEditB->setSize(CEGUI::USize(CEGUI::UDim(0, 50), CEGUI::UDim(0, 30)));
    BallMassValueEditB->setVerticalAlignment(CEGUI::VA_BOTTOM);
    BallMassValueEditB->setHorizontalAlignment(CEGUI::HA_CENTRE);

    MainLayout->addChild(BallMassValueEditB);

    ApplyMassChangesToBallPushB = CreateNewGUIComponent<CEGUI::PushButton>("OgreTray/Button");
    ApplyMassChangesToBallPushB->setText("Apply");
    ApplyMassChangesToBallPushB->setSize(CEGUI::USize(CEGUI::UDim(0, 100), CEGUI::UDim(0, 30)));
    ApplyMassChangesToBallPushB->setVerticalAlignment(CEGUI::VA_BOTTOM);
    ApplyMassChangesToBallPushB->setHorizontalAlignment(CEGUI::HA_CENTRE);
    ApplyMassChangesToBallPushB->subscribeEvent(CEGUI::PushButton::EventClicked,
    		CEGUI::Event::Subscriber(&BallGame::ApplyMassChangesToBallPushBCallback, this));

    MainLayout->addChild(ApplyMassChangesToBallPushB);

    SetWindowsPosNearToOther(BallMassValueEditB, ApplyMassChangesToBallPushB, 0, -1);
}

void BallGame::SetupGame(void)
{
//    // register our scene with the RTSS
//    RTShader::ShaderGenerator* shadergen = RTShader::ShaderGenerator::getSingletonPtr();
//    shadergen->addSceneManager(scnMgr);

	mSceneMgr->setAmbientLight(ColourValue(0.5, 0.5, 0.5));

    Light* light = mSceneMgr->createLight("MainLight");
    SceneNode* lightNode = mSceneMgr->getRootSceneNode()->createChildSceneNode("MainLight");
    lightNode->attachObject(light);
    lightNode->setPosition(20, 80, 50);

	mThumbnailSceneMgr->setAmbientLight(ColourValue(0.5, 0.5, 0.5));

    light = mThumbnailSceneMgr->createLight("MainThumbnailLight");
    lightNode = mThumbnailSceneMgr->getRootSceneNode()->createChildSceneNode("MainThumbnailLight");
    lightNode->attachObject(light);
    lightNode->setPosition(20, 80, 50);

    SetCam(-184, -253, 352);
    mThumbnailCamera->setPosition(-184, -253, 352);
    mCamera->setOrientation(Ogre::Quaternion(0.835422, 0.393051, -0.238709, -0.300998));
    mThumbnailCamera->setOrientation(Ogre::Quaternion(0.835422, 0.393051, -0.238709, -0.300998));

    CEGUI::Texture &guiTex = mRenderer->createTexture("textname", ptex);

    const CEGUI::Rectf rect(CEGUI::Vector2f(0.0f, 0.0f), guiTex.getOriginalDataSize());
    CEGUI::BasicImage* image = (CEGUI::BasicImage*)( &CEGUI::ImageManager::getSingleton().create("BasicImage", "ElementsThumbail"));
       image->setTexture(&guiTex);
       image->setArea(rect);
       image->setAutoScaled(CEGUI::ASM_Both);

   ThumbnailWindow->setProperty("Image", "ElementsThumbail");

    ChangeLevel();

    _StartPhysic();
//    _StopPhysic();
}

void BallGame::OnContactCollision (const NewtonJoint* contactJoint, dFloat timestep, int threadIndex)
{
	NewtonBody *body0 = NewtonJointGetBody0(contactJoint);
	NewtonBody *body1 = NewtonJointGetBody1(contactJoint);
	BallGame *Game = (BallGame*)NewtonWorldGetUserData(NewtonBodyGetWorld(body0));
	BallGameEntity *Entity0 = (BallGameEntity*)NewtonBodyGetUserData(body0);
	BallGameEntity *Entity1 = (BallGameEntity*)NewtonBodyGetUserData(body1);
	BallEntity *BallToCheck = NULL;
	CaseEntity *CaseToCheck = NULL;
	switch(Entity0->getType())
	{
	case Ball :
//		LOG << "Ball " << Entity0 << " Colliding with ";
		BallToCheck = (BallEntity*)Entity0;
		break;
	case Case :
//		LOG << "Case " << Entity0 << " Colliding with ";
		CaseToCheck = (CaseEntity*)Entity0;
		break;
	}
	switch(Entity1->getType())
	{
	case Ball :
//		LOG << "Ball " << Entity1 << std::endl;
		BallToCheck = (BallEntity*)Entity1;
		break;
	case Case :
//		LOG << "Case " << Entity1 << std::endl;
		CaseToCheck = (CaseEntity*)Entity1;
		break;
	}
	if(BallToCheck != NULL && CaseToCheck != NULL)
	{
		CaseToCheck->AddBallColliding(BallToCheck);
		Game->AddCaseColliding(CaseToCheck);
	}
}

void BallGame::AddCaseColliding(CaseEntity *ToAdd)
{
	if(ToAdd == NULL)
		return;
	if(CheckIfAlreadyColliding(ToAdd) == false)
		CasesUnderCollide.push_back(ToAdd);
}

bool BallGame::CheckIfAlreadyColliding(CaseEntity *ToCheck)
{
	bool ret = false;
	if(ToCheck == NULL)
		return ret;
	std::list<CaseEntity*>::iterator iter(CasesUnderCollide.begin());
	while(iter != CasesUnderCollide.end())
	{
		CaseEntity *Case = *(iter++);
		if(Case == ToCheck)
			return true;
	}
	return ret;
}

void BallGame::CheckforCollides(void)
{
	std::list<CaseEntity*>::iterator iter(CasesUnderCollide.begin());
	while(iter != CasesUnderCollide.end())
	{
		CaseEntity *Case = *iter;
		if(Case != NULL)
			Case->ApplyForceOnCollidingBalls();
		iter = CasesUnderCollide.erase(iter);
	}
}

void BallGame::AddGroup(GroupEntity *group)
{
	if(group == NULL)
		return;
	Groups.push_back(group);
	nb_entities++;
}

void BallGame::AddBall(BallEntity *ball)
{
	if(ball == NULL)
		return;
	Balls.push_back(ball);
	nb_entities++;
}

void BallGame::AddCase(CaseEntity *Wcase)
{
	if(Wcase == NULL)
		return;
	Cases.push_back(Wcase);
	nb_entities++;
}


bool BallGame::frameEnded(const Ogre::FrameEvent& fe)
{
//    LOG << "Render a frame" << std::endl;
	dFloat timestep = dGetElapsedSeconds();
	UpdatePhysics(timestep);

	// Needed for tool tips ? For the moment doesn't work and if uncommented, all buttons places are broke down !
//	CEGUI::System::getSingleton().getDefaultGUIContext().injectTimePulse( fe.timeSinceLastFrame );

	if(ThumbnailWindow->isVisible() == true && ogreThumbnailNode != NULL)
		ogreThumbnailNode->roll(Degree(0.01), Node::TS_WORLD);

	if(mode == Running)
	{
		std::list<BallEntity*>::iterator iter(Balls.begin());
		while(iter != Balls.end())
		{
			BallEntity *ball = *(iter++);
			if(ball == NULL)
				continue;
			Vector3 worldPos = ball->getAbsolutePosition();
			Vector3 hcsPosition = mCamera->getProjectionMatrix() * mCamera->getViewMatrix() * worldPos;
#define ECART 0.25
//#define ECART 0.8
			if(hcsPosition.x >= ECART)
			{
				MoveCam(0.1, 0, 0);
				hcsPosition = mCamera->getProjectionMatrix() * mCamera->getViewMatrix() * worldPos;
			}
			if(hcsPosition.x <= -1 * ECART)
			{
				MoveCam(-0.1, 0, 0);
				hcsPosition = mCamera->getProjectionMatrix() * mCamera->getViewMatrix() * worldPos;
			}
			if(hcsPosition.y >= ECART)
			{
				MoveCam(0, 0.1, 0);
				hcsPosition = mCamera->getProjectionMatrix() * mCamera->getViewMatrix() * worldPos;
			}
			if(hcsPosition.y <= - 1 * ECART)
			{
				MoveCam(0, -0.1, 0);
				hcsPosition = mCamera->getProjectionMatrix() * mCamera->getViewMatrix() * worldPos;
			}
		}
	}
	else
	{
		CEGUI::Vector2f mpos = CEGUI::System::getSingleton().getDefaultGUIContext().getMouseCursor().getPosition();
		// Will Mouse has not hit top left corner, there is a gap between OIS and CEGUI mouse coordinates. CEGUI is more reliable

		float xmov = 0, ymov = 0;

		if(mpos.d_x < 10)
			xmov = -0.1;
		if(mpos.d_x > mWindow->getWidth() - 10)
			xmov = 0.1;

		if(mpos.d_y < 10)
			ymov = 0.1;
		if(mpos.d_y > mWindow->getHeight() - 10)
			ymov = -0.1;

		if(xmov != 0 || ymov != 0)
			MoveCam(xmov,  ymov,  0);
	}

    return true;
}

bool BallGame::mouseMoved(const OIS::MouseEvent &arg)
{
    CEGUI::GUIContext& context = CEGUI::System::getSingleton().getDefaultGUIContext();
    context.injectMouseMove(arg.state.X.rel, arg.state.Y.rel);
	if(arg.state.Z.rel != 0)
		context.injectMouseWheelChange(arg.state.Z.rel);
    //BaseApplication::mouseMoved(arg);

    if(MouseOverButton == true)
    	return true;

	Real x, y;
	CEGUI::Vector2f mpos = CEGUI::System::getSingleton().getDefaultGUIContext().getMouseCursor().getPosition();
	// Will Mouse has not hit top left corner, there is a gap between OIS and CEGUI mouse coordinates. CEGUI is more reliable
	x = mpos.d_x / (float)mWindow->getWidth();
	y = mpos.d_y / (float)mWindow->getHeight();

	if(arg.state.Z.rel != 0)
		MoveCam(0,  0, -2 * arg.state.Z.rel);

	if(mode == Editing)
	{
		if(LastHighligted != NULL)
		{
			bool HideBoundingBox = true;

			if((UnderEditCase != NULL && UnderEditCase == LastHighligted)
					|| (UnderEditBall != NULL && UnderEditBall == LastHighligted)
					|| (ToBePlacedEntity != NULL && ToBePlacedEntity == LastHighligted)
					|| (ToBeDeletedEntity != NULL && ToBeDeletedEntity == LastHighligted))
				HideBoundingBox = false;

			if(UnderEditEntites.empty() == false)
			{
//				LOG << "Multi selection mode" << std::endl;
				std::list<BallGameEntity*>::iterator iter(UnderEditEntites.begin());
				while(iter != UnderEditEntites.end())
				{
					BallGameEntity *Entity = *(iter++);
					if(Entity != NULL && Entity == LastHighligted)
					{
						HideBoundingBox = false;
						break;
					}
				}
			}
			if(HideBoundingBox)
			{
//				LOG << "Entity under mouse not selected" << std::endl;
				LastHighligted->DisplaySelectedBox(false);
			}
			LastHighligted = NULL;
		}
		RaySceneQuery *mRayScanQuery = mSceneMgr->createRayQuery(Ogre::Ray());


//		LOG << "Coords Abs {" << arg.state.X.abs << ", " << arg.state.Y.abs << "}" << std::endl;
//		LOG << "Coords CEGUI {" << mpos.d_x << ", " << mpos.d_y << "}" << std::endl;
//		LOG << "Edit mode, try to pick {" << x << ", " << y << "}" << std::endl;
		Ray mouseRay = mCamera->getCameraToViewportRay(x, y);
		mRayScanQuery->setRay(mouseRay);
		mRayScanQuery->setSortByDistance(true, 1);
		RaySceneQueryResult &result = mRayScanQuery->execute();
		RaySceneQueryResult::iterator itr = result.begin();
//		LOG << "Picking Mouse : " << result.size() << std::endl;
		while(itr != result.end())
		{
			if(itr->movable != NULL)
			{
				SceneNode *PickedUpNode = itr->movable->getParentSceneNode();
				if(PickedUpNode != ForcesArrows)
				{
					LastHighligted = Ogre::any_cast<BallGameEntity*>(((Ogre::Entity*)PickedUpNode->getAttachedObject(0))->getUserObjectBindings().getUserAny());
					LastHighligted->DisplaySelectedBox(true);
				}
			}
			itr++;
		}
		delete mRayScanQuery;
	}
	return true;
}

CEGUI::MouseButton convertButton(OIS::MouseButtonID buttonID)
{
    switch (buttonID)
    {
    case OIS::MB_Left:
        return CEGUI::LeftButton;
        break;

    case OIS::MB_Right:
        return CEGUI::RightButton;
        break;

    case OIS::MB_Middle:
        return CEGUI::MiddleButton;
        break;

    default:
        return CEGUI::LeftButton;
        break;
    }
}

bool BallGame::ApplyForceChangesToCasePushBCallback(const CEGUI::EventArgs &event)
{
	dVector *force_dir = NULL;
	if(UnderEditCase == NULL)
		return true;
	UnderEditCaseForce = NAN;
	if(CaseHasForce == true)
	{
		UnderEditCaseForce = CEGUI::PropertyHelper<float>::fromString(CaseForceValueEditB->getText());
		if(force_directed == true)
			force_dir = new dVector(force_direction.m_x, force_direction.m_y, force_direction.m_z);
	}
	UnderEditCase->SetForceToApply(UnderEditCaseForce, force_dir);

	return true;
}

bool BallGame::CaseHasForceToggleBCallback(const CEGUI::EventArgs &event)
{
	const CEGUI::WindowEventArgs &e = (const CEGUI::WindowEventArgs &)event;
	LOG << "Update buttons by CE Callback of " << e.window->getName() << std::endl;
	CaseHasForceToggleB->setMutedState(true);
	CaseHasForceDirectionToggleB->setMutedState(true);
	if(CaseHasForceToggleB->isSelected())
		CaseHasForce = true;
	else
		CaseHasForce = false;
	UpdateEditButtons();
	CaseHasForceDirectionToggleB->setMutedState(false);
	CaseHasForceToggleB->setMutedState(false);
    return true;
}

bool BallGame::CaseHasForceDirectionToggleBCallback(const CEGUI::EventArgs &event)
{
	const CEGUI::WindowEventArgs &e = (const CEGUI::WindowEventArgs &)event;
	LOG << "Update buttons by CE Callback of " << e.window->getName() << std::endl;
	CaseHasForceToggleB->setMutedState(true);
	CaseHasForceDirectionToggleB->setMutedState(true);
	if(CaseHasForceDirectionToggleB->isSelected())
	{
		force_directed = true;
		ForcesArrows = UnderEditCase->CreateForceArrows(mSceneMgr);
	}
	else
	{
		force_directed = false;
		if(ForcesArrows != NULL)
		{
			LOG << "Remove Arrows child" << std::endl;
			mSceneMgr->getRootSceneNode()->removeAndDestroyChild("Arrows");
			ForcesArrows = NULL;
		}
	}
	UpdateEditButtons();
	CaseHasForceDirectionToggleB->setMutedState(false);
	CaseHasForceToggleB->setMutedState(false);
    return true;
}

void BallGame::UpdateEditButtons(void)
{
	LOG << "Update Edit buttons"<< std::endl;
	CaseHasForceToggleB->setVisible(true);
	CaseHasForceToggleB->setDisabled(false);
	CaseHasForceDirectionToggleB->setVisible(true);
	CaseForceValueEditB->setVisible(true);
	CaseForceDirectionXValueEditB->setVisible(true);
	CaseForceDirectionYValueEditB->setVisible(true);
	CaseForceDirectionZValueEditB->setVisible(true);
	NormalizeCaseForceDirectionPushB->setVisible(true);
	ApplyForceChangesToCasePushB->setVisible(true);
	ApplyForceChangesToCasePushB->setDisabled(false);

	if(CaseHasForce == false)
	{
		LOG << "No Base Force"<< std::endl;
		CaseHasForceToggleB->setSelected(false);
		CaseHasForceDirectionToggleB->setDisabled(true);
		CaseHasForceDirectionToggleB->setSelected(false);
		CaseForceValueEditB->setDisabled(true);
		CaseForceValueEditB->setText("");
		CaseForceDirectionXValueEditB->setDisabled(true);
		CaseForceDirectionXValueEditB->setText("");
		CaseForceDirectionYValueEditB->setDisabled(true);
		CaseForceDirectionYValueEditB->setText("");
		CaseForceDirectionZValueEditB->setDisabled(true);
		CaseForceDirectionZValueEditB->setText("");
	}
	else
	{
		LOG << "Base Force is present"<< std::endl;
		CaseHasForceToggleB->setSelected(true);
		CaseHasForceDirectionToggleB->setDisabled(false);
		CaseForceValueEditB->setText(toCEGUIString(UnderEditCaseForce));
		CaseForceValueEditB->setDisabled(false);

		if(force_directed == false)
		{
			LOG << "Not Directed Force"<< std::endl;
			CaseHasForceDirectionToggleB->setSelected(false);
			CaseForceDirectionXValueEditB->setDisabled(true);
			CaseForceDirectionYValueEditB->setText("");
			CaseForceDirectionYValueEditB->setDisabled(true);
			CaseForceDirectionXValueEditB->setText("");
			CaseForceDirectionZValueEditB->setDisabled(true);
			CaseForceDirectionZValueEditB->setText("");
			NormalizeCaseForceDirectionPushB->setDisabled(true);

		}
		else
		{
			LOG << "Directed Force"<< std::endl;
			CaseHasForceDirectionToggleB->setSelected(true);
			CaseForceDirectionXValueEditB->setDisabled(false);
			CaseForceDirectionYValueEditB->setDisabled(false);
			CaseForceDirectionZValueEditB->setDisabled(false);
			CaseForceDirectionXValueEditB->setText(toCEGUIString(force_direction.m_x));
			CaseForceDirectionYValueEditB->setText(toCEGUIString(force_direction.m_y));
			CaseForceDirectionZValueEditB->setText(toCEGUIString(force_direction.m_z));
			NormalizeCaseForceDirectionPushB->setDisabled(false);
		}
	}
}

Ogre::SceneNode *CaseEntity::CreateForceArrows(Ogre::SceneManager *Scene)
{
	Ogre::ManualObject *ForcesArrows = new ManualObject("Arrows");
	Vector3 coords = OgreEntity->_getWorldAABB().getCenter();
	coords.z = OgreEntity->_getWorldAABB().getMaximum().z;
	ForcesArrows->begin("Material.001", Ogre::RenderOperation::OT_LINE_STRIP);
	ForcesArrows->position (0.0, 0.0, 0.0);
	ForcesArrows->position (40.0, 0.0, 0.0);
	ForcesArrows->position (0.0, 0.0, 0.0);
	ForcesArrows->position (0.0, 40.0, 0.0);
	ForcesArrows->position (0.0, 0.0, 0.0);
	ForcesArrows->position (0.0, 0.0, 40.0);
	ForcesArrows->end();
	LOG << "Create Arrows child" << std::endl;
	Ogre::SceneNode *node = (Ogre::SceneNode*)Scene->getRootSceneNode()->createChildSceneNode("Arrows");
	node->attachObject(ForcesArrows);
	node->setPosition(coords);
	if(force_direction != NULL && !isnanf(force_direction->m_x) && !isnanf(force_direction->m_y) && !isnanf(force_direction->m_z))
		node->setScale(force_direction->m_x, force_direction->m_y, force_direction->m_z);

	return node;
}

void BallGame::UpdateForceArrows(void)
{
	if(ForcesArrows == NULL)
		return;
	LOG << "Scale Force Arrows" << std::endl;
	ForcesArrows->setScale(force_direction.m_x, force_direction.m_y, force_direction.m_z);
}

void BallGame::EditCase(CaseEntity *Entity)
{
	if(ForcesArrows != NULL)
	{
		LOG << "Remove Arrows child" << std::endl;
		mSceneMgr->getRootSceneNode()->removeAndDestroyChild("Arrows");
		ForcesArrows = NULL;
	}
	if(mode != Editing)
		return;
	if(UnderEditCase != NULL)
	{
		UnderEditCase->DisplaySelectedBox(false);
	}
	UnderEditCase = Entity;

	if(UnderEditCase != NULL && PlacementMode != Delete)
	{
		const dVector *case_force_direction = UnderEditCase->getForceDirection();
		CaseHasForceToggleB->setMutedState(true);
		CaseHasForceDirectionToggleB->setMutedState(true);

		UnderEditCase->DisplaySelectedBox(true);
		UnderEditCaseForce = UnderEditCase->getForce();
		if(isnan(UnderEditCaseForce))
		{
			LOG << "EditCase Force not present" << std::endl;
			CaseHasForce = false;
		}
		else
		{
			LOG << "EditCase Force present" << std::endl;
			CaseHasForce = true;
		}
		if(case_force_direction == NULL)
		{
			LOG << "EditCase Force not directed" << std::endl;
			force_directed = false;
		}
		else
		{
			LOG << "EditCase Force directed" << std::endl;
			force_directed = true;
			force_direction.m_x = case_force_direction->m_x;
			force_direction.m_y = case_force_direction->m_y;
			force_direction.m_z = case_force_direction->m_z;
			ForcesArrows = UnderEditCase->CreateForceArrows(mSceneMgr);
		}
		LOG << "Update buttons by Mouse Pressed" << std::endl;
		UpdateEditButtons();

		CaseHasForceDirectionToggleB->setMutedState(false);
		CaseHasForceToggleB->setMutedState(false);
	}
	else
	{
		CaseHasForceToggleB->setVisible(false);
		CaseHasForceDirectionToggleB->setVisible(false);
		CaseForceValueEditB->setVisible(false);
		CaseForceDirectionXValueEditB->setVisible(false);
		CaseForceDirectionYValueEditB->setVisible(false);
		CaseForceDirectionZValueEditB->setVisible(false);
		NormalizeCaseForceDirectionPushB->setVisible(false);
		ApplyForceChangesToCasePushB->setVisible(false);
	}


}

bool BallGame::ApplyMassChangesToBallPushBCallback(const CEGUI::EventArgs &event)
{
	if(UnderEditBall == NULL)
		return true;
	UnderEditBallMass =	CEGUI::PropertyHelper<float>::fromString(BallMassValueEditB->getText());
	UnderEditBall->setMass(UnderEditBallMass);
	return true;
}

void BallGame::EditBall(BallEntity *Entity)
{
	if(mode != Editing)
		return;
	if(UnderEditBall != NULL)
		UnderEditBall->DisplaySelectedBox(false);
	UnderEditBall = Entity;

	if(UnderEditBall != NULL && PlacementMode != Delete)
	{
		UnderEditBall->DisplaySelectedBox(true);
		UnderEditBallMass = UnderEditBall->getMass();
		BallMassValueEditB->setVisible(true);
		BallMassValueEditB->setDisabled(false);
		BallMassValueEditB->setText(toCEGUIString(UnderEditBallMass));
		ApplyMassChangesToBallPushB->setVisible(true);
		ApplyMassChangesToBallPushB->setDisabled(false);
	}
	else
	{
		BallMassValueEditB->setVisible(false);
		ApplyMassChangesToBallPushB->setVisible(false);
	}
}

void BallGame::MultiSelectionSetEmpty(void)
{
	std::list<BallGameEntity*>::iterator iter(UnderEditEntites.begin());
	while(iter != UnderEditEntites.end())
	{
		BallGameEntity *Entity = *iter;
		if(Entity == NULL)
			continue;
		Entity->DisplaySelectedBox(false);
		iter = UnderEditEntites.erase(iter);
	}
	GroupElementsB->setMutedState(true);
	GroupElementsB->setVisible(false);
	GroupElementsB->setMutedState(false);
}

bool BallGame::ManageMultiSelectionSet(BallGameEntity *entity)
{
	bool add_it = true;
	std::list<BallGameEntity*>::iterator iter(UnderEditEntites.begin());
	while(iter != UnderEditEntites.end())
	{
		BallGameEntity *got = *iter;
		if(got == NULL)
		{
			iter++;
			continue;
		}
		if(entity == got)
		{
			add_it = false;
			LOG << "Remove Entity " << entity << " from MultiSelection" << std::endl;
			entity->DisplaySelectedBox(false);
			UnderEditEntites.erase(iter);
			break;
		}
		iter++;
	}
	if(add_it == true)
	{
		LOG << "Add Entity " << entity << " to MultiSelection" << std::endl;
		entity->DisplaySelectedBox(true);
		UnderEditEntites.push_back(entity);
	}

	GroupEntity *to_check = NULL;
	bool group_are_identical = true;
	std::list<BallGameEntity*>::iterator iter2(UnderEditEntites.begin());
	while(iter2 != UnderEditEntites.end())
	{
		BallGameEntity *CheckEnt = *(iter2++);
		if(CheckEnt == NULL)
			continue;
		if(to_check == NULL)
		{
			to_check = CheckEnt->getGroup();
			if(to_check == NULL)
			{
				group_are_identical = false;
				break;
			}
		}
		else if(to_check != CheckEnt->getGroup())
		{
			group_are_identical = false;
			break;
		}
	}
	GroupElementsB->setMutedState(true);
	if(UnderEditEntites.size() > 1)
		GroupElementsB->setVisible(true);
	else
		GroupElementsB->setVisible(false);
	if(group_are_identical == true)
		GroupElementsB->setSelected(true);
	else
		GroupElementsB->setSelected(false);
	GroupElementsB->setMutedState(false);
	return add_it;
}

bool BallGame::mousePressed( const OIS::MouseEvent &arg, OIS::MouseButtonID id )
{
    CEGUI::GUIContext& context = CEGUI::System::getSingleton().getDefaultGUIContext();
    context.injectMouseButtonDown(convertButton(id));

    //BaseApplication::mousePressed(arg, id);
    if(mode == Editing && MouseOverButton == false)
    {
    	switch(PlacementMode)
    	{
    	case PlaceMove :
    	case PlaceRotate :
    	case PlaceScale :
    		break;
    	case Delete :
    		if(LastHighligted != NULL)
    			PrepareDeleteElement(LastHighligted);
    		break;
    	case EditMove :
    	case EditRotate :
    	case EditScale :
    		if(LastHighligted != NULL)
			{
				//Case Entity ?
				LOG << "Edit by Mouse Pressed" << std::endl;
				GroupEntity *HighlightedGroup = LastHighligted->getGroup();
				if(MultiSelectionMode == true)
				{
					if(HighlightedGroup != NULL)
					{
						std::list<BallGameEntity*> to_add;
						HighlightedGroup->FillListWithChilds(to_add);
						std::list<BallGameEntity*>::iterator iter(to_add.begin());
						while(iter != to_add.end())
						{
							BallGameEntity *child = *(iter++);
							if(child != NULL)
								ManageMultiSelectionSet(child);
						}
					}
					else
						ManageMultiSelectionSet(LastHighligted);
				}
				else
				{
					if(HighlightedGroup == NULL)
					{
						MultiSelectionSetEmpty();
						switch(LastHighligted->getType())
						{
						case Case :
								LOG << "Edit Case by Mouse Pressed" << std::endl;
								EditBall(NULL);//Hide Ball Editing buttons;
								EditCase((CaseEntity*)LastHighligted);
								break;
						case Ball :
								LOG << "Edit Ball by Mouse Pressed" << std::endl;
								EditCase(NULL);//Hide Case Editing buttons;
								EditBall((BallEntity*)LastHighligted);
								break;
						}
					}
					else
					{
						if(UnderEditBall != NULL)
							EditBall(NULL);//Hide Ball Editing buttons;
						if(UnderEditCase != NULL)
							EditCase(NULL);//Hide Case Editing buttons;
						std::list<BallGameEntity*> to_add;
						HighlightedGroup->FillListWithChilds(to_add);
						std::list<BallGameEntity*>::iterator iter(to_add.begin());
						while(iter != to_add.end())
						{
							BallGameEntity *child = *(iter++);
							if(child != NULL)
								ManageMultiSelectionSet(child);
						}
					}
				}
			}
    		break;
    	}
    }
    return true;
}

bool BallGame::mouseReleased( const OIS::MouseEvent &arg, OIS::MouseButtonID id )
{
    CEGUI::GUIContext& context = CEGUI::System::getSingleton().getDefaultGUIContext();
    context.injectMouseButtonUp(convertButton(id));
    //BaseApplication::mouseReleased(arg, id);
    return true;
}

inline void MoveNode(Node *node, Vector3 &addPos)
{
	LOG << "Move Node " << node << " by " << addPos.x << ", " << addPos.y << ", " << addPos.z << std::endl;
	Vector3 pos = node->getPosition();
	pos += addPos;
	node->setPosition(pos);
}

inline void MoveNode(Node *node, float x, float y, float z)
{
	Vector3 addPos(x, y, z);
	MoveNode(node, addPos);
}

void BallGameEntity::Move(float x, float y, float z)
{
	MoveNode(OgreEntity, x, y, z);
}

void BallGameEntity::Move(Vector3 &addPos)
{
	MoveNode(OgreEntity, addPos);
}

void GroupEntity::Move(float x, float y, float z)
{
	MoveNode(OgreEntity, x, y, z);
}

inline void ScaleNode(Node *node, Vector3 &addScale)
{
	LOG << "Scale Node " << node << " by " << addScale.x << ", " << addScale.y << ", " << addScale.z << std::endl;
	Vector3 sc = node->getScale();
	sc += addScale;
	node->setScale(sc);

//	sc = node->getScale();
//	Vector3 dsc = node->_getDerivedScale();
//	LOG << "=> Scale = " << sc.x << ", " << sc.y << ", " << sc.z << std::endl;
//	LOG << "=> DerivatedScale = " << dsc.x << ", " << dsc.y << ", " << dsc.z << std::endl;
}

inline void ScaleNode(Node *node, float x, float y, float z)
{
	Vector3 addScale(x, y, z);
	ScaleNode(node, addScale);
}

void BallGameEntity::Scale(float x, float y, float z)
{
	ScaleNode(OgreEntity, x, y, z);
}

void BallGameEntity::Scale(Vector3 &addScale)
{
	ScaleNode(OgreEntity, addScale);
}

void GroupEntity::Scale(float x, float y, float z)
{
	std::list<BallGameEntity*>::iterator iter(childs.begin());
	while(iter != childs.end())
	{
		BallGameEntity *Entity = *(iter++);
		if(Entity == NULL)
			continue;
		Vector3 childPos = Entity->getRelativePosition();
		Vector3 childScale = Entity->getRelativeScale();
		childPos.x *= x / childScale.x;
		childPos.y *= y / childScale.y;
		childPos.z *= z / childScale.z;
		Entity->Scale(x, y, z);
		//As scale parent is a factor and here we add scaling, not multiply it, we must do it ourself by scaling child and moving it consequently by the same factor (newscale / odlscale) !
		Entity->Move(childPos);
	}
}

inline void RotateNode(Node *node, float x, float y, float z)
{
	node->pitch(Degree(x));
	node->roll(Degree(z));
	node->yaw(Degree(y));
}

void BallGameEntity::Rotate(float x, float y, float z)
{
	RotateNode(OgreEntity, x, y, z);
}

void GroupEntity::Rotate(float x, float y, float z)
{
	RotateNode(OgreEntity, x, y, z);
}

void BallGame::MoveEntities(float x, float y, float z)
{
	if(UnderEditCase != NULL)
		UnderEditCase->Move(x, y, z);
	if(UnderEditBall != NULL)
		UnderEditBall->Move(x, y, z);
	std::list<GroupEntity*> ToMoveGroups;
	std::list<BallGameEntity*>::iterator iter(UnderEditEntites.begin());
	while(iter != UnderEditEntites.end())
	{
		BallGameEntity *Entity = *(iter++);
		GroupEntity *EntityGroup;
		if(Entity == NULL)
			continue;
		if((EntityGroup = Entity->getGroup()) != NULL)
		{
			bool to_add = true;
			std::list<GroupEntity*>::iterator iter(ToMoveGroups.begin());
			while(iter != ToMoveGroups.end())
			{
				GroupEntity *grp = *(iter++);
				if(grp != NULL && grp == EntityGroup)
				{
					to_add = false;
					break;
				}
			}
			if(to_add == true)
				ToMoveGroups.push_back(EntityGroup);
		}
		else
			Entity->Move(x, y, z);
	}
	std::list<GroupEntity*>::iterator iterG(ToMoveGroups.begin());
	while(iterG != ToMoveGroups.end())
	{
		GroupEntity *grp = *(iterG++);
		if(grp == NULL)
			continue;
		grp->Move(x, y, z);
	}
}

void BallGame::RotateEntities(float x, float y, float z)
{
	if(UnderEditCase != NULL)
		UnderEditCase->Rotate(x, y, z);
	if(UnderEditBall != NULL)
		UnderEditBall->Rotate(x, y, z);
	std::list<GroupEntity*> ToRotateGroups;
	std::list<BallGameEntity*>::iterator iter(UnderEditEntites.begin());
	while(iter != UnderEditEntites.end())
	{
		BallGameEntity *Entity = *(iter++);
		GroupEntity *EntityGroup;
		if(Entity == NULL)
			continue;
		if((EntityGroup = Entity->getGroup()) != NULL)
		{
			bool to_add = true;
			std::list<GroupEntity*>::iterator iter(ToRotateGroups.begin());
			while(iter != ToRotateGroups.end())
			{
				GroupEntity *grp = *(iter++);
				if(grp != NULL && grp == EntityGroup)
				{
					to_add = false;
					break;
				}
			}
			if(to_add == true)
				ToRotateGroups.push_back(EntityGroup);
		}
		else
			Entity->Rotate(x, y, z);
	}
	std::list<GroupEntity*>::iterator iterG(ToRotateGroups.begin());
	while(iterG != ToRotateGroups.end())
	{
		GroupEntity *grp = *(iterG++);
		if(grp == NULL)
			continue;
		grp->Rotate(x, y, z);
	}
}

void BallGame::ScaleEntities(float x, float y, float z)
{
	if(UnderEditCase != NULL)
		UnderEditCase->Scale(x, y, z);
	if(UnderEditBall != NULL)
		UnderEditBall->Scale(x, y, z);
	std::list<GroupEntity*> ToScaleGroups;
	std::list<BallGameEntity*>::iterator iter(UnderEditEntites.begin());
	while(iter != UnderEditEntites.end())
	{
		BallGameEntity *Entity = *(iter++);
		GroupEntity *EntityGroup;
		if(Entity == NULL)
			continue;
		if((EntityGroup = Entity->getGroup()) != NULL)
		{
			bool to_add = true;
			std::list<GroupEntity*>::iterator iter(ToScaleGroups.begin());
			while(iter != ToScaleGroups.end())
			{
				GroupEntity *grp = *(iter++);
				if(grp != NULL && grp == EntityGroup)
				{
					to_add = false;
					break;
				}
			}
			if(to_add == true)
				ToScaleGroups.push_back(EntityGroup);
		}
		else
			Entity->Scale(x, y, z);
	}
	std::list<GroupEntity*>::iterator iterG(ToScaleGroups.begin());
	while(iterG != ToScaleGroups.end())
	{
		GroupEntity *grp = *(iterG++);
		if(grp == NULL)
			continue;
		LOG << "Scale Group " << grp << std::endl;
		grp->Scale(x, y, z);
	}
}

bool BallGame::keyPressed(const OIS::KeyEvent &arg)
{
	LOG << "Key pressed " << arg.key << std::endl;

    CEGUI::GUIContext& context = CEGUI::System::getSingleton().getDefaultGUIContext();
    context.injectKeyDown((CEGUI::Key::Scan)arg.key);
    context.injectChar(arg.text);

	//BaseApplication::keyPressed(arg);
    switch (arg.key)
    {
    case OIS::KeyCode::KC_LCONTROL :
    case OIS::KeyCode::KC_RCONTROL :
    	if(MouseOverButton == false)
    	{
			if(PlacementMode == EditMove || PlacementMode == EditRotate || PlacementMode == EditScale || PlacementMode == Delete)
			{
				BallGameEntity *UnderEditEntity = NULL;
				MultiSelectionMode = true;
				LOG << "Activate Multi selection mode" << std::endl;
				if(UnderEditBall != NULL)
					UnderEditEntity = (BallGameEntity*)UnderEditBall;
				if(UnderEditCase != NULL)
					UnderEditEntity = (BallGameEntity*)UnderEditCase;
				EditBall(NULL);
				EditCase(NULL);
				if(UnderEditEntity != NULL)
					ManageMultiSelectionSet(UnderEditEntity);
			}
    	}
    	break;
    case OIS::KeyCode::KC_ESCAPE :
		if(QuitPushB->isVisible())
		{
			StopPhysicPushB->setVisible(false);
			EditModePushB->setVisible(false);
			StatesModePushB->setVisible(false);
			ChooseLevelComboB->setVisible(false);
			NewLevelEditB->setVisible(false);
			NewLevelCreateB->setVisible(false);
			SaveLevelPushB->setVisible(false);
			QuitPushB->setVisible(false);
		}
		else
		{
			StopPhysicPushB->setVisible(true);
			EditModePushB->setVisible(true);
			StatesModePushB->setVisible(true);
			ChooseLevelComboB->setVisible(true);
			NewLevelEditB->setVisible(true);
			NewLevelCreateB->setVisible(true);
			SaveLevelPushB->setVisible(true);
			QuitPushB->setVisible(true);
		}
	    break;
	case OIS::KeyCode::KC_UP:
    	if(MouseOverButton == false)
    	{
			switch(PlacementMode)
			{
			case PlaceMove :
				if(ToBePlacedEntity != NULL)
					ToBePlacedEntity->Move(0, 10, 0);
				break;
			case EditMove :
				MoveEntities(0, 10, 0);
				break;
			case PlaceRotate :
				if(ToBePlacedEntity != NULL)
					ToBePlacedEntity->Rotate(0, 10, 0);
				break;
			case EditRotate :
				RotateEntities(0, 10, 0);
				break;
			case PlaceScale :
				if(ToBePlacedEntity != NULL)
					ToBePlacedEntity->Scale(0, 10, 0);
				break;
			case EditScale :
				ScaleEntities(0, 10, 0);
				break;
			}
		}
	    break;
	case OIS::KeyCode::KC_DOWN:
    	if(MouseOverButton == false)
    	{
			switch(PlacementMode)
			{
			case PlaceMove :
				if(ToBePlacedEntity != NULL)
					ToBePlacedEntity->Move(0, -10, 0);
				break;
			case EditMove :
				MoveEntities(0, -10, 0);
				break;
			case PlaceRotate :
				if(ToBePlacedEntity != NULL)
					ToBePlacedEntity->Rotate(0, -10, 0);
				break;
			case EditRotate :
				RotateEntities(0, -10, 0);
				break;
			case PlaceScale :
				if(ToBePlacedEntity != NULL)
					ToBePlacedEntity->Scale(0, -10, 0);
				break;
			case EditScale :
				ScaleEntities(0, -10, 0);
				break;
			}
		}
	    break;
	case OIS::KeyCode::KC_RIGHT:
    	if(MouseOverButton == false)
    	{
			switch(PlacementMode)
			{
			case PlaceMove :
				if(ToBePlacedEntity != NULL)
					ToBePlacedEntity->Move(10, 0, 0);
				break;
			case EditMove :
				MoveEntities(10, 0, 0);
				break;
			case PlaceRotate :
				if(ToBePlacedEntity != NULL)
					ToBePlacedEntity->Rotate(10, 0, 0);
				break;
			case EditRotate :
				RotateEntities(10, 0, 0);
				break;
			case PlaceScale :
				if(ToBePlacedEntity != NULL)
					ToBePlacedEntity->Scale(10, 0, 0);
				break;
			case EditScale :
				ScaleEntities(10, 0, 0);
				break;
			}
		}
	    break;
	case OIS::KeyCode::KC_LEFT:
    	if(MouseOverButton == false)
    	{
			switch(PlacementMode)
			{
			case PlaceMove :
				if(ToBePlacedEntity != NULL)
					ToBePlacedEntity->Move(-10, 0, 0);
				break;
			case EditMove :
				MoveEntities(-10, 0, 0);
				break;
			case PlaceRotate :
				if(ToBePlacedEntity != NULL)
					ToBePlacedEntity->Rotate(-10, 0, 0);
				break;
			case EditRotate :
				RotateEntities(-10, 0, 0);
				break;
			case PlaceScale :
				if(ToBePlacedEntity != NULL)
					ToBePlacedEntity->Scale(-10, 0, 0);
				break;
			case EditScale :
				ScaleEntities(-10, 0, 0);
				break;
			}
		}
	    break;
	case OIS::KeyCode::KC_PGUP:
    	if(MouseOverButton == false)
    	{
			switch(PlacementMode)
			{
			case PlaceMove :
				if(ToBePlacedEntity != NULL)
					ToBePlacedEntity->Move(0, 0, 10);
				break;
			case EditMove :
				MoveEntities(0, 0, 10);
				break;
			case PlaceRotate :
				if(ToBePlacedEntity != NULL)
					ToBePlacedEntity->Rotate(0, 0, 10);
				break;
			case EditRotate :
				RotateEntities(0, 0, 10);
				break;
			case PlaceScale :
				if(ToBePlacedEntity != NULL)
					ToBePlacedEntity->Scale(0, 0, 10);
				break;
			case EditScale :
				ScaleEntities(0, 0, 10);
				break;
			}
		}
	    break;
	case OIS::KeyCode::KC_PGDOWN:
    	if(MouseOverButton == false)
    	{
			switch(PlacementMode)
			{
			case PlaceMove :
				if(ToBePlacedEntity != NULL)
					ToBePlacedEntity->Move(0, 0, -10);
				break;
			case EditMove :
				MoveEntities(0, 0, -10);
				break;
			case PlaceRotate :
				if(ToBePlacedEntity != NULL)
					ToBePlacedEntity->Rotate(0, 0, -10);
				break;
			case EditRotate :
				RotateEntities(0, 0, -10);
				break;
			case PlaceScale :
				if(ToBePlacedEntity != NULL)
					ToBePlacedEntity->Scale(0, 0, -10);
				break;
			case EditScale :
				ScaleEntities(0, 0, -10);
				break;
			}
		}
	    break;
	case OIS::KeyCode::KC_SPACE:
		if(mode == Editing && MouseOverButton == false)
		{
			switch(PlacementMode)
			{
			case PlaceMove :
			case PlaceRotate :
			case PlaceScale :
				PlaceNewElement();
				break;
			case EditMove :
			case EditRotate :
			case EditScale :
				PlaceUnderEditElement();
				break;
			}
		}
		break;
	case OIS::KeyCode::KC_DELETE:
		if(mode == Editing && MouseOverButton == false)
			DeleteElement();
		break;
	case OIS::KeyCode::KC_M:
		if(mode == Editing)
			SetMoveNewElement();
		break;
	case OIS::KeyCode::KC_R:
		if(mode == Editing)
			SetRotateNewElement();
		break;
	case OIS::KeyCode::KC_S:
		if(mode == Editing)
			SetScaleNewElement();
		break;
	case OIS::KeyCode::KC_PAUSE:
		if(m_suspendPhysicsUpdate)
			_StartPhysic();
		else
			_StopPhysic();
		break;
	case OIS::KeyCode::KC_F1:
		SwitchEditMode();
		break;
    }
    return true;
}

bool BallGame::keyReleased( const OIS::KeyEvent &arg )
{
    CEGUI::GUIContext& context = CEGUI::System::getSingleton().getDefaultGUIContext();
    if(context.injectKeyUp((CEGUI::Key::Scan)arg.key))
    	return true;
    switch (arg.key)
    {
    case OIS::KeyCode::KC_LCONTROL :
    case OIS::KeyCode::KC_RCONTROL :
		LOG << "Unactivate Multi selection mode" << std::endl;
		MultiSelectionMode = false;
    	break;
    }
	//BaseApplication::keyReleased(arg);
    return true;
}

bool BallGame::NewLevelCreateBCallback(const CEGUI::EventArgs &e)
{
	EmptyLevel();
	String level;
	String Filename;
	BuildLevelFilename(level, Filename);
	level = NewLevelEditB->getText().c_str();
	SetLevel(level, Filename);
	if(mode == Running)
		SwitchEditMode();
	return true;
}

bool BallGame::SaveLevelPushBCallback(const CEGUI::EventArgs &e)
{
	String export_str;
	ExportLevelIntoJson(export_str);
	std::ofstream myfile;
	myfile.open (LevelFilename.c_str());
	myfile << export_str;
	myfile.close();
	return true;
}

bool BallGame::ChooseLevelComboBCallback(const CEGUI::EventArgs &e)
{
	CEGUI::ListboxTextItem *item = (CEGUI::ListboxTextItem*)ChooseLevelComboB->getSelectedItem();
	String level(item->getText().c_str()), *filename = (String*)item->getUserData();
	SetLevel(level, *filename);
	ChangeLevel();
	return true;
}

void BallGame::SetLevel(String &level_name, String &levelFilename)
{
	Level = level_name;
	LevelFilename = levelFilename;
	LevelNameBanner->setText((CEGUI::utf8*)Level.c_str());
}

void BallEntity::ExportToJson(rapidjson::Value &v, rapidjson::Document::AllocatorType& allocator)
{
	BallGameEntity::ExportToJson(v, allocator);
	v.AddMember("Mass", InitialMass, allocator);
}

void CaseEntity::ExportToJson(rapidjson::Value &v, rapidjson::Document::AllocatorType& allocator)
{
	BallGameEntity::ExportToJson(v, allocator);
	v.AddMember("Type", (int)type, allocator);
	if(isnan(force_to_apply))
		v.AddMember("ForcePresent", false, allocator);
	else
	{
		v.AddMember("ForcePresent", true, allocator);
		v.AddMember("ForceValue", force_to_apply, allocator);
		if(force_direction == NULL)
			v.AddMember("ForceDirectionPresent", false, allocator);
		else
		{
			dVector *force_dir = force_direction;
			v.AddMember("ForceDirectionPresent", true, allocator);
			v.AddMember("ForceDirectionX", force_dir->m_x, allocator);
			v.AddMember("ForceDirectionY", force_dir->m_y, allocator);
			v.AddMember("ForceDirectionZ", force_dir->m_z, allocator);
			v.AddMember("ForceDirectionW", force_dir->m_w, allocator);
		}
	}
}

void CaseEntity::CreateFromJson(rapidjson::Value &v, BallGame *Game, NewtonWorld *m_world, Node *parent)
{
	ImportFromJson(v, Game, parent);
	dVector NewtonBodyLocation;
	dVector NewtonBodySize;
	NewtonBody *newtonBody;
	Entity* ogreEntity = (Ogre::Entity*)OgreEntity->getAttachedObject(0);
	dMatrix *casematrix = PrepareNewtonBody(NewtonBodyLocation, NewtonBodySize);
	NewtonCollision *collision_tree = NULL;

	Matrix4 ogre_matrix;
	ogre_matrix.makeTransform(Vector3::ZERO, InitialScale, Quaternion::IDENTITY);
	const MeshPtr ptr = ogreEntity->getMesh();
	collision_tree = ParseEntity(m_world, ptr, ogre_matrix);

	int defaultMaterialID = NewtonMaterialGetDefaultGroupID (m_world);
	newtonBody = WorldAddCase(m_world, NewtonBodySize, defaultMaterialID, *casematrix, collision_tree);

	setNewtonBody(newtonBody);
}

void CaseEntity::ImportFromJson(rapidjson::Value &v, BallGame *Game, Node *parent)
{
	BallGameEntity::ImportFromJson(v, Game, parent);
	float force_json =  NAN;
	dVector *direction_json = NULL;
	type = (CaseEntity::CaseType)v["Type"].GetInt();
	if(v["ForcePresent"].GetBool() == true)
	{
		force_json = v["ForceValue"].GetFloat();
		if(v["ForceDirectionPresent"].GetBool() == true)
		{
			float xjson, yjson, zjson, wjson;
			xjson = v["ForceDirectionX"].GetFloat();
			yjson = v["ForceDirectionY"].GetFloat();
			zjson = v["ForceDirectionZ"].GetFloat();
			wjson = v["ForceDirectionW"].GetFloat();
			direction_json = new dVector(xjson, yjson, zjson, wjson);
		}
	}
	SetForceToApply(force_json, direction_json);
}

void BallGameEntity::ExportToJson(rapidjson::Value &v, rapidjson::Document::AllocatorType& allocator)
{
	Ogre::Entity *entity = (Ogre::Entity*)OgreEntity->getAttachedObject(0);
	const char *cname = (const char*)entity->getMesh().get()->getName().c_str();
	rapidjson::Value name;
	name.SetString(cname, allocator);
	v.AddMember("Mesh", name, allocator);
	const char *ogrename = (const char*)OgreEntity->getName().c_str();
	name.SetString(ogrename, allocator);
	v.AddMember("NodeName", name, allocator);
	if(Group != NULL)
	{
		rapidjson::Value gname;
		const char *groupname = (const char*)Group->getName().c_str();
		gname.SetString(groupname, allocator);
		v.AddMember("GroupName", gname, allocator);
	}
	else
		v.AddMember("GroupName", "", allocator);
	v.AddMember("PosX", InitialPos.x, allocator);
	v.AddMember("PosY", InitialPos.y, allocator);
	v.AddMember("PosZ", InitialPos.z, allocator);
	v.AddMember("ScaleX", InitialScale.x, allocator);
	v.AddMember("ScaleY", InitialScale.y, allocator);
	v.AddMember("ScaleZ", InitialScale.z, allocator);
	v.AddMember("OrientationX", InitialOrientation.x, allocator);
	v.AddMember("OrientationY", InitialOrientation.y, allocator);
	v.AddMember("OrientationZ", InitialOrientation.z, allocator);
	v.AddMember("OrientationW", InitialOrientation.w, allocator);
}

void BallEntity::CreateFromJson(rapidjson::Value &v, BallGame *Game, NewtonWorld *m_world, Node *parent)
{
	ImportFromJson(v, Game, parent);
	dVector NewtonBodyLocation;
	dVector NewtonBodySize;
	NewtonBody *BallBody;

	dMatrix *ballmatrix = PrepareNewtonBody(NewtonBodyLocation, NewtonBodySize);

	int defaultMaterialID = NewtonMaterialGetDefaultGroupID (m_world);
	BallBody = WorldAddBall(m_world, InitialMass, NewtonBodySize, defaultMaterialID, *ballmatrix);

	setNewtonBody(BallBody);
}

void BallEntity::ImportFromJson(rapidjson::Value &v, BallGame *Game, Node *parent)
{
	BallGameEntity::ImportFromJson(v, Game, parent);
	InitialMass = v["Mass"].GetFloat();
}

void BallGameEntity::ImportFromJson(rapidjson::Value &v, BallGame *Game, Node *parent)
{
	const char *meshname = v["Mesh"].GetString();
	Ogre::SceneManager *mSceneMgr = Game->getSceneManager();
	Entity* ogreEntity = mSceneMgr->createEntity(meshname);
	InitialPos.x = v["PosX"].GetFloat();
	InitialPos.y = v["PosY"].GetFloat();
	InitialPos.z = v["PosZ"].GetFloat();
	InitialScale.x = v["ScaleX"].GetFloat();
	InitialScale.y = v["ScaleY"].GetFloat();
	InitialScale.z = v["ScaleZ"].GetFloat();
	InitialOrientation.x = v["OrientationX"].GetFloat();
	InitialOrientation.y = v["OrientationY"].GetFloat();
	InitialOrientation.z = v["OrientationZ"].GetFloat();
	InitialOrientation.w = v["OrientationW"].GetFloat();

	SceneNode* ogreNode;
	const char *nodename = v["NodeName"].GetString();
	if(parent == NULL)
		ogreNode = mSceneMgr->getRootSceneNode()->createChildSceneNode(nodename, InitialPos);
	else
		ogreNode = (SceneNode*)parent->createChild(InitialPos);
	ogreNode->attachObject(ogreEntity);
	setOgreNode(ogreNode);
	const char *Groupname = v["GroupName"].GetString();
	if(strcmp(Groupname, "") != 0)
	{
		GroupEntity *Grp = Game->findGroup(Groupname);
		if(Grp != NULL)
		{
			LOG << "Add Child " << nodename << " to Group " << Groupname << std::endl;
			Grp->AddChild(this);
		}
	}
}

void BallGame::DeleteGroup(GroupEntity *Entity, std::list<GroupEntity*>::iterator *iter)
{
	RemoveGroup(Entity, iter);
	Entity->Finalize();
	delete Entity;
}

void BallGame::RemoveGroup(GroupEntity *Entity, std::list<GroupEntity*>::iterator *iter)
{
	if(iter != NULL)
		*iter = Groups.erase(*iter);
	else
	{
		std::list<GroupEntity*>::iterator it(Groups.begin());
		while(it != Groups.end())
		{
			GroupEntity *B = *it;
			if(B == Entity)
			{
				it = Groups.erase(it);
				break;
			}
			it++;
		}
	}
}

GroupEntity *BallGame::findGroup(const char * const name_c)
{
	String name(name_c);
	std::list<GroupEntity*>::iterator it(Groups.begin());
	while(it != Groups.end())
	{
		GroupEntity *B = *it;
		if(B != NULL && B->getName() == name)
			return B;
			break;
		it++;
	}
	return NULL;
}

void BallGame::DeleteBall(BallEntity *Entity, std::list<BallEntity*>::iterator *iter)
{
	RemoveBall(Entity, iter);
	GroupEntity *Group = Entity->getGroup();
	if(Group != NULL)
		Group->DelChild((BallGameEntity*)Entity);
	Entity->Finalize();
	delete Entity;
}

void BallGame::RemoveBall(BallEntity *Entity, std::list<BallEntity*>::iterator *iter)
{
	if(iter != NULL)
		*iter = Balls.erase(*iter);
	else
	{
		std::list<BallEntity*>::iterator it(Balls.begin());
		while(it != Balls.end())
		{
			BallEntity *B = *it;
			if(B == Entity)
			{
				it = Balls.erase(it);
				break;
			}
			it++;
		}
	}
}

void BallGame::DeleteCase(CaseEntity *Entity, std::list<CaseEntity*>::iterator *iter)
{
	RemoveCase(Entity, iter);
	GroupEntity *Group = Entity->getGroup();
	if(Group != NULL)
		Group->DelChild((BallGameEntity*)Entity);
	Entity->Finalize();
	delete Entity;
}

void BallGame::RemoveCase(CaseEntity *Entity, std::list<CaseEntity*>::iterator *iter)
{
	if(iter != NULL)
		*iter = Cases.erase(*iter);
	else
	{
		std::list<CaseEntity*>::iterator it(Cases.begin());
		while(it != Cases.end())
		{
			CaseEntity *C = *it;
			if(C == Entity)
			{
				it = Cases.erase(it);
				break;
			}
			it++;
		}
	}
}

void BallGame::EmptyLevel(void)
{
	_StopPhysic();
	std::list<CaseEntity*>::iterator Cit(Cases.begin());
	while(Cit != Cases.end())
	{
		CaseEntity *Case = *Cit;
		if(Case != NULL)
			DeleteCase(Case, &Cit);
		else
			Cit++;
	}
	assert(Cases.empty());
	std::list<BallEntity*>::iterator Bit(Balls.begin());
	while(Bit != Balls.end())
	{
		BallEntity *Ball = *Bit;
		if(Ball != NULL)
			DeleteBall(Ball, &Bit);
		else
			Bit++;
	}
	assert(Balls.empty());
	std::list<GroupEntity*>::iterator Git(Groups.begin());
	while(Git != Groups.end())
	{
		GroupEntity *Group = *Git;
		if(Group != NULL)
			DeleteGroup(Group, &Git);
		else
			Git++;
	}
	assert(Groups.empty());
	CasesUnderCollide.clear();
	EmptyStatesList();
}

void BallGame::EmptyLevelsList(void)
{
	for (size_t cmpt = 0; cmpt < ChooseLevelComboB->getItemCount(); cmpt++)
	{
		CEGUI::ListboxTextItem *item = (CEGUI::ListboxTextItem*)ChooseLevelComboB->getListboxItemFromIndex(cmpt);
		String *str = (String*)item->getUserData();
		if(str != NULL)
			delete str;
		ChooseLevelComboB->removeItem(item);
	}
}

void BallGame::EmptyStatesList(void)
{
	for (size_t cmpt = 0; cmpt < ChooseStateToLoadB->getItemCount(); cmpt++)
	{
		CEGUI::ListboxTextItem *item = (CEGUI::ListboxTextItem*)ChooseStateToLoadB->getListboxItemFromIndex(cmpt);
		String *str = (String*)item->getUserData();
		if(str != NULL)
			delete str;
		ChooseStateToLoadB->removeItem(item);
	}
}

void BallGame::LoadStatesList(void)
{
	LOG << "Load selected state list" << std::endl;
	ChooseStateToLoadB->setMutedState(true);
	EmptyStatesList();
	SetupStatesButtons();
	String globfilter(LEVELS_FOLDER);
	globfilter += Level;
	globfilter += "*." STATES_EXTENSION;
    glob_t glob_result;
	memset(&glob_result, 0, sizeof(glob_result));
	glob(globfilter.c_str(), 0, NULL, &glob_result);
	CEGUI::ListboxTextItem *Selecteditem = NULL;
	for(size_t i = 0; i < glob_result.gl_pathc; ++i)
	{
		String *globname, filename;
		globname = new String(glob_result.gl_pathv[i]);
		size_t slashpos = globname->find_last_of('/'), dotpos = globname->find_last_of('.');
		filename = globname->substr(slashpos + 1, dotpos - (slashpos + 1));
		CEGUI::ListboxTextItem *item = new CEGUI::ListboxTextItem(filename);
		item->setUserData(globname);
		ChooseStateToLoadB->addItem(item);
		if(Selecteditem == NULL)
        	Selecteditem = item;
	}
	globfree(&glob_result);
	ChooseStateToLoadB->setSize(CEGUI::USize(CEGUI::UDim(0, 150), CEGUI::UDim(0, 40 * (ChooseStateToLoadB->getItemCount() + 1))));
	ChooseStateToLoadB->setMutedState(false);
	LOG << "Set default selected item" << std::endl;
	if(Selecteditem != NULL)
		ChooseStateToLoadB->setItemSelectState((CEGUI::ListboxItem*)Selecteditem, true);
	else
		ChooseStateToLoadB->setText("");
}

void BallGame::ChangeLevel(void)
{
	if(mode == Editing)
		SwitchEditMode();
	else
		_StopPhysic();
	EmptyLevel();
	ImportLevelFromJson();
	LoadStatesList();
}

void BallGame::ImportLevelFromJson(Node *parent)
{
	std::ifstream myfile;
	std::stringstream buffer;
	myfile.open (LevelFilename.c_str());
	buffer << myfile.rdbuf();
	myfile.close();
	rapidjson::Document in;
	in.Parse(buffer.str().c_str());
	int idx = 0;
	//Parsing Groups
	rapidjson::Value &groups = in[idx++];
	for(int cmpt = 0; cmpt < groups.Size(); cmpt++)
	{
		GroupEntity *newGroup = new GroupEntity();
		rapidjson::Value &groupjson = groups[cmpt];
		newGroup->ImportFromJson(groupjson, this);
		AddGroup(newGroup);
	}
	//Parsing Cases
	rapidjson::Value &cases = in[idx++];
	for(int cmpt = 0; cmpt < cases.Size(); cmpt++)
	{
		CaseEntity *newCase = new CaseEntity();
		rapidjson::Value &casejson = cases[cmpt];
		newCase->CreateFromJson(casejson, this, m_world, parent);
		AddCase(newCase);
	}
	//Parsing Balls
	rapidjson::Value &balls = in[idx++];
	for(int cmpt = 0; cmpt < balls.Size(); cmpt++)
	{
		BallEntity *newBall = new BallEntity();
		rapidjson::Value &balljson = balls[cmpt];
		newBall->CreateFromJson(balljson, this, m_world, parent);
		AddBall(newBall);
	}

	//Now we need to restore links between ogre nodes.
	std::list<GroupEntity*>::iterator iter(Groups.begin());
	while(iter != Groups.end())
	{
		GroupEntity *Grp = *(iter++);
		if(Grp != NULL)
			Grp->ComputeChilds();
	}
}

void BallGame::ExportLevelIntoJson(String &export_str)
{
	rapidjson::Document document;
	document.SetArray();

	rapidjson::Document::AllocatorType& allocator = document.GetAllocator();
	rapidjson::Value groups(rapidjson::kArrayType);

	std::list<GroupEntity*>::iterator Git(Groups.begin());
	while(Git != Groups.end())
	{
		GroupEntity *Entity = *(Git++);
		if(Entity == NULL)
			continue;
		rapidjson::Value JGroup(rapidjson::kObjectType);
		Entity->ExportToJson(JGroup, allocator);

		groups.PushBack(JGroup, allocator);
	}

	document.PushBack(groups, allocator);

	rapidjson::Value cases(rapidjson::kArrayType);

	std::list<CaseEntity*>::iterator Cit(Cases.begin());
	while(Cit != Cases.end())
	{
		CaseEntity *Entity = *(Cit++);
		if(Entity == NULL)
			continue;
		rapidjson::Value JCase(rapidjson::kObjectType);
//		JCase.AddMember("Type", Entity->type, allocator);
		Entity->ExportToJson(JCase, allocator);

		cases.PushBack(JCase, allocator);
	}

	document.PushBack(cases, allocator);

	rapidjson::Value balls(rapidjson::kArrayType);

	std::list<BallEntity*>::iterator Bit(Balls.begin());
	while(Bit != Balls.end())
	{
		BallEntity *Entity = *(Bit++);
		if(Entity == NULL)
			continue;
		rapidjson::Value JCase(rapidjson::kObjectType);
		Entity->ExportToJson(JCase, allocator);

		balls.PushBack(JCase, allocator);
	}

	document.PushBack(balls, allocator);

	rapidjson::StringBuffer strbuf;
	rapidjson::Writer<rapidjson::StringBuffer> writer(strbuf);
	document.Accept(writer);

	export_str = strbuf.GetString();

	LOG << strbuf.GetString() << std::endl;
}
