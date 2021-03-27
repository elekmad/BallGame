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
#include "GameEngine.h"


static int OnBodyAABBOverlap(const NewtonJoint* const contactJoint, dFloat timestep, int threadIndex)
{
	return 1;
}

static int OnSubShapeAABBOverlapTest (const NewtonJoint* const contact, dFloat timestep, const NewtonBody* const body0, const void* const collsionNode0, const NewtonBody* const body1, const void* const collsionNode1, int threadIndex)
{
	return 1;
}

namespace BallGame {

void GameEngine::CustomListenerPostUpdateCallback(dFloat timestep)
{
	if(m_suspendPhysicsUpdate == false)
	{
		std::list<CaseEntity*>::iterator iter1(CasesToBeMoved.begin());
		while(iter1 != CasesToBeMoved.end())
		{
			CaseEntity *Case = *(iter1++);
			if(Case != NULL)
				if(Case->CaseMove(m_microsecunds, timestep) == false)
					Case->ComputeMove();
		}
	}
}

void GameEngine::GameNewtonListener::PostUpdate(dFloat timestep)
{
//	LOG << "PostUpdate CustomListener @" << timestep << std::endl;
	Engine->CustomListenerPostUpdateCallback(timestep);
}


void GameEngine::BodySerialization (NewtonBody* const body, void* const bodyUserData, NewtonSerializeCallback serializeCallback, void* const serializeHandle)
{
	Entity *Ent = (Entity*)NewtonBodyGetUserData(body);

	const char* const bodyIndentification = Ent->getName().c_str();

	std::vector<uint8_t> ident;
	ident.push_back((uint8_t)Ent->getType());
	if(Ent->getType() == BaseEntity::Types::Case)
	{
		CaseEntity *C = (CaseEntity*)Ent;
		void *MovePtr = C->getActualMoveStep();
		uint8_t *MovePtr8 = (uint8_t*)&MovePtr;
		for(int cmpt = 0; cmpt < sizeof(void*); cmpt++)
		{
			ident.push_back(MovePtr8[cmpt]);
		}
	}
	for(int cmpt = 0; cmpt < Ent->getName().length(); cmpt++)
		ident.push_back(Ent->getName().at(cmpt));
	ident.push_back((uint8_t)'\0');//Add '\0' for end of the entity's name
	LOG << "Serialize Entity (" << Ent << "/" << body << ") name :" << Ent->getName() << std::endl;
	int size = (ident.size() + 3) & -4;
	serializeCallback (serializeHandle, &size, sizeof (size));
	serializeCallback (serializeHandle, (void*)ident.data(), size);
}

void GameEngine::BodyDeserialization (NewtonBody* const body, void* const bodyUserData, NewtonDeserializeCallback deserializecallback, void* const serializeHandle)
{
	int size;
	char bodyIndentification[256];
	char *ident;
	GameEngine *Game = (GameEngine*)bodyUserData;

	deserializecallback (serializeHandle, &size, sizeof (size));
	deserializecallback (serializeHandle, bodyIndentification, size);
	bodyIndentification[size] = 0;
	BaseEntity::Types type = (BaseEntity::Types)bodyIndentification[0];
	if(type != BaseEntity::Types::Case)
		ident = &bodyIndentification[1];
	else
		ident = &bodyIndentification[1+sizeof(void*)];

	Entity *Entity = Game->GetEntity(ident);
	LOG << "Deserialize Entity (" << Entity << "/" << body << ") name :" << ident << std::endl;

	if(type == BaseEntity::Types::Case)
	{
		void *MovePtr;
		uint8_t *ptr = (uint8_t*)(&MovePtr);
		for(int cmpt = 0; cmpt < sizeof(void*); cmpt++)
			ptr[cmpt] = (uint8_t)bodyIndentification[1+cmpt];
		((CaseEntity*)Entity)->setActualMoveStep(MovePtr);
	}


	NewtonBodySetUserData (body, Entity);
	Entity->setNewtonBody(body);
	NewtonBodySetTransformCallback(body, Entity::TransformCallback);
	if(Entity->getType() == Entity::Types::Ball)
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

Entity *GameEngine::GetEntity(char *name_c)
{
	String name(name_c);
	std::list<CaseEntity*>::iterator Citer(Cases.begin());
	while(Citer != Cases.end())
	{
		CaseEntity *Ent = *(Citer++);
		if(Ent != NULL && Ent->getName() == name)
			return (Entity*)Ent;
	}
	std::list<BallEntity*>::iterator Biter(Balls.begin());
	while(Biter != Balls.end())
	{
		BallEntity *Ent = *(Biter++);
		if(Ent != NULL && Ent->getName() == name)
			return (Entity*)Ent;
	}
	return NULL;
}

void GameEngine::SerializedPhysicScene(const String* const name)
{
//	NewtonSerializeToFile(m_world, name, NULL, NULL);
	_StopPhysic();
	NewtonSerializeToFile(m_world, name->c_str(), BodySerialization, NULL);
	_StartPhysic();
}

void GameEngine::DeserializedPhysicScene(const String* const name)
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

NewtonWorld* GameEngine::GetNewton(void)
{
	return m_world;
}

void GameEngine::SetCam(float x, float y, float z)
{
    mCamera->setPosition(x, y, z);
}

void GameEngine::MoveCam(float x, float y, float z)
{
	mCamera->moveRelative(Ogre::Vector3(x, y, z));
}

GameEngine::GameEngine() :
		m_asynchronousPhysicsUpdate(false)
		,m_suspendPhysicsUpdate(false)
		,m_physicsFramesCount(0)
		,m_microsecunds(0)
		,m_mainThreadPhysicsTime(0.0f)
		,m_mainThreadPhysicsTimeAcc(0.0f)
{
	m_world = NULL;

	nb_entities = 0;
	listener = NULL;
	SetupNewton();
}

void GameEngine::SetupNewton(void)
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

	listener = new GameNewtonListener(this);
}

GameEngine::~GameEngine()
{
	if(m_world != NULL)
		NewtonDestroy(m_world);//This causes CutomListener to be destroyed too !
}

inline void GameEngine::_updatePhysic(dFloat timestep)
{
	#ifdef DEMO_CHECK_ASYN_UPDATE
			g_checkAsyncUpdate = 1;
#endif
	if (m_asynchronousPhysicsUpdate)
	{
		NewtonUpdateAsync(m_world, timestep);
#ifdef DEMO_CHECK_ASYN_UPDATE
		NewtonWaitForUpdateToFinish(m_world);
		g_checkAsyncUpdate = 0;
#endif
	}
	else
	{
		NewtonUpdate(m_world, timestep);
	}
}

void GameEngine::PostUpdateCallback(const NewtonWorld* const world, dFloat timestep)
{
//	LOG << "Post Update Time " << timestep << std::endl;
/*	LevelEditor* const scene = (LevelEditor*) NewtonWorldGetUserData(world);
	scene->m_cameraManager->FixUpdate(scene->GetNewton(), timestep);
	if (scene->m_updateCamera) {
		scene->m_updateCamera(scene, scene->m_updateCameraContext, timestep);
	}*/
}

void GameEngine::UpdatePhysics(dFloat timestep)
{
	// update the physics
//	LOG << " Update Time " << timestep << std::endl;
	if (m_world && !m_suspendPhysicsUpdate)
	{
		D_TRACKTIME();

		dFloat timestepInSecunds = 1.0f / MAX_PHYSICS_FPS;
		unsigned64 timestepMicrosecunds = unsigned64 (timestepInSecunds * 1000000.0f);

		unsigned64 currentTime = dGetTimeInMicrosenconds ();
		unsigned64 nextTime = currentTime - m_microsecunds;
		if (nextTime > timestepMicrosecunds * 2)
		{
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

			_updatePhysic(timestepInSecunds);

			physicsTime += NewtonGetLastUpdateTime(m_world);

			nextTime -= timestepMicrosecunds;
			m_microsecunds += timestepMicrosecunds;
//			LOG << "Microseconds : " << m_microsecunds << std::endl;
		}

		if (newUpdate)
		{
			m_physicsFramesCount ++;
			m_mainThreadPhysicsTimeAcc += physicsTime;
			if (m_physicsFramesCount >= 16)
			{
				m_mainThreadPhysicsTime = m_mainThreadPhysicsTimeAcc / m_physicsFramesCount;
				m_physicsFramesCount = 0;
				m_mainThreadPhysicsTimeAcc = 0.0f;
			}
		}

//		LOG << "Main Thread Physic Time : " << m_mainThreadPhysicsTime << std::endl;
	}
}

void GameEngine::_StartPhysic(void)
{
	m_suspendPhysicsUpdate = false;
}

void GameEngine::_StopPhysic(void)
{
	m_suspendPhysicsUpdate = true;
	NewtonWaitForUpdateToFinish(m_world);
}

void GameEngine::SetupGame(void)
{
	mSceneMgr->setAmbientLight(ColourValue(0.5, 0.5, 0.5));

    Light* light = mSceneMgr->createLight("MainLight");
    SceneNode* lightNode = mSceneMgr->getRootSceneNode()->createChildSceneNode("MainLight");
    lightNode->attachObject(light);
    lightNode->setPosition(20, 80, 50);

    SetCam(-184, -253, 352);
    mCamera->setOrientation(Ogre::Quaternion(0.835422, 0.393051, -0.238709, -0.300998));
}

void GameEngine::OnContactCollision (const NewtonJoint* contactJoint, dFloat timestep, int threadIndex)
{
	NewtonBody *body0 = NewtonJointGetBody0(contactJoint);
	NewtonBody *body1 = NewtonJointGetBody1(contactJoint);
	GameEngine *Game = (GameEngine*)NewtonWorldGetUserData(NewtonBodyGetWorld(body0));
	Entity *Entity0 = (Entity*)NewtonBodyGetUserData(body0);
	Entity *Entity1 = (Entity*)NewtonBodyGetUserData(body1);
	BallEntity *BallToCheck = NULL;
	CaseEntity *CaseToCheck = NULL;
	switch(Entity0->getType())
	{
	case Entity::Types::Ball :
//		LOG << "Ball " << Entity0 << " Colliding with ";
		BallToCheck = (BallEntity*)Entity0;
		break;
	case Entity::Types::Case :
//		LOG << "Case " << Entity0 << " Colliding with ";
		CaseToCheck = (CaseEntity*)Entity0;
		break;
	}
	switch(Entity1->getType())
	{
	case Entity::Types::Ball :
//		LOG << "Ball " << Entity1 << std::endl;
		BallToCheck = (BallEntity*)Entity1;
		break;
	case Entity::Types::Case :
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

void GameEngine::BuildRefMove(CaseEntity *ToAdd)
{
	if(ToAdd->getRefMove() == NULL)
	{
		Vector3 ActuPos = ToAdd->getAbsolutePosition();
		Quaternion ActuAngle = ToAdd->getAbsoluteOrientation();
		String name("MoveRef-" + ToAdd->getName() + "-Group");
		GroupEntity *Grp = new GroupEntity(name, mSceneMgr);
		GroupEntity *OldGroup = ToAdd->getGroup();
		LOG << "Entity " << ToAdd->getName() << " Add RefMove " << name << std::endl;
		Grp->AddChild(ToAdd);
		ToAdd->setRefMove(Grp);
		Grp->setisRefMove(true);
		Grp->ComputeAndEquilibrateChilds();
		AddGroup(Grp);
		if(OldGroup != NULL)
		{
			OldGroup->AddChild(Grp);
			OldGroup->ComputeAndEquilibrateChilds();
		}
		ToAdd->setAbsolutePosition(ActuPos);
		ToAdd->setAbsoluteOrientation(ActuAngle);
	}
}

void GameEngine::AddCaseToBeMoved(CaseEntity *ToAdd)
{
	if(ToAdd == NULL)
		return;
	BuildRefMove(ToAdd);
	std::list<CaseEntity*>::iterator iter(CasesToBeMoved.begin());
	while(iter != CasesToBeMoved.end())
	{
		CaseEntity *Case = *(iter++);
		if(Case == ToAdd)
			return;
	}
	CasesToBeMoved.push_back(ToAdd);
}

void GameEngine::DelCaseToBeMoved(CaseEntity *ToDel)
{
	if(ToDel == NULL)
		return;
	std::list<CaseEntity*>::iterator iter(CasesToBeMoved.begin());
	while(iter != CasesToBeMoved.end())
	{
		CaseEntity *Case = *iter;
		if(Case == ToDel)
		{
			GroupEntity *RefMove = Case->getRefMove();
			if(RefMove != NULL)
			{
				GroupEntity *NewGroup = RefMove->getGroup();
				if(NewGroup != NULL)
				{
					NewGroup->DelChild(RefMove);
					NewGroup->AddChild(Case);
					NewGroup->ComputeAndEquilibrateChilds();
				}
				DeleteGroup(RefMove);
			}
			iter = CasesToBeMoved.erase(iter);
			return;
		}
		iter++;
	}
}

void GameEngine::AddCaseColliding(CaseEntity *ToAdd)
{
	if(ToAdd == NULL)
		return;
	if(CheckIfAlreadyColliding(ToAdd) == false)
		CasesUnderCollide.push_back(ToAdd);
}

bool GameEngine::CheckIfAlreadyColliding(CaseEntity *ToCheck)
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

void GameEngine::CheckforCollides(void)
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

void GameEngine::AddGroup(GroupEntity *group, bool recursive)
{
	if(group == NULL)
		return;

	auto iter(Groups.begin());
	while(iter != Groups.end())
	{
		GroupEntity *G = *(iter++);
		if(G == group)
			return;
	}

	Groups.push_back(group);
	group->setEngine(this);
	nb_entities++;

	if(recursive == true)
	{
		std::list<BaseEntity*> Childs;
		group->FillListWithChilds(Childs, false);

		auto iterG(Childs.begin());
		while(iterG != Childs.end())
		{
			BaseEntity *E = *(iterG++);
			if(E == NULL)
				continue;
			switch(E->getType())
			{
			case BaseEntity::Types::Ball :
				AddBall((BallEntity*)E);
				break;
			case BaseEntity::Types::Case :
				AddCase((CaseEntity*)E);
				break;
			case BaseEntity::Types::Group :
				AddGroup((GroupEntity*)E, true);
				break;
			}
		}
	}
}

void GameEngine::AddBall(BallEntity *ball)
{
	if(ball == NULL)
		return;

	auto iter(Balls.begin());
	while(iter != Balls.end())
	{
		BallEntity *B = *(iter++);
		if(B == ball)
			return;
	}

	Balls.push_back(ball);
	ball->setEngine(this);
	nb_entities++;
}

void GameEngine::AddCase(CaseEntity *Wcase)
{
	if(Wcase == NULL)
		return;

	auto iter(Cases.begin());
	while(iter != Cases.end())
	{
		CaseEntity *C = *(iter++);
		if(C == Wcase)
			return;
	}

	Cases.push_back(Wcase);
	Wcase->setEngine(this);
	nb_entities++;

	if(Wcase->CaseToMove() == true)
		AddCaseToBeMoved(Wcase);
}

bool GameEngine::frameEnded(const Ogre::FrameEvent& fe)
{
//    LOG << "Render a frame" << std::endl;
	dFloat timestep = dGetElapsedSeconds();
	UpdatePhysics(timestep);

	if(m_suspendPhysicsUpdate == false)
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

inline void GameEngine::DeleteGroup(GroupEntity *Grp)
{
	do
	{
		Grp = DeleteGroup(Grp, NULL);
	}
	while(Grp != NULL);
}

GroupEntity *GameEngine::DeleteGroup(GroupEntity *Entity, std::list<GroupEntity*>::iterator *iter)
{
	GroupEntity *Sup = Entity->getGroup();
	if(Sup != NULL)
	{
		if(Sup->DelChild(Entity) == false)
			Sup = NULL;
	}
	RemoveGroup(Entity, iter);
	Entity->Finalize();
	delete Entity;
	return Sup;
}

void GameEngine::RemoveGroup(GroupEntity *Entity, std::list<GroupEntity*>::iterator *iter)
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

GroupEntity *GameEngine::findGroup(String &name, bool is_for_import)
{
	std::list<GroupEntity*> *GrpList;
	if(is_for_import == false)
		GrpList = &Groups;
	else
		GrpList = &ImportLevelGroups;

	std::list<GroupEntity*>::iterator it(GrpList->begin());
	while(it != GrpList->end())
	{
		GroupEntity *B = *(it++);
		if(B != NULL && B->getName() == name)
			return B;
	}

	return NULL;
}

inline GroupEntity *GameEngine::findGroup(const char * const name_c, bool is_for_import)
{
	String name(name_c);
	return findGroup(name, is_for_import);
}

void GameEngine::DeleteBall(BallEntity *Ent, std::list<BallEntity*>::iterator *iter)
{
	RemoveBall(Ent, iter);
	GroupEntity *Group = Ent->getGroup();
	if(Group != NULL)
	{
		bool tobedel = Group->DelChild((Entity*)Ent);
		if(tobedel)
			DeleteGroup(Group);
	}
	Ent->Finalize();
	delete Ent;
}

void GameEngine::RemoveBall(BallEntity *Entity, std::list<BallEntity*>::iterator *iter)
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

void GameEngine::DeleteCase(CaseEntity *Ent, std::list<CaseEntity*>::iterator *iter)
{
	RemoveCase(Ent, iter);
	GroupEntity *Group = Ent->getGroup();
	if(Group != NULL)
	{
		bool tobedel = Group->DelChild((Entity*)Ent);
		if(tobedel)
			DeleteGroup(Group);
	}
	Ent->Finalize();
	delete Ent;
}

void GameEngine::RemoveCase(CaseEntity *Entity, std::list<CaseEntity*>::iterator *iter)
{
	DelCaseToBeMoved(Entity);
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

void GameEngine::EmptyLevel(void)
{
	_StopPhysic();

	CasesUnderCollide.clear();
	CasesToBeMoved.clear();

	while(Cases.empty() == false)
		DeleteCase(*(Cases.begin()));
	assert(Cases.empty());

	while(Balls.empty() == false)
		DeleteBall(*(Balls.begin()));
	assert(Balls.empty());

	while(Groups.empty() == false)
		DeleteGroup(*(Groups.begin()));
	assert(Groups.empty());

	//Force an update of Physic to force garbage collecting !
	_updatePhysic(1.0f / MAX_PHYSICS_FPS);
	_StopPhysic();
}

void GameEngine::ImportLevelFromJson(Node *parent, String &nodeNamePrefix, bool isForImport)
{
	std::ifstream myfile;
	std::stringstream buffer;
	std::list<GroupEntity*> *GroupList = NULL;
	if(isForImport)
	{
		myfile.open (ImportLevelFilename.c_str());
		GroupList = &ImportLevelGroups;
	}
	else
	{
		myfile.open (LevelFilename.c_str());
		GroupList = &Groups;
	}
	buffer << myfile.rdbuf();
	myfile.close();
	rapidjson::Document in;
	in.Parse(buffer.str().c_str());

	unsigned long nb_parsed = in[COUNTER_JSON_FIELD].GetUint();

	nb_entities = std::max(nb_parsed, nb_entities);
	//Parsing Groups
	for(int cmpt = 0; cmpt < in[GROUPS_JSON_FIELD].GetArray().Size(); cmpt++)
	{
		GroupEntity *newGroup = new GroupEntity();
		rapidjson::Value &groupjson = in[GROUPS_JSON_FIELD].GetArray()[cmpt];
		newGroup->ImportFromJson(groupjson, this, parent, nodeNamePrefix);
		if(isForImport)
			ImportLevelGroups.push_back(newGroup);
		else
			AddGroup(newGroup);
	}
	//Parsing Cases
	for(int cmpt = 0; cmpt < in[CASES_JSON_FIELD].GetArray().Size(); cmpt++)
	{
		CaseEntity *newCase = new CaseEntity();
		rapidjson::Value &casejson = in[CASES_JSON_FIELD].GetArray()[cmpt];
		if(isForImport)
		{
			newCase->ImportFromJson(casejson, this, parent, nodeNamePrefix);
			ImportLevelCases.push_back(newCase);
		}
		else
		{
			newCase->CreateFromJson(casejson, this, m_world, parent, nodeNamePrefix);
			AddCase(newCase);
		}
	}
	//Parsing Balls
	for(int cmpt = 0; cmpt < in[BALLS_JSON_FIELD].GetArray().Size(); cmpt++)
	{
		BallEntity *newBall = new BallEntity();
		rapidjson::Value &balljson = in[BALLS_JSON_FIELD].GetArray()[cmpt];
		if(isForImport)
		{
			newBall->ImportFromJson(balljson, this, parent, nodeNamePrefix);
			ImportLevelBalls.push_back(newBall);
		}
		else
		{
			newBall->CreateFromJson(balljson, this, m_world, parent, nodeNamePrefix);
			AddBall(newBall);
		}
	}

	//Now we need to restore links between ogre nodes.
	std::list<GroupEntity*>::iterator iter(GroupList->begin());
	while(iter != GroupList->end())
	{
		GroupEntity *Grp = *(iter++);
		if(Grp != NULL)
			Grp->ComputeChilds();
	}
}

}

