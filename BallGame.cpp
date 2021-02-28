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

void inline ButtonsSetVisible(std::list<CEGUI::Window*> &list, bool state)
{
	std::list<CEGUI::Window*>::iterator iter(list.begin());
	while(iter != list.end())
	{
		CEGUI::Window *W = *(iter++);
		if(W != NULL)
			W->setVisible(state);
	}
}

void inline ButtonsMoveToFront(std::list<CEGUI::Window*> &list)
{
	std::list<CEGUI::Window*>::iterator iter(list.begin());
	while(iter != list.end())
	{
		CEGUI::Window *W = *(iter++);
		if(W != NULL)
			W->moveToFront();
	}
}

void inline ButtonsMoveToBack(std::list<CEGUI::Window*> &list)
{
	std::list<CEGUI::Window*>::iterator iter(list.begin());
	while(iter != list.end())
	{
		CEGUI::Window *W = *(iter++);
		if(W != NULL)
			W->moveToBack();
	}
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

void BallGame::CustomListenerPostUpdateCallback(dFloat timestep)
{
	std::list<CaseEntity*>::iterator iter1(CasesToBeMoved.begin());
	while(iter1 != CasesToBeMoved.end())
	{
		CaseEntity *Case = *(iter1++);
		if(Case != NULL)
			Case->CaseMove(m_microsecunds, timestep);
	}
}

void BallGame::GameNewtonListener::PostUpdate(dFloat timestep)
{
//	LOG << "PostUpdate CustomListener @" << timestep << std::endl;
	Engine->CustomListenerPostUpdateCallback(timestep);
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
	listener = NULL;
	LevelEditMode = Place;
	EntityEditMode = Simple;
	EntityEditAction = Move;
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

	listener = new GameNewtonListener(this);
}

BallGame::~BallGame()
{
	EmptyLevel();
	EmptyLevelsList();
	if(m_world != NULL)
		NewtonDestroy(m_world);//This causes CutomListener to be destroyed too !
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

inline void BallGame::_updatePhysic(dFloat timestep)
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
		ButtonsSetVisible(EditButtons, true);
	    GroupElementsB->setVisible(false);
	    CaractsEditElementB->setVisible(false);
	    SimpleEditElementB->setVisible(false);
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
		ButtonsSetVisible(EditButtons, false);
		ButtonsSetVisible(EditCaseButtons, false);
		ButtonsSetVisible(EditBallButtons, false);
	}
}

void BallGame::BuildImportLevelWindowContent(Node *parent)
{
	mImportLevelSceneMgr->getRootSceneNode()->removeAndDestroyAllChildren();
	std::list<GroupEntity*>::iterator Giter(ImportLevelGroups.begin());
	while(Giter != ImportLevelGroups.end())
	{
		GroupEntity *Grp = *Giter;
		if(Grp != NULL)
			delete Grp;
		Giter = ImportLevelGroups.erase(Giter);
	}

	std::list<CaseEntity*>::iterator Citer(ImportLevelCases.begin());
	while(Citer != ImportLevelCases.end())
	{
		CaseEntity *Grp = *Citer;
		if(Grp != NULL)
			delete Grp;
		Citer = ImportLevelCases.erase(Citer);
	}

	std::list<BallEntity*>::iterator Biter(ImportLevelBalls.begin());
	while(Biter != ImportLevelBalls.end())
	{
		BallEntity *Grp = *Biter;
		if(Grp != NULL)
			delete Grp;
		Biter = ImportLevelBalls.erase(Biter);
	}

	if(parent != NULL && ImportLevelName.empty() == false && ImportLevelFilename.empty() == false)
	{
		String Prefix;
		Prefix = ImportLevelName;
		Prefix += "-" + std::to_string(nb_entities);
		Prefix += ":";
		LOG << "Building Window Content for " << ImportLevelName << " (" << ImportLevelFilename << ") with Prefix " << Prefix << std::endl;
		ImportLevelFromJson(parent, Prefix, true);
	}
}

inline void BallGame::ActivateLevelImportInterface(void)
{
	ButtonsSetVisible(ImportLevelButtons, true);
	ButtonsMoveToFront(ImportLevelButtons);
	ButtonsMoveToBack(MainMenuButtons);
	ButtonsMoveToBack(EditButtons);
	ButtonsMoveToBack(EditBallButtons);
	ButtonsMoveToBack(EditCaseButtons);
	ImportLevelActivateInterfacePushB->setText("-");
}

inline void BallGame::UnactivateLevelImportInterface(void)
{
	ButtonsSetVisible(ImportLevelButtons, false);
	ImportLevelActivateInterfacePushB->setText("+");
	ImportLevelName.clear();
	ImportLevelFilename.clear();
	ButtonsMoveToFront(MainMenuButtons);
	ButtonsMoveToFront(EditButtons);
	ButtonsMoveToFront(EditBallButtons);
	ButtonsMoveToFront(EditCaseButtons);
	BuildImportLevelWindowContent(NULL);
}

bool BallGame::ImportLevelActivateInterfacePushBCallback(const CEGUI::EventArgs &e)
{
	if(ImportLevelActivateInterfacePushB->getText() == "+")//Menu not activated
		ActivateLevelImportInterface();
	else
		UnactivateLevelImportInterface();
	return true;
}

bool BallGame::ImportLevelPushBCallback(const CEGUI::EventArgs &e)
{
	//Clear import level scene manager content
	BuildImportLevelWindowContent(NULL);
	//Re import content into main manager !
	BuildImportLevelWindowContent((Node*)mSceneMgr->getRootSceneNode());

	String GrpName = ImportLevelName;
	GrpName += "-" + std::to_string(nb_entities);
	GrpName += ":ImportGroup";
	GroupEntity *ImportGroup = new GroupEntity(GrpName, mSceneMgr);

	std::list<CaseEntity*> selected;
	std::list<CaseEntity*>::iterator Citer(ImportLevelCases.begin());
	while(Citer != ImportLevelCases.end())
	{
		CaseEntity *Case = *Citer;
		if(Case != NULL)
		{
//			LOG << "Import of Case " << Case << " '" << Case->getName() << "'" << std::endl;
			Case->CreateNewtonBody(m_world);
			AddCase(Case);
			ImportGroup->AddChild(Case);
			selected.push_back(Case);
		}
		Citer = ImportLevelCases.erase(Citer);
	}

	ImportGroup->ComputeAndEquilibrateChilds();
	AddGroup(ImportGroup);

	std::list<GroupEntity*>::iterator Giter(ImportLevelGroups.begin());
	while(Giter != ImportLevelGroups.end())
	{
		GroupEntity *Group = *Giter;
		if(Group != NULL)
		{
			Group->Finalize();
			delete Group;
		}
		Giter = ImportLevelGroups.erase(Giter);
	}

	std::list<BallEntity*>::iterator Biter(ImportLevelBalls.begin());
	while(Biter != ImportLevelBalls.end())
	{
		BallEntity *Ball = *Biter;
		if(Ball != NULL)
		{
			Ball->Finalize();
			delete Ball;
		}
		Biter = ImportLevelBalls.erase(Biter);
	}

	UnactivateLevelImportInterface();
	MouseOverButton = false;
	EditElementSetupButtons();
	std::list<CaseEntity*>::iterator Siter(selected.begin());
	while(Siter != selected.end())
	{
		CaseEntity *Case = *Siter;
		if(Case != NULL)
			ManageMultiSelectionSet((BallGameEntity*)Case);
		Siter = selected.erase(Siter);
	}
	return true;
}

bool BallGame::ChooseLevelToImportComboBCallback(const CEGUI::EventArgs &e)
{
	CEGUI::ListboxTextItem *item = (CEGUI::ListboxTextItem*)ChooseLevelToImportComboB->getSelectedItem();
	if(item != NULL)
	{
		ImportLevelFilename = *(String*)item->getUserData();
		ImportLevelName = item->getText().c_str();
		BuildImportLevelWindowContent((Node*)mImportLevelSceneMgr->getRootSceneNode());
	}
	return true;
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
	if(LevelEditMode != Delete)
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
	std::list<BallGameEntity*>::iterator delIter(UnderEditEntites.begin());
	while(delIter != UnderEditEntites.end())
	{
		BallGameEntity *Entity = *delIter;
		if(Entity != NULL)
		{
			if(Entity == LastHighligted)
				LastHighligted = NULL;
			switch(Entity->getType())
			{
			case Ball :
				DeleteBall((BallEntity*)Entity);
				break;
			case Case :
				DeleteCase((CaseEntity*)Entity);
				break;
			}
		}
		delIter = UnderEditEntites.erase(delIter);
	}
	GroupElementsB->setMutedState(true);
	GroupElementsB->setVisible(false);
	GroupElementsB->setMutedState(false);
}

bool BallGame::DeleteElementBCallback(const CEGUI::EventArgs &e)
{
	switch(LevelEditMode)
	{
	case Edit :
		MultiSelectionMode = false;
		MultiSelectionSetEmpty();
		EditBall(NULL);
		EditCase(NULL);
		break;
	case Place :
		UnprepareNewElement();
		break;
	}
	LevelEditMode = Delete;
	DeleteElementB->setDisabled(true);
	PlaceNewElementB->setDisabled(false);
	EditElementB->setDisabled(false);
	SimpleEditElementB->setVisible(false);
	CaractsEditElementB->setVisible(false);
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
	SimpleEditElementB->setVisible(false);
	CaractsEditElementB->setVisible(false);
	MoveElementB->setVisible(true);
	RotateElementB->setVisible(true);
	ScaleElementB->setVisible(true);
	GroupElementsB->setVisible(false);
	UnprepareNewElement();
	SetMoveNewElement();
	return true;
}

bool BallGame::EditElementBCallback(const CEGUI::EventArgs &e)
{
	EditElementSetupButtons();
	return true;
}

void BallGame::EditElementSetupButtons(void)
{
	DeleteElementB->setDisabled(false);
	PlaceNewElementB->setDisabled(false);
	EditElementB->setDisabled(true);
	SimpleEditElementB->setVisible(true);
	CaractsEditElementB->setVisible(true);
	MoveElementB->setVisible(true);
	RotateElementB->setVisible(true);
	ScaleElementB->setVisible(true);
	GroupElementsB->setVisible(false);
	SetMoveElement();
}

void BallGame::SetMoveElement(void)
{
	switch(LevelEditMode)
	{
	case Place :
		UnprepareNewElement();
		break;
	case Delete :
		UnprepareDeleteElement();
		MultiSelectionSetEmpty();
		break;
	}
	LevelEditMode = Edit;
	EntityEditMode = Simple;
	EntityEditAction = Move;
	DeleteElementB->setDisabled(false);
	PlaceNewElementB->setDisabled(false);
	SimpleEditElementB->setDisabled(true);
	CaractsEditElementB->setDisabled(false);
	EditElementB->setDisabled(true);
	MoveElementB->setDisabled(true);
	RotateElementB->setDisabled(false);
	ScaleElementB->setDisabled(false);
}

void BallGame::SetMoveNewElement(void)
{
	switch(LevelEditMode)
	{
	case Edit :
		MultiSelectionMode = false;
		MultiSelectionSetEmpty();
		EditBall(NULL);
		EditCase(NULL);
		break;
	case Delete :
		UnprepareDeleteElement();
		MultiSelectionSetEmpty();
		break;
	}
	LevelEditMode = Place;
	EntityEditMode = Simple;
	EntityEditAction = Move;
	PrepareNewElement();
	DeleteElementB->setDisabled(false);
	PlaceNewElementB->setDisabled(true);
	EditElementB->setDisabled(false);
	MoveElementB->setDisabled(true);
	RotateElementB->setDisabled(false);
	ScaleElementB->setDisabled(false);
}

bool BallGame::SimpleEditElementBCallback(const CEGUI::EventArgs &e)
{
	EntityEditMode = Simple;
	EntityEditAction = Move;
	EditBall(UnderEditBall);
	EditCase(UnderEditCase);
	CaractsEditElementB->setEnabled(true);
	SimpleEditElementB->setEnabled(false);

	MoveElementB->setVisible(true);
	MoveElementB->setEnabled(true);
	RotateElementB->setVisible(true);
	RotateElementB->setEnabled(false);
	ScaleElementB->setVisible(true);
	ScaleElementB->setEnabled(false);
	return true;
}

bool BallGame::MoveEditElementBCallback(const CEGUI::EventArgs &e)
{
	EntityEditMode = Moves;
	EntityEditAction = Move;
	EditBall(UnderEditBall);
	EditCase(UnderEditCase);
	CaractsEditElementB->setEnabled(false);
	SimpleEditElementB->setEnabled(true);

	MoveElementB->setVisible(true);
	MoveElementB->setEnabled(true);
	RotateElementB->setVisible(true);
	RotateElementB->setEnabled(false);
	ScaleElementB->setVisible(true);
	ScaleElementB->setEnabled(false);
	return true;
}

bool BallGame::CaractsEditElementBCallback(const CEGUI::EventArgs &e)
{
	EntityEditMode = Caracts;
	EditBall(UnderEditBall);
	EditCase(UnderEditCase);
	CaractsEditElementB->setEnabled(false);
	SimpleEditElementB->setEnabled(true);

	MoveElementB->setVisible(false);
	RotateElementB->setVisible(false);
	ScaleElementB->setVisible(false);
	return true;
}

inline void BallGame::SetMoveElementAction(void)
{
	EntityEditAction = Move;
	MoveElementB->setDisabled(true);
	RotateElementB->setDisabled(false);
	ScaleElementB->setDisabled(false);
}

bool BallGame::MoveElementBCallback(const CEGUI::EventArgs &e)
{
	SetMoveElementAction();
	return true;
}

inline void BallGame::SetRotateElementAction(void)
{
	EntityEditAction = Rotate;
	MoveElementB->setDisabled(false);
	RotateElementB->setDisabled(true);
	ScaleElementB->setDisabled(false);
}

bool BallGame::RotateElementBCallback(const CEGUI::EventArgs &e)
{
	SetRotateElementAction();
	return true;
}

inline void BallGame::SetScaleElementAction(void)
{
	EntityEditAction = Scale;
	MoveElementB->setDisabled(false);
	RotateElementB->setDisabled(false);
	ScaleElementB->setDisabled(true);
}

bool BallGame::ScaleElementBCallback(const CEGUI::EventArgs &e)
{
	SetScaleElementAction();
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
		ButtonsSetVisible(StatesButtons, true);
	else
		ButtonsSetVisible(StatesButtons, false);
	return true;
}

void SetWindowsPosNearToOther(CEGUI::Window *self, CEGUI::Window *other, int H_factor, int V_factor)
{
	CEGUI::UVector2 pos(other->getPosition());

	if(H_factor != 0)
	{
		switch(self->getHorizontalAlignment())
		{
		case CEGUI::HA_LEFT :
			break;
		case CEGUI::HA_CENTRE :
			pos.d_x += (H_factor / 2.0f) * self->getWidth();
			break;
		case CEGUI::HA_RIGHT :
			pos.d_x += H_factor * self->getWidth();
			break;
		}
		switch(other->getHorizontalAlignment())
		{
		case CEGUI::HA_LEFT :
			pos.d_x += H_factor * other->getWidth();
			break;
		case CEGUI::HA_CENTRE :
			pos.d_x += (H_factor / 2.0f) * other->getWidth();
			break;
		case CEGUI::HA_RIGHT :
			break;
		}
	}

	if(V_factor != 0)
	{
		switch(self->getVerticalAlignment())
		{
		case CEGUI::HA_LEFT :
			break;
		case CEGUI::HA_CENTRE :
			pos.d_y += (V_factor / 2.0f) * self->getHeight();
			break;
		case CEGUI::HA_RIGHT :
			pos.d_y += V_factor * self->getHeight();
			break;
		}
		switch(other->getVerticalAlignment())
		{
		case CEGUI::HA_LEFT :
			pos.d_y += V_factor * other->getHeight();
			break;
		case CEGUI::HA_CENTRE :
			pos.d_y += (V_factor / 2.0f) * other->getHeight();
			break;
		case CEGUI::HA_RIGHT :
			break;
		}
	}

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

#define ButtonSetAddButton(S, B) S.push_back((CEGUI::Window*)B)

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
    ButtonSetAddButton(MainMenuButtons, StopPhysicPushB);

    StopPhysicPushB->subscribeEvent(CEGUI::PushButton::EventClicked,
    		CEGUI::Event::Subscriber(&BallGame::StopPhysicPushBCallback, this));

    EditModePushB = CreateNewGUIComponent<CEGUI::PushButton>("OgreTray/Button");
    EditModePushB->setText("Edit");
    EditModePushB->setSize(CEGUI::USize(CEGUI::UDim(0, 75), CEGUI::UDim(0, 30)));

    MainLayout->addChild(EditModePushB);
    ButtonSetAddButton(MainMenuButtons, EditModePushB);

    EditModePushB->subscribeEvent(CEGUI::PushButton::EventClicked,
    		CEGUI::Event::Subscriber(&BallGame::EditModePushBCallback, this));

    SetWindowsPosNearToOther(EditModePushB, StopPhysicPushB, 0, 1);

    StatesModePushB = CreateNewGUIComponent<CEGUI::PushButton>("OgreTray/Button");
    StatesModePushB->setText("States");
    StatesModePushB->setSize(CEGUI::USize(CEGUI::UDim(0, 75), CEGUI::UDim(0, 30)));

    MainLayout->addChild(StatesModePushB);
    ButtonSetAddButton(MainMenuButtons, StatesModePushB);

    StatesModePushB->subscribeEvent(CEGUI::PushButton::EventClicked,
    		CEGUI::Event::Subscriber(&BallGame::StatesModePushBCallback, this));

    SetWindowsPosNearToOther(StatesModePushB, EditModePushB, 1, 0);

    ChooseLevelComboB = CreateNewGUIComponent<CEGUI::Combobox>("OgreTray/Combobox");
    ChooseLevelComboB->setReadOnly(true);
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
    ButtonSetAddButton(MainMenuButtons, ChooseLevelComboB);

    ChooseLevelComboB->subscribeEvent(CEGUI::Combobox::EventListSelectionAccepted,
    		CEGUI::Event::Subscriber(&BallGame::ChooseLevelComboBCallback, this));

    SetWindowsPosNearToOther(ChooseLevelComboB, EditModePushB, 0, 1);

    NewLevelEditB = CreateNewGUIComponent<CEGUI::Editbox>("OgreTray/Editbox");
    NewLevelEditB->setText("NewLevel");
    NewLevelEditB->setSize(CEGUI::USize(CEGUI::UDim(0, 90), CEGUI::UDim(0, 30)));

    MainLayout->addChild(NewLevelEditB);
    ButtonSetAddButton(MainMenuButtons, NewLevelEditB);

    SetWindowsPosNearToOther(NewLevelEditB, EditModePushB, 0, 2);// Be Carefull, Combobox size is size with combo expanded !

    NewLevelCreateB = CreateNewGUIComponent<CEGUI::PushButton>("OgreTray/Button");
    NewLevelCreateB->setText("Create");
    NewLevelCreateB->setSize(CEGUI::USize(CEGUI::UDim(0, 60), CEGUI::UDim(0, 30)));

    MainLayout->addChild(NewLevelCreateB);
    ButtonSetAddButton(MainMenuButtons, NewLevelCreateB);

    NewLevelCreateB->subscribeEvent(CEGUI::PushButton::EventClicked,
			CEGUI::Event::Subscriber(&BallGame::NewLevelCreateBCallback, this));

    SetWindowsPosNearToOther(NewLevelCreateB, NewLevelEditB, 1, 0);

    SaveLevelPushB = CreateNewGUIComponent<CEGUI::PushButton>("OgreTray/Button");
    SaveLevelPushB->setText("Save");
    SaveLevelPushB->setSize(CEGUI::USize(CEGUI::UDim(0, 150), CEGUI::UDim(0, 30)));

    MainLayout->addChild(SaveLevelPushB);
    ButtonSetAddButton(MainMenuButtons, SaveLevelPushB);

    SaveLevelPushB->subscribeEvent(CEGUI::PushButton::EventClicked,
    		CEGUI::Event::Subscriber(&BallGame::SaveLevelPushBCallback, this));

    SetWindowsPosNearToOther(SaveLevelPushB, NewLevelEditB, 0, 1);

    QuitPushB = CreateNewGUIComponent<CEGUI::PushButton>("OgreTray/Button");
    QuitPushB->setText("Quit");
    QuitPushB->setTooltipText("Quit");
    QuitPushB->setSize(CEGUI::USize(CEGUI::UDim(0, 150), CEGUI::UDim(0, 30)));

    MainLayout->addChild(QuitPushB);
    ButtonSetAddButton(MainMenuButtons, QuitPushB);

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
    ButtonSetAddButton(StatesButtons, StatesBanner);

    ChooseStateToLoadB = CreateNewGUIComponent<CEGUI::Combobox>("OgreTray/Combobox");
    ChooseStateToLoadB->setReadOnly(true);
    ChooseStateToLoadB->setSize(CEGUI::USize(CEGUI::UDim(0, 150), CEGUI::UDim(0, 30)));
    ChooseStateToLoadB->setVerticalAlignment(CEGUI::VA_TOP);
    ChooseStateToLoadB->setHorizontalAlignment(CEGUI::HA_RIGHT);

    MainLayout->addChild(ChooseStateToLoadB);
    ButtonSetAddButton(StatesButtons, ChooseStateToLoadB);
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
    ButtonSetAddButton(StatesButtons, LoadStatePushB);
    LoadStatePushB->subscribeEvent(CEGUI::PushButton::EventClicked,
    		CEGUI::Event::Subscriber(&BallGame::LoadStatePushBCallback, this));
    SetWindowsPosNearToOther(LoadStatePushB, StatesBanner, 0, 2);

    DelStatePushB = CreateNewGUIComponent<CEGUI::PushButton>("OgreTray/Button");
    DelStatePushB->setText("Delete");
    DelStatePushB->setSize(CEGUI::USize(CEGUI::UDim(0, 150), CEGUI::UDim(0, 30)));
    DelStatePushB->setVerticalAlignment(CEGUI::VA_TOP);
    DelStatePushB->setHorizontalAlignment(CEGUI::HA_RIGHT);

    MainLayout->addChild(DelStatePushB);
    ButtonSetAddButton(StatesButtons, DelStatePushB);
    DelStatePushB->subscribeEvent(CEGUI::PushButton::EventClicked,
    		CEGUI::Event::Subscriber(&BallGame::DelStatePushBCallback, this));
    SetWindowsPosNearToOther(DelStatePushB, LoadStatePushB, 0, 1);

    SaveStatePushB = CreateNewGUIComponent<CEGUI::PushButton>("OgreTray/Button");
    SaveStatePushB->setText("Save");
    SaveStatePushB->setSize(CEGUI::USize(CEGUI::UDim(0, 150), CEGUI::UDim(0, 30)));
    SaveStatePushB->setVerticalAlignment(CEGUI::VA_TOP);
    SaveStatePushB->setHorizontalAlignment(CEGUI::HA_RIGHT);

    MainLayout->addChild(SaveStatePushB);
    ButtonSetAddButton(StatesButtons, SaveStatePushB);
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
    ButtonSetAddButton(EditButtons, EditingModeTitleBanner);

    SetWindowsPosNearToOther(EditingModeTitleBanner, LevelNameBanner, 0, 1);

    ///ImportLevel
    ImportLevelActivateInterfacePushB = CreateNewGUIComponent<CEGUI::PushButton>("OgreTray/Button");
    ImportLevelActivateInterfacePushB->setText("+");
    ImportLevelActivateInterfacePushB->setSize(CEGUI::USize(CEGUI::UDim(0, 30), CEGUI::UDim(0, 30)));
    ImportLevelActivateInterfacePushB->setVerticalAlignment(CEGUI::VA_TOP);
    ImportLevelActivateInterfacePushB->setHorizontalAlignment(CEGUI::HA_CENTRE);

    MainLayout->addChild(ImportLevelActivateInterfacePushB);
    ButtonSetAddButton(EditButtons, ImportLevelActivateInterfacePushB);

    ImportLevelActivateInterfacePushB->subscribeEvent(CEGUI::PushButton::EventClicked,
    		CEGUI::Event::Subscriber(&BallGame::ImportLevelActivateInterfacePushBCallback, this));

    SetWindowsPosNearToOther(ImportLevelActivateInterfacePushB, EditingModeTitleBanner, 1, 0);


    ChooseLevelToImportComboB = CreateNewGUIComponent<CEGUI::Combobox>("OgreTray/Combobox");
    ChooseLevelToImportComboB->setReadOnly(true);
    ChooseLevelToImportComboB->setVerticalAlignment(CEGUI::VA_TOP);
    ChooseLevelToImportComboB->setHorizontalAlignment(CEGUI::HA_CENTRE);
	for (size_t cmpt = 0; cmpt < ChooseLevelComboB->getItemCount(); cmpt++)//All levels that can be imported are levels that can be loaded !
	{
		CEGUI::ListboxTextItem *item = (CEGUI::ListboxTextItem*)ChooseLevelComboB->getListboxItemFromIndex(cmpt);
		String *str = (String*)item->getUserData();
		CEGUI::ListboxTextItem *newitem = new CEGUI::ListboxTextItem(item->getText());
		String *newstr = new String(str->c_str());
		newitem->setUserData(newstr);
		ChooseLevelToImportComboB->addItem((CEGUI::ListboxItem*)newitem);
	}
    ChooseLevelToImportComboB->setSize(CEGUI::USize(CEGUI::UDim(0, 150), CEGUI::UDim(0, 40 * (ChooseLevelComboB->getItemCount() + 1))));

    MainLayout->addChild(ChooseLevelToImportComboB);
    ButtonSetAddButton(ImportLevelButtons, ChooseLevelToImportComboB);

    ChooseLevelToImportComboB->subscribeEvent(CEGUI::Combobox::EventListSelectionAccepted,
    		CEGUI::Event::Subscriber(&BallGame::ChooseLevelToImportComboBCallback, this));

    //set windows near to other function works for integer !
    {
		CEGUI::UVector2 pos(EditingModeTitleBanner->getPosition());
		pos.d_x -= 0.5 * ChooseLevelToImportComboB->getWidth();
		pos.d_y += EditingModeTitleBanner->getHeight();
		ChooseLevelToImportComboB->setPosition(pos);
    }


    ImportLevelPushB = CreateNewGUIComponent<CEGUI::PushButton>("OgreTray/Button");
    ImportLevelPushB->setText("Import");
    ImportLevelPushB->setSize(CEGUI::USize(CEGUI::UDim(0, 150), CEGUI::UDim(0, 30)));
    ImportLevelPushB->setVerticalAlignment(CEGUI::VA_TOP);
    ImportLevelPushB->setHorizontalAlignment(CEGUI::HA_CENTRE);

    MainLayout->addChild(ImportLevelPushB);
    ButtonSetAddButton(ImportLevelButtons, ImportLevelPushB);

    ImportLevelPushB->subscribeEvent(CEGUI::PushButton::EventClicked,
    		CEGUI::Event::Subscriber(&BallGame::ImportLevelPushBCallback, this));
    SetWindowsPosNearToOther(ImportLevelPushB, ChooseLevelToImportComboB, 1, 0);


    ImportLevelWindow = CreateNewGUIComponent<CEGUI::Window>("OgreTray/StaticImage", "ImportLevelWindow");
    ImportLevelWindow->setSize(CEGUI::USize(CEGUI::UDim(0.8, 0),
 						   CEGUI::UDim(0.8, 0)));
    ImportLevelWindow->setVerticalAlignment(CEGUI::VA_TOP);
    ImportLevelWindow->setHorizontalAlignment(CEGUI::HA_CENTRE);

    ButtonSetAddButton(ImportLevelButtons, ImportLevelWindow);

    SetWindowsPosNearToOther(ImportLevelWindow, EditingModeTitleBanner, 0, 2);

    sheet->addChild(ImportLevelWindow);


    /// Add new Element GUI
    AddElementTitleBanner = CreateNewGUIComponent<CEGUI::Titlebar>("OgreTray/Titlebar");
    AddElementTitleBanner->setText("Add");
    AddElementTitleBanner->setSize(CEGUI::USize(CEGUI::UDim(0, 150), CEGUI::UDim(0, 30)));
    AddElementTitleBanner->setVerticalAlignment(CEGUI::VA_TOP);
    AddElementTitleBanner->setHorizontalAlignment(CEGUI::HA_RIGHT);
    {
		CEGUI::UVector2 pos = AddElementTitleBanner->getPosition();
		pos.d_y = CEGUI::UDim(0, (mWindow->getHeight() / 2) - 120);
		AddElementTitleBanner->setPosition(pos);
    }

    MainLayout->addChild(AddElementTitleBanner);
    ButtonSetAddButton(EditButtons, AddElementTitleBanner);


    ChooseTypeOfElementToAddB = CreateNewGUIComponent<CEGUI::Combobox>("OgreTray/Combobox");
    ChooseTypeOfElementToAddB->setReadOnly(true);
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
    ChooseTypeOfElementToAddB->setSize(CEGUI::USize(CEGUI::UDim(0, 150), CEGUI::UDim(0, 40 * (ChooseTypeOfElementToAddB->getItemCount() + 1))));
    ChooseTypeOfElementToAddB->setVerticalAlignment(CEGUI::VA_TOP);
    ChooseTypeOfElementToAddB->setHorizontalAlignment(CEGUI::HA_RIGHT);

    ChooseTypeOfElementToAddB->subscribeEvent(CEGUI::Combobox::EventListSelectionAccepted,
    		CEGUI::Event::Subscriber(&BallGame::ChooseTypeOfElementToAddBCallback, this));

    MainLayout->addChild(ChooseTypeOfElementToAddB);
    ButtonSetAddButton(EditButtons, ChooseTypeOfElementToAddB);

    ThumbnailWindow = CreateNewGUIComponent<CEGUI::Window>("OgreTray/StaticImage", "ThumbnailWindow");
    ThumbnailWindow->setSize(CEGUI::USize(CEGUI::UDim(0, 150),
 						   CEGUI::UDim(0, 150)));
    ThumbnailWindow->setVerticalAlignment(CEGUI::VA_TOP);
    ThumbnailWindow->setHorizontalAlignment(CEGUI::HA_RIGHT);

    sheet->addChild(ThumbnailWindow);
    ButtonSetAddButton(EditButtons, ThumbnailWindow);

    PlaceNewElementB = CreateNewGUIComponent<CEGUI::PushButton>("OgreTray/Button");
    PlaceNewElementB->setText("Place");
    PlaceNewElementB->setSize(CEGUI::USize(CEGUI::UDim(0, 75), CEGUI::UDim(0, 30)));
    PlaceNewElementB->setVerticalAlignment(CEGUI::VA_TOP);
    PlaceNewElementB->setHorizontalAlignment(CEGUI::HA_RIGHT);

    PlaceNewElementB->subscribeEvent(CEGUI::PushButton::EventClicked,
			CEGUI::Event::Subscriber(&BallGame::PlaceNewElementBCallback, this));

    MainLayout->addChild(PlaceNewElementB);
    ButtonSetAddButton(EditButtons, PlaceNewElementB);

    EditElementB = CreateNewGUIComponent<CEGUI::PushButton>("OgreTray/Button");
    EditElementB->setText("Edit");
    EditElementB->setSize(CEGUI::USize(CEGUI::UDim(0, 75), CEGUI::UDim(0, 30)));
    EditElementB->setVerticalAlignment(CEGUI::VA_TOP);
    EditElementB->setHorizontalAlignment(CEGUI::HA_RIGHT);

    EditElementB->subscribeEvent(CEGUI::PushButton::EventClicked,
			CEGUI::Event::Subscriber(&BallGame::EditElementBCallback, this));

    MainLayout->addChild(EditElementB);
    ButtonSetAddButton(EditButtons, EditElementB);

    DeleteElementB = CreateNewGUIComponent<CEGUI::PushButton>("OgreTray/Button");
    DeleteElementB->setText("Delete");
    DeleteElementB->setSize(CEGUI::USize(CEGUI::UDim(0, 150), CEGUI::UDim(0, 30)));
    DeleteElementB->setVerticalAlignment(CEGUI::VA_TOP);
    DeleteElementB->setHorizontalAlignment(CEGUI::HA_RIGHT);

    DeleteElementB->subscribeEvent(CEGUI::PushButton::EventClicked,
			CEGUI::Event::Subscriber(&BallGame::DeleteElementBCallback, this));

    MainLayout->addChild(DeleteElementB);
    ButtonSetAddButton(EditButtons, DeleteElementB);


    SimpleEditElementB = CreateNewGUIComponent<CEGUI::PushButton>("OgreTray/Button");
    SimpleEditElementB->setText("S");
    SimpleEditElementB->setTooltipText("Simple Edit Element.");
    SimpleEditElementB->setSize(CEGUI::USize(CEGUI::UDim(0, 75), CEGUI::UDim(0, 30)));
    SimpleEditElementB->setVerticalAlignment(CEGUI::VA_TOP);
    SimpleEditElementB->setHorizontalAlignment(CEGUI::HA_RIGHT);

    SimpleEditElementB->subscribeEvent(CEGUI::PushButton::EventClicked,
			CEGUI::Event::Subscriber(&BallGame::SimpleEditElementBCallback, this));

    MainLayout->addChild(SimpleEditElementB);
    ButtonSetAddButton(EditButtons, SimpleEditElementB);

    /*
    MoveEditElementB = CreateNewGUIComponent<CEGUI::PushButton>("OgreTray/Button");
    MoveEditElementB->setText("M");
    MoveEditElementB->setTooltipText("Moves Edit Element.");
    MoveEditElementB->setSize(CEGUI::USize(CEGUI::UDim(0, 75), CEGUI::UDim(0, 30)));
    MoveEditElementB->setVerticalAlignment(CEGUI::VA_TOP);
    MoveEditElementB->setHorizontalAlignment(CEGUI::HA_RIGHT);

    MoveEditElementB->subscribeEvent(CEGUI::PushButton::EventClicked,
			CEGUI::Event::Subscriber(&BallGame::MoveEditElementBCallback, this));

    MainLayout->addChild(MoveEditElementB);
    ButtonSetAddButton(EditButtons, MoveEditElementB);
    */

    CaractsEditElementB = CreateNewGUIComponent<CEGUI::PushButton>("OgreTray/Button");
    CaractsEditElementB->setText("C");
    CaractsEditElementB->setTooltipText("Caracts Edit Element.");
    CaractsEditElementB->setSize(CEGUI::USize(CEGUI::UDim(0, 75), CEGUI::UDim(0, 30)));
    CaractsEditElementB->setVerticalAlignment(CEGUI::VA_TOP);
    CaractsEditElementB->setHorizontalAlignment(CEGUI::HA_RIGHT);

    CaractsEditElementB->subscribeEvent(CEGUI::PushButton::EventClicked,
			CEGUI::Event::Subscriber(&BallGame::CaractsEditElementBCallback, this));

    MainLayout->addChild(CaractsEditElementB);
    ButtonSetAddButton(EditButtons, CaractsEditElementB);

    MoveElementB = CreateNewGUIComponent<CEGUI::PushButton>("OgreTray/Button");
    MoveElementB->setText("M");
    MoveElementB->setTooltipText("Move Element.");
    MoveElementB->setSize(CEGUI::USize(CEGUI::UDim(0, 50), CEGUI::UDim(0, 30)));
    MoveElementB->setVerticalAlignment(CEGUI::VA_TOP);
    MoveElementB->setHorizontalAlignment(CEGUI::HA_RIGHT);

    MoveElementB->subscribeEvent(CEGUI::PushButton::EventClicked,
			CEGUI::Event::Subscriber(&BallGame::MoveElementBCallback, this));

    MainLayout->addChild(MoveElementB);
    ButtonSetAddButton(EditButtons, MoveElementB);

    RotateElementB = CreateNewGUIComponent<CEGUI::PushButton>("OgreTray/Button");
    RotateElementB->setText("R");
    RotateElementB->setTooltipText("Rotate Element.");
    RotateElementB->setSize(CEGUI::USize(CEGUI::UDim(0, 50), CEGUI::UDim(0, 30)));
    RotateElementB->setVerticalAlignment(CEGUI::VA_TOP);
    RotateElementB->setHorizontalAlignment(CEGUI::HA_RIGHT);

    RotateElementB->subscribeEvent(CEGUI::PushButton::EventClicked,
			CEGUI::Event::Subscriber(&BallGame::RotateElementBCallback, this));

    MainLayout->addChild(RotateElementB);
    ButtonSetAddButton(EditButtons, RotateElementB);


    ScaleElementB = CreateNewGUIComponent<CEGUI::PushButton>("OgreTray/Button");
    ScaleElementB->setText("S");
    ScaleElementB->setTooltipText("Scale Element.");
    ScaleElementB->setSize(CEGUI::USize(CEGUI::UDim(0, 50), CEGUI::UDim(0, 30)));
    ScaleElementB->setVerticalAlignment(CEGUI::VA_TOP);
    ScaleElementB->setHorizontalAlignment(CEGUI::HA_RIGHT);

    ScaleElementB->subscribeEvent(CEGUI::PushButton::EventClicked,
			CEGUI::Event::Subscriber(&BallGame::ScaleElementBCallback, this));

    MainLayout->addChild(ScaleElementB);
    ButtonSetAddButton(EditButtons, ScaleElementB);


    GroupElementsB = CreateNewGUIComponent<CEGUI::ToggleButton>("OgreTray/Checkbox");
    GroupElementsB->setText("Grouped");
    GroupElementsB->setSize(CEGUI::USize(CEGUI::UDim(0, 150), CEGUI::UDim(0, 30)));
    GroupElementsB->setVerticalAlignment(CEGUI::VA_TOP);
    GroupElementsB->setHorizontalAlignment(CEGUI::HA_RIGHT);

    GroupElementsB->subscribeEvent(CEGUI::ToggleButton::EventSelectStateChanged,
			CEGUI::Event::Subscriber(&BallGame::GroupElementsBCallback, this));

    MainLayout->addChild(GroupElementsB);
    ButtonSetAddButton(EditButtons, GroupElementsB);

    SetWindowsPosNearToOther(ChooseTypeOfElementToAddB, AddElementTitleBanner, 0, 1);
    SetWindowsPosNearToOther(ThumbnailWindow, AddElementTitleBanner, 0, 2);
    SetWindowsPosNearToOther(EditElementB, ThumbnailWindow, 0, 1);
    SetWindowsPosNearToOther(PlaceNewElementB, EditElementB, -1, 0);
    SetWindowsPosNearToOther(DeleteElementB, EditElementB, 0, 1);
    SetWindowsPosNearToOther(SimpleEditElementB, DeleteElementB, 0, 1);
    SetWindowsPosNearToOther(CaractsEditElementB, SimpleEditElementB, -1, 0);
    SetWindowsPosNearToOther(ScaleElementB, SimpleEditElementB, 0, 1);
    SetWindowsPosNearToOther(RotateElementB, ScaleElementB, -1, 0);
    SetWindowsPosNearToOther(MoveElementB, RotateElementB, -1, 0);
    SetWindowsPosNearToOther(GroupElementsB, ScaleElementB, 0, 1);


    /// Edit Case GUI

    CaseHasForceToggleB = CreateNewGUIComponent<CEGUI::ToggleButton>("OgreTray/Checkbox");
    CaseHasForceToggleB->setText("Has Force");
    CaseHasForceToggleB->setSelected(false);
    CaseHasForceToggleB->setSize(CEGUI::USize(CEGUI::UDim(0, 200), CEGUI::UDim(0, 30)));
    CaseHasForceToggleB->setVerticalAlignment(CEGUI::VA_BOTTOM);
    CaseHasForceToggleB->setHorizontalAlignment(CEGUI::HA_CENTRE);

    CaseHasForceToggleB->subscribeEvent(CEGUI::ToggleButton::EventSelectStateChanged,
			CEGUI::Event::Subscriber(&BallGame::CaseHasForceToggleBCallback, this));

    MainLayout->addChild(CaseHasForceToggleB);
    ButtonSetAddButton(EditCaseButtons, CaseHasForceToggleB);

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
    ButtonSetAddButton(EditCaseButtons, CaseForceValueEditB);

    CaseHasForceDirectionToggleB = CreateNewGUIComponent<CEGUI::ToggleButton>("OgreTray/Checkbox");
    CaseHasForceDirectionToggleB->setText("Has Force Directed");
    CaseHasForceDirectionToggleB->setSelected(false);
    CaseHasForceDirectionToggleB->setSize(CEGUI::USize(CEGUI::UDim(0, 200), CEGUI::UDim(0, 30)));
    CaseHasForceDirectionToggleB->setVerticalAlignment(CEGUI::VA_BOTTOM);
    CaseHasForceDirectionToggleB->setHorizontalAlignment(CEGUI::HA_CENTRE);

	CaseHasForceDirectionToggleB->subscribeEvent(CEGUI::ToggleButton::EventSelectStateChanged,
			CEGUI::Event::Subscriber(&BallGame::CaseHasForceDirectionToggleBCallback, this));

    MainLayout->addChild(CaseHasForceDirectionToggleB);
    ButtonSetAddButton(EditCaseButtons, CaseHasForceDirectionToggleB);

    CaseForceDirectionXValueEditB = CreateNewGUIComponent<CEGUI::Editbox>("OgreTray/Editbox");
    CaseForceDirectionXValueEditB->setSize(CEGUI::USize(CEGUI::UDim(0, 50), CEGUI::UDim(0, 30)));
    CaseForceDirectionXValueEditB->setVerticalAlignment(CEGUI::VA_BOTTOM);
    CaseForceDirectionXValueEditB->setHorizontalAlignment(CEGUI::HA_CENTRE);
    CaseForceDirectionXValueEditB->setValidationString(numRegex);

    CaseForceDirectionXValueEditB->subscribeEvent(CEGUI::Editbox::EventMouseWheel,
			CEGUI::Event::Subscriber(&BallGame::CaseForceDirectionXValueEditBMouseWheelCallback, this));


    MainLayout->addChild(CaseForceDirectionXValueEditB);
    ButtonSetAddButton(EditCaseButtons, CaseForceDirectionXValueEditB);

    CaseForceDirectionYValueEditB = CreateNewGUIComponent<CEGUI::Editbox>("OgreTray/Editbox");
    CaseForceDirectionYValueEditB->setSize(CEGUI::USize(CEGUI::UDim(0, 50), CEGUI::UDim(0, 30)));
    CaseForceDirectionYValueEditB->setVerticalAlignment(CEGUI::VA_BOTTOM);
    CaseForceDirectionYValueEditB->setHorizontalAlignment(CEGUI::HA_CENTRE);
    CaseForceDirectionYValueEditB->setValidationString(numRegex);

    CaseForceDirectionYValueEditB->subscribeEvent(CEGUI::Editbox::EventMouseWheel,
			CEGUI::Event::Subscriber(&BallGame::CaseForceDirectionYValueEditBMouseWheelCallback, this));


    MainLayout->addChild(CaseForceDirectionYValueEditB);
    ButtonSetAddButton(EditCaseButtons, CaseForceDirectionYValueEditB);

    CaseForceDirectionZValueEditB = CreateNewGUIComponent<CEGUI::Editbox>("OgreTray/Editbox");
    CaseForceDirectionZValueEditB->setSize(CEGUI::USize(CEGUI::UDim(0, 50), CEGUI::UDim(0, 30)));
    CaseForceDirectionZValueEditB->setVerticalAlignment(CEGUI::VA_BOTTOM);
    CaseForceDirectionZValueEditB->setHorizontalAlignment(CEGUI::HA_CENTRE);
    CaseForceDirectionZValueEditB->setValidationString(numRegex);

    CaseForceDirectionZValueEditB->subscribeEvent(CEGUI::Editbox::EventMouseWheel,
			CEGUI::Event::Subscriber(&BallGame::CaseForceDirectionZValueEditBMouseWheelCallback, this));


    MainLayout->addChild(CaseForceDirectionZValueEditB);
    ButtonSetAddButton(EditCaseButtons, CaseForceDirectionZValueEditB);

    NormalizeCaseForceDirectionPushB = CreateNewGUIComponent<CEGUI::PushButton>("OgreTray/Button");
    NormalizeCaseForceDirectionPushB->setText("Norm");
    NormalizeCaseForceDirectionPushB->setSize(CEGUI::USize(CEGUI::UDim(0, 50), CEGUI::UDim(0, 30)));
    NormalizeCaseForceDirectionPushB->setVerticalAlignment(CEGUI::VA_BOTTOM);
    NormalizeCaseForceDirectionPushB->setHorizontalAlignment(CEGUI::HA_CENTRE);
    NormalizeCaseForceDirectionPushB->subscribeEvent(CEGUI::PushButton::EventClicked,
    		CEGUI::Event::Subscriber(&BallGame::NormalizeCaseForceDirectionPushBCallback, this));


    MainLayout->addChild(NormalizeCaseForceDirectionPushB);
    ButtonSetAddButton(EditCaseButtons, NormalizeCaseForceDirectionPushB);

    ApplyForceChangesToCasePushB = CreateNewGUIComponent<CEGUI::PushButton>("OgreTray/Button");
    ApplyForceChangesToCasePushB->setText("Apply");
    ApplyForceChangesToCasePushB->setSize(CEGUI::USize(CEGUI::UDim(0, 100), CEGUI::UDim(0, 30)));
    ApplyForceChangesToCasePushB->setVerticalAlignment(CEGUI::VA_BOTTOM);
    ApplyForceChangesToCasePushB->setHorizontalAlignment(CEGUI::HA_CENTRE);

    ApplyForceChangesToCasePushB->subscribeEvent(CEGUI::PushButton::EventClicked,
    		CEGUI::Event::Subscriber(&BallGame::ApplyForceChangesToCasePushBCallback, this));

    MainLayout->addChild(ApplyForceChangesToCasePushB);
    ButtonSetAddButton(EditCaseButtons, ApplyForceChangesToCasePushB);

    SetWindowsPosNearToOther(CaseHasForceDirectionToggleB, ApplyForceChangesToCasePushB, 0, -1);
    SetWindowsPosNearToOther(CaseHasForceToggleB, CaseHasForceDirectionToggleB, 0, -1);
    SetWindowsPosNearToOther(CaseForceValueEditB, CaseHasForceToggleB, 1, 0);
    SetWindowsPosNearToOther(CaseForceDirectionXValueEditB, CaseHasForceDirectionToggleB, 1, 0);
    SetWindowsPosNearToOther(CaseForceDirectionYValueEditB, CaseForceDirectionXValueEditB, 1, 0);
    SetWindowsPosNearToOther(CaseForceDirectionZValueEditB, CaseForceDirectionYValueEditB, 1, 0);
    SetWindowsPosNearToOther(NormalizeCaseForceDirectionPushB, CaseForceDirectionZValueEditB, 1, 0);

    /// Edit Ball GUI

    BallMassValueEditB = CreateNewGUIComponent<CEGUI::Editbox>("OgreTray/Editbox");
    BallMassValueEditB->setSize(CEGUI::USize(CEGUI::UDim(0, 50), CEGUI::UDim(0, 30)));
    BallMassValueEditB->setText("Mass");
    BallMassValueEditB->setVerticalAlignment(CEGUI::VA_BOTTOM);
    BallMassValueEditB->setHorizontalAlignment(CEGUI::HA_CENTRE);

    MainLayout->addChild(BallMassValueEditB);
    ButtonSetAddButton(EditBallButtons, BallMassValueEditB);

    ApplyMassChangesToBallPushB = CreateNewGUIComponent<CEGUI::PushButton>("OgreTray/Button");
    ApplyMassChangesToBallPushB->setText("Apply");
    ApplyMassChangesToBallPushB->setSize(CEGUI::USize(CEGUI::UDim(0, 100), CEGUI::UDim(0, 30)));
    ApplyMassChangesToBallPushB->setVerticalAlignment(CEGUI::VA_BOTTOM);
    ApplyMassChangesToBallPushB->setHorizontalAlignment(CEGUI::HA_CENTRE);
    ApplyMassChangesToBallPushB->subscribeEvent(CEGUI::PushButton::EventClicked,
    		CEGUI::Event::Subscriber(&BallGame::ApplyMassChangesToBallPushBCallback, this));

    MainLayout->addChild(ApplyMassChangesToBallPushB);
    ButtonSetAddButton(EditBallButtons, ApplyMassChangesToBallPushB);

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

    SetCam(-184, -253, 352);
    mCamera->setOrientation(Ogre::Quaternion(0.835422, 0.393051, -0.238709, -0.300998));


    //Thumbnail Window
	mThumbnailSceneMgr->setAmbientLight(ColourValue(0.5, 0.5, 0.5));

	light = mThumbnailSceneMgr->createLight("MainThumbnailLight");
	lightNode = mThumbnailSceneMgr->getRootSceneNode()->createChildSceneNode("MainThumbnailLight");
	lightNode->attachObject(light);
	lightNode->setPosition(20, 80, 50);

    mThumbnailCamera->setPosition(-184, -253, 352);
    mThumbnailCamera->setOrientation(Ogre::Quaternion(0.835422, 0.393051, -0.238709, -0.300998));

    CEGUI::Texture &guiThumbnailTex = mRenderer->createTexture("Thumbnailtexture", pThumbnailtex);

    const CEGUI::Rectf Thumbnailrect(CEGUI::Vector2f(0.0f, 0.0f), guiThumbnailTex.getOriginalDataSize());
    CEGUI::BasicImage* Thumbnailimage = (CEGUI::BasicImage*)( &CEGUI::ImageManager::getSingleton().create("BasicImage", "ElementsThumbnail"));
		Thumbnailimage->setTexture(&guiThumbnailTex);
		Thumbnailimage->setArea(Thumbnailrect);
		Thumbnailimage->setAutoScaled(CEGUI::ASM_Both);

	ThumbnailWindow->setProperty("Image", "ElementsThumbnail");

	//ImportLevel Window
	mImportLevelSceneMgr->setAmbientLight(ColourValue(0.5, 0.5, 0.5));

    light = mImportLevelSceneMgr->createLight("MainImportLevelLight");
    lightNode = mImportLevelSceneMgr->getRootSceneNode()->createChildSceneNode("MainImportLevelLight");
    lightNode->attachObject(light);
    lightNode->setPosition(20, 80, 50);

	mImportLevelCamera->setPosition(-184, -253, 352);
	mImportLevelCamera->setOrientation(Ogre::Quaternion(0.835422, 0.393051, -0.238709, -0.300998));

	CEGUI::Texture &guiImportLevelTex = mRenderer->createTexture("ImportLeveltexture", pImportLeveltex);

	const CEGUI::Rectf ImportLevelrect(CEGUI::Vector2f(0.0f, 0.0f), guiImportLevelTex.getOriginalDataSize());
	CEGUI::BasicImage* ImportLevelimage = (CEGUI::BasicImage*)( &CEGUI::ImageManager::getSingleton().create("BasicImage", "ImportLevelWindow"));
	   ImportLevelimage->setTexture(&guiImportLevelTex);
	   ImportLevelimage->setArea(ImportLevelrect);
	   ImportLevelimage->setAutoScaled(CEGUI::ASM_Both);

	ImportLevelWindow->setProperty("Image", "ImportLevelWindow");

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
		Game->AddCaseToBeMoved(CaseToCheck);
	}
}

void BallGame::AddCaseToBeMoved(CaseEntity *ToAdd)
{
	if(ToAdd == NULL)
		return;
	std::list<CaseEntity*>::iterator iter(CasesToBeMoved.begin());
	while(iter != CasesToBeMoved.end())
	{
		CaseEntity *Case = *(iter++);
		if(Case == ToAdd)
			return;
	}
	CasesToBeMoved.push_back(ToAdd);
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
	{
		CaseHasForce = true;
		if(force_directed == true)
			ForcesArrows = UnderEditCase->CreateForceArrows(mSceneMgr);
	}
	else
	{
		CaseHasForce = false;
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
		UnderEditCase->ResetToInitial();
	}
	UnderEditCase = Entity;

	if(UnderEditCase != NULL && LevelEditMode == Edit && EntityEditMode == Caracts)
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
		ButtonsSetVisible(EditCaseButtons, false);
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
	{
		UnderEditBall->DisplaySelectedBox(false);
		UnderEditBall->ResetToInitial();
	}
	UnderEditBall = Entity;

	if(UnderEditBall != NULL && LevelEditMode == Edit && EntityEditMode == Caracts)
	{
		UnderEditBall->DisplaySelectedBox(true);
		UnderEditBallMass = UnderEditBall->getMass();
		ButtonsSetVisible(EditBallButtons, true);
		BallMassValueEditB->setDisabled(false);
		BallMassValueEditB->setText(toCEGUIString(UnderEditBallMass));
		ApplyMassChangesToBallPushB->setDisabled(false);
	}
	else
		ButtonsSetVisible(EditBallButtons, false);
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
		Entity->ResetToInitial();
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
    	switch(LevelEditMode)
    	{
    	case Place :
    		break;
    	case Delete :
    	case Edit :
    		if(LastHighligted != NULL)
			{
				//Case Entity ?
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
					//Hide and deselect all that was showed and selected from last time.
					MultiSelectionSetEmpty();
					if(LevelEditMode != Delete)
					{
						if(UnderEditBall != NULL)
							EditBall(NULL);//Hide Ball Editing buttons;
						if(UnderEditCase != NULL)
							EditCase(NULL);//Hide Case Editing buttons;
					}
					else
						UnprepareDeleteElement();

					//Then deal with what is to be shown and selected now.
					if(HighlightedGroup == NULL)
					{
						if(LevelEditMode != Delete)
						{
							switch(LastHighligted->getType())
							{
							case Case :
									LOG << "Edit Case by Mouse Pressed" << std::endl;
									EditCase((CaseEntity*)LastHighligted);
									break;
							case Ball :
									LOG << "Edit Ball by Mouse Pressed" << std::endl;
									EditBall((BallEntity*)LastHighligted);
									break;
							}
						}
						else
							PrepareDeleteElement(LastHighligted);
					}
					else
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
			if(LevelEditMode == Edit || LevelEditMode == Delete)
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
			ButtonsSetVisible(MainMenuButtons, false);
		else
			ButtonsSetVisible(MainMenuButtons, true);
	    break;
	case OIS::KeyCode::KC_UP:
    	if(MouseOverButton == false && EntityEditMode != Caracts)
    	{
    		switch(LevelEditMode)
    		{
    		case Place :
				switch(EntityEditAction)
				{
				case Move :
					if(ToBePlacedEntity != NULL)
						ToBePlacedEntity->Move(0, 10, 0);
					break;
				case Rotate :
					if(ToBePlacedEntity != NULL)
						ToBePlacedEntity->Rotate(0, 10, 0);
					break;
				case Scale :
					if(ToBePlacedEntity != NULL)
						ToBePlacedEntity->Scale(0, 10, 0);
					break;
				}
				break;
			case Edit :
				switch(EntityEditAction)
				{
				case Move :
					MoveEntities(0, 10, 0);
					break;
				case Rotate :
					RotateEntities(0, 10, 0);
					break;
				case Scale :
					ScaleEntities(0, 10, 0);
					break;
				}
				break;
    		}
		}
	    break;
	case OIS::KeyCode::KC_DOWN:
    	if(MouseOverButton == false && EntityEditMode != Caracts)
    	{
    		switch(LevelEditMode)
    		{
    		case Place :
				switch(EntityEditAction)
				{
				case Move :
					if(ToBePlacedEntity != NULL)
						ToBePlacedEntity->Move(0, -10, 0);
					break;
				case Rotate :
					if(ToBePlacedEntity != NULL)
						ToBePlacedEntity->Rotate(0, -10, 0);
					break;
				case Scale :
					if(ToBePlacedEntity != NULL)
						ToBePlacedEntity->Scale(0, -10, 0);
					break;
				}
				break;
			case Edit :
				switch(EntityEditAction)
				{
				case Move :
					MoveEntities(0, -10, 0);
					break;
				case Rotate :
					RotateEntities(0, -10, 0);
					break;
				case Scale :
					ScaleEntities(0, -10, 0);
					break;
				}
				break;
    		}
		}
	    break;
	case OIS::KeyCode::KC_RIGHT:
    	if(MouseOverButton == false && EntityEditMode != Caracts)
    	{
    		switch(LevelEditMode)
    		{
    		case Place :
				switch(EntityEditAction)
				{
				case Move :
					if(ToBePlacedEntity != NULL)
						ToBePlacedEntity->Move(10, 0, 0);
					break;
				case Rotate :
					if(ToBePlacedEntity != NULL)
						ToBePlacedEntity->Rotate(10, 0, 0);
					break;
				case Scale :
					if(ToBePlacedEntity != NULL)
						ToBePlacedEntity->Scale(10, 0, 0);
					break;
				}
				break;
			case Edit :
				switch(EntityEditAction)
				{
				case Move :
					MoveEntities(10, 0, 0);
					break;
				case Rotate :
					RotateEntities(10, 0, 0);
					break;
				case Scale :
					ScaleEntities(10, 0, 0);
					break;
				}
				break;
    		}
		}
	    break;
	case OIS::KeyCode::KC_LEFT:
    	if(MouseOverButton == false && EntityEditMode != Caracts)
    	{
    		switch(LevelEditMode)
    		{
    		case Place :
				switch(EntityEditAction)
				{
				case Move :
					if(ToBePlacedEntity != NULL)
						ToBePlacedEntity->Move(-10, 0, 0);
					break;
				case Rotate :
					if(ToBePlacedEntity != NULL)
						ToBePlacedEntity->Rotate(-10, 0, 0);
					break;
				case Scale :
					if(ToBePlacedEntity != NULL)
						ToBePlacedEntity->Scale(-10, 0, 0);
					break;
				}
				break;
			case Edit :
				switch(EntityEditAction)
				{
				case Move :
					MoveEntities(-10, 0, 0);
					break;
				case Rotate :
					RotateEntities(-10, 0, 0);
					break;
				case Scale :
					ScaleEntities(-10, 0, 0);
					break;
				}
				break;
    		}
		}
	    break;
	case OIS::KeyCode::KC_PGUP:
    	if(MouseOverButton == false && EntityEditMode != Caracts)
    	{
    		switch(LevelEditMode)
    		{
    		case Place :
				switch(EntityEditAction)
				{
				case Move :
					if(ToBePlacedEntity != NULL)
						ToBePlacedEntity->Move(0, 0, 10);
					break;
				case Rotate :
					if(ToBePlacedEntity != NULL)
						ToBePlacedEntity->Rotate(0, 0, 10);
					break;
				case Scale :
					if(ToBePlacedEntity != NULL)
						ToBePlacedEntity->Scale(0, 0, 10);
					break;
				}
				break;
			case Edit :
				switch(EntityEditAction)
				{
				case Move :
					MoveEntities(0, 0, 10);
					break;
				case Rotate :
					RotateEntities(0, 0, 10);
					break;
				case Scale :
					ScaleEntities(0, 0, 10);
					break;
				}
				break;
    		}
		}
	    break;
	case OIS::KeyCode::KC_PGDOWN:
    	if(MouseOverButton == false && EntityEditMode != Caracts)
    	{
    		switch(LevelEditMode)
    		{
    		case Place :
				switch(EntityEditAction)
				{
				case Move :
					if(ToBePlacedEntity != NULL)
						ToBePlacedEntity->Move(0, 0, -10);
					break;
				case Rotate :
					if(ToBePlacedEntity != NULL)
						ToBePlacedEntity->Rotate(0, 0, -10);
					break;
				case Scale :
					if(ToBePlacedEntity != NULL)
						ToBePlacedEntity->Scale(0, 0, -10);
					break;
				}
				break;
			case Edit :
				switch(EntityEditAction)
				{
				case Move :
					MoveEntities(0, 0, -10);
					break;
				case Rotate :
					RotateEntities(0, 0, -10);
					break;
				case Scale :
					ScaleEntities(0, 0, -10);
					break;
				}
				break;
    		}
		}
	    break;
	case OIS::KeyCode::KC_SPACE:
		if(mode == Editing && MouseOverButton == false)
		{
			switch(LevelEditMode)
			{
			case Place :
				PlaceNewElement();
				break;
			case Edit :
				switch(EntityEditMode)
				{
				case Simple :
					PlaceUnderEditElement();
					break;
				case Moves :
					break;
				}
				break;
			}
		}
		break;
	case OIS::KeyCode::KC_DELETE:
		if(mode == Editing && MouseOverButton == false && EntityEditMode != Caracts)
			DeleteElement();
		break;
	case OIS::KeyCode::KC_M:
		if(mode == Editing)
			SetMoveElementAction();
		break;
	case OIS::KeyCode::KC_R:
		if(mode == Editing)
			SetRotateElementAction();
		break;
	case OIS::KeyCode::KC_S:
		if(mode == Editing)
			SetScaleElementAction();
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
		GroupEntity *B = *(it++);
		if(B != NULL && B->getName() == name)
			return B;
	}
	return NULL;
}

void BallGame::DeleteBall(BallEntity *Entity, std::list<BallEntity*>::iterator *iter)
{
	RemoveBall(Entity, iter);
	GroupEntity *Group = Entity->getGroup();
	if(Group != NULL)
	{
		bool tobedel = Group->DelChild((BallGameEntity*)Entity);
		if(tobedel)
			DeleteGroup(Group);
	}
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
	{
		bool tobedel = Group->DelChild((BallGameEntity*)Entity);
		if(tobedel)
			DeleteGroup(Group);
	}
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
	CasesToBeMoved.clear();
	EmptyStatesList();

	//Force an update of Physic to force garbage collecting !
	_updatePhysic(1.0f / MAX_PHYSICS_FPS);
	_StopPhysic();
}

void BallGame::EmptyLevelsList(void)
{
	while(ChooseLevelComboB->getItemCount())
	{
		CEGUI::ListboxTextItem *item = (CEGUI::ListboxTextItem*)ChooseLevelComboB->getListboxItemFromIndex(0);
		String *str = (String*)item->getUserData();
		if(str != NULL)
			delete str;
		ChooseLevelComboB->removeItem(item);
	}
}

void BallGame::EmptyStatesList(void)
{
	while (ChooseStateToLoadB->getItemCount())
	{
		CEGUI::ListboxTextItem *item = (CEGUI::ListboxTextItem*)ChooseStateToLoadB->getListboxItemFromIndex(0);
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
	String nodeNamePrefix;
	ImportLevelFromJson((Node*)mSceneMgr->getRootSceneNode(), nodeNamePrefix);
	LoadStatesList();
}

#define COUNTER_JSON_FIELD "InternalCounter"
#define GROUPS_JSON_FIELD "Groups"
#define CASES_JSON_FIELD "Cases"
#define BALLS_JSON_FIELD "Balls"

void BallGame::ImportLevelFromJson(Node *parent, String &nodeNamePrefix, bool isForImport)
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
		newGroup->ImportFromJson(groupjson, parent, nodeNamePrefix);
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

		if(newCase->CaseToMove() == true)
			AddCaseToBeMoved(newCase);
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

void BallGame::ExportLevelIntoJson(String &export_str)
{
	rapidjson::Document document;
	document.SetObject();

	rapidjson::Document::AllocatorType& allocator = document.GetAllocator();

	document.AddMember(COUNTER_JSON_FIELD, nb_entities, allocator);

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

	document.AddMember(GROUPS_JSON_FIELD, groups, allocator);

	rapidjson::Value cases(rapidjson::kArrayType);

	std::list<CaseEntity*>::iterator Cit(Cases.begin());
	while(Cit != Cases.end())
	{
		CaseEntity *Entity = *(Cit++);
		if(Entity == NULL)
			continue;
		rapidjson::Value JCase(rapidjson::kObjectType);
		Entity->ExportToJson(JCase, allocator);

		cases.PushBack(JCase, allocator);
	}

	document.AddMember(CASES_JSON_FIELD, cases, allocator);

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

	document.AddMember(BALLS_JSON_FIELD, balls, allocator);

	rapidjson::StringBuffer strbuf;
	rapidjson::Writer<rapidjson::StringBuffer> writer(strbuf);
	document.Accept(writer);

	export_str = strbuf.GetString();

	LOG << strbuf.GetString() << std::endl;
}
