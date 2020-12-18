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
//#include <sys/types.h>
#include <glob.h>
//Put BallGame.h in last because of Xlib defines (True False Bool None) which must be undef
#include "BallGame.h"


void EquilibrateAABBAroundOrigin(Node *node)
{
	Vector3 min(NAN, NAN, NAN), max(NAN, NAN, NAN);
	Node::ChildNodeIterator ite(node->getChildIterator());
	while ( ite.hasMoreElements() )
	{
	       SceneNode* child = static_cast<SceneNode*>(ite.getNext());
	       Vector3 childmin = child->_getWorldAABB().getMinimum();
	       Vector3 childmax = child->_getWorldAABB().getMaximum();

	       childmin *= child->getScale();
	       childmin += child->getPosition();
	       childmax *= child->getScale();
	       childmax += child->getPosition();

	       if(min.isNaN())
	    	   min = childmin;
	       if(max.isNaN())
	    	   max = childmax;

	       min.x = min2(min.x, childmin.x);
	       min.y = min2(min.y, childmin.y);
	       min.z = min2(min.z, childmin.z);

	       max.x = max2(max.x, childmax.x);
	       max.y = max2(max.y, childmax.y);
	       max.z = max2(max.z, childmax.z);
	}

	node->translate((min + max) / 2);

	Node::ChildNodeIterator ite2(node->getChildIterator());
	while ( ite2.hasMoreElements() )
	{
		   SceneNode* child = static_cast<SceneNode*>(ite2.getNext());
		   child->translate(-1 * (min + max) / 2);
	}
}



void BallGame::BodySerialization (NewtonBody* const body, void* const bodyUserData, NewtonSerializeCallback serializeCallback, void* const serializeHandle)
{
	BallGameEntity *Entity = (BallGameEntity*)NewtonBodyGetUserData(body);

	const char* const bodyIndentification = Entity->OgreEntity->getName().c_str();
	std::cout << "Entity (" << Entity << "/" << body << ") name :" << bodyIndentification << std::endl;
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

	BallGameEntity *Entity = Game->GetEntity(bodyIndentification);

	NewtonBodySetUserData (body, Entity);
	Entity->SetNewtonBody(body);
	NewtonBodySetTransformCallback(body, BallGameEntity::TransformCallback);
	if(Entity->type == Ball)
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
		if(Entity != NULL && Entity->OgreEntity->getName() == name)
			return (BallGameEntity*)Entity;
	}
	std::list<BallEntity*>::iterator Biter(Balls.begin());
	while(Biter != Balls.end())
	{
		BallEntity *Entity = *(Biter++);
		if(Entity != NULL && Entity->OgreEntity->getName() == name)
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
	std::cout << "Load '" << (*name) << "'" << std::endl;
	_StopPhysic();
	std::list<CaseEntity*>::iterator Citer(Cases.begin());
	while(Citer != Cases.end())
	{
		CaseEntity *Entity = *(Citer++);
		if(Entity != NULL)
			Entity->SetNewtonBody(NULL);
	}
	std::list<BallEntity*>::iterator Biter(Balls.begin());
	while(Biter != Balls.end())
	{
		BallEntity *Entity = *(Biter++);
		if(Entity == NULL)
			continue;
		Entity->SetNewtonBody(NULL);
		Entity->CleanupForces();
	}
	NewtonDestroyAllBodies(m_world);
	NewtonMaterialDestroyAllGroupID(m_world);
	NewtonDeserializeFromFile(m_world, name->c_str(), BodyDeserialization, this);
	_StartPhysic();
}


bool BallGame::SaveStatePushBCallback(const CEGUI::EventArgs &e)
{
	String state_name = Level;
	state_name += "-";
	state_name += std::to_string(ChooseStateToLoadB->getItemCount());
	SerializedPhysicScene(&state_name);
	ChooseStateToLoadB->addItem(new CEGUI::ListboxTextItem(state_name));
	ChooseStateToLoadB->setSize(CEGUI::USize(CEGUI::UDim(0, 150), CEGUI::UDim(0, 40 * (ChooseStateToLoadB->getItemCount() + 1))));
	return true;
}

bool BallGame::LoadStatePushBCallback(const CEGUI::EventArgs &e)
{
	CEGUI::ListboxItem *item = ChooseStateToLoadB->getSelectedItem();
	if(item == NULL)
		return true;
	String state_name = item->getText().c_str();
	if(state_name.empty() == false)
		DeserializedPhysicScene(&state_name);
	return true;
}

BallGameEntity::BallGameEntity(const dMatrix& matrix) :// m_matrix(matrix),
	m_curPosition (matrix.m_posit),
	m_nextPosition (matrix.m_posit),
	m_curRotation (dQuaternion (matrix)),
	m_nextRotation (dQuaternion (matrix))
{
	OgreEntity = NULL;
	Body = NULL;
	type = Case;
}

BallGameEntity::BallGameEntity()
{
	OgreEntity = NULL;
	Body = NULL;
	type = Case;
}

void BallGameEntity::Finalize(Ogre::SceneManager* mSceneMgr)
{
	NewtonDestroyBody(Body);
	mSceneMgr->getRootSceneNode()->removeChild(OgreEntity);
}

void BallGameEntity::SetOgreNode(SceneNode *node)
{
	OgreEntity = node;
	if(node != NULL)
	{
		node->setPosition(InitialPos);
		node->setScale(InitialScale);
		node->setOrientation(InitialOrientation);
		((Ogre::Entity*)node->getAttachedObject(0))->getUserObjectBindings().setUserAny(Ogre::Any(this));
	}
}

void BallGameEntity::SetNewtonBody(NewtonBody *body)
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
		std::cout << "angle < 0 : " << angle << std::endl;
	}
}

void BallGameEntity::TransformCallback(const NewtonBody* body, const dFloat* matrix, int threadIndex)
{
//	std::cout << "TransformCallback" << std::endl;
	BallGameEntity* const ent = (BallGameEntity*) NewtonBodyGetUserData(body);
	if (ent) {
		BallGame* const scene = (BallGame*)NewtonWorldGetUserData(NewtonBodyGetWorld(body));
		dMatrix transform(matrix);
		dQuaternion rot;
		NewtonBodyGetRotation(body, &rot.m_x);

		//scene->Lock(ent->m_lock);
		ent->SetMatrixUsafe(rot, transform.m_posit);
		//scene->Unlock(ent->m_lock);
//		std::cout << "Entity transform " << "position {" << ent->m_curPosition.m_x << ", " << ent->m_curPosition.m_y << ", " << ent->m_curPosition.m_z << "}";
//		std::cout << " Orientation {" << ent->m_curRotation.m_w << ", " << ent->m_curRotation.m_x << ", " << ent->m_curRotation.m_y << ", " << ent->m_curRotation.m_z << "}" << std::endl;
		ent->OgreEntity->setPosition(ent->m_curPosition.m_x, ent->m_curPosition.m_y, ent->m_curPosition.m_z);
		ent->OgreEntity->setOrientation(ent->m_curRotation.m_w, ent->m_curRotation.m_x, ent->m_curRotation.m_y, ent->m_curRotation.m_z);
	}
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
	std::cout << "Add Force {" << (*force)[0] << ", " << (*force)[1] << ", " << (*force)[2] << "} On ball" << std::endl;
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

/*
void CaseEntity::AddBallColliding(NewtonBody *ball)
{
	if(ball == NULL)
		return;
	int id = NewtonBodyGetID(ball), size = BallsUnderCollide.GetSize();
	while(size < id)
		BallsUnderCollide[size++] = NULL;
	BallsUnderCollide[id] = ball;
	size = id + 1;
	while(size < BallsUnderCollide.GetSize())
		BallsUnderCollide[size++] = NULL;
}

bool CaseEntity::CheckIfAlreadyColliding(NewtonBody *ball)
{
	bool ret = false;
	if(ball == NULL)
		return ret;
	int id = NewtonBodyGetID(ball);
	if(id < BallsUnderCollide.GetSize())
	{
		if(BallsUnderCollide[id] == ball)
			ret = true;
	}
	return ret;
}
*/
void CaseEntity::ApplyForceOnBall(BallEntity *ball)
{
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
			dFloat velocity[3], sum;
			NewtonBodyGetVelocity(ball->Body, velocity);
			sum = Normalize3(velocity[0], velocity[1], velocity[2]);
			velocity[0] /= sum;
			velocity[1] /= sum;
			velocity[2] /= sum;//Like that we have normalization of velocity into percents, we can use it to scale force.
			dVector *force = new dVector(velocity[0], velocity[1], velocity[2]);
			*force = force->Scale(force_to_apply);
			ball->AddForceVector(force);
		}
	}
	//AddBallColliding(ball);
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
	UnderEditBall = NULL;
	ToBePlacedEntity = NULL;
	LastPlacedEntity = NULL;
	ToBePlacedEntityType = NULL;
	ogreThumbnailNode = NULL;
	PlacementMode = PlaceMove;
	mode = Running;
	MouseOverButton = false;
	// create the newton world
	SetupNewton();
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
}

BallGame::~BallGame()
{
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
		iter++;
	}
	EntityTypes.clear();
}

void BallGame::PostUpdateCallback(const NewtonWorld* const world, dFloat timestep)
{
//	std::cout << "Post Update Time " << timestep << std::endl;
/*	BallGame* const scene = (BallGame*) NewtonWorldGetUserData(world);
	scene->m_cameraManager->FixUpdate(scene->GetNewton(), timestep);
	if (scene->m_updateCamera) {
		scene->m_updateCamera(scene, scene->m_updateCameraContext, timestep);
	}*/
}

void BallGame::UpdatePhysics(dFloat timestep)
{
	// update the physics
//	std::cout << " Update Time " << timestep << std::endl;
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
	MouseOverButton = true;
	return true;
}

bool BallGame::LeavingArea(const CEGUI::EventArgs &event)
{
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
	UnderEditCaseForce = strtof(CaseForceValueEditB->getText().c_str(), NULL);
	return true;
}

inline void BallGame::CaseForceValueEditBSetText(float value)
{
	char force_c[20];
	snprintf(force_c, 19, "%.3f", value);
	CaseForceValueEditB->setText(force_c);
}

inline void BallGame::CaseForceValueEditBSetText(double value)
{
	char force_c[20];
	snprintf(force_c, 19, "%.3f", value);
	CaseForceValueEditB->setText(force_c);
}

inline void BallGame::CaseForceDirectionXValueEditBSetText(float value)
{
	char force_c[20];
	snprintf(force_c, 19, "%.3f", value);
	CaseForceDirectionXValueEditB->setText(force_c);
}

inline void BallGame::CaseForceDirectionXValueEditBSetText(double value)
{
	char force_c[20];
	snprintf(force_c, 19, "%.3f", value);
	CaseForceDirectionXValueEditB->setText(force_c);
}

inline void BallGame::CaseForceDirectionYValueEditBSetText(float value)
{
	char force_c[20];
	snprintf(force_c, 19, "%.3f", value);
	CaseForceDirectionYValueEditB->setText(force_c);
}

inline void BallGame::CaseForceDirectionYValueEditBSetText(double value)
{
	char force_c[20];
	snprintf(force_c, 19, "%.3f", value);
	CaseForceDirectionYValueEditB->setText(force_c);
}

inline void BallGame::CaseForceDirectionZValueEditBSetText(float value)
{
	char force_c[20];
	snprintf(force_c, 19, "%.3f", value);
	CaseForceDirectionZValueEditB->setText(force_c);
}

inline void BallGame::CaseForceDirectionZValueEditBSetText(double value)
{
	char force_c[20];
	snprintf(force_c, 19, "%.3f", value);
	CaseForceDirectionZValueEditB->setText(force_c);
}

bool BallGame::NormalizeCaseForceDirectionPushBCallback(const CEGUI::EventArgs &e)
{
	char force_c[20];
	double direction_x, direction_y, direction_z, direction;
	direction_x = strtod(CaseForceDirectionXValueEditB->getText().c_str(), NULL);
	direction_y = strtod(CaseForceDirectionYValueEditB->getText().c_str(), NULL);
	direction_z = strtod(CaseForceDirectionZValueEditB->getText().c_str(), NULL);
	direction = Normalize3(direction_x, direction_y, direction_z);

	CaseForceDirectionXValueEditBSetText(direction_x / direction);

	CaseForceDirectionYValueEditBSetText(direction_y / direction);

	CaseForceDirectionZValueEditBSetText(direction_z / direction);
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
		std::cout << "Edit Mode" << std::endl;
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
		SetMoveNewElement();
		PrepareNewElement();
	}
	else
	{
		std::cout << "Running Mode" << std::endl;
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
		if(LastHighligted != NULL)
		{
			LastHighligted->showBoundingBox(false);
			LastHighligted = NULL;
		}
		if(UnderEditCase != NULL)
		{
			UnderEditCase->OgreEntity->showBoundingBox(false);
			UnderEditCase = NULL;
		}
		LastPlacedEntity = NULL;
		UnprepareNewElement();
	}
}

void BallGame::CreateThumbnail(String meshname)
{
	Entity *ogreEntity = mThumbnailSceneMgr->createEntity(meshname);
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
	mThumbnailSceneMgr->getRootSceneNode()->removeAllChildren();

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
	if(ToBePlacedEntity != NULL)
	{
		mSceneMgr->getRootSceneNode()->removeChild(ToBePlacedEntity->OgreEntity);
		delete ToBePlacedEntity;
		ToBePlacedEntity = NULL;
		LastPlacedEntity = NULL;
	}
	PrepareNewElement();
	return true;
}

void BallGame::DeleteElement(void)
{
	if(PlacementMode != Delete)
		return;
	if(UnderEditBall != NULL)
	{
		RemoveBall(UnderEditBall);
		EditBall(NULL);
	}
	else if(UnderEditCase != NULL)
	{
		RemoveCase(UnderEditCase);
		EditCase(NULL);
	}
}

bool BallGame::DeleteElementBCallback(const CEGUI::EventArgs &e)
{
	PlacementMode = Delete;
	DeleteElementB->setDisabled(true);
	PlaceNewElementB->setDisabled(false);
	EditElementB->setDisabled(false);
	MoveElementB->setVisible(false);
	RotateElementB->setVisible(false);
	ScaleElementB->setVisible(false);
	EditBall(NULL);
	EditCase(NULL);
	UnprepareNewElement();
	DeleteElement();
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
	SetMoveNewElement();
	PrepareNewElement();
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
	SetMoveElement();
	UnprepareNewElement();
	return true;
}

void BallGame::SetMoveElement(void)
{
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
	PlacementMode = PlaceMove;
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
		mSceneMgr->getRootSceneNode()->removeChild(ToBePlacedEntity->OgreEntity);
		delete ToBePlacedEntity;
		ToBePlacedEntity = NULL;
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

	std::cout << "Placing new element ?" << std::endl;
	switch(ToBePlacedEntityType->Type)
	{
	case Case :
		ToBePlacedEntity = new CaseEntity();
		break;
	case Ball :
		ToBePlacedEntity = new BallEntity();
		break;
	}
	std::cout << "Pos = " << Pos.x << ", " << Pos.y << ", " << Pos.z << std::endl;
	std::cout << "Scale = " << Scale.x << ", " << Scale.y << ", " << Scale.z << std::endl;
	std::cout << "Orient = " << Orient.x << ", " << Orient.y << ", " << Orient.z << ", " << Orient.w << std::endl;
	std::cout << "Mesh = " << ToBePlacedEntityType->MeshName << std::endl;
	ogreEntity = mSceneMgr->createEntity(ToBePlacedEntityType->MeshName);

	if(LastPlacedEntity != NULL)
	{
		Pos = LastPlacedEntity->InitialPos;
		Scale = LastPlacedEntity->InitialScale;
		Orient = LastPlacedEntity->InitialOrientation;
	}

	ToBePlacedEntity->InitialPos = Pos;
	ToBePlacedEntity->InitialScale = Scale;
	ToBePlacedEntity->InitialOrientation = Orient;

	ogreNode = mSceneMgr->getRootSceneNode()->createChildSceneNode(ToBePlacedEntity->InitialPos);
	ogreNode->attachObject(ogreEntity);
	ogreNode->showBoundingBox(true);
	ToBePlacedEntity->SetOgreNode(ogreNode);
}

void BallGame::PlaceUnderEditElement(void)
{
	BallGameEntity *EditingEntity = NULL;
	if(UnderEditBall != NULL)
	{
		RemoveBall(UnderEditBall, NULL);
		EditingEntity = UnderEditBall;
	}
	else if(UnderEditCase != NULL)
	{
		RemoveCase(UnderEditCase, NULL);
		EditingEntity = UnderEditCase;
	}
	PlaceElement(EditingEntity);
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
	std::cout << "Placing new element !" << std::endl;
	ToBePlaced->OgreEntity->showBoundingBox(false);


	dVector NewtonBodyLocation;
	dVector NewtonBodySize;
	NewtonBody *newtonBody;

	ToBePlaced->InitialPos = ToBePlaced->OgreEntity->getPosition();
	ToBePlaced->InitialScale = ToBePlaced->OgreEntity->getScale();
	ToBePlaced->InitialOrientation = ToBePlaced->OgreEntity->getOrientation();
	NewtonBodyLocation.m_x = ToBePlaced->InitialPos.x;
	NewtonBodyLocation.m_y = ToBePlaced->InitialPos.y;
	NewtonBodyLocation.m_z = ToBePlaced->InitialPos.z;
	NewtonBodyLocation.m_w = 1;
	Entity *ogreEntity = (Ogre::Entity*)ToBePlaced->OgreEntity->getAttachedObject(0);
	Vector3 AABB(ogreEntity->getBoundingBox().getSize());
	NewtonBodySize.m_x = AABB.x * ToBePlaced->InitialScale.x;
	NewtonBodySize.m_y = AABB.y * ToBePlaced->InitialScale.y;
	NewtonBodySize.m_z = AABB.z * ToBePlaced->InitialScale.z;
	NewtonBodySize.m_w = 0.0f;

	dMatrix bodymatrix(ToBePlaced->InitialOrientation.getPitch(false).valueRadians(), ToBePlaced->InitialOrientation.getYaw(false).valueRadians(), ToBePlaced->InitialOrientation.getRoll(false).valueRadians(), NewtonBodyLocation);
	switch(ToBePlaced->type)
	{
	case Case :
		{
			CaseEntity *Case = (CaseEntity*)ToBePlaced;
			NewtonCollision *collision_tree = NULL;
			Matrix4 ogre_matrix;
			if(Case->type == CaseEntity::CaseType::typeRamp)
				std::cout << "Place a Ramp" << std::endl;
			else
				std::cout << "Place a Box" << std::endl;

			ogre_matrix.makeTransform(Vector3::ZERO, ToBePlaced->InitialScale, Quaternion::IDENTITY);
			const MeshPtr ptr = ogreEntity->getMesh();
			collision_tree = ParseEntity(m_world, ptr, ogre_matrix);

			newtonBody = WorldAddCase(m_world, NewtonBodySize, 0, bodymatrix, collision_tree);
			ToBePlaced->SetNewtonBody(newtonBody);
			AddCase(Case);
		}
		break;
	case Ball :
		std::cout << "Place a Ball" << std::endl;
		newtonBody = WorldAddBall(m_world, ToBePlacedEntityType->InitialMass, NewtonBodySize, 0, bodymatrix);
		ToBePlaced->SetNewtonBody(newtonBody);
		AddBall((BallEntity*)ToBePlaced);
		break;
	}
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
		SaveStatePushB->setVisible(true);
	}
	else
	{
		StatesBanner->setVisible(false);
		ChooseStateToLoadB->setVisible(false);
		LoadStatePushB->setVisible(false);
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

    StopPhysicPushB = (CEGUI::PushButton*)wmgr.createWindow("OgreTray/Button");
    StopPhysicPushB->setText("Start/Stop Physic");
    StopPhysicPushB->setSize(CEGUI::USize(CEGUI::UDim(0, 150), CEGUI::UDim(0, 30)));
    StopPhysicPushB->setVisible(false);

    MainLayout->addChild(StopPhysicPushB);

    StopPhysicPushB->subscribeEvent(CEGUI::PushButton::EventClicked,
    		CEGUI::Event::Subscriber(&BallGame::StopPhysicPushBCallback, this));

    EditModePushB = (CEGUI::PushButton*)wmgr.createWindow("OgreTray/Button");
    EditModePushB->setText("Edit");
    EditModePushB->setSize(CEGUI::USize(CEGUI::UDim(0, 75), CEGUI::UDim(0, 30)));
    EditModePushB->setVisible(false);

    MainLayout->addChild(EditModePushB);

    EditModePushB->subscribeEvent(CEGUI::PushButton::EventClicked,
    		CEGUI::Event::Subscriber(&BallGame::EditModePushBCallback, this));

    SetWindowsPosNearToOther(EditModePushB, StopPhysicPushB, 0, 1);

    StatesModePushB = (CEGUI::PushButton*)wmgr.createWindow("OgreTray/Button");
    StatesModePushB->setText("States");
    StatesModePushB->setSize(CEGUI::USize(CEGUI::UDim(0, 75), CEGUI::UDim(0, 30)));
    StatesModePushB->setVisible(false);

    MainLayout->addChild(StatesModePushB);

    StatesModePushB->subscribeEvent(CEGUI::PushButton::EventClicked,
    		CEGUI::Event::Subscriber(&BallGame::StatesModePushBCallback, this));

    SetWindowsPosNearToOther(StatesModePushB, EditModePushB, 1, 0);

    ChooseLevelComboB = (CEGUI::Combobox*)wmgr.createWindow("OgreTray/Combobox");
    glob_t glob_result;
	memset(&glob_result, 0, sizeof(glob_result));
	glob("Levels/*.json", 0, NULL, &glob_result);
	for(size_t i = 0; i < glob_result.gl_pathc; ++i)
	{
		String globname(glob_result.gl_pathv[i]), filename;
		size_t slashpos = globname.find_last_of('/'), dotpos = globname.find_last_of('.');
		filename = globname.substr(slashpos + 1, dotpos - (slashpos + 1));
		ChooseLevelComboB->addItem(new CEGUI::ListboxTextItem(filename));
        if(ChooseLevelComboB->getText().empty() == true)
        	ChooseLevelComboB->setText(filename);
	}
	globfree(&glob_result);
    ChooseLevelComboB->setSize(CEGUI::USize(CEGUI::UDim(0, 150), CEGUI::UDim(0, 40 * (ChooseLevelComboB->getItemCount() + 1))));
    ChooseLevelComboB->setVisible(false);

    MainLayout->addChild(ChooseLevelComboB);

    ChooseLevelComboB->subscribeEvent(CEGUI::Combobox::EventListSelectionAccepted,
    		CEGUI::Event::Subscriber(&BallGame::ChooseLevelComboBCallback, this));

    SetWindowsPosNearToOther(ChooseLevelComboB, EditModePushB, 0, 1);

    NewLevelEditB = (CEGUI::Editbox*)wmgr.createWindow("OgreTray/Editbox");
    NewLevelEditB->setText("NewLevel");
    NewLevelEditB->setSize(CEGUI::USize(CEGUI::UDim(0, 90), CEGUI::UDim(0, 30)));
    NewLevelEditB->setVisible(false);

    MainLayout->addChild(NewLevelEditB);

//    NewLevelEditB->subscribeEvent(CEGUI::Editbox::EventClicked,
//    		CEGUI::Event::Subscriber(&BallGame::SaveLevelPushBCallback, this));

    SetWindowsPosNearToOther(NewLevelEditB, EditModePushB, 0, 2);// Be Carefull, Combobox size is size with combo expanded !

    NewLevelCreateB = (CEGUI::PushButton*)wmgr.createWindow("OgreTray/Button");
    NewLevelCreateB->setText("Create");
    NewLevelCreateB->setSize(CEGUI::USize(CEGUI::UDim(0, 60), CEGUI::UDim(0, 30)));
    NewLevelCreateB->setVisible(false);

    MainLayout->addChild(NewLevelCreateB);

    NewLevelCreateB->subscribeEvent(CEGUI::PushButton::EventClicked,
			CEGUI::Event::Subscriber(&BallGame::NewLevelCreateBCallback, this));

    SetWindowsPosNearToOther(NewLevelCreateB, NewLevelEditB, 1, 0);

    SaveLevelPushB = (CEGUI::PushButton*)wmgr.createWindow("OgreTray/Button");
    SaveLevelPushB->setText("Save");
    SaveLevelPushB->setSize(CEGUI::USize(CEGUI::UDim(0, 150), CEGUI::UDim(0, 30)));
    SaveLevelPushB->setVisible(false);

    MainLayout->addChild(SaveLevelPushB);

    SaveLevelPushB->subscribeEvent(CEGUI::PushButton::EventClicked,
    		CEGUI::Event::Subscriber(&BallGame::SaveLevelPushBCallback, this));

    SetWindowsPosNearToOther(SaveLevelPushB, NewLevelEditB, 0, 1);

    QuitPushB = (CEGUI::PushButton*)wmgr.createWindow("OgreTray/Button");
    QuitPushB->setText("Quit");
    QuitPushB->setTooltipText("Quit");
    QuitPushB->setSize(CEGUI::USize(CEGUI::UDim(0, 150), CEGUI::UDim(0, 30)));
    QuitPushB->setVisible(false);

    MainLayout->addChild(QuitPushB);

    QuitPushB->subscribeEvent(CEGUI::PushButton::EventClicked,
    		CEGUI::Event::Subscriber(&BallGame::QuitPushBCallback, this));

    SetWindowsPosNearToOther(QuitPushB, SaveLevelPushB, 0, 1);

    LevelNameBanner = (CEGUI::Titlebar*)wmgr.createWindow("OgreTray/Titlebar");
    LevelNameBanner->setText("Level");
    LevelNameBanner->setSize(CEGUI::USize(CEGUI::UDim(0, 150), CEGUI::UDim(0, 30)));
    LevelNameBanner->setVerticalAlignment(CEGUI::VA_TOP);
    LevelNameBanner->setHorizontalAlignment(CEGUI::HA_CENTRE);

    MainLayout->addChild(LevelNameBanner);

    StatesBanner = (CEGUI::Titlebar*)wmgr.createWindow("OgreTray/Titlebar");
    StatesBanner->setText("States");
    StatesBanner->setSize(CEGUI::USize(CEGUI::UDim(0, 150), CEGUI::UDim(0, 30)));
    StatesBanner->setVerticalAlignment(CEGUI::VA_TOP);
    StatesBanner->setHorizontalAlignment(CEGUI::HA_RIGHT);
    StatesBanner->setVisible(false);

    MainLayout->addChild(StatesBanner);

    ChooseStateToLoadB = (CEGUI::Combobox*)wmgr.createWindow("OgreTray/Combobox");
    ChooseStateToLoadB->setSize(CEGUI::USize(CEGUI::UDim(0, 150), CEGUI::UDim(0, 30)));
    ChooseStateToLoadB->setVerticalAlignment(CEGUI::VA_TOP);
    ChooseStateToLoadB->setHorizontalAlignment(CEGUI::HA_RIGHT);
    ChooseStateToLoadB->setVisible(false);

    MainLayout->addChild(ChooseStateToLoadB);
    SetWindowsPosNearToOther(ChooseStateToLoadB, StatesBanner, 0, 1);

    LoadStatePushB = (CEGUI::PushButton*)wmgr.createWindow("OgreTray/Button");
    LoadStatePushB->setText("Load");
    LoadStatePushB->setSize(CEGUI::USize(CEGUI::UDim(0, 150), CEGUI::UDim(0, 30)));
    LoadStatePushB->setVerticalAlignment(CEGUI::VA_TOP);
    LoadStatePushB->setHorizontalAlignment(CEGUI::HA_RIGHT);
    LoadStatePushB->setVisible(false);

    MainLayout->addChild(LoadStatePushB);
    LoadStatePushB->subscribeEvent(CEGUI::PushButton::EventClicked,
    		CEGUI::Event::Subscriber(&BallGame::LoadStatePushBCallback, this));
    SetWindowsPosNearToOther(LoadStatePushB, StatesBanner, 0, 2);

    SaveStatePushB = (CEGUI::PushButton*)wmgr.createWindow("OgreTray/Button");
    SaveStatePushB->setText("Save");
    SaveStatePushB->setSize(CEGUI::USize(CEGUI::UDim(0, 150), CEGUI::UDim(0, 30)));
    SaveStatePushB->setVerticalAlignment(CEGUI::VA_TOP);
    SaveStatePushB->setHorizontalAlignment(CEGUI::HA_RIGHT);
    SaveStatePushB->setVisible(false);

    MainLayout->addChild(SaveStatePushB);
    SaveStatePushB->subscribeEvent(CEGUI::PushButton::EventClicked,
    		CEGUI::Event::Subscriber(&BallGame::SaveStatePushBCallback, this));
    SetWindowsPosNearToOther(SaveStatePushB, LoadStatePushB, 0, 1);

    //Now LevelNameBanner exist, we can call SetLevel !
    String default_level = ChooseLevelComboB->getText().c_str();
    SetLevel(default_level);


    //Edit GUI

    EditingModeTitleBanner = (CEGUI::Titlebar*)wmgr.createWindow("OgreTray/Title");
    EditingModeTitleBanner->setText("Edit Mode");
    EditingModeTitleBanner->setSize(CEGUI::USize(CEGUI::UDim(0, 150), CEGUI::UDim(0, 30)));
    EditingModeTitleBanner->setVerticalAlignment(CEGUI::VA_TOP);
    EditingModeTitleBanner->setHorizontalAlignment(CEGUI::HA_CENTRE);
    EditingModeTitleBanner->setVisible(false);

    MainLayout->addChild(EditingModeTitleBanner);

    SetWindowsPosNearToOther(EditingModeTitleBanner, EditingModeTitleBanner, 0, 1);

    // Add new Element GUI
    AddElementTitleBanner = (CEGUI::Titlebar*)wmgr.createWindow("OgreTray/Titlebar");
    AddElementTitleBanner->setText("Add");
    AddElementTitleBanner->setSize(CEGUI::USize(CEGUI::UDim(0, 150), CEGUI::UDim(0, 30)));
    AddElementTitleBanner->setVerticalAlignment(CEGUI::VA_TOP);
    AddElementTitleBanner->setHorizontalAlignment(CEGUI::HA_RIGHT);
    CEGUI::UVector2 pos = AddElementTitleBanner->getPosition();
    pos.d_y = CEGUI::UDim(0, (mWindow->getHeight() / 2) - 120);
    AddElementTitleBanner->setPosition(pos);
    AddElementTitleBanner->setVisible(false);

    MainLayout->addChild(AddElementTitleBanner);


    ChooseTypeOfElementToAddB = (CEGUI::Combobox*)wmgr.createWindow("OgreTray/Combobox");
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
    ChooseTypeOfElementToAddB->setVisible(false);

    ChooseTypeOfElementToAddB->subscribeEvent(CEGUI::Combobox::EventListSelectionAccepted,
    		CEGUI::Event::Subscriber(&BallGame::ChooseTypeOfElementToAddBCallback, this));

    MainLayout->addChild(ChooseTypeOfElementToAddB);

    ThumbnailWindow = CEGUI::WindowManager::getSingleton().createWindow("OgreTray/StaticImage", "RTTWindow");
    ThumbnailWindow->setSize(CEGUI::USize(CEGUI::UDim(0, 150),
 						   CEGUI::UDim(0, 150)));
    ThumbnailWindow->setVerticalAlignment(CEGUI::VA_TOP);
    ThumbnailWindow->setHorizontalAlignment(CEGUI::HA_RIGHT);
    ThumbnailWindow->setVisible(false);

    sheet->addChild(ThumbnailWindow);

    PlaceNewElementB = (CEGUI::PushButton*)wmgr.createWindow("OgreTray/Button");
    PlaceNewElementB->setText("Place");
    PlaceNewElementB->setSize(CEGUI::USize(CEGUI::UDim(0, 75), CEGUI::UDim(0, 30)));
    PlaceNewElementB->setVerticalAlignment(CEGUI::VA_TOP);
    PlaceNewElementB->setHorizontalAlignment(CEGUI::HA_RIGHT);
    PlaceNewElementB->setVisible(false);

    PlaceNewElementB->subscribeEvent(CEGUI::PushButton::EventClicked,
			CEGUI::Event::Subscriber(&BallGame::PlaceNewElementBCallback, this));

    MainLayout->addChild(PlaceNewElementB);

    EditElementB = (CEGUI::PushButton*)wmgr.createWindow("OgreTray/Button");
    EditElementB->setText("Edit");
    EditElementB->setSize(CEGUI::USize(CEGUI::UDim(0, 75), CEGUI::UDim(0, 30)));
    EditElementB->setVerticalAlignment(CEGUI::VA_TOP);
    EditElementB->setHorizontalAlignment(CEGUI::HA_RIGHT);
    EditElementB->setVisible(false);

    EditElementB->subscribeEvent(CEGUI::PushButton::EventClicked,
			CEGUI::Event::Subscriber(&BallGame::EditElementBCallback, this));

    MainLayout->addChild(EditElementB);

    DeleteElementB = (CEGUI::PushButton*)wmgr.createWindow("OgreTray/Button");
    DeleteElementB->setText("Delete");
    DeleteElementB->setSize(CEGUI::USize(CEGUI::UDim(0, 150), CEGUI::UDim(0, 30)));
    DeleteElementB->setVerticalAlignment(CEGUI::VA_TOP);
    DeleteElementB->setHorizontalAlignment(CEGUI::HA_RIGHT);
    DeleteElementB->setVisible(false);

    DeleteElementB->subscribeEvent(CEGUI::PushButton::EventClicked,
			CEGUI::Event::Subscriber(&BallGame::DeleteElementBCallback, this));

    MainLayout->addChild(DeleteElementB);


    MoveElementB = (CEGUI::PushButton*)wmgr.createWindow("OgreTray/Button");
    MoveElementB->setText("M");
    MoveElementB->setTooltipText("Move Element. Press M to enter this mode.");
    MoveElementB->setSize(CEGUI::USize(CEGUI::UDim(0, 50), CEGUI::UDim(0, 30)));
    MoveElementB->setVerticalAlignment(CEGUI::VA_TOP);
    MoveElementB->setHorizontalAlignment(CEGUI::HA_RIGHT);
    MoveElementB->setVisible(false);

    MoveElementB->subscribeEvent(CEGUI::PushButton::EventClicked,
			CEGUI::Event::Subscriber(&BallGame::MoveElementBCallback, this));

    MainLayout->addChild(MoveElementB);


    RotateElementB = (CEGUI::PushButton*)wmgr.createWindow("OgreTray/Button");
    RotateElementB->setText("R");
    RotateElementB->setTooltipText("Rotate Element. Press R to enter this mode.");
    RotateElementB->setSize(CEGUI::USize(CEGUI::UDim(0, 50), CEGUI::UDim(0, 30)));
    RotateElementB->setVerticalAlignment(CEGUI::VA_TOP);
    RotateElementB->setHorizontalAlignment(CEGUI::HA_RIGHT);
    RotateElementB->setVisible(false);

    RotateElementB->subscribeEvent(CEGUI::PushButton::EventClicked,
			CEGUI::Event::Subscriber(&BallGame::RotateElementBCallback, this));

    MainLayout->addChild(RotateElementB);


    ScaleElementB = (CEGUI::PushButton*)wmgr.createWindow("OgreTray/Button");
    ScaleElementB->setText("S");
    ScaleElementB->setTooltipText("Scale Element. Press S to enter this mode.");
    ScaleElementB->setSize(CEGUI::USize(CEGUI::UDim(0, 50), CEGUI::UDim(0, 30)));
    ScaleElementB->setVerticalAlignment(CEGUI::VA_TOP);
    ScaleElementB->setHorizontalAlignment(CEGUI::HA_RIGHT);
    ScaleElementB->setVisible(false);

    ScaleElementB->subscribeEvent(CEGUI::PushButton::EventClicked,
			CEGUI::Event::Subscriber(&BallGame::ScaleElementBCallback, this));

    MainLayout->addChild(ScaleElementB);

    SetWindowsPosNearToOther(ChooseTypeOfElementToAddB, AddElementTitleBanner, 0, 1);
    SetWindowsPosNearToOther(ThumbnailWindow, AddElementTitleBanner, 0, 2);
    SetWindowsPosNearToOther(EditElementB, ThumbnailWindow, 0, 1);
    SetWindowsPosNearToOther(PlaceNewElementB, EditElementB, -1, 0);
    SetWindowsPosNearToOther(DeleteElementB, EditElementB, 0, 1);
    SetWindowsPosNearToOther(ScaleElementB, DeleteElementB, 0, 1);
    SetWindowsPosNearToOther(RotateElementB, ScaleElementB, -1, 0);
    SetWindowsPosNearToOther(MoveElementB, RotateElementB, -1, 0);


    // Edit Case GUI

    CaseHasForceToggleB = (CEGUI::ToggleButton*)wmgr.createWindow("OgreTray/Checkbox");
    CaseHasForceToggleB->setText("Has Force");
    CaseHasForceToggleB->setSelected(false);
    CaseHasForceToggleB->setSize(CEGUI::USize(CEGUI::UDim(0, 200), CEGUI::UDim(0, 30)));
    CaseHasForceToggleB->setVerticalAlignment(CEGUI::VA_BOTTOM);
    CaseHasForceToggleB->setHorizontalAlignment(CEGUI::HA_CENTRE);
    CaseHasForceToggleB->setVisible(false);

    CaseHasForceToggleB->subscribeEvent(CEGUI::ToggleButton::EventSelectStateChanged,
			CEGUI::Event::Subscriber(&BallGame::ToggleForceCallback, this));
    CaseHasForceToggleB->subscribeEvent(CEGUI::ToggleButton::EventMouseEntersArea,
			CEGUI::Event::Subscriber(&BallGame::EnteringArea, this));
    CaseHasForceToggleB->subscribeEvent(CEGUI::ToggleButton::EventMouseLeavesArea,
			CEGUI::Event::Subscriber(&BallGame::LeavingArea, this));

    MainLayout->addChild(CaseHasForceToggleB);

    CaseForceValueEditB = (CEGUI::Editbox*)wmgr.createWindow("OgreTray/Editbox");
    CaseForceValueEditB->setSize(CEGUI::USize(CEGUI::UDim(0, 50), CEGUI::UDim(0, 30)));
    CaseForceValueEditB->setVerticalAlignment(CEGUI::VA_BOTTOM);
    CaseForceValueEditB->setHorizontalAlignment(CEGUI::HA_CENTRE);
    String numRegex("^(\\-?[0-9]*(\\.[0-9]*)?)?");
    CaseForceValueEditB->setValidationString(numRegex);
    CaseForceValueEditB->setVisible(false);

    CaseForceValueEditB->subscribeEvent(CEGUI::Editbox::EventTextAccepted,
			CEGUI::Event::Subscriber(&BallGame::CaseForceValueEditBCallback, this));
    CaseForceValueEditB->subscribeEvent(CEGUI::Editbox::EventMouseLeavesArea,
			CEGUI::Event::Subscriber(&BallGame::CaseForceValueEditBCallback, this));
    CaseForceValueEditB->subscribeEvent(CEGUI::Editbox::EventMouseEntersArea,
			CEGUI::Event::Subscriber(&BallGame::EnteringArea, this));
    CaseForceValueEditB->subscribeEvent(CEGUI::Editbox::EventMouseLeavesArea,
			CEGUI::Event::Subscriber(&BallGame::LeavingArea, this));

    MainLayout->addChild(CaseForceValueEditB);

    CaseHasForceDirectionToggleB = (CEGUI::ToggleButton*)wmgr.createWindow("OgreTray/Checkbox");
    CaseHasForceDirectionToggleB->setText("Has Force Directed");
    CaseHasForceDirectionToggleB->setSelected(false);
    CaseHasForceDirectionToggleB->setSize(CEGUI::USize(CEGUI::UDim(0, 200), CEGUI::UDim(0, 30)));
    CaseHasForceDirectionToggleB->setVerticalAlignment(CEGUI::VA_BOTTOM);
    CaseHasForceDirectionToggleB->setHorizontalAlignment(CEGUI::HA_CENTRE);
    CaseHasForceDirectionToggleB->setVisible(false);

	CaseHasForceDirectionToggleB->subscribeEvent(CEGUI::ToggleButton::EventSelectStateChanged,
			CEGUI::Event::Subscriber(&BallGame::ToggleForceDirectedCallback, this));
	CaseHasForceDirectionToggleB->subscribeEvent(CEGUI::ToggleButton::EventMouseEntersArea,
			CEGUI::Event::Subscriber(&BallGame::EnteringArea, this));
	CaseHasForceDirectionToggleB->subscribeEvent(CEGUI::ToggleButton::EventMouseLeavesArea,
			CEGUI::Event::Subscriber(&BallGame::LeavingArea, this));

    MainLayout->addChild(CaseHasForceDirectionToggleB);

    CaseForceDirectionXValueEditB = (CEGUI::Editbox*)wmgr.createWindow("OgreTray/Editbox");
    CaseForceDirectionXValueEditB->setSize(CEGUI::USize(CEGUI::UDim(0, 50), CEGUI::UDim(0, 30)));
    CaseForceDirectionXValueEditB->setVerticalAlignment(CEGUI::VA_BOTTOM);
    CaseForceDirectionXValueEditB->setHorizontalAlignment(CEGUI::HA_CENTRE);
    CaseForceDirectionXValueEditB->setValidationString(numRegex);
    CaseForceDirectionXValueEditB->setVisible(false);

    CaseForceDirectionXValueEditB->subscribeEvent(CEGUI::Editbox::EventMouseEntersArea,
			CEGUI::Event::Subscriber(&BallGame::EnteringArea, this));
    CaseForceDirectionXValueEditB->subscribeEvent(CEGUI::Editbox::EventMouseLeavesArea,
			CEGUI::Event::Subscriber(&BallGame::LeavingArea, this));

    MainLayout->addChild(CaseForceDirectionXValueEditB);

    CaseForceDirectionYValueEditB = (CEGUI::Editbox*)wmgr.createWindow("OgreTray/Editbox");
    CaseForceDirectionYValueEditB->setSize(CEGUI::USize(CEGUI::UDim(0, 50), CEGUI::UDim(0, 30)));
    CaseForceDirectionYValueEditB->setVerticalAlignment(CEGUI::VA_BOTTOM);
    CaseForceDirectionYValueEditB->setHorizontalAlignment(CEGUI::HA_CENTRE);
    CaseForceDirectionYValueEditB->setValidationString(numRegex);
    CaseForceDirectionYValueEditB->setVisible(false);

    CaseForceDirectionYValueEditB->subscribeEvent(CEGUI::Editbox::EventMouseEntersArea,
			CEGUI::Event::Subscriber(&BallGame::EnteringArea, this));
    CaseForceDirectionYValueEditB->subscribeEvent(CEGUI::Editbox::EventMouseLeavesArea,
			CEGUI::Event::Subscriber(&BallGame::LeavingArea, this));

    MainLayout->addChild(CaseForceDirectionYValueEditB);

    CaseForceDirectionZValueEditB = (CEGUI::Editbox*)wmgr.createWindow("OgreTray/Editbox");
    CaseForceDirectionZValueEditB->setSize(CEGUI::USize(CEGUI::UDim(0, 50), CEGUI::UDim(0, 30)));
    CaseForceDirectionZValueEditB->setVerticalAlignment(CEGUI::VA_BOTTOM);
    CaseForceDirectionZValueEditB->setHorizontalAlignment(CEGUI::HA_CENTRE);
    CaseForceDirectionZValueEditB->setValidationString(numRegex);
    CaseForceDirectionZValueEditB->setVisible(false);

    CaseForceDirectionZValueEditB->subscribeEvent(CEGUI::Editbox::EventMouseEntersArea,
			CEGUI::Event::Subscriber(&BallGame::EnteringArea, this));
    CaseForceDirectionZValueEditB->subscribeEvent(CEGUI::Editbox::EventMouseLeavesArea,
			CEGUI::Event::Subscriber(&BallGame::LeavingArea, this));

    MainLayout->addChild(CaseForceDirectionZValueEditB);

    NormalizeCaseForceDirectionPushB = (CEGUI::PushButton*)wmgr.createWindow("OgreTray/Button");
    NormalizeCaseForceDirectionPushB->setText("Norm");
    NormalizeCaseForceDirectionPushB->setSize(CEGUI::USize(CEGUI::UDim(0, 50), CEGUI::UDim(0, 30)));
    NormalizeCaseForceDirectionPushB->setVerticalAlignment(CEGUI::VA_BOTTOM);
    NormalizeCaseForceDirectionPushB->setHorizontalAlignment(CEGUI::HA_CENTRE);
    NormalizeCaseForceDirectionPushB->setVisible(false);
    NormalizeCaseForceDirectionPushB->subscribeEvent(CEGUI::PushButton::EventClicked,
    		CEGUI::Event::Subscriber(&BallGame::NormalizeCaseForceDirectionPushBCallback, this));

    NormalizeCaseForceDirectionPushB->subscribeEvent(CEGUI::PushButton::EventMouseEntersArea,
			CEGUI::Event::Subscriber(&BallGame::EnteringArea, this));
    NormalizeCaseForceDirectionPushB->subscribeEvent(CEGUI::PushButton::EventMouseLeavesArea,
			CEGUI::Event::Subscriber(&BallGame::LeavingArea, this));

    MainLayout->addChild(NormalizeCaseForceDirectionPushB);

    ApplyForceChangesToCasePushB = (CEGUI::PushButton*)wmgr.createWindow("OgreTray/Button");
    ApplyForceChangesToCasePushB->setText("Apply");
    ApplyForceChangesToCasePushB->setSize(CEGUI::USize(CEGUI::UDim(0, 100), CEGUI::UDim(0, 30)));
    ApplyForceChangesToCasePushB->setVerticalAlignment(CEGUI::VA_BOTTOM);
    ApplyForceChangesToCasePushB->setHorizontalAlignment(CEGUI::HA_CENTRE);
    ApplyForceChangesToCasePushB->setVisible(false);

    ApplyForceChangesToCasePushB->subscribeEvent(CEGUI::PushButton::EventClicked,
    		CEGUI::Event::Subscriber(&BallGame::ApplyForceChangesToCasePushBCallback, this));
    ApplyForceChangesToCasePushB->subscribeEvent(CEGUI::PushButton::EventMouseEntersArea,
			CEGUI::Event::Subscriber(&BallGame::EnteringArea, this));
    ApplyForceChangesToCasePushB->subscribeEvent(CEGUI::PushButton::EventMouseLeavesArea,
			CEGUI::Event::Subscriber(&BallGame::LeavingArea, this));

    MainLayout->addChild(ApplyForceChangesToCasePushB);

    SetWindowsPosNearToOther(CaseHasForceDirectionToggleB, ApplyForceChangesToCasePushB, 0, -1);
    SetWindowsPosNearToOther(CaseHasForceToggleB, CaseHasForceDirectionToggleB, 0, -1);
    SetWindowsPosNearToOther(CaseForceValueEditB, CaseHasForceToggleB, 1, 0);
    SetWindowsPosNearToOther(CaseForceDirectionXValueEditB, CaseHasForceDirectionToggleB, 1, 0);
    SetWindowsPosNearToOther(CaseForceDirectionYValueEditB, CaseForceDirectionXValueEditB, 1, 0);
    SetWindowsPosNearToOther(CaseForceDirectionZValueEditB, CaseForceDirectionYValueEditB, 1, 0);
    SetWindowsPosNearToOther(NormalizeCaseForceDirectionPushB, CaseForceDirectionZValueEditB, 1, 0);

    // Edit Ball GUI

    BallMassValueEditB = (CEGUI::Editbox*)wmgr.createWindow("OgreTray/Editbox");
    BallMassValueEditB->setSize(CEGUI::USize(CEGUI::UDim(0, 50), CEGUI::UDim(0, 30)));
    BallMassValueEditB->setVerticalAlignment(CEGUI::VA_BOTTOM);
    BallMassValueEditB->setHorizontalAlignment(CEGUI::HA_CENTRE);
    BallMassValueEditB->setVisible(false);

    MainLayout->addChild(BallMassValueEditB);

    ApplyMassChangesToBallPushB = (CEGUI::PushButton*)wmgr.createWindow("OgreTray/Button");
    ApplyMassChangesToBallPushB->setText("Apply");
    ApplyMassChangesToBallPushB->setSize(CEGUI::USize(CEGUI::UDim(0, 100), CEGUI::UDim(0, 30)));
    ApplyMassChangesToBallPushB->setVerticalAlignment(CEGUI::VA_BOTTOM);
    ApplyMassChangesToBallPushB->setHorizontalAlignment(CEGUI::HA_CENTRE);
    ApplyMassChangesToBallPushB->setVisible(false);
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
    SceneNode* lightNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
    lightNode->attachObject(light);
    lightNode->setPosition(20, 80, 50);

	mThumbnailSceneMgr->setAmbientLight(ColourValue(0.5, 0.5, 0.5));

    light = mThumbnailSceneMgr->createLight("MainThumbnailLight");
    lightNode = mThumbnailSceneMgr->getRootSceneNode()->createChildSceneNode();
    lightNode->attachObject(light);
    lightNode->setPosition(20, 80, 50);

    SetCam(-184, -253, 352);
    mThumbnailCamera->setPosition(-184, -253, 352);
    mCamera->setOrientation(Ogre::Quaternion(0.835422, 0.393051, -0.238709, -0.300998));
    mThumbnailCamera->setOrientation(Ogre::Quaternion(0.835422, 0.393051, -0.238709, -0.300998));

    CEGUI::Texture &guiTex = mRenderer->createTexture("textname", tex);

    const CEGUI::Rectf rect(CEGUI::Vector2f(0.0f, 0.0f), guiTex.getOriginalDataSize());
    CEGUI::BasicImage* image = (CEGUI::BasicImage*)( &CEGUI::ImageManager::getSingleton().create("BasicImage", "foobar"));
       image->setTexture(&guiTex);
       image->setArea(rect);
       image->setAutoScaled(CEGUI::ASM_Both);

   ThumbnailWindow->setProperty("Image", "foobar");

    ChangeLevel();

    _StartPhysic();
//    _StopPhysic();
}

void BallGame::CheckforCollides(void)
{
	std::list<BallEntity*>::iterator Biter(Balls.begin());
	while(Biter != Balls.end())
	{
		BallEntity *ball = *(Biter++);
		if(ball == NULL)
			continue;
		std::list<CaseEntity*>::iterator Citer(Cases.begin());
		while(Citer != Cases.end())
		{
			CaseEntity *Case = *(Citer++);
			if(Case == NULL)
				continue;
//          if(NewtonBodyFindContact(ball, Case) != NULL)
//			if(DoBodiesCollide(m_world, ball->Body, Case->Body))
			if(CheckIfBodiesCollide(ball->Body, Case->Body) != NULL)
			{
				int idb, idc;
				idb = NewtonBodyGetID(ball->Body);
				idc = NewtonBodyGetID(Case->Body);
				Case->ApplyForceOnBall(ball);

				std::cout << ball << " id " << idb << " and " << Case << " id " << idc << " Collides" << std::endl;
			}
		}
	}
}

void BallGame::AddBall(BallEntity *ball)
{
	if(ball == NULL)
		return;
	Balls.push_back(ball);
}

void BallGame::AddCase(CaseEntity *Wcase)
{
	if(Wcase == NULL)
		return;
	Cases.push_back(Wcase);
}


bool BallGame::frameEnded(const Ogre::FrameEvent& fe)
{
//    std::cout << "Render a frame" << std::endl;
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
			Vector3 worldPos = ball->OgreEntity->getPosition();
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
    //BaseApplication::mouseMoved(arg);

	Real x, y;
	CEGUI::Vector2f mpos = CEGUI::System::getSingleton().getDefaultGUIContext().getMouseCursor().getPosition();
	// Will Mouse has not hit top left corner, there is a gap between OIS and CEGUI mouse coordinates. CEGUI is more reliable
	x = mpos.d_x / (float)mWindow->getWidth();
	y = mpos.d_y / (float)mWindow->getHeight();

	if(arg.state.Z.rel != 0)
		MoveCam(0,  0, -10 * arg.state.Z.rel);

	if(mode == Editing)
	{
		if(LastHighligted != NULL)
		{
			if((UnderEditCase == NULL || UnderEditCase->OgreEntity != LastHighligted)
					&& (UnderEditBall == NULL || UnderEditBall->OgreEntity != LastHighligted)
					&& (ToBePlacedEntity == NULL || ToBePlacedEntity->OgreEntity != LastHighligted))
				LastHighligted->showBoundingBox(false);
			LastHighligted = NULL;
		}
		RaySceneQuery *mRayScanQuery = mSceneMgr->createRayQuery(Ogre::Ray());


//		std::cout << "Coords Abs {" << arg.state.X.abs << ", " << arg.state.Y.abs << "}" << std::endl;
//		std::cout << "Coords CEGUI {" << mpos.d_x << ", " << mpos.d_y << "}" << std::endl;
//		std::cout << "Edit mode, try to pick {" << x << ", " << y << "}" << std::endl;
		Ray mouseRay = mCamera->getCameraToViewportRay(x, y);
		mRayScanQuery->setRay(mouseRay);
		mRayScanQuery->setSortByDistance(true, 1);
		RaySceneQueryResult &result = mRayScanQuery->execute();
		RaySceneQueryResult::iterator itr = result.begin();
//		std::cout << "Picking Mouse : " << result.size() << std::endl;
		while(itr != result.end())
		{
			if(itr->movable != NULL)
			{
				LastHighligted = itr->movable->getParentSceneNode();
				LastHighligted->showBoundingBox(true);
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
		UnderEditCaseForce = strtof(CaseForceValueEditB->getText().c_str(), NULL);
		if(force_directed == true)
		{
			float dir = strtof(CaseForceDirectionXValueEditB->getText().c_str(), NULL);
			force_direction.m_x = dir;
			dir = strtof(CaseForceDirectionYValueEditB->getText().c_str(), NULL);
			force_direction.m_y = dir;
			dir = strtof(CaseForceDirectionZValueEditB->getText().c_str(), NULL);
			force_direction.m_z = dir;
			force_dir = new dVector(force_direction.m_x, force_direction.m_y, force_direction.m_z);
		}
	}
	UnderEditCase->SetForceToApply(UnderEditCaseForce, force_dir);

	return true;
}

bool BallGame::ToggleForceCallback(const CEGUI::EventArgs &event)
{
	const CEGUI::WindowEventArgs &e = (const CEGUI::WindowEventArgs &)event;
	std::cout << "Update buttons by CE Callback of " << e.window->getName() << std::endl;
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

bool BallGame::ToggleForceDirectedCallback(const CEGUI::EventArgs &event)
{
	const CEGUI::WindowEventArgs &e = (const CEGUI::WindowEventArgs &)event;
	std::cout << "Update buttons by CE Callback of " << e.window->getName() << std::endl;
	CaseHasForceToggleB->setMutedState(true);
	CaseHasForceDirectionToggleB->setMutedState(true);
	if(CaseHasForceDirectionToggleB->isSelected())
		force_directed = true;
	else
		force_directed = false;
	UpdateEditButtons();
	CaseHasForceDirectionToggleB->setMutedState(false);
	CaseHasForceToggleB->setMutedState(false);
    return true;
}

void BallGame::UpdateEditButtons(void)
{
	std::cout << "Update Edit buttons"<< std::endl;
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
		std::cout << "No Base Force"<< std::endl;
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
		std::cout << "Base Force is present"<< std::endl;
		CaseHasForceToggleB->setSelected(true);
		CaseHasForceDirectionToggleB->setDisabled(false);
		CaseForceValueEditBSetText(UnderEditCaseForce);
		CaseForceValueEditB->setDisabled(false);

		if(force_directed == false)
		{
			std::cout << "Not Directed Force"<< std::endl;
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
			std::cout << "Directed Force"<< std::endl;
			CaseHasForceDirectionToggleB->setSelected(true);
			CaseForceDirectionXValueEditB->setDisabled(false);
			CaseForceDirectionYValueEditB->setDisabled(false);
			CaseForceDirectionZValueEditB->setDisabled(false);
			CaseForceDirectionXValueEditBSetText(force_direction.m_x);
			CaseForceDirectionYValueEditBSetText(force_direction.m_y);
			CaseForceDirectionZValueEditBSetText(force_direction.m_z);
			NormalizeCaseForceDirectionPushB->setDisabled(false);
		}
	}
}

void BallGame::EditCase(CaseEntity *Entity)
{
	if(mode != Editing)
		return;
	if(LastHighligted == NULL)
		return;
	if(UnderEditCase != NULL)
		UnderEditCase->OgreEntity->showBoundingBox(false);
	UnderEditCase = Entity;

	if(UnderEditCase != NULL)
	{
		CaseHasForceToggleB->setMutedState(true);
		CaseHasForceDirectionToggleB->setMutedState(true);

		UnderEditCase->OgreEntity->showBoundingBox(true);
		UnderEditCaseForce = UnderEditCase->force_to_apply;
		if(isnan(UnderEditCaseForce))
		{
			std::cout << "EditCase Force not present" << std::endl;
			CaseHasForce = false;
		}
		else
		{
			std::cout << "EditCase Force present" << std::endl;
			CaseHasForce = true;
		}
		if(UnderEditCase->force_direction == NULL)
		{
			std::cout << "EditCase Force not directed" << std::endl;
			force_directed = false;
		}
		else
		{
			std::cout << "EditCase Force directed" << std::endl;
			force_directed = true;
			force_direction.m_x = UnderEditCase->force_direction->m_x;
			force_direction.m_y = UnderEditCase->force_direction->m_y;
			force_direction.m_z = UnderEditCase->force_direction->m_z;
		}
		std::cout << "Update buttons by Mouse Pressed" << std::endl;
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
	UnderEditBallMass = strtof(BallMassValueEditB->getText().c_str(), NULL);
	std::cout << "Ball new mass : " << UnderEditBallMass << std::endl;
	NewtonBody *Nball = UnderEditBall->Body;
	NewtonCollision *collision = NewtonBodyGetCollision(Nball);
	NewtonBodySetMassProperties(Nball, UnderEditBallMass, collision);
	return true;
}

void BallGame::EditBall(BallEntity *Entity)
{
	if(mode != Editing)
		return;
	if(LastHighligted == NULL)
		return;
	if(UnderEditBall != NULL)
		UnderEditBall->OgreEntity->showBoundingBox(false);
	UnderEditBall = Entity;

	if(UnderEditBall != NULL)
	{
		UnderEditBall->OgreEntity->showBoundingBox(true);
		NewtonBody *Nball = UnderEditBall->Body;
		dFloat inertx, inerty, inertz;
		NewtonBodyGetMass(Nball, &UnderEditBallMass, &inertx, &inerty, &inertz);
		BallMassValueEditB->setVisible(true);
		BallMassValueEditB->setDisabled(false);
		char mass_c[20];
		snprintf(mass_c, 19, "%f", UnderEditBallMass);
		BallMassValueEditB->setText(mass_c);
		ApplyMassChangesToBallPushB->setVisible(true);
		ApplyMassChangesToBallPushB->setDisabled(false);
	}
	else
	{
		BallMassValueEditB->setVisible(false);
		ApplyMassChangesToBallPushB->setVisible(false);
	}
}

bool BallGame::mousePressed( const OIS::MouseEvent &arg, OIS::MouseButtonID id )
{
    CEGUI::GUIContext& context = CEGUI::System::getSingleton().getDefaultGUIContext();
    context.injectMouseButtonDown(convertButton(id));

    //BaseApplication::mousePressed(arg, id);
    if(mode == Editing && MouseOverButton == false)
    {
    	if(LastHighligted != NULL && (ToBePlacedEntity == NULL || LastHighligted != ToBePlacedEntity->OgreEntity))
    	{
			//Case Entity ?
    		std::cout << "Edit by Mouse Pressed" << std::endl;
    		BallGameEntity *Entity = Ogre::any_cast<BallGameEntity*>(((Ogre::Entity*)LastHighligted->getAttachedObject(0))->getUserObjectBindings().getUserAny());
    		switch(Entity->type)
    		{
    		case Case :
					std::cout << "Edit Case by Mouse Pressed" << std::endl;
					EditBall(NULL);//Hide Ball Editing buttons;
					EditCase((CaseEntity*)Entity);
					break;
    		case Ball :
					std::cout << "Edit Ball by Mouse Pressed" << std::endl;
					EditCase(NULL);//Hide Case Editing buttons;
					EditBall((BallEntity*)Entity);
					break;
    		}
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

bool BallGame::keyPressed(const OIS::KeyEvent &arg)
{
	std::cout << "Key pressed " << arg.key << std::endl;

    CEGUI::GUIContext& context = CEGUI::System::getSingleton().getDefaultGUIContext();
    context.injectKeyDown((CEGUI::Key::Scan)arg.key);
    context.injectChar(arg.text);

	//BaseApplication::keyPressed(arg);
    switch (arg.key)
    {
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
		switch(PlacementMode)
		{
		case PlaceMove :
			if(ToBePlacedEntity != NULL)
			{
				Vector3 pos = ToBePlacedEntity->OgreEntity->getPosition();
				pos.y += 10;
				ToBePlacedEntity->OgreEntity->setPosition(pos);
			}
			break;
		case EditMove :
			{
				BallGameEntity *Entity = UnderEditBall == NULL ? (BallGameEntity*)UnderEditCase : (BallGameEntity*)UnderEditBall;
				if(Entity != NULL)
				{
					Vector3 pos = Entity->OgreEntity->getPosition();
					pos.y += 10;
					Entity->OgreEntity->setPosition(pos);
				}
			}
			break;
		case PlaceRotate :
			if(ToBePlacedEntity != NULL)
				ToBePlacedEntity->OgreEntity->yaw(Degree(10));
			break;
		case EditRotate :
			{
				BallGameEntity *Entity = UnderEditBall == NULL ? (BallGameEntity*)UnderEditCase : (BallGameEntity*)UnderEditBall;
				if(Entity != NULL)
					Entity->OgreEntity->yaw(Degree(10));
			}
			break;
		case PlaceScale :
			if(ToBePlacedEntity != NULL)
			{
				Vector3 pos = ToBePlacedEntity->OgreEntity->getScale();
				pos.y += 10;
				ToBePlacedEntity->OgreEntity->setScale(pos);
			}
			break;
		case EditScale :
			{
				BallGameEntity *Entity = UnderEditBall == NULL ? (BallGameEntity*)UnderEditCase : (BallGameEntity*)UnderEditBall;
				if(Entity != NULL)
				{
					Vector3 pos = Entity->OgreEntity->getScale();
					pos.y += 10;
					Entity->OgreEntity->setScale(pos);
				}
			}
			break;
		}
	    break;
	case OIS::KeyCode::KC_DOWN:
		switch(PlacementMode)
		{
		case PlaceMove :
			if(ToBePlacedEntity != NULL)
			{
				Vector3 pos = ToBePlacedEntity->OgreEntity->getPosition();
				pos.y -= 10;
				ToBePlacedEntity->OgreEntity->setPosition(pos);
			}
			break;
		case EditMove :
			{
				BallGameEntity *Entity = UnderEditBall == NULL ? (BallGameEntity*)UnderEditCase : (BallGameEntity*)UnderEditBall;
				if(Entity != NULL)
				{
					Vector3 pos = Entity->OgreEntity->getPosition();
					pos.y -= 10;
					Entity->OgreEntity->setPosition(pos);
				}
			}
			break;
		case PlaceRotate :
			if(ToBePlacedEntity != NULL)
				ToBePlacedEntity->OgreEntity->yaw(Degree(-10));
			break;
		case EditRotate :
			{
				BallGameEntity *Entity = UnderEditBall == NULL ? (BallGameEntity*)UnderEditCase : (BallGameEntity*)UnderEditBall;
				if(Entity != NULL)
					Entity->OgreEntity->yaw(Degree(-10));
			}
			break;
		case PlaceScale :
			if(ToBePlacedEntity != NULL)
			{
				Vector3 pos = ToBePlacedEntity->OgreEntity->getScale();
				pos.y -= 10;
				ToBePlacedEntity->OgreEntity->setScale(pos);
			}
			break;
		case EditScale :
			{
				BallGameEntity *Entity = UnderEditBall == NULL ? (BallGameEntity*)UnderEditCase : (BallGameEntity*)UnderEditBall;
				if(Entity != NULL)
				{
					Vector3 pos = Entity->OgreEntity->getScale();
					pos.y -= 10;
					Entity->OgreEntity->setScale(pos);
				}
			}
			break;
		}
	    break;
	case OIS::KeyCode::KC_LEFT:
		switch(PlacementMode)
		{
		case PlaceMove :
			if(ToBePlacedEntity != NULL)
			{
				Vector3 pos = ToBePlacedEntity->OgreEntity->getPosition();
				pos.x -= 10;
				ToBePlacedEntity->OgreEntity->setPosition(pos);
			}
			break;
		case EditMove :
			{
				BallGameEntity *Entity = UnderEditBall == NULL ? (BallGameEntity*)UnderEditCase : (BallGameEntity*)UnderEditBall;
				if(Entity != NULL)
				{
					Vector3 pos = Entity->OgreEntity->getPosition();
					pos.x -= 10;
					Entity->OgreEntity->setPosition(pos);
				}
			}
			break;
		case PlaceRotate :
			if(ToBePlacedEntity != NULL)
				ToBePlacedEntity->OgreEntity->pitch(Degree(10));
			break;
		case EditRotate :
			{
				BallGameEntity *Entity = UnderEditBall == NULL ? (BallGameEntity*)UnderEditCase : (BallGameEntity*)UnderEditBall;
				if(Entity != NULL)
					Entity->OgreEntity->pitch(Degree(10));
			}
			break;
		case PlaceScale :
			if(ToBePlacedEntity != NULL)
			{
				Vector3 pos = ToBePlacedEntity->OgreEntity->getScale();
				pos.x -= 10;
				ToBePlacedEntity->OgreEntity->setScale(pos);
			}
			break;
		case EditScale :
			{
				BallGameEntity *Entity = UnderEditBall == NULL ? (BallGameEntity*)UnderEditCase : (BallGameEntity*)UnderEditBall;
				if(Entity != NULL)
				{
					Vector3 pos = Entity->OgreEntity->getScale();
					pos.x -= 10;
					Entity->OgreEntity->setScale(pos);
				}
			}
			break;
		}
	    break;
	case OIS::KeyCode::KC_RIGHT:
		switch(PlacementMode)
		{
		case PlaceMove :
			if(ToBePlacedEntity != NULL)
			{
				Vector3 pos = ToBePlacedEntity->OgreEntity->getPosition();
				pos.x += 10;
				ToBePlacedEntity->OgreEntity->setPosition(pos);
			}
			break;
		case EditMove :
			{
				BallGameEntity *Entity = UnderEditBall == NULL ? (BallGameEntity*)UnderEditCase : (BallGameEntity*)UnderEditBall;
				if(Entity != NULL)
				{
					Vector3 pos = Entity->OgreEntity->getPosition();
					pos.x += 10;
					Entity->OgreEntity->setPosition(pos);
				}
			}
			break;
		case PlaceRotate :
			if(ToBePlacedEntity != NULL)
				ToBePlacedEntity->OgreEntity->pitch(Degree(-10));
			break;
		case EditRotate :
			{
				BallGameEntity *Entity = UnderEditBall == NULL ? (BallGameEntity*)UnderEditCase : (BallGameEntity*)UnderEditBall;
				if(Entity != NULL)
					Entity->OgreEntity->pitch(Degree(-10));
			}
			break;
		case PlaceScale :
			if(ToBePlacedEntity != NULL)
			{
				Vector3 pos = ToBePlacedEntity->OgreEntity->getScale();
				pos.x += 10;
				ToBePlacedEntity->OgreEntity->setScale(pos);
			}
			break;
		case EditScale :
			{
				BallGameEntity *Entity = UnderEditBall == NULL ? (BallGameEntity*)UnderEditCase : (BallGameEntity*)UnderEditBall;
				if(Entity != NULL)
				{
					Vector3 pos = Entity->OgreEntity->getScale();
					pos.x += 10;
					Entity->OgreEntity->setScale(pos);
				}
			}
			break;
		}
	    break;
	case OIS::KeyCode::KC_PGUP:
		switch(PlacementMode)
		{
		case PlaceMove :
			if(ToBePlacedEntity != NULL)
			{
				Vector3 pos = ToBePlacedEntity->OgreEntity->getPosition();
				pos.z += 10;
				ToBePlacedEntity->OgreEntity->setPosition(pos);
			}
			break;
		case EditMove :
			{
				BallGameEntity *Entity = UnderEditBall == NULL ? (BallGameEntity*)UnderEditCase : (BallGameEntity*)UnderEditBall;
				if(Entity != NULL)
				{
					Vector3 pos = Entity->OgreEntity->getPosition();
					pos.z += 10;
					Entity->OgreEntity->setPosition(pos);
				}
			}
			break;
		case PlaceRotate :
			if(ToBePlacedEntity != NULL)
				ToBePlacedEntity->OgreEntity->roll(Degree(10));
			break;
		case EditRotate :
			{
				BallGameEntity *Entity = UnderEditBall == NULL ? (BallGameEntity*)UnderEditCase : (BallGameEntity*)UnderEditBall;
				if(Entity != NULL)
					Entity->OgreEntity->roll(Degree(10));
			}
			break;
		case PlaceScale :
			if(ToBePlacedEntity != NULL)
			{
				Vector3 pos = ToBePlacedEntity->OgreEntity->getScale();
				pos.z += 10;
				ToBePlacedEntity->OgreEntity->setScale(pos);
			}
			break;
		case EditScale :
			{
				BallGameEntity *Entity = UnderEditBall == NULL ? (BallGameEntity*)UnderEditCase : (BallGameEntity*)UnderEditBall;
				if(Entity != NULL)
				{
					Vector3 pos = Entity->OgreEntity->getScale();
					pos.z += 10;
					Entity->OgreEntity->setScale(pos);
				}
			}
			break;
		}
	    break;
	case OIS::KeyCode::KC_PGDOWN:
		switch(PlacementMode)
		{
		case PlaceMove :
			if(ToBePlacedEntity != NULL)
			{
				Vector3 pos = ToBePlacedEntity->OgreEntity->getPosition();
				pos.z -= 10;
				ToBePlacedEntity->OgreEntity->setPosition(pos);
			}
			break;
		case EditMove :
			{
				BallGameEntity *Entity = UnderEditBall == NULL ? (BallGameEntity*)UnderEditCase : (BallGameEntity*)UnderEditBall;
				if(Entity != NULL)
				{
					Vector3 pos = Entity->OgreEntity->getPosition();
					pos.z -= 10;
					Entity->OgreEntity->setPosition(pos);
				}
			}
			break;
		case PlaceRotate :
			if(ToBePlacedEntity != NULL)
				ToBePlacedEntity->OgreEntity->roll(Degree(-10));
			break;
		case EditRotate :
			{
				BallGameEntity *Entity = UnderEditBall == NULL ? (BallGameEntity*)UnderEditCase : (BallGameEntity*)UnderEditBall;
				if(Entity != NULL)
					Entity->OgreEntity->roll(Degree(-10));
			}
			break;
		case PlaceScale :
			if(ToBePlacedEntity != NULL)
			{
				Vector3 pos = ToBePlacedEntity->OgreEntity->getScale();
				pos.z -= 10;
				ToBePlacedEntity->OgreEntity->setScale(pos);
			}
			break;
		case EditScale :
			{
				BallGameEntity *Entity = UnderEditBall == NULL ? (BallGameEntity*)UnderEditCase : (BallGameEntity*)UnderEditBall;
				if(Entity != NULL)
				{
					Vector3 pos = Entity->OgreEntity->getScale();
					pos.z -= 10;
					Entity->OgreEntity->setScale(pos);
				}
			}
			break;
		}
	    break;
	case OIS::KeyCode::KC_SPACE:
		if(mode == Editing)
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
		if(mode == Editing)
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
	//BaseApplication::keyReleased(arg);
    return true;
}

bool BallGame::NewLevelCreateBCallback(const CEGUI::EventArgs &e)
{
	EmptyLevel();
	String level;
	level = NewLevelEditB->getText().c_str();
	SetLevel(level);
	if(mode == Running)
		SwitchEditMode();
	return true;
}

bool BallGame::SaveLevelPushBCallback(const CEGUI::EventArgs &e)
{
	String export_str;
	ExportLevelIntoJson(export_str);
	std::ofstream myfile;
	String Filename("Levels/");
	Filename += Level;
	Filename += ".json";
	myfile.open (Filename.c_str());
	myfile << export_str;
	myfile.close();
	return true;
}

bool BallGame::ChooseLevelComboBCallback(const CEGUI::EventArgs &e)
{
	String level;
	level = ChooseLevelComboB->getSelectedItem()->getText().c_str();
	SetLevel(level);
	ChangeLevel();
	return true;
}

void BallGame::SetLevel(String &level_name)
{
	Level = level_name;
	LevelNameBanner->setText(Level);
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

void CaseEntity::CreateFromJson(rapidjson::Value &v, Ogre::SceneManager* mSceneMgr, NewtonWorld *m_world, Node *parent)
{
	ImportFromJson(v, mSceneMgr, parent);
	dVector NewtonBodyLocation;
	dVector NewtonBodySize;
	NewtonBody *newtonBody;
	NewtonBodyLocation.m_x = InitialPos.x;
	NewtonBodyLocation.m_y = InitialPos.y;
	NewtonBodyLocation.m_z = InitialPos.z;
	NewtonBodyLocation.m_w = 1;
	Entity* ogreEntity = (Ogre::Entity*)OgreEntity->getAttachedObject(0);
	Vector3 AABB(ogreEntity->getBoundingBox().getSize());
	NewtonBodySize.m_x = AABB.x * InitialScale.x;
	NewtonBodySize.m_y = AABB.y * InitialScale.y;
	NewtonBodySize.m_z = AABB.z * InitialScale.z;
	NewtonBodySize.m_w = 0.0f;

	dMatrix casematrix(InitialOrientation.getPitch(false).valueRadians(), InitialOrientation.getYaw(false).valueRadians(), InitialOrientation.getRoll(false).valueRadians(), NewtonBodyLocation);
	NewtonCollision *collision_tree = NULL;

	Matrix4 ogre_matrix;
	ogre_matrix.makeTransform(Vector3::ZERO, InitialScale, Quaternion::IDENTITY);
	const MeshPtr ptr = ogreEntity->getMesh();
	collision_tree = ParseEntity(m_world, ptr, ogre_matrix);

	newtonBody = WorldAddCase(m_world, NewtonBodySize, 0, casematrix, collision_tree);

	SetNewtonBody(newtonBody);
}

void CaseEntity::ImportFromJson(rapidjson::Value &v, Ogre::SceneManager* mSceneMgr, Node *parent)
{
	BallGameEntity::ImportFromJson(v, mSceneMgr, parent);
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

void BallEntity::CreateFromJson(rapidjson::Value &v, Ogre::SceneManager* mSceneMgr, NewtonWorld *m_world, Node *parent)
{
	ImportFromJson(v, mSceneMgr, parent);
	dVector NewtonBodyLocation;
	dVector NewtonBodySize;
	NewtonBody *BallBody;
	NewtonBodyLocation.m_x = InitialPos.x;
	NewtonBodyLocation.m_y = InitialPos.y;
	NewtonBodyLocation.m_z = InitialPos.z;
	NewtonBodyLocation.m_w = 1;
	Entity* ogreEntity = (Ogre::Entity*)OgreEntity->getAttachedObject(0);
	Vector3 AABB(ogreEntity->getBoundingBox().getSize());
	NewtonBodySize.m_x = AABB.x * InitialScale.x;
	NewtonBodySize.m_y = AABB.y * InitialScale.y;
	NewtonBodySize.m_z = AABB.z * InitialScale.z;
	NewtonBodySize.m_w = 0.0f;
	dMatrix ballmatrix(InitialOrientation.getPitch(false).valueRadians(), InitialOrientation.getYaw(false).valueRadians(), InitialOrientation.getRoll(false).valueRadians(), NewtonBodyLocation);
	BallBody = WorldAddBall(m_world, InitialMass, NewtonBodySize, 0, ballmatrix);

	SetNewtonBody(BallBody);
}

void BallEntity::ImportFromJson(rapidjson::Value &v, Ogre::SceneManager* mSceneMgr, Node *parent)
{
	BallGameEntity::ImportFromJson(v, mSceneMgr, parent);
	InitialMass = v["Mass"].GetFloat();
}

void BallGameEntity::ImportFromJson(rapidjson::Value &v, Ogre::SceneManager* mSceneMgr, Node *parent)
{
	const char *meshname = v["Mesh"].GetString();
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
	if(parent == NULL)
		ogreNode = mSceneMgr->getRootSceneNode()->createChildSceneNode(InitialPos);
	else
		ogreNode = (SceneNode*)parent->createChild(InitialPos);
	ogreNode->attachObject(ogreEntity);
	SetOgreNode(ogreNode);
}

void BallGame::DeleteBall(BallEntity *Entity, std::list<BallEntity*>::iterator *iter)
{
	RemoveBall(Entity, iter);
	Entity->Finalize(mSceneMgr);
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
	Entity->Finalize(mSceneMgr);
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
			RemoveCase(Case, &Cit);
		else
			Cit++;
	}
	std::list<BallEntity*>::iterator Bit(Balls.begin());
	while(Bit != Balls.end())
	{
		BallEntity *Ball = *Bit;
		if(Ball != NULL)
			RemoveBall(Ball, &Bit);
		else
			Cit++;
	}
}

void BallGame::ChangeLevel(void)
{
	if(mode == Editing)
		SwitchEditMode();
	else
		_StopPhysic();
	EmptyLevel();
	ImportLevelFromJson();
}

void BallGame::ImportLevelFromJson(Node *parent)
{
	std::ifstream myfile;
	std::stringstream buffer;
	String Filename("Levels/");
	Filename += Level;
	Filename += ".json";
	myfile.open (Filename.c_str());
	buffer << myfile.rdbuf();
	myfile.close();
	rapidjson::Document in;
	in.Parse(buffer.str().c_str());
	//Parsing Cases
	rapidjson::Value &cases = in[0];
	for(int cmpt = 0; cmpt < cases.Size(); cmpt++)
	{
		CaseEntity *newCase = new CaseEntity();
		rapidjson::Value &casejson = cases[cmpt];
		newCase->CreateFromJson(casejson, mSceneMgr, m_world, parent);
		AddCase(newCase);
	}
	//Parsing Balls
	rapidjson::Value &balls = in[1];
	for(int cmpt = 0; cmpt < balls.Size(); cmpt++)
	{
		BallEntity *newBall = new BallEntity();
		rapidjson::Value &balljson = balls[cmpt];
		newBall->CreateFromJson(balljson, mSceneMgr, m_world, parent);
		AddBall(newBall);
	}
}

void BallGame::ExportLevelIntoJson(String &export_str)
{
	rapidjson::Document document;
	document.SetArray();

	rapidjson::Document::AllocatorType& allocator = document.GetAllocator();
	rapidjson::Value cases(rapidjson::kArrayType);

	std::list<CaseEntity*>::iterator Cit(Cases.begin());
	while(Cit != Cases.end())
	{
		CaseEntity *Entity = *(Cit++);
		if(Entity == NULL)
			continue;
		rapidjson::Value JCase(rapidjson::kObjectType);
		JCase.AddMember("Type", Entity->type, allocator);
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

	std::cout << strbuf.GetString() << std::endl;
}
