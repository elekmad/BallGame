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
			sum = velocity[0] + velocity[1] + velocity[2];
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
	PlacementMode = Move;
	mode = Running;
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
}

void BallGame::PostUpdateCallback(const NewtonWorld* const world, dFloat timestep)
{
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

void BallGame::createScene(void)
{
	SetupGUI();

	SetupGame();
}

bool BallGame::NormalizeCaseForceDirectionPushBCallback(const CEGUI::EventArgs &e)
{
	char force_c[20];
	double direction_x, direction_y, direction_z, direction;
	direction_x = strtod(CaseForceDirectionXValueEditB->getText().c_str(), NULL);
	direction_y = strtod(CaseForceDirectionYValueEditB->getText().c_str(), NULL);
	direction_z = strtod(CaseForceDirectionZValueEditB->getText().c_str(), NULL);
	direction = sqrt(direction_x * direction_x + direction_y * direction_y + direction_z * direction_z);

	direction_x /= direction;
	snprintf(force_c, 19, "%f", direction_x);
	CaseForceDirectionXValueEditB->setText(force_c);

	direction_y /= direction;
	snprintf(force_c, 19, "%f", direction_y);
	CaseForceDirectionYValueEditB->setText(force_c);

	direction_z /= direction;
	snprintf(force_c, 19, "%f", direction_z);
	CaseForceDirectionZValueEditB->setText(force_c);
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
		ChooseLevelComboB->setVisible(false);
		PlaceNewElementB->setVisible(false);
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
	}
}

bool BallGame::PlaceNewElementBCallback(const CEGUI::EventArgs &e)
{
	if(ToBePlacedEntity != NULL)
		return true;
	std::cout << "Placing new element ?" << std::endl;
	ToBePlacedEntity = new CaseEntity();

	ToBePlacedEntity->InitialPos.x = 0;
	ToBePlacedEntity->InitialPos.y = 0;
	ToBePlacedEntity->InitialPos.z = 0;
	ToBePlacedEntity->InitialScale.x = 25;
	ToBePlacedEntity->InitialScale.y = 25;
	ToBePlacedEntity->InitialScale.z = 25;
	ToBePlacedEntity->InitialOrientation.x = 0;
	ToBePlacedEntity->InitialOrientation.y = 0;
	ToBePlacedEntity->InitialOrientation.z = 0;
	ToBePlacedEntity->InitialOrientation.w = 1;

	Entity *ogreEntity;
	SceneNode *ogreNode;
	ogreEntity = mSceneMgr->createEntity("Cube.mesh");
	ogreNode = mSceneMgr->getRootSceneNode()->createChildSceneNode(ToBePlacedEntity->InitialPos);
	ogreNode->attachObject(ogreEntity);
	ogreNode->showBoundingBox(true);
	ToBePlacedEntity->SetOgreNode(ogreNode);

	return true;
}

void BallGame::PlaceNewElement(void)
{
	if(ToBePlacedEntity == NULL)
		return;
	std::cout << "Placing new element !" << std::endl;
	ToBePlacedEntity->OgreEntity->showBoundingBox(false);


	dVector NewtonBodyLocation;
	dVector NewtonBodySize;
	NewtonBody *newtonBody;

	ToBePlacedEntity->InitialPos = ToBePlacedEntity->OgreEntity->getPosition();
	ToBePlacedEntity->InitialScale = ToBePlacedEntity->OgreEntity->getScale();
	ToBePlacedEntity->InitialOrientation = ToBePlacedEntity->OgreEntity->getOrientation();
	NewtonBodyLocation.m_x = ToBePlacedEntity->InitialPos.x;
	NewtonBodyLocation.m_y = ToBePlacedEntity->InitialPos.y;
	NewtonBodyLocation.m_z = ToBePlacedEntity->InitialPos.z;
	NewtonBodyLocation.m_w = 1;
	Entity *ogreEntity = (Ogre::Entity*)ToBePlacedEntity->OgreEntity->getAttachedObject(0);
	Vector3 AABB(ogreEntity->getBoundingBox().getSize());
	NewtonBodySize.m_x = AABB.x * ToBePlacedEntity->InitialScale.x;
	NewtonBodySize.m_y = AABB.y * ToBePlacedEntity->InitialScale.y;
	NewtonBodySize.m_z = AABB.z * ToBePlacedEntity->InitialScale.z;
	NewtonBodySize.m_w = 0.0f;

	dMatrix casematrix(ToBePlacedEntity->InitialOrientation.getPitch(false).valueRadians(), ToBePlacedEntity->InitialOrientation.getYaw(false).valueRadians(), ToBePlacedEntity->InitialOrientation.getRoll(false).valueRadians(), NewtonBodyLocation);
	if(ToBePlacedEntity->type == Case)
	{
		CaseEntity *Case = (CaseEntity*)ToBePlacedEntity;
		NewtonCollision *collision_tree = NULL;
		if(Case->type == CaseEntity::CaseType::typeRamp)
		{
			Matrix4 ident_ogre_matrix = Matrix4::IDENTITY;
			const MeshPtr ptr = ogreEntity->getMesh();
			collision_tree = ParseEntity(m_world, ptr, ident_ogre_matrix);
		}
		newtonBody = WorldAddCase(m_world, NewtonBodySize, 0, casematrix, Case->type, collision_tree);
		ToBePlacedEntity->SetNewtonBody(newtonBody);
		AddCase(Case);
	}
	ToBePlacedEntity = NULL;
}

bool BallGame::EditModePushBCallback(const CEGUI::EventArgs &e)
{
	SwitchEditMode();
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
    EditModePushB->setSize(CEGUI::USize(CEGUI::UDim(0, 150), CEGUI::UDim(0, 30)));
    EditModePushB->setVisible(false);

    MainLayout->addChild(EditModePushB);

    EditModePushB->subscribeEvent(CEGUI::PushButton::EventClicked,
    		CEGUI::Event::Subscriber(&BallGame::EditModePushBCallback, this));

    SetWindowsPosNearToOther(EditModePushB, StopPhysicPushB, 0, 1);

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

    SaveLevelPushB = (CEGUI::PushButton*)wmgr.createWindow("OgreTray/Button");
    SaveLevelPushB->setText("Save");
    SaveLevelPushB->setSize(CEGUI::USize(CEGUI::UDim(0, 150), CEGUI::UDim(0, 30)));
    SaveLevelPushB->setVisible(false);

    MainLayout->addChild(SaveLevelPushB);

    SaveLevelPushB->subscribeEvent(CEGUI::PushButton::EventClicked,
    		CEGUI::Event::Subscriber(&BallGame::SaveLevelPushBCallback, this));

    SetWindowsPosNearToOther(SaveLevelPushB, EditModePushB, 0, 2);// Be Carefull, Combobox size is size with combo expanded !

    QuitPushB = (CEGUI::PushButton*)wmgr.createWindow("OgreTray/Button");
    QuitPushB->setText("Quit");
    QuitPushB->setSize(CEGUI::USize(CEGUI::UDim(0, 150), CEGUI::UDim(0, 30)));
    QuitPushB->setVisible(false);

    MainLayout->addChild(QuitPushB);

    QuitPushB->subscribeEvent(CEGUI::PushButton::EventClicked,
    		CEGUI::Event::Subscriber(&BallGame::QuitPushBCallback, this));

    SetWindowsPosNearToOther(QuitPushB, SaveLevelPushB, 0, 1);

    LevelNameBanner = (CEGUI::Titlebar*)wmgr.createWindow("OgreTray/Title");
    LevelNameBanner->setText("Level");
    LevelNameBanner->setSize(CEGUI::USize(CEGUI::UDim(0, 150), CEGUI::UDim(0, 30)));
    LevelNameBanner->setVerticalAlignment(CEGUI::VA_TOP);
    LevelNameBanner->setHorizontalAlignment(CEGUI::HA_CENTRE);

    MainLayout->addChild(LevelNameBanner);

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
    AddElementTitleBanner = (CEGUI::Titlebar*)wmgr.createWindow("OgreTray/Title");
    AddElementTitleBanner->setText("Add");
    AddElementTitleBanner->setSize(CEGUI::USize(CEGUI::UDim(0, 150), CEGUI::UDim(0, 30)));
    AddElementTitleBanner->setVerticalAlignment(CEGUI::VA_CENTRE);
    AddElementTitleBanner->setHorizontalAlignment(CEGUI::HA_RIGHT);
    AddElementTitleBanner->setVisible(false);

    MainLayout->addChild(AddElementTitleBanner);


    ChooseTypeOfElementToAddB = (CEGUI::Combobox*)wmgr.createWindow("OgreTray/Combobox");
    ChooseTypeOfElementToAddB->addItem(new CEGUI::ListboxTextItem("Case"));
    ChooseTypeOfElementToAddB->setText("Case");
    ChooseTypeOfElementToAddB->addItem(new CEGUI::ListboxTextItem("Ball"));
    ChooseTypeOfElementToAddB->setSize(CEGUI::USize(CEGUI::UDim(0, 150), CEGUI::UDim(0, 40 * (ChooseLevelComboB->getItemCount() + 1))));
    ChooseTypeOfElementToAddB->setVerticalAlignment(CEGUI::VA_CENTRE);
    ChooseTypeOfElementToAddB->setHorizontalAlignment(CEGUI::HA_RIGHT);
    ChooseTypeOfElementToAddB->setVisible(false);

    MainLayout->addChild(ChooseTypeOfElementToAddB);

    SetWindowsPosNearToOther(ChooseTypeOfElementToAddB, AddElementTitleBanner, 0, 1);

    PlaceNewElementB = (CEGUI::PushButton*)wmgr.createWindow("OgreTray/Button");
    PlaceNewElementB->setText("Place");
    PlaceNewElementB->setSize(CEGUI::USize(CEGUI::UDim(0, 150), CEGUI::UDim(0, 30)));
    PlaceNewElementB->setVerticalAlignment(CEGUI::VA_CENTRE);
    PlaceNewElementB->setHorizontalAlignment(CEGUI::HA_RIGHT);
    PlaceNewElementB->setVisible(false);

    PlaceNewElementB->subscribeEvent(CEGUI::PushButton::EventClicked,
			CEGUI::Event::Subscriber(&BallGame::PlaceNewElementBCallback, this));

    MainLayout->addChild(PlaceNewElementB);

    SetWindowsPosNearToOther(PlaceNewElementB, AddElementTitleBanner, 0, 2);

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

    MainLayout->addChild(CaseHasForceToggleB);

    CaseForceValueEditB = (CEGUI::Editbox*)wmgr.createWindow("OgreTray/Editbox");
    CaseForceValueEditB->setSize(CEGUI::USize(CEGUI::UDim(0, 50), CEGUI::UDim(0, 30)));
    CaseForceValueEditB->setVerticalAlignment(CEGUI::VA_BOTTOM);
    CaseForceValueEditB->setHorizontalAlignment(CEGUI::HA_CENTRE);
    CaseForceValueEditB->setVisible(false);

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

    MainLayout->addChild(CaseHasForceDirectionToggleB);

    CaseForceDirectionXValueEditB = (CEGUI::Editbox*)wmgr.createWindow("OgreTray/Editbox");
    CaseForceDirectionXValueEditB->setSize(CEGUI::USize(CEGUI::UDim(0, 50), CEGUI::UDim(0, 30)));
    CaseForceDirectionXValueEditB->setVerticalAlignment(CEGUI::VA_BOTTOM);
    CaseForceDirectionXValueEditB->setHorizontalAlignment(CEGUI::HA_CENTRE);
    CaseForceDirectionXValueEditB->setVisible(false);

    MainLayout->addChild(CaseForceDirectionXValueEditB);

    CaseForceDirectionYValueEditB = (CEGUI::Editbox*)wmgr.createWindow("OgreTray/Editbox");
    CaseForceDirectionYValueEditB->setSize(CEGUI::USize(CEGUI::UDim(0, 50), CEGUI::UDim(0, 30)));
    CaseForceDirectionYValueEditB->setVerticalAlignment(CEGUI::VA_BOTTOM);
    CaseForceDirectionYValueEditB->setHorizontalAlignment(CEGUI::HA_CENTRE);
    CaseForceDirectionYValueEditB->setVisible(false);

    MainLayout->addChild(CaseForceDirectionYValueEditB);

    CaseForceDirectionZValueEditB = (CEGUI::Editbox*)wmgr.createWindow("OgreTray/Editbox");
    CaseForceDirectionZValueEditB->setSize(CEGUI::USize(CEGUI::UDim(0, 50), CEGUI::UDim(0, 30)));
    CaseForceDirectionZValueEditB->setVerticalAlignment(CEGUI::VA_BOTTOM);
    CaseForceDirectionZValueEditB->setHorizontalAlignment(CEGUI::HA_CENTRE);
    CaseForceDirectionZValueEditB->setVisible(false);

    MainLayout->addChild(CaseForceDirectionZValueEditB);

    NormalizeCaseForceDirectionPushB = (CEGUI::PushButton*)wmgr.createWindow("OgreTray/Button");
    NormalizeCaseForceDirectionPushB->setText("Norm");
    NormalizeCaseForceDirectionPushB->setSize(CEGUI::USize(CEGUI::UDim(0, 50), CEGUI::UDim(0, 30)));
    NormalizeCaseForceDirectionPushB->setVerticalAlignment(CEGUI::VA_BOTTOM);
    NormalizeCaseForceDirectionPushB->setHorizontalAlignment(CEGUI::HA_CENTRE);
    NormalizeCaseForceDirectionPushB->setVisible(false);
    NormalizeCaseForceDirectionPushB->subscribeEvent(CEGUI::PushButton::EventClicked,
    		CEGUI::Event::Subscriber(&BallGame::NormalizeCaseForceDirectionPushBCallback, this));

    MainLayout->addChild(NormalizeCaseForceDirectionPushB);

    ApplyForceChangesToCasePushB = (CEGUI::PushButton*)wmgr.createWindow("OgreTray/Button");
    ApplyForceChangesToCasePushB->setText("Apply");
    ApplyForceChangesToCasePushB->setSize(CEGUI::USize(CEGUI::UDim(0, 100), CEGUI::UDim(0, 30)));
    ApplyForceChangesToCasePushB->setVerticalAlignment(CEGUI::VA_BOTTOM);
    ApplyForceChangesToCasePushB->setHorizontalAlignment(CEGUI::HA_CENTRE);
    ApplyForceChangesToCasePushB->setVisible(false);
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

    // -- tutorial section start --
    //! [turnlights]
	mSceneMgr->setAmbientLight(ColourValue(0.5, 0.5, 0.5));
    //! [turnlights]

    //! [newlight]
    Light* light = mSceneMgr->createLight("MainLight");
    SceneNode* lightNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
    lightNode->attachObject(light);
    //! [newlight]

    //! [lightpos]
    lightNode->setPosition(20, 80, 50);
    //! [lightpos]

    ChangeLevel();

#if 0
    //////////////   ADD CASES ///////////////////

	for(int cmpt = 0; cmpt < WORLD_LENGTH; cmpt++)
    {
		for(int cmpt2 = 0; cmpt2 < WORLD_DEPTH; cmpt2++)
		{
				dVector location;
				dVector tsize;
				NewtonBody *tableBody;
				Entity* ogreEntity;
				enum CaseEntity::CaseType type = CaseEntity::CaseType::typeBox;
				if(cmpt == 5)
				{
					ogreEntity = mSceneMgr->createEntity("Rampe.mesh");
					type = CaseEntity::CaseType::typeRamp;
				}
				else
					ogreEntity = mSceneMgr->createEntity("Cube.mesh");
				SceneNode* ogreNode = mSceneMgr->getRootSceneNode()->createChildSceneNode(Vector3(cmpt * 50, cmpt2 * 50, 0));
				ogreNode->setScale(25, 25, 25);
				if(cmpt == 5)
				{
					ogreNode->roll(Degree(-90));
					ogreNode->setPosition(cmpt * 50, cmpt2 * 50, 25);
				}
				ogreNode->attachObject(ogreEntity);
				Vector3 pos(ogreNode->getPosition());
				location.m_x = pos.x;
				location.m_y = pos.y;
				location.m_z = pos.z;
				location.m_w = 1;
				Vector3 AABB(ogreEntity->getBoundingBox().getSize()), scale(ogreNode->getScale());
				tsize.m_x = AABB.x * scale.x;
				tsize.m_y = AABB.y * scale.y;
				tsize.m_z = AABB.z * scale.z;
				tsize.m_w = 0.0f;
				//ident_matrix.m_posit = location;
				Quaternion orientation = ogreNode->getOrientation();
				dMatrix casematrix(orientation.getPitch(false).valueRadians(), orientation.getYaw(false).valueRadians(), orientation.getRoll(false).valueRadians(), location);
				NewtonCollision *collision_tree = NULL;
				if(type == CaseEntity::CaseType::typeRamp)
				{
					Matrix4 ident_ogre_matrix = Matrix4::IDENTITY;
					const MeshPtr ptr = ogreEntity->getMesh();
					collision_tree = ParseEntity(m_world, ptr, ident_ogre_matrix);
				}
				tableBody = WorldAddCase(m_world, tsize, 0, casematrix, type, collision_tree);
				CaseEntity *Entity = new CaseEntity(casematrix, type);
				Entity->InitialPos = ogreNode->getPosition();
				Entity->InitialScale = ogreNode->getScale();
				Entity->InitialOrientation = ogreNode->getOrientation();
				if(cmpt == 0)
				{
					dVector *direction = new dVector(1.0, 0.0, 0.0);
					Entity->SetForceToApply(100.0, direction);
						//					dVector *force = new dVector(0.0, 0.0, 1000.0);
				}
				if((cmpt == 4 && cmpt2 == 0) || (cmpt == 3 && cmpt2 == 1))
				{
					Entity->SetForceToApply(100.0, NULL);
				}
				if(cmpt >= 10)
				{
					dVector *direction = new dVector(0.0, 0.0, 1.0);
					Entity->SetForceToApply(1000.0, direction);
						//					dVector *force = new dVector(0.0, 0.0, 1000.0);
				}
				Entity->SetOgreNode(ogreNode);
				Entity->SetNewtonBody(tableBody);
				AddCase(Entity);
		}
    }

    //////////////   ADD BALLS ///////////////////

	for(int cmpt = 0; cmpt < 1; cmpt++)
    {
		for(int cmpt2 = 0; cmpt2 < WORLD_DEPTH; cmpt2++)
		{
			dVector location;
			dVector tsize;
			NewtonBody *BallBody;
			Entity* ogreEntity = mSceneMgr->createEntity("Sphere.mesh");
			SceneNode* ogreNode = mSceneMgr->getRootSceneNode()->createChildSceneNode(Vector3(cmpt * 50, cmpt2 * 50, 55));
			ogreNode->setScale(25, 25, 25);
			ogreNode->attachObject(ogreEntity);
			Vector3 pos(ogreNode->getPosition());
			location.m_x = pos.x;
			location.m_y = pos.y;
			location.m_z = pos.z;
			location.m_w = 1;
			Vector3 AABB(ogreEntity->getBoundingBox().getSize()), scale(ogreNode->getScale());
			tsize.m_x = AABB.x * scale.x;
			tsize.m_y = AABB.y * scale.y;
			tsize.m_z = AABB.z * scale.z;
			tsize.m_w = 0.0f;
			Quaternion orientation = ogreNode->getOrientation();
			dMatrix ballmatrix(orientation.getPitch(false).valueRadians(), orientation.getYaw(false).valueRadians(), orientation.getRoll(false).valueRadians(), location);
			if(cmpt2 == 0)
			    BallBody = WorldAddBall(m_world, 10, tsize, 0, ballmatrix);
			else
			    BallBody = WorldAddBall(m_world, 1000, tsize, 0, ballmatrix);
			BallEntity *Entity = new BallEntity(ballmatrix);
			Entity->InitialPos = ogreNode->getPosition();
			Entity->InitialScale = ogreNode->getScale();
			Entity->InitialOrientation = ogreNode->getOrientation();
			Entity->InitialMass = cmpt2 == 0 ? 10 : 1000;
			Entity->SetOgreNode(ogreNode);
			Entity->SetNewtonBody(BallBody);
			AddBall(Entity);
		}
    }
#endif
    //! [cameramove]
    SetCam(-184, -253, 352);
    mCamera->setOrientation(Ogre::Quaternion(0.835422, 0.393051, -0.238709, -0.300998));

    _StartPhysic();
//    _StopPhysic();
}

void BallGame::CheckforCollides(void)
{
	for(int cmpt = 0; cmpt < Balls.GetSize(); cmpt++)
	{
		BallEntity *ball = Balls[cmpt];
		if(ball == NULL)
			continue;
		for(int cmpt2 = 0; cmpt2 < Cases.GetSize(); cmpt2++)
		{
			CaseEntity *Case = Cases[cmpt2];
			if(Case == NULL)
				continue;
//          if(NewtonBodyFindContact(ball, Case) != NULL)
			if(CheckIfBodiesCollide(ball->Body, Case->Body) != NULL)
			{
				//std::cout << ball << " id " << cmpt << " and " << Case << " id " << cmpt2 << " Collides by joints" << std::endl;
			}
			if(DoBodiesCollide(m_world, ball->Body, Case->Body))
			{
				//if(!CEntity->CheckIfAlreadyColliding(ball))
				{
					Case->ApplyForceOnBall(ball);

					//std::cout << ball << " id " << cmpt << " and " << Case << " id " << cmpt2 << " Collides" << std::endl;
				}
//				else
//					std::cout << ball << " id " << cmpt << " and " << Case << " id " << cmpt2 << " Already Colliding" << std::endl;
			}
		}
	}
}

void BallGame::AddBall(BallEntity *ball)
{
	if(ball == NULL)
		return;
	int id = NewtonBodyGetID(ball->Body), size = Balls.GetSize();
	while(size < id)
		Balls[size++] = NULL;
	Balls[id] = ball;
	size = id + 1;
	while(size < Balls.GetSize())
		Balls[size++] = NULL;
}

void BallGame::AddCase(CaseEntity *Wcase)
{
	if(Wcase == NULL)
		return;
	int id = NewtonBodyGetID(Wcase->Body), size = Cases.GetSize();
	while(size < id)
		Cases[size++] = NULL;
	Cases[id] = Wcase;
	size = id + 1;
	while(size < Cases.GetSize())
		Cases[size++] = NULL;
}


bool BallGame::frameEnded(const Ogre::FrameEvent& fe)
{
//    std::cout << "Render a frame" << std::endl;
	dFloat timestep = dGetElapsedSeconds();
	UpdatePhysics(timestep);

	if(mode == Running)
	{
		for(int cmpt = 0; cmpt < Balls.GetSize(); cmpt++)
		{
			BallEntity *ball = Balls[cmpt];
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
					&& (UnderEditBall == NULL || UnderEditBall->OgreEntity != LastHighligted))
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
		String ForceStr;
		char force_c[50];
		snprintf(force_c, 49, "%f", UnderEditCaseForce);
		ForceStr = force_c;
		std::cout << "Force " <<  UnderEditCaseForce << ", " << force_c << ", " << ForceStr << std::endl;
		CaseForceValueEditB->setDisabled(false);
		CaseForceValueEditB->setText(ForceStr);

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
			snprintf(force_c, 49, "%f", force_direction.m_x);
			ForceStr = force_c;
			CaseForceDirectionXValueEditB->setText(ForceStr);
			snprintf(force_c, 49, "%f", force_direction.m_y);
			ForceStr = force_c;
			CaseForceDirectionYValueEditB->setText(ForceStr);
			snprintf(force_c, 49, "%f", force_direction.m_z);
			ForceStr = force_c;
			CaseForceDirectionZValueEditB->setText(ForceStr);
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
    if(mode == Editing)
    {
    	if(LastHighligted != NULL)
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
			mRoot->queueEndRendering();
		}
		else
		{
			StopPhysicPushB->setVisible(true);
			EditModePushB->setVisible(true);
			ChooseLevelComboB->setVisible(true);
			SaveLevelPushB->setVisible(true);
			QuitPushB->setVisible(true);
		}
	    break;
	case OIS::KeyCode::KC_UP:
		if(ToBePlacedEntity != NULL)
		{
			switch(PlacementMode)
			{
			case Move :
				{
					Vector3 pos = ToBePlacedEntity->OgreEntity->getPosition();
					pos.y += 10;
					ToBePlacedEntity->OgreEntity->setPosition(pos);
				}
				break;
			case Rotate :
				ToBePlacedEntity->OgreEntity->yaw(Degree(10));
				break;
			case Scale :
				{
					Vector3 pos = ToBePlacedEntity->OgreEntity->getScale();
					pos.y += 10;
					ToBePlacedEntity->OgreEntity->setScale(pos);
				}
				break;
			}
		}
	    break;
	case OIS::KeyCode::KC_DOWN:
		if(ToBePlacedEntity != NULL)
		{
			switch(PlacementMode)
			{
			case Move :
				{
					Vector3 pos = ToBePlacedEntity->OgreEntity->getPosition();
					pos.y -= 10;
					ToBePlacedEntity->OgreEntity->setPosition(pos);
				}
				break;
			case Rotate :
				ToBePlacedEntity->OgreEntity->yaw(Degree(-10));
				break;
			case Scale :
				{
					Vector3 pos = ToBePlacedEntity->OgreEntity->getScale();
					pos.y -= 10;
					ToBePlacedEntity->OgreEntity->setScale(pos);
				}
				break;
			}
		}
	    break;
	case OIS::KeyCode::KC_LEFT:
		if(ToBePlacedEntity != NULL)
		{
			switch(PlacementMode)
			{
			case Move :
				{
					Vector3 pos = ToBePlacedEntity->OgreEntity->getPosition();
					pos.x -= 10;
					ToBePlacedEntity->OgreEntity->setPosition(pos);
				}
				break;
			case Rotate :
				ToBePlacedEntity->OgreEntity->pitch(Degree(10));
				break;
			case Scale :
				{
					Vector3 pos = ToBePlacedEntity->OgreEntity->getScale();
					pos.x -= 10;
					ToBePlacedEntity->OgreEntity->setScale(pos);
				}
				break;
			}
		}
	    break;
	case OIS::KeyCode::KC_RIGHT:
		if(ToBePlacedEntity != NULL)
		{
			switch(PlacementMode)
			{
			case Move :
				{
					Vector3 pos = ToBePlacedEntity->OgreEntity->getPosition();
					pos.x += 10;
					ToBePlacedEntity->OgreEntity->setPosition(pos);
				}
				break;
			case Rotate :
				ToBePlacedEntity->OgreEntity->pitch(Degree(-10));
				break;
			case Scale :
				{
					Vector3 pos = ToBePlacedEntity->OgreEntity->getScale();
					pos.x += 10;
					ToBePlacedEntity->OgreEntity->setScale(pos);
				}
				break;
			}
		}
	    break;
	case OIS::KeyCode::KC_PGUP:
		if(ToBePlacedEntity != NULL)
		{
			switch(PlacementMode)
			{
			case Move :
				{
					Vector3 pos = ToBePlacedEntity->OgreEntity->getPosition();
					pos.z += 10;
					ToBePlacedEntity->OgreEntity->setPosition(pos);
				}
				break;
			case Rotate :
				ToBePlacedEntity->OgreEntity->roll(Degree(10));
				break;
			case Scale :
				{
					Vector3 pos = ToBePlacedEntity->OgreEntity->getScale();
					pos.z += 10;
					ToBePlacedEntity->OgreEntity->setScale(pos);
				}
				break;
			}
		}
	    break;
	case OIS::KeyCode::KC_PGDOWN:
		if(ToBePlacedEntity != NULL)
		{
			switch(PlacementMode)
			{
			case Move :
				{
					Vector3 pos = ToBePlacedEntity->OgreEntity->getPosition();
					pos.z -= 10;
					ToBePlacedEntity->OgreEntity->setPosition(pos);
				}
				break;
			case Rotate :
				ToBePlacedEntity->OgreEntity->roll(Degree(-10));
				break;
			case Scale :
				{
					Vector3 pos = ToBePlacedEntity->OgreEntity->getScale();
					pos.z -= 10;
					ToBePlacedEntity->OgreEntity->setScale(pos);
				}
				break;
			}
		}
	    break;
	case OIS::KeyCode::KC_SPACE:
		if(mode == Editing)
			PlaceNewElement();
		break;
	case OIS::KeyCode::KC_M:
		if(mode == Editing)
		{
			switch(PlacementMode)
			{
			case Move :
				PlacementMode = Rotate;
				break;
			case Rotate :
				PlacementMode = Scale;
				break;
			case Scale :
				PlacementMode = Move;
				break;
			}
		}
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

void CaseEntity::CreateFromJson(rapidjson::Value &v, Ogre::SceneManager* mSceneMgr, NewtonWorld *m_world)
{
	ImportFromJson(v);
	dVector NewtonBodyLocation;
	dVector NewtonBodySize;
	NewtonBody *newtonBody;
	Entity *ogreEntity;
	SceneNode *ogreNode;
	switch(type)
	{
		case CaseEntity::typeRamp :
			ogreEntity = mSceneMgr->createEntity("Rampe.mesh");
			break;
		case CaseEntity::typeBox :
			ogreEntity = mSceneMgr->createEntity("Cube.mesh");
			break;
	}
	ogreNode = mSceneMgr->getRootSceneNode()->createChildSceneNode(InitialPos);
	ogreNode->attachObject(ogreEntity);
	SetOgreNode(ogreNode);
	NewtonBodyLocation.m_x = InitialPos.x;
	NewtonBodyLocation.m_y = InitialPos.y;
	NewtonBodyLocation.m_z = InitialPos.z;
	NewtonBodyLocation.m_w = 1;
	Vector3 AABB(ogreEntity->getBoundingBox().getSize());
	NewtonBodySize.m_x = AABB.x * InitialScale.x;
	NewtonBodySize.m_y = AABB.y * InitialScale.y;
	NewtonBodySize.m_z = AABB.z * InitialScale.z;
	NewtonBodySize.m_w = 0.0f;

	dMatrix casematrix(InitialOrientation.getPitch(false).valueRadians(), InitialOrientation.getYaw(false).valueRadians(), InitialOrientation.getRoll(false).valueRadians(), NewtonBodyLocation);
	NewtonCollision *collision_tree = NULL;
	if(type == CaseEntity::CaseType::typeRamp)
	{
		Matrix4 ident_ogre_matrix = Matrix4::IDENTITY;
		const MeshPtr ptr = ogreEntity->getMesh();
		collision_tree = ParseEntity(m_world, ptr, ident_ogre_matrix);
	}
	newtonBody = WorldAddCase(m_world, NewtonBodySize, 0, casematrix, type, collision_tree);

	SetNewtonBody(newtonBody);
}

void CaseEntity::ImportFromJson(rapidjson::Value &v)
{
	BallGameEntity::ImportFromJson(v);
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

void BallEntity::CreateFromJson(rapidjson::Value &v, Ogre::SceneManager* mSceneMgr, NewtonWorld *m_world)
{
	ImportFromJson(v);
	dVector NewtonBodyLocation;
	dVector NewtonBodySize;
	NewtonBody *BallBody;
	Entity* ogreEntity = mSceneMgr->createEntity("Sphere.mesh");
	SceneNode* ogreNode = mSceneMgr->getRootSceneNode()->createChildSceneNode(InitialPos);
	ogreNode->attachObject(ogreEntity);
	SetOgreNode(ogreNode);
	NewtonBodyLocation.m_x = InitialPos.x;
	NewtonBodyLocation.m_y = InitialPos.y;
	NewtonBodyLocation.m_z = InitialPos.z;
	NewtonBodyLocation.m_w = 1;
	Vector3 AABB(ogreEntity->getBoundingBox().getSize());
	NewtonBodySize.m_x = AABB.x * InitialScale.x;
	NewtonBodySize.m_y = AABB.y * InitialScale.y;
	NewtonBodySize.m_z = AABB.z * InitialScale.z;
	NewtonBodySize.m_w = 0.0f;
	dMatrix ballmatrix(InitialOrientation.getPitch(false).valueRadians(), InitialOrientation.getYaw(false).valueRadians(), InitialOrientation.getRoll(false).valueRadians(), NewtonBodyLocation);
	BallBody = WorldAddBall(m_world, InitialMass, NewtonBodySize, 0, ballmatrix);

	SetNewtonBody(BallBody);
}

void BallEntity::ImportFromJson(rapidjson::Value &v)
{
	BallGameEntity::ImportFromJson(v);
	InitialMass = v["Mass"].GetFloat();
}

void BallGameEntity::ImportFromJson(rapidjson::Value &v)
{
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
}


void BallGame::ChangeLevel(void)
{
	_StopPhysic();
	for(int cmpt = 0; cmpt < Cases.GetSize(); cmpt++)
	{
		CaseEntity *Case = Cases[cmpt];
		if(Case == NULL)
			continue;
		NewtonDestroyBody(Case->Body);
		mSceneMgr->getRootSceneNode()->removeChild(Case->OgreEntity);
		delete Case;
		Cases[cmpt] = NULL;
	}
	for(int cmpt = 0; cmpt < Balls.GetSize(); cmpt++)
	{
		BallEntity *Ball = Balls[cmpt];
		if(Ball == NULL)
			continue;
		NewtonDestroyBody(Ball->Body);
		mSceneMgr->getRootSceneNode()->removeChild(Ball->OgreEntity);
		delete Ball;
		Balls[cmpt] = NULL;
	}
	ImportLevelFromJson();
}

void BallGame::ImportLevelFromJson(void)
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
		newCase->CreateFromJson(casejson, mSceneMgr, m_world);
		AddCase(newCase);
	}
	//Parsing Balls
	rapidjson::Value &balls = in[1];
	for(int cmpt = 0; cmpt < balls.Size(); cmpt++)
	{
		BallEntity *newBall = new BallEntity();
		rapidjson::Value &balljson = balls[cmpt];
		newBall->CreateFromJson(balljson, mSceneMgr, m_world);
		AddBall(newBall);
	}
}

void BallGame::ExportLevelIntoJson(String &export_str)
{
	rapidjson::Document document;
	document.SetArray();

	rapidjson::Document::AllocatorType& allocator = document.GetAllocator();
	rapidjson::Value cases(rapidjson::kArrayType);

	for(int cmpt = 0; cmpt < Cases.GetSize(); cmpt++)
	{
		CaseEntity *Entity = Cases[cmpt];
		if(Entity == NULL)
			continue;
		rapidjson::Value JCase(rapidjson::kObjectType);
		JCase.AddMember("Type", Entity->type, allocator);
		Entity->ExportToJson(JCase, allocator);

		cases.PushBack(JCase, allocator);
	}

	document.PushBack(cases, allocator);

	rapidjson::Value balls(rapidjson::kArrayType);

	for(int cmpt = 0; cmpt < Balls.GetSize(); cmpt++)
	{
		BallEntity *Entity = Balls[cmpt];
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
