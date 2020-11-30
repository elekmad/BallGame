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
#include <dNewtonScopeBuffer.h>
#include <Newton.h>
#include <OgreRay.h>
//Put BallGame.h in last because of Xlib defines (True False Bool None) which must be undef
#include "BallGame.h"


BallGameEntity::BallGameEntity(const dMatrix& matrix) : m_matrix(matrix)
	,m_curPosition (matrix.m_posit)
	,m_nextPosition (matrix.m_posit)
	,m_curRotation (dQuaternion (matrix))
	,m_nextRotation (dQuaternion (matrix))
{
	OgreEntity = NULL;
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

void CaseEntity::ApplyForceOnBall(NewtonBody *ball)
{
	BallEntity *BEntity = (BallEntity*)NewtonBodyGetUserData(ball);
	if(!isnanf(force_to_apply))
	{
		if(force_direction != NULL)
		{
			dVector *force = new dVector();
			*force = force_direction->Scale(force_to_apply);
			BEntity->AddForceVector(force);
		}
		else
		{
			dFloat velocity[3], sum;
			NewtonBodyGetVelocity(ball, velocity);
			sum = velocity[0] + velocity[1] + velocity[2];
			velocity[0] /= sum;
			velocity[1] /= sum;
			velocity[2] /= sum;//Like that we have normalization of velocity into percents, we can use it to scale force.
			dVector *force = new dVector(velocity[0], velocity[1], velocity[2]);
			*force = force->Scale(force_to_apply);
			BEntity->AddForceVector(force);
		}
	}
	AddBallColliding(ball);
}

NewtonWorld* BallGame::GetNewton(void)
{
	return m_world;
}

void BallGame::SetCam(int x, int y, int z)
{
    camx = x;
    camy = y;
    camz = z;
    //camNode->translate(camx, camx, camz, Ogre::Node::TransformSpace::TS_LOCAL);
    mCamera->setPosition(camx, camy, camz);
    GetCamParams();
}

void BallGame::MoveCam(int x, int y, int z)
{
	mCamera->moveRelative(Ogre::Vector3(x, y, z));
    //SetCam(camx + x, camy + y, camz + z);
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

    camx = camy = camz = 0;

	m_world = NULL;
	mWindow = NULL;
	LastHighligted = NULL;
	UnderEditCase = NULL;
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

bool BallGame::NormalizeForceDirection(const CEGUI::EventArgs &e)
{
	char force_c[20];
	double direction_x, direction_y, direction_z, direction;
	direction_x = strtod(direction_x_edit->getText().c_str(), NULL);
	direction_y = strtod(direction_y_edit->getText().c_str(), NULL);
	direction_z = strtod(direction_z_edit->getText().c_str(), NULL);
	direction = sqrt(direction_x * direction_x + direction_y * direction_y + direction_z * direction_z);

	direction_x /= direction;
	snprintf(force_c, 19, "%f", direction_x);
	direction_x_edit->setText(force_c);

	direction_y /= direction;
	snprintf(force_c, 19, "%f", direction_y);
	direction_y_edit->setText(force_c);

	direction_z /= direction;
	snprintf(force_c, 19, "%f", direction_z);
	direction_z_edit->setText(force_c);
    return true;
}

bool BallGame::quit(const CEGUI::EventArgs &e)
{
	mRoot->queueEndRendering();
    return true;
}

void BallGame::_StartPhysic(void)
{
	StopPhysicB->setText("Stop Physic");
	m_suspendPhysicsUpdate = false;
}

void BallGame::_StopPhysic(void)
{
	StopPhysicB->setText("Start Physic");
	m_suspendPhysicsUpdate = true;
}

bool BallGame::StopPhysic(const CEGUI::EventArgs &e)
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
		Editing_banner->setVisible(true);
	}
	else
	{
		std::cout << "Running Mode" << std::endl;
		mode = Running;
		Editing_banner->setVisible(false);
		ApplyToCase->setVisible(false);
		has_force_button->setVisible(false);
		force_edit->setVisible(false);
		has_direction_button->setVisible(false);
		direction_x_edit->setVisible(false);
		direction_y_edit->setVisible(false);
		direction_z_edit->setVisible(false);
		Normalize_direction->setVisible(false);
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

bool BallGame::EditCallback(const CEGUI::EventArgs &e)
{
	SwitchEditMode();
    return true;
}

void SetWindowsPosNearToOther(CEGUI::Window *self, CEGUI::Window *other, int H_factor, int V_factor)
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
    main_layout = static_cast<CEGUI::LayoutContainer*>(wmgr.createWindow("VerticalLayoutContainer"));
    sheet->addChild(main_layout);
    main_layout->setVisible(false);

    StopPhysicB = (CEGUI::PushButton*)wmgr.createWindow("OgreTray/Button");
    StopPhysicB->setText("Start/Stop Physic");
    StopPhysicB->setSize(CEGUI::USize(CEGUI::UDim(0, 150), CEGUI::UDim(0, 30)));

    main_layout->addChild(StopPhysicB);

    StopPhysicB->subscribeEvent(CEGUI::PushButton::EventClicked,
    		CEGUI::Event::Subscriber(&BallGame::StopPhysic, this));

    CEGUI::PushButton *Editing = (CEGUI::PushButton*)wmgr.createWindow("OgreTray/Button");
    Editing->setText("Edit");
    Editing->setSize(CEGUI::USize(CEGUI::UDim(0, 150), CEGUI::UDim(0, 30)));

    main_layout->addChild(Editing);

    Editing->subscribeEvent(CEGUI::PushButton::EventClicked,
    		CEGUI::Event::Subscriber(&BallGame::EditCallback, this));

    SetWindowsPosNearToOther(Editing, StopPhysicB, 0, 1);

    CEGUI::PushButton *Quit = (CEGUI::PushButton*)wmgr.createWindow("OgreTray/Button");
    Quit->setText("Quit");
    Quit->setSize(CEGUI::USize(CEGUI::UDim(0, 150), CEGUI::UDim(0, 30)));

    main_layout->addChild(Quit);

    Quit->subscribeEvent(CEGUI::PushButton::EventClicked,
    		CEGUI::Event::Subscriber(&BallGame::quit, this));

    SetWindowsPosNearToOther(Quit, Editing, 0, 1);


    //Edit GUI

    Editing_banner = (CEGUI::Titlebar*)wmgr.createWindow("OgreTray/Title");
    Editing_banner->setText("Edit Mode");
    Editing_banner->setSize(CEGUI::USize(CEGUI::UDim(0, 150), CEGUI::UDim(0, 30)));
    Editing_banner->setVerticalAlignment(CEGUI::VA_TOP);
    Editing_banner->setHorizontalAlignment(CEGUI::HA_CENTRE);
    Editing_banner->setVisible(false);

    main_layout->addChild(Editing_banner);

#define TOGGLE_FORCE_NAME "HasForceToggle"
    has_force_button = (CEGUI::ToggleButton*)wmgr.createWindow("OgreTray/Checkbox", TOGGLE_FORCE_NAME);
    has_force_button->setText("Has Force");
    has_force_button->setSelected(false);
    has_force_button->setSize(CEGUI::USize(CEGUI::UDim(0, 200), CEGUI::UDim(0, 30)));
    has_force_button->setVerticalAlignment(CEGUI::VA_BOTTOM);
    has_force_button->setHorizontalAlignment(CEGUI::HA_CENTRE);
    has_force_button->setVisible(false);

    has_force_button->subscribeEvent(CEGUI::ToggleButton::EventSelectStateChanged,
			CEGUI::Event::Subscriber(&BallGame::UpdateEditButtonsCallback, this));

    main_layout->addChild(has_force_button);

    force_edit = (CEGUI::Editbox*)wmgr.createWindow("OgreTray/Editbox");
    force_edit->setSize(CEGUI::USize(CEGUI::UDim(0, 50), CEGUI::UDim(0, 30)));
    force_edit->setVerticalAlignment(CEGUI::VA_BOTTOM);
    force_edit->setHorizontalAlignment(CEGUI::HA_CENTRE);
    force_edit->setVisible(false);

    main_layout->addChild(force_edit);

#define TOGGLE_FORCE_DIRECTION_NAME "HasForceDirectedToggle"
    has_direction_button = (CEGUI::ToggleButton*)wmgr.createWindow("OgreTray/Checkbox", TOGGLE_FORCE_DIRECTION_NAME);
    has_direction_button->setText("Has Force Directed");
    has_direction_button->setSelected(false);
    has_direction_button->setSize(CEGUI::USize(CEGUI::UDim(0, 200), CEGUI::UDim(0, 30)));
    has_direction_button->setVerticalAlignment(CEGUI::VA_BOTTOM);
    has_direction_button->setHorizontalAlignment(CEGUI::HA_CENTRE);
    has_direction_button->setVisible(false);

	has_direction_button->subscribeEvent(CEGUI::ToggleButton::EventSelectStateChanged,
			CEGUI::Event::Subscriber(&BallGame::UpdateEditButtonsCallback, this));

    main_layout->addChild(has_direction_button);

    direction_x_edit = (CEGUI::Editbox*)wmgr.createWindow("OgreTray/Editbox");
    direction_x_edit->setSize(CEGUI::USize(CEGUI::UDim(0, 50), CEGUI::UDim(0, 30)));
    direction_x_edit->setVerticalAlignment(CEGUI::VA_BOTTOM);
    direction_x_edit->setHorizontalAlignment(CEGUI::HA_CENTRE);
    direction_x_edit->setVisible(false);

    main_layout->addChild(direction_x_edit);

    direction_y_edit = (CEGUI::Editbox*)wmgr.createWindow("OgreTray/Editbox");
    direction_y_edit->setSize(CEGUI::USize(CEGUI::UDim(0, 50), CEGUI::UDim(0, 30)));
    direction_y_edit->setVerticalAlignment(CEGUI::VA_BOTTOM);
    direction_y_edit->setHorizontalAlignment(CEGUI::HA_CENTRE);
    direction_y_edit->setVisible(false);

    main_layout->addChild(direction_y_edit);

    direction_z_edit = (CEGUI::Editbox*)wmgr.createWindow("OgreTray/Editbox");
    direction_z_edit->setSize(CEGUI::USize(CEGUI::UDim(0, 50), CEGUI::UDim(0, 30)));
    direction_z_edit->setVerticalAlignment(CEGUI::VA_BOTTOM);
    direction_z_edit->setHorizontalAlignment(CEGUI::HA_CENTRE);
    direction_z_edit->setVisible(false);

    main_layout->addChild(direction_z_edit);

    Normalize_direction = (CEGUI::PushButton*)wmgr.createWindow("OgreTray/Button");
    Normalize_direction->setText("Norm");
    Normalize_direction->setSize(CEGUI::USize(CEGUI::UDim(0, 50), CEGUI::UDim(0, 30)));
    Normalize_direction->setVerticalAlignment(CEGUI::VA_BOTTOM);
    Normalize_direction->setHorizontalAlignment(CEGUI::HA_CENTRE);
    Normalize_direction->setVisible(false);
    Normalize_direction->subscribeEvent(CEGUI::PushButton::EventClicked,
    		CEGUI::Event::Subscriber(&BallGame::NormalizeForceDirection, this));

    SetWindowsPosNearToOther(Quit, Editing, 0, 1);

    main_layout->addChild(Normalize_direction);

    ApplyToCase = (CEGUI::PushButton*)wmgr.createWindow("OgreTray/Button");
    ApplyToCase->setText("Apply");
    ApplyToCase->setSize(CEGUI::USize(CEGUI::UDim(0, 100), CEGUI::UDim(0, 30)));
    ApplyToCase->setVerticalAlignment(CEGUI::VA_BOTTOM);
    ApplyToCase->setHorizontalAlignment(CEGUI::HA_CENTRE);
    ApplyToCase->setVisible(false);
    ApplyToCase->subscribeEvent(CEGUI::PushButton::EventClicked,
    		CEGUI::Event::Subscriber(&BallGame::ApplyChangesToCaseCallback, this));

    main_layout->addChild(ApplyToCase);

    SetWindowsPosNearToOther(has_direction_button, ApplyToCase, 0, -1);
    SetWindowsPosNearToOther(has_force_button, has_direction_button, 0, -1);
    SetWindowsPosNearToOther(force_edit, has_force_button, 1, 0);
    SetWindowsPosNearToOther(direction_x_edit, has_direction_button, 1, 0);
    SetWindowsPosNearToOther(direction_y_edit, direction_x_edit, 1, 0);
    SetWindowsPosNearToOther(direction_z_edit, direction_y_edit, 1, 0);
    SetWindowsPosNearToOther(Normalize_direction, direction_z_edit, 1, 0);

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
				if(cmpt2 == 0 && cmpt == 5)
				{
					ogreEntity = mSceneMgr->createEntity("Rampe.mesh");
					type = CaseEntity::CaseType::typeRamp;
				}
				else
					ogreEntity = mSceneMgr->createEntity("Cube.mesh");
				SceneNode* ogreNode = mSceneMgr->getRootSceneNode()->createChildSceneNode(Vector3(cmpt * 50, cmpt2 * 50, 0));
				ogreNode->setScale(25, 25, 25);
				if(cmpt2 == 0 && cmpt == 5)
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
				CaseEntity *Entity = (CaseEntity*)NewtonBodyGetUserData(tableBody);
				if(cmpt == 0 && cmpt2 == 0)
				{
					dVector *direction = new dVector(1.0, 0.0, 0.0);
					Entity->SetForceToApply(100.0, direction);
						//					dVector *force = new dVector(0.0, 0.0, 1000.0);
				}
				if(cmpt == 4 && cmpt2 == 0)
				{
					Entity->SetForceToApply(100.0, NULL);
				}
				if(cmpt >= 10 && cmpt2 == 0)
				{
					dVector *direction = new dVector(0.0, 0.0, 1.0);
					Entity->SetForceToApply(1000.0, direction);
						//					dVector *force = new dVector(0.0, 0.0, 1000.0);
				}
				Entity->OgreEntity = ogreNode;
				ogreEntity->getUserObjectBindings().setUserAny(Ogre::Any(Entity));
				AddCase(tableBody);
		}
    }

    //////////////   ADD BALLS ///////////////////

	for(int cmpt = 0; cmpt < 1; cmpt++)
    {
		for(int cmpt2 = 0; cmpt2 < 1; cmpt2++)
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
			BallBody = WorldAddBall(m_world, 10, tsize, 0, ballmatrix);
			BallGameEntity *Entity = (BallGameEntity*)NewtonBodyGetUserData(BallBody);
			Entity->OgreEntity = ogreNode;
			ogreEntity->getUserObjectBindings().setUserAny(Ogre::Any(Entity));
			AddBall(BallBody);
		}
    }

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
		NewtonBody *ball = Balls[cmpt];
		if(ball == NULL)
			continue;
		for(int cmpt2 = 0; cmpt2 < Cases.GetSize(); cmpt2++)
		{
			NewtonBody *Case = Cases[cmpt2];
			if(Case == NULL)
				continue;
//          if(NewtonBodyFindContact(ball, Case) != NULL)
			if(CheckIfBodiesCollide(ball, Case) != NULL)
			{
				std::cout << ball << " id " << cmpt << " and " << Case << " id " << cmpt2 << " Collides by joints" << std::endl;
			}
			if(DoBodiesCollide(m_world, ball, Case))
			{
				CaseEntity *CEntity = (CaseEntity*)NewtonBodyGetUserData(Case);
				//if(!CEntity->CheckIfAlreadyColliding(ball))
				{
					CEntity->ApplyForceOnBall(ball);

					std::cout << ball << " id " << cmpt << " and " << Case << " id " << cmpt2 << " Collides" << std::endl;
				}
//				else
//					std::cout << ball << " id " << cmpt << " and " << Case << " id " << cmpt2 << " Already Colliding" << std::endl;
			}
		}
	}
}

void BallGame::AddBall(NewtonBody *ball)
{
	if(ball == NULL)
		return;
	int id = NewtonBodyGetID(ball), size = Balls.GetSize();
	while(size < id)
		Balls[size++] = NULL;
	Balls[id] = ball;
	size = id + 1;
	while(size < Balls.GetSize())
		Balls[size++] = NULL;
}

void BallGame::AddCase(NewtonBody *Wcase)
{
	if(Wcase == NULL)
		return;
	int id = NewtonBodyGetID(Wcase), size = Cases.GetSize();
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

	for(int cmpt = 0; cmpt < Balls.GetSize(); cmpt++)
	{
		NewtonBody *ball = Balls[cmpt];
		if(ball == NULL)
			continue;
		Vector3 worldPos = ((BallGameEntity*)NewtonBodyGetUserData(ball))->OgreEntity->getPosition();
		Vector3 hcsPosition = mCamera->getProjectionMatrix() * mCamera->getViewMatrix() * worldPos;
#define ECART 0.25
//#define ECART 0.8
		while(hcsPosition.x >= ECART)
		{
			MoveCam(1, 0, 0);
			hcsPosition = mCamera->getProjectionMatrix() * mCamera->getViewMatrix() * worldPos;
		}
		while(hcsPosition.x <= -1 * ECART)
		{
			MoveCam(-1, 0, 0);
			hcsPosition = mCamera->getProjectionMatrix() * mCamera->getViewMatrix() * worldPos;
		}
		while(hcsPosition.y >= ECART)
		{
			MoveCam(0, 1, 0);
			hcsPosition = mCamera->getProjectionMatrix() * mCamera->getViewMatrix() * worldPos;
		}
		while(hcsPosition.y <= - 1 * ECART)
		{
			MoveCam(0, -1, 0);
			hcsPosition = mCamera->getProjectionMatrix() * mCamera->getViewMatrix() * worldPos;
		}
	}

    return true;
}

bool BallGame::mouseMoved(const OIS::MouseEvent &arg)
{
    CEGUI::GUIContext& context = CEGUI::System::getSingleton().getDefaultGUIContext();
    context.injectMouseMove(arg.state.X.rel, arg.state.Y.rel);
    //BaseApplication::mouseMoved(arg);


	if(arg.state.Z.rel != 0)
		MoveCam(0,  0,  -10 * arg.state.Z.rel);
	if(mode == Editing)
	{
		if(LastHighligted != NULL)
		{
			if(UnderEditCase == NULL || UnderEditCase->OgreEntity != LastHighligted)
				LastHighligted->showBoundingBox(false);
			LastHighligted = NULL;
		}
		RaySceneQuery *mRayScanQuery = mSceneMgr->createRayQuery(Ogre::Ray());

		Real x, y;
		CEGUI::Vector2f mpos = CEGUI::System::getSingleton().getDefaultGUIContext().getMouseCursor().getPosition();
		// Will Mouse has not hit top left corner, there is a gap between OIS and CEGUI mouse coordinates. CEGUI is more reliable
		x = mpos.d_x / (float)mWindow->getWidth();
		y = mpos.d_y / (float)mWindow->getHeight();
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

bool BallGame::ApplyChangesToCaseCallback(const CEGUI::EventArgs &event)
{
	dVector *force_dir = NULL;
	if(UnderEditCase == NULL)
		return true;

	if(CaseHasForce == true)
	{
		UnderEditCaseForce = strtof(force_edit->getText().c_str(), NULL);
		if(force_directed == true)
		{
			float dir = strtof(direction_x_edit->getText().c_str(), NULL);
			force_direction.m_x = dir;
			dir = strtof(direction_y_edit->getText().c_str(), NULL);
			force_direction.m_y = dir;
			dir = strtof(direction_z_edit->getText().c_str(), NULL);
			force_direction.m_z = dir;
			force_dir = new dVector(force_direction.m_x, force_direction.m_y, force_direction.m_z);
		}
	}
	UnderEditCase->SetForceToApply(UnderEditCaseForce, force_dir);

	return true;
}

bool BallGame::UpdateEditButtonsCallback(const CEGUI::EventArgs &event)
{
	const CEGUI::WindowEventArgs &e = (const CEGUI::WindowEventArgs &)event;
	std::cout << "Update buttons by CE Callback of " << e.window->getName() << std::endl;
	has_force_button->setMutedState(true);
	has_direction_button->setMutedState(true);
	if(e.window->getName() == TOGGLE_FORCE_NAME)
	{
		if(has_force_button->isSelected())
			CaseHasForce = true;
		else
			CaseHasForce = false;
	}
	if(e.window->getName() == TOGGLE_FORCE_DIRECTION_NAME)
	{
		if(has_direction_button->isSelected())
			force_directed = true;
		else
			force_directed = false;
	}
	UpdateEditButtons();
	has_direction_button->setMutedState(false);
	has_force_button->setMutedState(false);
    return true;
}

void BallGame::UpdateEditButtons(void)
{
	std::cout << "Update Edit buttons"<< std::endl;
	has_force_button->setVisible(true);
	has_force_button->setDisabled(false);
	has_direction_button->setVisible(true);
	force_edit->setVisible(true);
	direction_x_edit->setVisible(true);
	direction_y_edit->setVisible(true);
	direction_z_edit->setVisible(true);
	Normalize_direction->setVisible(true);
	ApplyToCase->setVisible(true);
	ApplyToCase->setDisabled(false);

	if(CaseHasForce == false)
	{
		std::cout << "No Base Force"<< std::endl;
		has_force_button->setSelected(false);
		has_direction_button->setDisabled(true);
		has_direction_button->setSelected(false);
		force_edit->setDisabled(true);
		force_edit->setText("");
		direction_x_edit->setDisabled(true);
		direction_x_edit->setText("");
		direction_y_edit->setDisabled(true);
		direction_y_edit->setText("");
		direction_z_edit->setDisabled(true);
		direction_z_edit->setText("");
	}
	else
	{
		std::cout << "Base Force is present"<< std::endl;
		has_force_button->setSelected(true);
		has_direction_button->setDisabled(false);
		String ForceStr;
		char force_c[50];
		snprintf(force_c, 49, "%f", UnderEditCaseForce);
		ForceStr = force_c;
		std::cout << "Force " <<  UnderEditCaseForce << ", " << force_c << ", " << ForceStr << std::endl;
		force_edit->setDisabled(false);
		force_edit->setText(ForceStr);

		if(force_directed == false)
		{
			std::cout << "Not Directed Force"<< std::endl;
			has_direction_button->setSelected(false);
			direction_x_edit->setDisabled(true);
			direction_y_edit->setText("");
			direction_y_edit->setDisabled(true);
			direction_x_edit->setText("");
			direction_z_edit->setDisabled(true);
			direction_z_edit->setText("");
			Normalize_direction->setDisabled(true);

		}
		else
		{
			std::cout << "Directed Force"<< std::endl;
			has_direction_button->setSelected(true);
			direction_x_edit->setDisabled(false);
			direction_y_edit->setDisabled(false);
			direction_z_edit->setDisabled(false);
			snprintf(force_c, 49, "%f", force_direction.m_x);
			ForceStr = force_c;
			direction_x_edit->setText(ForceStr);
			snprintf(force_c, 49, "%f", force_direction.m_y);
			ForceStr = force_c;
			direction_y_edit->setText(ForceStr);
			snprintf(force_c, 49, "%f", force_direction.m_z);
			ForceStr = force_c;
			direction_z_edit->setText(ForceStr);
			Normalize_direction->setDisabled(false);
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
	has_force_button->setMutedState(true);
	has_direction_button->setMutedState(true);
	if(UnderEditCase != NULL)
	{
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
	}

	std::cout << "Update buttons by Mouse Pressed" << std::endl;
	UpdateEditButtons();
	has_direction_button->setMutedState(false);
	has_force_button->setMutedState(false);
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
			try
			{
				CaseEntity *Entity = Ogre::any_cast<CaseEntity*>(((Ogre::Entity*)LastHighligted->getAttachedObject(0))->getUserObjectBindings().getUserAny());
				if(Entity != NULL)
				{
					std::cout << "Edit Case by Mouse Pressed" << std::endl;
					EditCase(Entity);
				}
			}
			catch(Ogre::Exception &e)
			{
				//Ball Entity ?
				try
				{
					BallEntity *Entity = Ogre::any_cast<BallEntity*>(((Ogre::Entity*)LastHighligted->getAttachedObject(0))->getUserObjectBindings().getUserAny());
				}
				catch(Ogre::Exception &e)
				{
				}
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
		if(main_layout->isVisible())
			mRoot->queueEndRendering();
		else
			main_layout->setVisible(true);
	    break;
	case OIS::KeyCode::KC_UP:
	    MoveCam(0, 0, 10);
	    break;
	case OIS::KeyCode::KC_DOWN:
	    MoveCam(0, 0, -10);
	    break;
	case OIS::KeyCode::KC_LEFT:
	    MoveCam(-10, 0, 0);
	    break;
	case OIS::KeyCode::KC_RIGHT:
	    MoveCam(10, 0, 0);
	    break;
//	case OIS::KeyCode::KC_KP_8:
//	    CamPitch(10);
//	    break;
//	case OIS::KeyCode::KC_KP_5:
//	    CamPitch(-10);
//	    break;
//	case OIS::KeyCode::KC_KP_4:
//	    CamYaw(10);
//	    break;
//	case OIS::KeyCode::KC_KP_6:
//	    CamYaw(-10);
//	    break;
//	case OIS::KeyCode::KC_KP_7:
//	    CamRoll(10);
//	    break;
//	case OIS::KeyCode::KC_KP_9:
//	    CamRoll(-10);
//	    break;
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
