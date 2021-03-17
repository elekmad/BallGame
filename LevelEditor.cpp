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
#include "LevelEditor.h"


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

void inline ButtonsSetMutedState(std::list<CEGUI::Window*> &list, bool state)
{
	std::list<CEGUI::Window*>::iterator iter(list.begin());
	while(iter != list.end())
	{
		CEGUI::Window *W = *(iter++);
		if(W != NULL)
			W->setMutedState(state);
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

namespace BallGame {

bool LevelEditor::SaveStatePushBCallback(const CEGUI::EventArgs &e)
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

bool LevelEditor::LoadStatePushBCallback(const CEGUI::EventArgs &e)
{
	CEGUI::ListboxItem *item = ChooseStateToLoadB->getSelectedItem();
	if(item == NULL)
		return true;
	String *state_filename = (String*)item->getUserData();
	if(state_filename != NULL && state_filename->empty() == false)
		DeserializedPhysicScene(state_filename);
	return true;
}

bool LevelEditor::DelStatePushBCallback(const CEGUI::EventArgs &e)
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

bool LevelEditor::ChooseStateToLoadBCallback(const CEGUI::EventArgs &e)
{
	SetupStatesButtons();
	return true;
}

void LevelEditor::SetupStatesButtons(void)
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


LevelEditor::LevelEditor() :
	mThumbnailCamera(0),
	mThumbnailSceneMgr(0),
	rThumbnailtex(0),
	mImportLevelCamera(0),
	mImportLevelSceneMgr(0),
	rImportLeveltex(0)
{
	mRenderer = NULL;

	mWindow = NULL;
	ThumbnailWindow = NULL;
	ImportLevelWindow = NULL;
	MainLayout = NULL;
	LastHighlighted = NULL;
	LastHighlightedGroup = NULL;
	UnderEditCase = NULL;
	ForcesArrows = NULL;
	UnderEditBall = NULL;
	ToBePlacedEntity = NULL;
	LastPlacedEntity = NULL;
	ToBePlacedEntityType = NULL;
	ToBeDeletedEntity = NULL;
	ogreThumbnailNode = NULL;
	LoadStatePushB = NULL;
	ChooseStateToLoadB = NULL;
	DelStatePushB = NULL;
	SaveStatePushB = NULL;
	StatesBanner = NULL;
	LevelEditMode = Place;
	EntityEditMode = Simple;
	EntityEditAction = Move;
	LevelNameBanner = NULL;
	ImportLevelPushB = NULL;
	ImportLevelActivateInterfacePushB = NULL;
	ChooseLevelToImportComboB = NULL;
	EditingModeTitleBanner = NULL;
	MoveElementB = NULL;
	ScaleElementB = NULL;
	RotateElementB = NULL;
	EditElementB = NULL;
	PlaceNewElementB = NULL;
	DeleteElementB = NULL;
	GroupElementsB = NULL;
	CaractsEditElementB = NULL;
	SimpleEditElementB = NULL;
	MoveEditElementB = NULL;
	AddElementTitleBanner = NULL;
	ChooseTypeOfElementToAddB = NULL;
	ChooseMoveComboB = NULL;
	AddMoveStepPushB = NULL;
	DelMoveStepPushB = NULL;
	MoveRSpeedEditB = NULL;
	MoveRSpeedTitleB = NULL;
	MoveTSpeedEditB = NULL;
	MoveTSpeedTitleB = NULL;
	CorrelateSpeedsToggleB = NULL;
	MoveWaitTimeEditB = NULL;
	MoveWaitTimeTitleB = NULL;
	IsMoveTriggeredToggleB = NULL;
	ApplyToMoveStepPushB = NULL;
	UnderEditBall = NULL;
	UnderEditBallMass = NAN;
	BallMassValueEditB = NULL;
	ApplyMassChangesToBallPushB = NULL;
	ApplyForceChangesToCasePushB = NULL;
	NormalizeCaseForceDirectionPushB = NULL;
	UnderEditCaseForce = NAN;
	CaseHasForce = NULL;
	CaseForceValueEditB = NULL;
	CaseHasForceToggleB = NULL;
	CaseHasForceDirectionToggleB = NULL;
	CaseForceDirectionXValueEditB = NULL;
	CaseForceDirectionYValueEditB = NULL;
	CaseForceDirectionZValueEditB = NULL;
	StopPhysicPushB = NULL;
	StatesModePushB = NULL;
	EditModePushB = NULL;
	ChooseLevelComboB = NULL;
	NewLevelEditB = NULL;
	NewLevelCreateB = NULL;
	SaveLevelPushB = NULL;
	QuitPushB = NULL;
	MultiSelectionMode = false;
	mode = Running;
	MouseOverButton = false;
	force_directed = false;
	is_new_level = false;
}

void LevelEditor::chooseSceneManager(void)
{
	GameEngine::chooseSceneManager();
	mThumbnailSceneMgr = mRoot->createSceneManager(Ogre::ST_GENERIC);
	mImportLevelSceneMgr = mRoot->createSceneManager(Ogre::ST_GENERIC);
}

void LevelEditor::createCamera(void)
{
	GameEngine::createCamera();
	mThumbnailCamera = mThumbnailSceneMgr->createCamera("ThumbnailCam");
	mThumbnailCamera->setNearClipDistance(5);

	mImportLevelCamera = mImportLevelSceneMgr->createCamera("ImportLevelCam");
	mImportLevelCamera->setNearClipDistance(5);
}

void LevelEditor::createViewports(void)
{
	GameEngine::createViewports();

    pThumbnailtex = mRoot->getTextureManager()->createManual(
        "RTT",
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        Ogre::TEX_TYPE_2D,
        512,
        512,
        0,
        Ogre::PF_R8G8B8,
        Ogre::TU_RENDERTARGET);
    rThumbnailtex = pThumbnailtex->getBuffer()->getRenderTarget();


    Ogre::Viewport* vp = rThumbnailtex->addViewport(mThumbnailCamera);
	vp->setBackgroundColour(Ogre::ColourValue(0,0,0));

	// Alter the camera aspect ratio to match the viewport
	mThumbnailCamera->setAspectRatio(
		Ogre::Real(vp->getActualWidth()) / Ogre::Real(vp->getActualHeight()));

    pImportLeveltex = mRoot->getTextureManager()->createManual(
        "RTT",
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        Ogre::TEX_TYPE_2D,
		vp->getActualWidth(),
		vp->getActualHeight(),
        0,
        Ogre::PF_R8G8B8,
        Ogre::TU_RENDERTARGET);
    rImportLeveltex = pImportLeveltex->getBuffer()->getRenderTarget();


	vp = rImportLeveltex->addViewport(mImportLevelCamera);
	vp->setBackgroundColour(Ogre::ColourValue(0,0,0));

	// Alter the camera aspect ratio to match the viewport
	mImportLevelCamera->setAspectRatio(
		Ogre::Real(vp->getActualWidth()) / Ogre::Real(vp->getActualHeight()));
}

LevelEditor::~LevelEditor()
{
	EmptyLevel();
	EmptyLevelsList();
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

	if (mThumbnailCamera) delete mThumbnailCamera;
	if (mThumbnailSceneMgr)
	{
		mThumbnailSceneMgr->getRootSceneNode()->removeAndDestroyAllChildren();
		delete mThumbnailSceneMgr;
	}
	if (rThumbnailtex)delete rThumbnailtex;

	if (mImportLevelCamera) delete mImportLevelCamera;
	if (mImportLevelSceneMgr)
	{
		mImportLevelSceneMgr->getRootSceneNode()->removeAndDestroyAllChildren();
		delete mImportLevelSceneMgr;
	}
	if (rImportLeveltex)delete rImportLeveltex;
}

bool LevelEditor::EnteringArea(const CEGUI::EventArgs &event)
{
//	LOG << "Enter Button Area" << std::endl;
	MouseOverButton = true;
	return true;
}

bool LevelEditor::LeavingArea(const CEGUI::EventArgs &event)
{
//	LOG << "Leave Button Area" << std::endl;
	MouseOverButton = false;
	return true;
}

void LevelEditor::LoadBallGameEntityTypes(void)
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
		type->Type = strcmp(in["Type"].GetString(), "Case") == 0 ? Entity::Types::Case : Entity::Types::Ball;
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

void LevelEditor::createScene(void)
{
	LoadBallGameEntityTypes();

	SetupGUI();

	SetupGame();
}

bool LevelEditor::CaseForceValueEditBCallback(const CEGUI::EventArgs &event)
{
	UnderEditCaseForce = CEGUI::PropertyHelper<float>::fromString(CaseForceValueEditB->getText());
	return true;
}

bool LevelEditor::CaseForceDirectionXValueEditBMouseWheelCallback(const CEGUI::EventArgs &e)
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

bool LevelEditor::CaseForceDirectionYValueEditBMouseWheelCallback(const CEGUI::EventArgs &e)
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

bool LevelEditor::CaseForceDirectionZValueEditBMouseWheelCallback(const CEGUI::EventArgs &e)
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

void LevelEditor::NormalizeForceDirection(void)
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

bool LevelEditor::NormalizeCaseForceDirectionPushBCallback(const CEGUI::EventArgs &e)
{
	NormalizeForceDirection();
	return true;
}

bool LevelEditor::QuitPushBCallback(const CEGUI::EventArgs &e)
{
	mRoot->queueEndRendering();
    return true;
}

void LevelEditor::_StartPhysic(void)
{
	GameEngine::_StartPhysic();
	StopPhysicPushB->setText("Stop Physic");
}

void LevelEditor::_StopPhysic(void)
{
	GameEngine::_StopPhysic();
	StopPhysicPushB->setText("Start Physic");
}

bool LevelEditor::StopPhysicPushBCallback(const CEGUI::EventArgs &e)
{
	if(m_suspendPhysicsUpdate)
		_StartPhysic();
	else
		_StopPhysic();
    return true;
}

void LevelEditor::SwitchEditMode(void)
{
	if(mode == Running)
	{
		LOG << "Edit Mode" << std::endl;
		mode = Editing;
		_StopPhysic();
		ButtonsSetVisible(EditButtons, true);
	    GroupElementsB->setVisible(false);
		MoveEditElementB->setVisible(false);
	    CaractsEditElementB->setVisible(false);
	    SimpleEditElementB->setVisible(false);
		SetMoveNewElement();
		SaveLevelPushB->setEnabled(true);
	}
	else
	{
		LOG << "Running Mode" << std::endl;
		if(LastHighlighted != NULL)
		{
			LastHighlighted->DisplaySelectedBox(false);
			LastHighlighted = NULL;
			if(LastHighlightedGroup != NULL)
			{
				LastHighlightedGroup->DisplaySelectedBox(false);
				LastHighlightedGroup = NULL;
			}
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
		ogreThumbnailNode->removeAndDestroyAllChildren();
	}
}

void LevelEditor::BuildImportLevelWindowContent(Node *parent)
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
		CaseEntity *CaseE = *Citer;
		if(CaseE != NULL)
			delete CaseE;
		Citer = ImportLevelCases.erase(Citer);
	}

	std::list<BallEntity*>::iterator Biter(ImportLevelBalls.begin());
	while(Biter != ImportLevelBalls.end())
	{
		BallEntity *BallE = *Biter;
		if(BallE != NULL)
			delete BallE;
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

inline void LevelEditor::ActivateLevelImportInterface(void)
{
	ButtonsSetVisible(ImportLevelButtons, true);
	ButtonsMoveToFront(ImportLevelButtons);
	ButtonsMoveToBack(MainMenuButtons);
	ButtonsMoveToBack(EditButtons);
	ButtonsMoveToBack(EditBallButtons);
	ButtonsMoveToBack(EditCaseButtons);
	ImportLevelActivateInterfacePushB->setText("-");
}

inline void LevelEditor::UnactivateLevelImportInterface(void)
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

bool LevelEditor::ImportLevelActivateInterfacePushBCallback(const CEGUI::EventArgs &e)
{
	if(ImportLevelActivateInterfacePushB->getText() == "+")//Menu not activated
		ActivateLevelImportInterface();
	else
		UnactivateLevelImportInterface();
	return true;
}

bool LevelEditor::ImportLevelPushBCallback(const CEGUI::EventArgs &e)
{
	//Clear import level scene manager content
	BuildImportLevelWindowContent(NULL);
	//Re import content into main manager !
	BuildImportLevelWindowContent((Node*)mSceneMgr->getRootSceneNode());

	String GrpName = ImportLevelName;
	GrpName += "-" + std::to_string(nb_entities);
	GrpName += ":ImportGroup";
	GroupEntity *ImportGroup = new GroupEntity(GrpName, mSceneMgr);

	LOG << "Create " << GrpName << " Group for the Import" << std::endl;

	AddGroup(ImportGroup);

	std::list<CaseEntity*>::iterator Citer(ImportLevelCases.begin());
	while(Citer != ImportLevelCases.end())
	{
		CaseEntity *Case = *Citer;
		if(Case != NULL)
		{
//			LOG << "Import of Case " << Case << " '" << Case->getName() << "'" << std::endl;
			LOG << "Case " << Case->getName() << " Pos " << Case->getAbsolutePosition() << " Ori " << Case->getAbsoluteOrientation() << std::endl;
			GroupEntity *refMove = Case->getRefMove();
			Case->CreateNewtonBody(m_world);
			AddCase(Case);
			if(refMove == NULL)
				ImportGroup->AddChild(Case);
			else
			{
				AddGroup(refMove);
				ImportGroup->AddChild(refMove);
				ImportLevelGroups.remove(refMove);
				AddCaseToBeMoved(Case);
			}
			LOG << "Case " << Case->getName() << " Pos " << Case->getAbsolutePosition() << " Ori " << Case->getAbsoluteOrientation() << std::endl;
		}
		Citer = ImportLevelCases.erase(Citer);
	}

	std::list<GroupEntity*>::iterator Giter(ImportLevelGroups.begin());
	while(Giter != ImportLevelGroups.end())
	{
		GroupEntity *Group = *Giter;
		if(Group != NULL && Group->getisRefMove() == false)
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

	LastHighlightedGroup = ImportGroup;
	ImportGroup->ComputeAndEquilibrateChilds();
	AddGroupToBeEquilibrated(ImportGroup);

	FillMultiselectionSetWithGroup(LastHighlightedGroup);
	LOG << "Import Finished !!!!!" << std::endl;
	return true;
}

void LevelEditor::AddGroupToBeEquilibrated(GroupEntity *Grp)
{
	Grp->setForceRecomputeChilds();
	GroupsToBeEquilibrated.push_back(Grp);//AABB is only up to date after the SceneManager has called _update.
}

bool LevelEditor::ChooseLevelToImportComboBCallback(const CEGUI::EventArgs &e)
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

void LevelEditor::CreateThumbnail(String meshname)
{
	Ogre::Entity *ogreEntity = mThumbnailSceneMgr->createEntity(meshname);
	mThumbnailSceneMgr->getRootSceneNode()->removeAndDestroyAllChildren();
	ogreThumbnailNode = mThumbnailSceneMgr->getRootSceneNode()->createChildSceneNode();
	ogreThumbnailNode->attachObject(ogreEntity);
	ogreThumbnailNode->scale(130, 130, 130);
	ogreThumbnailNode->setPosition(270, 130, -30);
}

bool LevelEditor::ChooseTypeOfElementToAddBCallback(const CEGUI::EventArgs &e)
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

void LevelEditor::DeleteElement(void)
{
	if(LevelEditMode != Delete)
		return;
	if(ToBeDeletedEntity != NULL)
	{
		LOG << "Delete Entity : " << ToBeDeletedEntity->getName() << std::endl;
		if(ToBeDeletedEntity == LastHighlighted)
		{
			LastHighlighted = NULL;
			LastHighlightedGroup = NULL;
		}
		switch(ToBeDeletedEntity->getType())
		{
		case Entity::Types::Ball :
			DeleteBall((BallEntity*)ToBeDeletedEntity);
			break;
		case Entity::Types::Case :
			DeleteCase((CaseEntity*)ToBeDeletedEntity);
			break;
		}
		ToBeDeletedEntity = NULL;
	}
	std::list<BaseEntity*>::iterator delIter(UnderEditEntites.begin());
	while(delIter != UnderEditEntites.end())
	{
		BaseEntity *Entity = *delIter;
		if(Entity != NULL)
		{
			if(Entity == LastHighlighted)
			{
				LastHighlighted = NULL;
				LastHighlightedGroup = NULL;
			}
			switch(Entity->getType())
			{
			case Entity::Types::Ball :
				DeleteBall((BallEntity*)Entity);
				break;
			case Entity::Types::Case :
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

bool LevelEditor::DeleteElementBCallback(const CEGUI::EventArgs &e)
{
	ButtonsSetMutedState(EditButtons, true);
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
	MoveEditElementB->setVisible(false);
	MoveElementB->setVisible(false);
	RotateElementB->setVisible(false);
	ScaleElementB->setVisible(false);
	GroupElementsB->setVisible(false);
	ChooseTypeOfElementToAddB->setEnabled(false);
	ThumbnailWindow->setEnabled(false);
	ButtonsSetMutedState(EditButtons, false);

	return true;
}

bool LevelEditor::PlaceNewElementBCallback(const CEGUI::EventArgs &e)
{
	ButtonsSetMutedState(EditButtons, true);
	DeleteElementB->setDisabled(false);
	PlaceNewElementB->setDisabled(true);
	EditElementB->setDisabled(false);
	SimpleEditElementB->setVisible(false);
	CaractsEditElementB->setVisible(false);
	MoveEditElementB->setVisible(false);
	MoveElementB->setVisible(true);
	RotateElementB->setVisible(true);
	ScaleElementB->setVisible(true);
	GroupElementsB->setVisible(false);
	ChooseTypeOfElementToAddB->setEnabled(true);
	ThumbnailWindow->setEnabled(true);
	UnprepareNewElement();
	SetMoveNewElement();
	ButtonsSetMutedState(EditButtons, false);
	return true;
}

bool LevelEditor::EditElementBCallback(const CEGUI::EventArgs &e)
{
	EditElementSetupButtons();
	return true;
}

void LevelEditor::EditElementSetupButtons(void)
{
	ButtonsSetMutedState(EditButtons, true);
	DeleteElementB->setDisabled(false);
	PlaceNewElementB->setDisabled(false);
	EditElementB->setDisabled(true);
	SimpleEditElementB->setVisible(true);
	CaractsEditElementB->setVisible(true);
	MoveEditElementB->setVisible(true);
	MoveElementB->setVisible(true);
	RotateElementB->setVisible(true);
	ScaleElementB->setVisible(true);
	GroupElementsB->setVisible(false);
	ChooseTypeOfElementToAddB->setEnabled(false);
	ThumbnailWindow->setEnabled(false);
	SetMoveElement();
	ButtonsSetMutedState(EditButtons, false);
}

void LevelEditor::SetMoveElement(void)
{
	ButtonsSetMutedState(EditButtons, true);
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
	MoveEditElementB->setDisabled(false);
	EditElementB->setDisabled(true);
	MoveElementB->setDisabled(true);
	RotateElementB->setDisabled(false);
	ScaleElementB->setDisabled(false);
	ButtonsSetMutedState(EditButtons, false);
}

void LevelEditor::SetMoveNewElement(void)
{
	ButtonsSetMutedState(EditButtons, true);
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
	ButtonsSetMutedState(EditButtons, false);
}

inline void LevelEditor::ReSetUnderEditEntities(void)
{
	EditBall(UnderEditBall);
	EditCase(UnderEditCase);
	MultiSelectionSetEmpty();
	BaseEntity *UnderEdit = (BaseEntity*)UnderEditBall;
	if(UnderEdit == NULL)
		UnderEdit = (BaseEntity*)UnderEditCase;
	if(UnderEdit != NULL)
	{
		GroupEntity *Grp = UnderEdit->getGroup();
		if(UnderEdit->getType() == BaseEntity::Types::Case && ((CaseEntity*)UnderEdit)->getRefMove() != NULL)
			Grp = Grp->getGroup();
		if(Grp != NULL)
			FillMultiselectionSetWithGroup(Grp);
	}
}

bool LevelEditor::SimpleEditElementBCallback(const CEGUI::EventArgs &e)
{
	ButtonsSetMutedState(EditButtons, true);
	EntityEditMode = Simple;
	EntityEditAction = Move;
	ReSetUnderEditEntities();
	CaractsEditElementB->setEnabled(true);
	SimpleEditElementB->setEnabled(false);
	MoveEditElementB->setEnabled(true);

	MoveElementB->setVisible(true);
	MoveElementB->setEnabled(false);
	RotateElementB->setVisible(true);
	RotateElementB->setEnabled(true);
	ScaleElementB->setVisible(true);
	ScaleElementB->setEnabled(true);
	ButtonsSetMutedState(EditButtons, false);
	return true;
}

bool LevelEditor::MoveEditElementBCallback(const CEGUI::EventArgs &e)
{
	ButtonsSetMutedState(EditButtons, true);
	EntityEditMode = Moves;
	EntityEditAction = Move;
	ReSetUnderEditEntities();
	CaractsEditElementB->setEnabled(true);
	SimpleEditElementB->setEnabled(true);
	MoveEditElementB->setEnabled(false);

	MoveElementB->setVisible(true);
	MoveElementB->setEnabled(false);
	RotateElementB->setVisible(true);
	RotateElementB->setEnabled(true);
	ScaleElementB->setVisible(true);
	ScaleElementB->setEnabled(true);
	ButtonsSetMutedState(EditButtons, false);
	return true;
}

bool LevelEditor::CaractsEditElementBCallback(const CEGUI::EventArgs &e)
{
	ButtonsSetMutedState(EditButtons, true);
	EntityEditMode = Caracts;
	ReSetUnderEditEntities();
	CaractsEditElementB->setEnabled(false);
	SimpleEditElementB->setEnabled(true);
	MoveEditElementB->setEnabled(true);

	MoveElementB->setVisible(false);
	RotateElementB->setVisible(false);
	ScaleElementB->setVisible(false);
	ButtonsSetMutedState(EditButtons, false);
	return true;
}

inline void LevelEditor::SetMoveElementAction(void)
{
	EntityEditAction = Move;
	MoveElementB->setDisabled(true);
	RotateElementB->setDisabled(false);
	ScaleElementB->setDisabled(false);
}

bool LevelEditor::MoveElementBCallback(const CEGUI::EventArgs &e)
{
	SetMoveElementAction();
	return true;
}

inline void LevelEditor::SetRotateElementAction(void)
{
	EntityEditAction = Rotate;
	MoveElementB->setDisabled(false);
	RotateElementB->setDisabled(true);
	ScaleElementB->setDisabled(false);
}

bool LevelEditor::RotateElementBCallback(const CEGUI::EventArgs &e)
{
	SetRotateElementAction();
	return true;
}

inline void LevelEditor::SetScaleElementAction(void)
{
	EntityEditAction = Scale;
	MoveElementB->setDisabled(false);
	RotateElementB->setDisabled(false);
	ScaleElementB->setDisabled(true);
}

bool LevelEditor::ScaleElementBCallback(const CEGUI::EventArgs &e)
{
	SetScaleElementAction();
	return true;
}

inline void LevelEditor::UnprepareNewElement(void)
{
	if(ToBePlacedEntity != NULL)
	{
		LOG << "Unprepare new element : " << ToBePlacedEntity->getName() << std::endl;
		mSceneMgr->getRootSceneNode()->removeAndDestroyChild(ToBePlacedEntity->getName());
		delete ToBePlacedEntity;
		ToBePlacedEntity = NULL;
	}
}

inline void LevelEditor::UnprepareDeleteElement(void)
{
	if(ToBeDeletedEntity != NULL)
	{
		if(ToBeDeletedEntity != LastHighlighted)
			ToBeDeletedEntity->DisplaySelectedBox(false);
		ToBeDeletedEntity = NULL;
	}
}

inline void LevelEditor::PrepareDeleteElement(Entity *Entity)
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

void LevelEditor::PrepareNewElement(void)
{
	Ogre::Entity *ogreEntity;
	SceneNode *ogreNode;
	Vector3 Pos, Scale;
	Quaternion Orient;

	Pos = ToBePlacedEntityType->InitialPos;
	Scale = ToBePlacedEntityType->InitialScale;
	Orient = ToBePlacedEntityType->InitialOrientation;

	LOG << "Placing new element ?" << std::endl;
	switch(ToBePlacedEntityType->Type)
	{
	case Entity::Types::Case :
		ToBePlacedEntity = new CaseEntity();
		break;
	case Entity::Types::Ball :
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

	((Ogre::Entity*)ogreNode->getAttachedObject(0))->getUserObjectBindings().setUserAny(Ogre::Any(ToBePlacedEntity));
	ToBePlacedEntity->setOgreNode(ogreNode);
	ToBePlacedEntity->DisplaySelectedBox(true);
}

void LevelEditor::PlaceUnderEditElement(void)
{
	if(UnderEditBall != NULL)
		UnderEditBall->CreateNewtonBody(m_world);

	if(UnderEditCase != NULL)
		UnderEditCase->CreateNewtonBody(m_world);

	if(UnderEditEntites.empty() == false)
	{
		std::list<GroupEntity*> GroupsToBeMoved;
		std::list<BaseEntity*>::iterator iter(UnderEditEntites.begin());
		while(iter != UnderEditEntites.end())
		{
			GroupEntity *Grp;
			BaseEntity *Entity = *(iter++);
			if(Entity == NULL)
				continue;
			Entity->copyOgreToInitial();
			Grp = Entity->getGroup();
			if(Grp != NULL)
			{
				bool found = false;
				std::list<GroupEntity*>::iterator Giter(GroupsToBeMoved.begin());
				while(Giter != GroupsToBeMoved.end())
				{
					GroupEntity *G = *(Giter++);
					if(G == Grp)
					{
						found = true;
						break;
					}
				}
				if(found == false)
					GroupsToBeMoved.push_back(Grp);
			}
			Entity->CreateNewtonBody(m_world);
		}
		std::list<GroupEntity*>::iterator iterG(GroupsToBeMoved.begin());
		while(iterG != GroupsToBeMoved.end())
		{
			GroupEntity *Grp = *(iterG++);
			if(Grp != NULL)
				Grp->copyOgreToInitial();
		}
	}
}

void LevelEditor::PlaceNewElement(void)
{
	PlaceElement(ToBePlacedEntity);
	LastPlacedEntity = ToBePlacedEntity;
	PrepareNewElement();
}

void LevelEditor::PlaceElement(Entity *ToBePlaced)
{
	if(ToBePlaced == NULL)
		return;
	LOG << "Placing new element !" << std::endl;

	ToBePlaced->DisplaySelectedBox(false);

	switch(ToBePlaced->getType())
	{
	case Entity::Types::Case :
		((CaseEntity*)ToBePlaced)->CreateNewtonBody(m_world);
		AddCase((CaseEntity*)ToBePlaced);
		break;
	case Entity::Types::Ball :
		((BallEntity*)ToBePlaced)->CreateNewtonBody(m_world);
		AddBall((BallEntity*)ToBePlaced);
		break;
	}
}

bool LevelEditor::GroupElementsBCallback(const CEGUI::EventArgs &e)
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
	std:list<BaseEntity*>::iterator iter(UnderEditEntites.begin());
	while(iter != UnderEditEntites.end())
	{
		BaseEntity *Entity = *(iter++);
		GroupEntity *old;
		if(Entity == NULL)
			continue;
		if((old = Entity->getGroup()) != NULL)
		{
			if(Entity->getType() == BaseEntity::Types::Case)
			{
				if(((CaseEntity*)Entity)->getRefMove() == old)
				{
					if(Grp != NULL)
						Grp->AddChild(old);
					continue;
				}
			}
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

bool LevelEditor::EditModePushBCallback(const CEGUI::EventArgs &e)
{
	SwitchEditMode();
    return true;
}

bool LevelEditor::StatesModePushBCallback(const CEGUI::EventArgs &e)
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

template<typename T> T* LevelEditor::CreateNewGUIComponent(std::string &TypeName, std::string &Name)
{
    return CreateNewGUIComponent<T>(TypeName.c_str(), Name.c_str());
}

template<typename T> T* LevelEditor::CreateNewGUIComponent(const char *TypeName, const char *Name)
{
    CEGUI::WindowManager &wmgr = CEGUI::WindowManager::getSingleton();
	T* ret = (T*)wmgr.createWindow(TypeName, Name);
    ret->subscribeEvent(T::EventMouseEntersArea,
			CEGUI::Event::Subscriber(&LevelEditor::EnteringArea, this));
    ret->subscribeEvent(T::EventMouseLeavesArea,
			CEGUI::Event::Subscriber(&LevelEditor::LeavingArea, this));
    ret->setVisible(false);
    return ret;
}

#define ButtonSetAddButton(S, B) S.push_back((CEGUI::Window*)B)

void LevelEditor::SetupGUI(void)
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
    		CEGUI::Event::Subscriber(&LevelEditor::StopPhysicPushBCallback, this));

    EditModePushB = CreateNewGUIComponent<CEGUI::PushButton>("OgreTray/Button");
    EditModePushB->setText("Edit");
    EditModePushB->setSize(CEGUI::USize(CEGUI::UDim(0, 75), CEGUI::UDim(0, 30)));

    MainLayout->addChild(EditModePushB);
    ButtonSetAddButton(MainMenuButtons, EditModePushB);

    EditModePushB->subscribeEvent(CEGUI::PushButton::EventClicked,
    		CEGUI::Event::Subscriber(&LevelEditor::EditModePushBCallback, this));

    SetWindowsPosNearToOther(EditModePushB, StopPhysicPushB, 0, 1);

    StatesModePushB = CreateNewGUIComponent<CEGUI::PushButton>("OgreTray/Button");
    StatesModePushB->setText("States");
    StatesModePushB->setSize(CEGUI::USize(CEGUI::UDim(0, 75), CEGUI::UDim(0, 30)));

    MainLayout->addChild(StatesModePushB);
    ButtonSetAddButton(MainMenuButtons, StatesModePushB);

    StatesModePushB->subscribeEvent(CEGUI::PushButton::EventClicked,
    		CEGUI::Event::Subscriber(&LevelEditor::StatesModePushBCallback, this));

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
    		CEGUI::Event::Subscriber(&LevelEditor::ChooseLevelComboBCallback, this));

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
			CEGUI::Event::Subscriber(&LevelEditor::NewLevelCreateBCallback, this));

    SetWindowsPosNearToOther(NewLevelCreateB, NewLevelEditB, 1, 0);

    SaveLevelPushB = CreateNewGUIComponent<CEGUI::PushButton>("OgreTray/Button");
    SaveLevelPushB->setText("Save");
    SaveLevelPushB->setSize(CEGUI::USize(CEGUI::UDim(0, 150), CEGUI::UDim(0, 30)));
    SaveLevelPushB->setEnabled(false);

    MainLayout->addChild(SaveLevelPushB);
    ButtonSetAddButton(MainMenuButtons, SaveLevelPushB);

    SaveLevelPushB->subscribeEvent(CEGUI::PushButton::EventClicked,
    		CEGUI::Event::Subscriber(&LevelEditor::SaveLevelPushBCallback, this));

    SetWindowsPosNearToOther(SaveLevelPushB, NewLevelEditB, 0, 1);

    QuitPushB = CreateNewGUIComponent<CEGUI::PushButton>("OgreTray/Button");
    QuitPushB->setText("Quit");
    QuitPushB->setTooltipText("Quit");
    QuitPushB->setSize(CEGUI::USize(CEGUI::UDim(0, 150), CEGUI::UDim(0, 30)));

    MainLayout->addChild(QuitPushB);
    ButtonSetAddButton(MainMenuButtons, QuitPushB);

    QuitPushB->subscribeEvent(CEGUI::PushButton::EventClicked,
    		CEGUI::Event::Subscriber(&LevelEditor::QuitPushBCallback, this));

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
    		CEGUI::Event::Subscriber(&LevelEditor::ChooseStateToLoadBCallback, this));
    ChooseStateToLoadB->subscribeEvent(CEGUI::Combobox::EventListContentsChanged,
    		CEGUI::Event::Subscriber(&LevelEditor::ChooseStateToLoadBCallback, this));
    SetWindowsPosNearToOther(ChooseStateToLoadB, StatesBanner, 0, 1);

    LoadStatePushB = CreateNewGUIComponent<CEGUI::PushButton>("OgreTray/Button");
    LoadStatePushB->setText("Load");
    LoadStatePushB->setSize(CEGUI::USize(CEGUI::UDim(0, 150), CEGUI::UDim(0, 30)));
    LoadStatePushB->setVerticalAlignment(CEGUI::VA_TOP);
    LoadStatePushB->setHorizontalAlignment(CEGUI::HA_RIGHT);

    MainLayout->addChild(LoadStatePushB);
    ButtonSetAddButton(StatesButtons, LoadStatePushB);
    LoadStatePushB->subscribeEvent(CEGUI::PushButton::EventClicked,
    		CEGUI::Event::Subscriber(&LevelEditor::LoadStatePushBCallback, this));
    SetWindowsPosNearToOther(LoadStatePushB, StatesBanner, 0, 2);

    DelStatePushB = CreateNewGUIComponent<CEGUI::PushButton>("OgreTray/Button");
    DelStatePushB->setText("Delete");
    DelStatePushB->setSize(CEGUI::USize(CEGUI::UDim(0, 150), CEGUI::UDim(0, 30)));
    DelStatePushB->setVerticalAlignment(CEGUI::VA_TOP);
    DelStatePushB->setHorizontalAlignment(CEGUI::HA_RIGHT);

    MainLayout->addChild(DelStatePushB);
    ButtonSetAddButton(StatesButtons, DelStatePushB);
    DelStatePushB->subscribeEvent(CEGUI::PushButton::EventClicked,
    		CEGUI::Event::Subscriber(&LevelEditor::DelStatePushBCallback, this));
    SetWindowsPosNearToOther(DelStatePushB, LoadStatePushB, 0, 1);

    SaveStatePushB = CreateNewGUIComponent<CEGUI::PushButton>("OgreTray/Button");
    SaveStatePushB->setText("Save");
    SaveStatePushB->setSize(CEGUI::USize(CEGUI::UDim(0, 150), CEGUI::UDim(0, 30)));
    SaveStatePushB->setVerticalAlignment(CEGUI::VA_TOP);
    SaveStatePushB->setHorizontalAlignment(CEGUI::HA_RIGHT);

    MainLayout->addChild(SaveStatePushB);
    ButtonSetAddButton(StatesButtons, SaveStatePushB);
    SaveStatePushB->subscribeEvent(CEGUI::PushButton::EventClicked,
    		CEGUI::Event::Subscriber(&LevelEditor::SaveStatePushBCallback, this));
    SetWindowsPosNearToOther(SaveStatePushB, DelStatePushB, 0, 1);

    //Now LevelNameBanner exist, we can call SetLevel !
    if(actuallevel != NULL)
    {
		String levelname(actuallevel->getText().c_str());
		String *levelfilename = (String*)actuallevel->getUserData();
		SetLevel(levelname, *levelfilename);
    }


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
    		CEGUI::Event::Subscriber(&LevelEditor::ImportLevelActivateInterfacePushBCallback, this));

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
    		CEGUI::Event::Subscriber(&LevelEditor::ChooseLevelToImportComboBCallback, this));

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
    		CEGUI::Event::Subscriber(&LevelEditor::ImportLevelPushBCallback, this));
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
    		CEGUI::Event::Subscriber(&LevelEditor::ChooseTypeOfElementToAddBCallback, this));

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
			CEGUI::Event::Subscriber(&LevelEditor::PlaceNewElementBCallback, this));

    MainLayout->addChild(PlaceNewElementB);
    ButtonSetAddButton(EditButtons, PlaceNewElementB);

    EditElementB = CreateNewGUIComponent<CEGUI::PushButton>("OgreTray/Button");
    EditElementB->setText("Edit");
    EditElementB->setSize(CEGUI::USize(CEGUI::UDim(0, 75), CEGUI::UDim(0, 30)));
    EditElementB->setVerticalAlignment(CEGUI::VA_TOP);
    EditElementB->setHorizontalAlignment(CEGUI::HA_RIGHT);

    EditElementB->subscribeEvent(CEGUI::PushButton::EventClicked,
			CEGUI::Event::Subscriber(&LevelEditor::EditElementBCallback, this));

    MainLayout->addChild(EditElementB);
    ButtonSetAddButton(EditButtons, EditElementB);

    DeleteElementB = CreateNewGUIComponent<CEGUI::PushButton>("OgreTray/Button");
    DeleteElementB->setText("Delete");
    DeleteElementB->setSize(CEGUI::USize(CEGUI::UDim(0, 150), CEGUI::UDim(0, 30)));
    DeleteElementB->setVerticalAlignment(CEGUI::VA_TOP);
    DeleteElementB->setHorizontalAlignment(CEGUI::HA_RIGHT);

    DeleteElementB->subscribeEvent(CEGUI::PushButton::EventClicked,
			CEGUI::Event::Subscriber(&LevelEditor::DeleteElementBCallback, this));

    MainLayout->addChild(DeleteElementB);
    ButtonSetAddButton(EditButtons, DeleteElementB);


    SimpleEditElementB = CreateNewGUIComponent<CEGUI::PushButton>("OgreTray/Button");
    SimpleEditElementB->setText("S");
    SimpleEditElementB->setTooltipText("Simple Edit Element.");
    SimpleEditElementB->setSize(CEGUI::USize(CEGUI::UDim(0, 50), CEGUI::UDim(0, 30)));
    SimpleEditElementB->setVerticalAlignment(CEGUI::VA_TOP);
    SimpleEditElementB->setHorizontalAlignment(CEGUI::HA_RIGHT);

    SimpleEditElementB->subscribeEvent(CEGUI::PushButton::EventClicked,
			CEGUI::Event::Subscriber(&LevelEditor::SimpleEditElementBCallback, this));

    MainLayout->addChild(SimpleEditElementB);
    ButtonSetAddButton(EditButtons, SimpleEditElementB);

    MoveEditElementB = CreateNewGUIComponent<CEGUI::PushButton>("OgreTray/Button");
    MoveEditElementB->setText("M");
    MoveEditElementB->setTooltipText("Moves Edit Element.");
    MoveEditElementB->setSize(CEGUI::USize(CEGUI::UDim(0, 50), CEGUI::UDim(0, 30)));
    MoveEditElementB->setVerticalAlignment(CEGUI::VA_TOP);
    MoveEditElementB->setHorizontalAlignment(CEGUI::HA_RIGHT);

    MoveEditElementB->subscribeEvent(CEGUI::PushButton::EventClicked,
			CEGUI::Event::Subscriber(&LevelEditor::MoveEditElementBCallback, this));

    MainLayout->addChild(MoveEditElementB);
    ButtonSetAddButton(EditButtons, MoveEditElementB);

    CaractsEditElementB = CreateNewGUIComponent<CEGUI::PushButton>("OgreTray/Button");
    CaractsEditElementB->setText("C");
    CaractsEditElementB->setTooltipText("Caracts Edit Element.");
    CaractsEditElementB->setSize(CEGUI::USize(CEGUI::UDim(0, 50), CEGUI::UDim(0, 30)));
    CaractsEditElementB->setVerticalAlignment(CEGUI::VA_TOP);
    CaractsEditElementB->setHorizontalAlignment(CEGUI::HA_RIGHT);

    CaractsEditElementB->subscribeEvent(CEGUI::PushButton::EventClicked,
			CEGUI::Event::Subscriber(&LevelEditor::CaractsEditElementBCallback, this));

    MainLayout->addChild(CaractsEditElementB);
    ButtonSetAddButton(EditButtons, CaractsEditElementB);

    MoveElementB = CreateNewGUIComponent<CEGUI::PushButton>("OgreTray/Button");
    MoveElementB->setText("M");
    MoveElementB->setTooltipText("Move Element.");
    MoveElementB->setSize(CEGUI::USize(CEGUI::UDim(0, 50), CEGUI::UDim(0, 30)));
    MoveElementB->setVerticalAlignment(CEGUI::VA_TOP);
    MoveElementB->setHorizontalAlignment(CEGUI::HA_RIGHT);

    MoveElementB->subscribeEvent(CEGUI::PushButton::EventClicked,
			CEGUI::Event::Subscriber(&LevelEditor::MoveElementBCallback, this));

    MainLayout->addChild(MoveElementB);
    ButtonSetAddButton(EditButtons, MoveElementB);

    RotateElementB = CreateNewGUIComponent<CEGUI::PushButton>("OgreTray/Button");
    RotateElementB->setText("R");
    RotateElementB->setTooltipText("Rotate Element.");
    RotateElementB->setSize(CEGUI::USize(CEGUI::UDim(0, 50), CEGUI::UDim(0, 30)));
    RotateElementB->setVerticalAlignment(CEGUI::VA_TOP);
    RotateElementB->setHorizontalAlignment(CEGUI::HA_RIGHT);

    RotateElementB->subscribeEvent(CEGUI::PushButton::EventClicked,
			CEGUI::Event::Subscriber(&LevelEditor::RotateElementBCallback, this));

    MainLayout->addChild(RotateElementB);
    ButtonSetAddButton(EditButtons, RotateElementB);


    ScaleElementB = CreateNewGUIComponent<CEGUI::PushButton>("OgreTray/Button");
    ScaleElementB->setText("S");
    ScaleElementB->setTooltipText("Scale Element.");
    ScaleElementB->setSize(CEGUI::USize(CEGUI::UDim(0, 50), CEGUI::UDim(0, 30)));
    ScaleElementB->setVerticalAlignment(CEGUI::VA_TOP);
    ScaleElementB->setHorizontalAlignment(CEGUI::HA_RIGHT);

    ScaleElementB->subscribeEvent(CEGUI::PushButton::EventClicked,
			CEGUI::Event::Subscriber(&LevelEditor::ScaleElementBCallback, this));

    MainLayout->addChild(ScaleElementB);
    ButtonSetAddButton(EditButtons, ScaleElementB);


    GroupElementsB = CreateNewGUIComponent<CEGUI::ToggleButton>("OgreTray/Checkbox");
    GroupElementsB->setText("Grouped");
    GroupElementsB->setSize(CEGUI::USize(CEGUI::UDim(0, 150), CEGUI::UDim(0, 30)));
    GroupElementsB->setVerticalAlignment(CEGUI::VA_TOP);
    GroupElementsB->setHorizontalAlignment(CEGUI::HA_RIGHT);

    GroupElementsB->subscribeEvent(CEGUI::ToggleButton::EventSelectStateChanged,
			CEGUI::Event::Subscriber(&LevelEditor::GroupElementsBCallback, this));

    MainLayout->addChild(GroupElementsB);
    ButtonSetAddButton(EditButtons, GroupElementsB);

    SetWindowsPosNearToOther(ChooseTypeOfElementToAddB, AddElementTitleBanner, 0, 1);
    SetWindowsPosNearToOther(ThumbnailWindow, AddElementTitleBanner, 0, 2);
    SetWindowsPosNearToOther(EditElementB, ThumbnailWindow, 0, 1);
    SetWindowsPosNearToOther(PlaceNewElementB, EditElementB, -1, 0);
    SetWindowsPosNearToOther(DeleteElementB, EditElementB, 0, 1);
    SetWindowsPosNearToOther(SimpleEditElementB, DeleteElementB, 0, 1);
    SetWindowsPosNearToOther(CaractsEditElementB, SimpleEditElementB, -1, 0);
    SetWindowsPosNearToOther(MoveEditElementB, CaractsEditElementB, -1, 0);
    SetWindowsPosNearToOther(ScaleElementB, SimpleEditElementB, 0, 1);
    SetWindowsPosNearToOther(RotateElementB, ScaleElementB, -1, 0);
    SetWindowsPosNearToOther(MoveElementB, RotateElementB, -1, 0);
    SetWindowsPosNearToOther(GroupElementsB, DeleteElementB, -1, 2);


    /// Edit Case GUI

    CaseHasForceToggleB = CreateNewGUIComponent<CEGUI::ToggleButton>("OgreTray/Checkbox");
    CaseHasForceToggleB->setText("Has Force");
    CaseHasForceToggleB->setSelected(false);
    CaseHasForceToggleB->setSize(CEGUI::USize(CEGUI::UDim(0, 200), CEGUI::UDim(0, 30)));
    CaseHasForceToggleB->setVerticalAlignment(CEGUI::VA_BOTTOM);
    CaseHasForceToggleB->setHorizontalAlignment(CEGUI::HA_CENTRE);

    CaseHasForceToggleB->subscribeEvent(CEGUI::ToggleButton::EventSelectStateChanged,
			CEGUI::Event::Subscriber(&LevelEditor::CaseHasForceToggleBCallback, this));

    MainLayout->addChild(CaseHasForceToggleB);
    ButtonSetAddButton(EditCaseButtons, CaseHasForceToggleB);

    CaseForceValueEditB = CreateNewGUIComponent<CEGUI::Editbox>("OgreTray/Editbox");
    CaseForceValueEditB->setSize(CEGUI::USize(CEGUI::UDim(0, 50), CEGUI::UDim(0, 30)));
    CaseForceValueEditB->setVerticalAlignment(CEGUI::VA_BOTTOM);
    CaseForceValueEditB->setHorizontalAlignment(CEGUI::HA_CENTRE);
    String FloatingNumRegex("^(\\-?[0-9]*(\\.[0-9]*)?|NAN)?");
    CaseForceValueEditB->setValidationString(FloatingNumRegex);
    String IntegerNumRegex("^(\\-?[0-9]*(\\.[0-9]*)?)?");
    CaseForceValueEditB->setValidationString(FloatingNumRegex);

    CaseForceValueEditB->subscribeEvent(CEGUI::Editbox::EventTextAccepted,
			CEGUI::Event::Subscriber(&LevelEditor::CaseForceValueEditBCallback, this));
    CaseForceValueEditB->subscribeEvent(CEGUI::Editbox::EventMouseLeavesArea,
			CEGUI::Event::Subscriber(&LevelEditor::CaseForceValueEditBCallback, this));

    MainLayout->addChild(CaseForceValueEditB);
    ButtonSetAddButton(EditCaseButtons, CaseForceValueEditB);

    CaseHasForceDirectionToggleB = CreateNewGUIComponent<CEGUI::ToggleButton>("OgreTray/Checkbox");
    CaseHasForceDirectionToggleB->setText("Has Force Directed");
    CaseHasForceDirectionToggleB->setSelected(false);
    CaseHasForceDirectionToggleB->setSize(CEGUI::USize(CEGUI::UDim(0, 200), CEGUI::UDim(0, 30)));
    CaseHasForceDirectionToggleB->setVerticalAlignment(CEGUI::VA_BOTTOM);
    CaseHasForceDirectionToggleB->setHorizontalAlignment(CEGUI::HA_CENTRE);

	CaseHasForceDirectionToggleB->subscribeEvent(CEGUI::ToggleButton::EventSelectStateChanged,
			CEGUI::Event::Subscriber(&LevelEditor::CaseHasForceDirectionToggleBCallback, this));

    MainLayout->addChild(CaseHasForceDirectionToggleB);
    ButtonSetAddButton(EditCaseButtons, CaseHasForceDirectionToggleB);

    CaseForceDirectionXValueEditB = CreateNewGUIComponent<CEGUI::Editbox>("OgreTray/Editbox");
    CaseForceDirectionXValueEditB->setSize(CEGUI::USize(CEGUI::UDim(0, 50), CEGUI::UDim(0, 30)));
    CaseForceDirectionXValueEditB->setVerticalAlignment(CEGUI::VA_BOTTOM);
    CaseForceDirectionXValueEditB->setHorizontalAlignment(CEGUI::HA_CENTRE);
    CaseForceDirectionXValueEditB->setValidationString(FloatingNumRegex);

    CaseForceDirectionXValueEditB->subscribeEvent(CEGUI::Editbox::EventMouseWheel,
			CEGUI::Event::Subscriber(&LevelEditor::CaseForceDirectionXValueEditBMouseWheelCallback, this));


    MainLayout->addChild(CaseForceDirectionXValueEditB);
    ButtonSetAddButton(EditCaseButtons, CaseForceDirectionXValueEditB);

    CaseForceDirectionYValueEditB = CreateNewGUIComponent<CEGUI::Editbox>("OgreTray/Editbox");
    CaseForceDirectionYValueEditB->setSize(CEGUI::USize(CEGUI::UDim(0, 50), CEGUI::UDim(0, 30)));
    CaseForceDirectionYValueEditB->setVerticalAlignment(CEGUI::VA_BOTTOM);
    CaseForceDirectionYValueEditB->setHorizontalAlignment(CEGUI::HA_CENTRE);
    CaseForceDirectionYValueEditB->setValidationString(FloatingNumRegex);

    CaseForceDirectionYValueEditB->subscribeEvent(CEGUI::Editbox::EventMouseWheel,
			CEGUI::Event::Subscriber(&LevelEditor::CaseForceDirectionYValueEditBMouseWheelCallback, this));


    MainLayout->addChild(CaseForceDirectionYValueEditB);
    ButtonSetAddButton(EditCaseButtons, CaseForceDirectionYValueEditB);

    CaseForceDirectionZValueEditB = CreateNewGUIComponent<CEGUI::Editbox>("OgreTray/Editbox");
    CaseForceDirectionZValueEditB->setSize(CEGUI::USize(CEGUI::UDim(0, 50), CEGUI::UDim(0, 30)));
    CaseForceDirectionZValueEditB->setVerticalAlignment(CEGUI::VA_BOTTOM);
    CaseForceDirectionZValueEditB->setHorizontalAlignment(CEGUI::HA_CENTRE);
    CaseForceDirectionZValueEditB->setValidationString(FloatingNumRegex);

    CaseForceDirectionZValueEditB->subscribeEvent(CEGUI::Editbox::EventMouseWheel,
			CEGUI::Event::Subscriber(&LevelEditor::CaseForceDirectionZValueEditBMouseWheelCallback, this));


    MainLayout->addChild(CaseForceDirectionZValueEditB);
    ButtonSetAddButton(EditCaseButtons, CaseForceDirectionZValueEditB);

    NormalizeCaseForceDirectionPushB = CreateNewGUIComponent<CEGUI::PushButton>("OgreTray/Button");
    NormalizeCaseForceDirectionPushB->setText("Norm");
    NormalizeCaseForceDirectionPushB->setSize(CEGUI::USize(CEGUI::UDim(0, 50), CEGUI::UDim(0, 30)));
    NormalizeCaseForceDirectionPushB->setVerticalAlignment(CEGUI::VA_BOTTOM);
    NormalizeCaseForceDirectionPushB->setHorizontalAlignment(CEGUI::HA_CENTRE);
    NormalizeCaseForceDirectionPushB->subscribeEvent(CEGUI::PushButton::EventClicked,
    		CEGUI::Event::Subscriber(&LevelEditor::NormalizeCaseForceDirectionPushBCallback, this));


    MainLayout->addChild(NormalizeCaseForceDirectionPushB);
    ButtonSetAddButton(EditCaseButtons, NormalizeCaseForceDirectionPushB);

    ApplyForceChangesToCasePushB = CreateNewGUIComponent<CEGUI::PushButton>("OgreTray/Button");
    ApplyForceChangesToCasePushB->setText("Apply");
    ApplyForceChangesToCasePushB->setSize(CEGUI::USize(CEGUI::UDim(0, 100), CEGUI::UDim(0, 30)));
    ApplyForceChangesToCasePushB->setVerticalAlignment(CEGUI::VA_BOTTOM);
    ApplyForceChangesToCasePushB->setHorizontalAlignment(CEGUI::HA_CENTRE);

    ApplyForceChangesToCasePushB->subscribeEvent(CEGUI::PushButton::EventClicked,
    		CEGUI::Event::Subscriber(&LevelEditor::ApplyForceChangesToCasePushBCallback, this));

    MainLayout->addChild(ApplyForceChangesToCasePushB);
    ButtonSetAddButton(EditCaseButtons, ApplyForceChangesToCasePushB);

    SetWindowsPosNearToOther(CaseHasForceDirectionToggleB, ApplyForceChangesToCasePushB, 0, -1);
    SetWindowsPosNearToOther(CaseHasForceToggleB, CaseHasForceDirectionToggleB, 0, -1);
    SetWindowsPosNearToOther(CaseForceValueEditB, CaseHasForceToggleB, 1, 0);
    SetWindowsPosNearToOther(CaseForceDirectionXValueEditB, CaseHasForceDirectionToggleB, 1, 0);
    SetWindowsPosNearToOther(CaseForceDirectionYValueEditB, CaseForceDirectionXValueEditB, 1, 0);
    SetWindowsPosNearToOther(CaseForceDirectionZValueEditB, CaseForceDirectionYValueEditB, 1, 0);
    SetWindowsPosNearToOther(NormalizeCaseForceDirectionPushB, CaseForceDirectionZValueEditB, 1, 0);

    /// Edit Moves GUI

    AddMoveStepPushB = CreateNewGUIComponent<CEGUI::PushButton>("OgreTray/Button");
    AddMoveStepPushB->setText("+");
    AddMoveStepPushB->setSize(CEGUI::USize(CEGUI::UDim(0, 30), CEGUI::UDim(0, 30)));
    AddMoveStepPushB->setVerticalAlignment(CEGUI::VA_TOP);
    AddMoveStepPushB->setHorizontalAlignment(CEGUI::HA_RIGHT);
    AddMoveStepPushB->subscribeEvent(CEGUI::PushButton::EventClicked,
    		CEGUI::Event::Subscriber(&LevelEditor::AddMoveStepPushBCallback, this));

    MainLayout->addChild(AddMoveStepPushB);
    ButtonSetAddButton(EditMovesButtons, AddMoveStepPushB);

    DelMoveStepPushB = CreateNewGUIComponent<CEGUI::PushButton>("OgreTray/Button");
    DelMoveStepPushB->setText("-");
    DelMoveStepPushB->setSize(CEGUI::USize(CEGUI::UDim(0, 30), CEGUI::UDim(0, 30)));
    DelMoveStepPushB->setVerticalAlignment(CEGUI::VA_TOP);
    DelMoveStepPushB->setHorizontalAlignment(CEGUI::HA_RIGHT);
    DelMoveStepPushB->subscribeEvent(CEGUI::PushButton::EventClicked,
    		CEGUI::Event::Subscriber(&LevelEditor::DelMoveStepPushBCallback, this));

    MainLayout->addChild(DelMoveStepPushB);
    ButtonSetAddButton(EditMovesButtons, DelMoveStepPushB);

    ChooseMoveComboB = CreateNewGUIComponent<CEGUI::Combobox>("OgreTray/Combobox");
    ChooseMoveComboB->setReadOnly(true);
    ChooseMoveComboB->setVerticalAlignment(CEGUI::VA_TOP);
    ChooseMoveComboB->setHorizontalAlignment(CEGUI::HA_RIGHT);
    ChooseMoveComboB->setSize(CEGUI::USize(CEGUI::UDim(0, 150), CEGUI::UDim(0, 30)));


    ChooseMoveComboB->subscribeEvent(CEGUI::Combobox::EventListSelectionAccepted,
    		CEGUI::Event::Subscriber(&LevelEditor::ChooseMoveComboBCallback, this));
    MainLayout->addChild(ChooseMoveComboB);
    ButtonSetAddButton(EditMovesButtons, ChooseMoveComboB);

    MoveTSpeedTitleB = CreateNewGUIComponent<CEGUI::Titlebar>("OgreTray/Title");
    MoveTSpeedTitleB->setText("Translate speed");
    MoveTSpeedTitleB->setSize(CEGUI::USize(CEGUI::UDim(0, 150), CEGUI::UDim(0, 30)));
    MoveTSpeedTitleB->setVerticalAlignment(CEGUI::VA_TOP);
    MoveTSpeedTitleB->setHorizontalAlignment(CEGUI::HA_RIGHT);

    MainLayout->addChild(MoveTSpeedTitleB);
    ButtonSetAddButton(EditMovesButtons, MoveTSpeedTitleB);

    MoveTSpeedEditB = CreateNewGUIComponent<CEGUI::Editbox>("OgreTray/Editbox");
    MoveTSpeedEditB->setSize(CEGUI::USize(CEGUI::UDim(0, 150), CEGUI::UDim(0, 30)));
    MoveTSpeedEditB->setVerticalAlignment(CEGUI::VA_TOP);
    MoveTSpeedEditB->setHorizontalAlignment(CEGUI::HA_RIGHT);
    MoveTSpeedEditB->setValidationString(FloatingNumRegex);

    MainLayout->addChild(MoveTSpeedEditB);
    ButtonSetAddButton(EditMovesButtons, MoveTSpeedEditB);

    MoveRSpeedTitleB = CreateNewGUIComponent<CEGUI::Titlebar>("OgreTray/Title");
    MoveRSpeedTitleB->setText("Rotate speed");
    MoveRSpeedTitleB->setSize(CEGUI::USize(CEGUI::UDim(0, 150), CEGUI::UDim(0, 30)));
    MoveRSpeedTitleB->setVerticalAlignment(CEGUI::VA_TOP);
    MoveRSpeedTitleB->setHorizontalAlignment(CEGUI::HA_RIGHT);

    MainLayout->addChild(MoveRSpeedTitleB);
    ButtonSetAddButton(EditMovesButtons, MoveRSpeedTitleB);

    MoveRSpeedEditB = CreateNewGUIComponent<CEGUI::Editbox>("OgreTray/Editbox");
    MoveRSpeedEditB->setSize(CEGUI::USize(CEGUI::UDim(0, 150), CEGUI::UDim(0, 30)));
    MoveRSpeedEditB->setVerticalAlignment(CEGUI::VA_TOP);
    MoveRSpeedEditB->setHorizontalAlignment(CEGUI::HA_RIGHT);
    MoveRSpeedEditB->setValidationString(FloatingNumRegex);

    MainLayout->addChild(MoveRSpeedEditB);
    ButtonSetAddButton(EditMovesButtons, MoveRSpeedEditB);

    MoveWaitTimeTitleB = CreateNewGUIComponent<CEGUI::Titlebar>("OgreTray/Title");
    MoveWaitTimeTitleB->setText("Wait time");
    MoveWaitTimeTitleB->setSize(CEGUI::USize(CEGUI::UDim(0, 150), CEGUI::UDim(0, 30)));
    MoveWaitTimeTitleB->setVerticalAlignment(CEGUI::VA_TOP);
    MoveWaitTimeTitleB->setHorizontalAlignment(CEGUI::HA_RIGHT);

    MainLayout->addChild(MoveWaitTimeTitleB);
    ButtonSetAddButton(EditMovesButtons, MoveWaitTimeTitleB);

    MoveWaitTimeEditB = CreateNewGUIComponent<CEGUI::Editbox>("OgreTray/Editbox");
    MoveWaitTimeEditB->setSize(CEGUI::USize(CEGUI::UDim(0, 150), CEGUI::UDim(0, 30)));
    MoveWaitTimeEditB->setVerticalAlignment(CEGUI::VA_TOP);
    MoveWaitTimeEditB->setHorizontalAlignment(CEGUI::HA_RIGHT);
    MoveWaitTimeEditB->setValidationString(IntegerNumRegex);

    MainLayout->addChild(MoveWaitTimeEditB);
    ButtonSetAddButton(EditMovesButtons, MoveWaitTimeEditB);

    ApplyToMoveStepPushB = CreateNewGUIComponent<CEGUI::PushButton>("OgreTray/Button");
    ApplyToMoveStepPushB->setText("Modify");
    ApplyToMoveStepPushB->setSize(CEGUI::USize(CEGUI::UDim(0, 75), CEGUI::UDim(0, 30)));
    ApplyToMoveStepPushB->setVerticalAlignment(CEGUI::VA_TOP);
    ApplyToMoveStepPushB->setHorizontalAlignment(CEGUI::HA_RIGHT);
    ApplyToMoveStepPushB->subscribeEvent(CEGUI::PushButton::EventClicked,
    		CEGUI::Event::Subscriber(&LevelEditor::ApplyToMoveStepPushBCallback, this));

    MainLayout->addChild(ApplyToMoveStepPushB);
    ButtonSetAddButton(EditMovesButtons, ApplyToMoveStepPushB);

    IsMoveTriggeredToggleB = CreateNewGUIComponent<CEGUI::ToggleButton>("OgreTray/Checkbox");
    IsMoveTriggeredToggleB->setText("Move triggered ?");
    IsMoveTriggeredToggleB->setSelected(false);
    IsMoveTriggeredToggleB->setSize(CEGUI::USize(CEGUI::UDim(0, 200), CEGUI::UDim(0, 30)));
    IsMoveTriggeredToggleB->setVerticalAlignment(CEGUI::VA_TOP);
    IsMoveTriggeredToggleB->setHorizontalAlignment(CEGUI::HA_RIGHT);

	IsMoveTriggeredToggleB->subscribeEvent(CEGUI::ToggleButton::EventSelectStateChanged,
			CEGUI::Event::Subscriber(&LevelEditor::IsMoveTriggeredToggleBCallback, this));

    MainLayout->addChild(IsMoveTriggeredToggleB);
    ButtonSetAddButton(EditMovesButtons, IsMoveTriggeredToggleB);

    CorrelateSpeedsToggleB = CreateNewGUIComponent<CEGUI::ToggleButton>("OgreTray/Checkbox");
    CorrelateSpeedsToggleB->setText("Link");
    CorrelateSpeedsToggleB->setSelected(false);
    CorrelateSpeedsToggleB->setSize(CEGUI::USize(CEGUI::UDim(0, 100), CEGUI::UDim(0, 30)));
    CorrelateSpeedsToggleB->setVerticalAlignment(CEGUI::VA_TOP);
    CorrelateSpeedsToggleB->setHorizontalAlignment(CEGUI::HA_RIGHT);

	CorrelateSpeedsToggleB->subscribeEvent(CEGUI::ToggleButton::EventSelectStateChanged,
			CEGUI::Event::Subscriber(&LevelEditor::CorrelateSpeedsToggleBCallback, this));

    MainLayout->addChild(CorrelateSpeedsToggleB);
    ButtonSetAddButton(EditMovesButtons, CorrelateSpeedsToggleB);

    SetWindowsPosNearToOther(ChooseMoveComboB, ScaleElementB, 0, 1);
    SetWindowsPosNearToOther(DelMoveStepPushB, ChooseMoveComboB, 0, 1);
    SetWindowsPosNearToOther(AddMoveStepPushB, DelMoveStepPushB, -1, 0);
    SetWindowsPosNearToOther(MoveTSpeedEditB, ChooseMoveComboB, -1, 0);
    SetWindowsPosNearToOther(MoveTSpeedTitleB, MoveTSpeedEditB, -1, 0);
    SetWindowsPosNearToOther(MoveRSpeedEditB, MoveTSpeedEditB, 0, 1);
    SetWindowsPosNearToOther(MoveRSpeedTitleB, MoveRSpeedEditB, -1, 0);
    SetWindowsPosNearToOther(MoveWaitTimeEditB, MoveRSpeedEditB, 0, 1);
    SetWindowsPosNearToOther(MoveWaitTimeTitleB, MoveWaitTimeEditB, -1, 0);
    SetWindowsPosNearToOther(ApplyToMoveStepPushB, MoveWaitTimeEditB, 0, 1);
    SetWindowsPosNearToOther(CorrelateSpeedsToggleB, ApplyToMoveStepPushB, -1, 0);
    SetWindowsPosNearToOther(IsMoveTriggeredToggleB, ApplyToMoveStepPushB, 0, 1);

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
    		CEGUI::Event::Subscriber(&LevelEditor::ApplyMassChangesToBallPushBCallback, this));

    MainLayout->addChild(ApplyMassChangesToBallPushB);
    ButtonSetAddButton(EditBallButtons, ApplyMassChangesToBallPushB);

    SetWindowsPosNearToOther(BallMassValueEditB, ApplyMassChangesToBallPushB, 0, -1);
}

void LevelEditor::SetupGame(void)
{
	GameEngine::SetupGame();
    //Thumbnail Window
	mThumbnailSceneMgr->setAmbientLight(ColourValue(0.5, 0.5, 0.5));

	Light* light = mThumbnailSceneMgr->createLight("MainThumbnailLight");
	SceneNode* lightNode = mThumbnailSceneMgr->getRootSceneNode()->createChildSceneNode("MainThumbnailLight");
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

bool LevelEditor::ChooseMoveComboBCallback(const CEGUI::EventArgs &e)
{
	if(UnderEditCase == NULL)
		return true;
	CEGUI::ListboxTextItem *item = (CEGUI::ListboxTextItem*)ChooseMoveComboB->getSelectedItem();
	if(item == NULL)
		return true;
	UnderEditCase->DisplaySelectedMove(item->getUserData(), MoveTSpeedEditB, MoveRSpeedEditB, MoveWaitTimeEditB, CorrelateSpeedsToggleB);
	ApplyToMoveStepPushB->setEnabled(true);
	DelMoveStepPushB->setEnabled(true);
	return true;
}

bool LevelEditor::CorrelateSpeedsToggleBCallback(const CEGUI::EventArgs &e)
{
	MoveRSpeedEditB->setEnabled(CorrelateSpeedsToggleB->isSelected() == false);
	MoveRSpeedTitleB->setEnabled(CorrelateSpeedsToggleB->isSelected() == false);
	return true;
}

bool LevelEditor::IsMoveTriggeredToggleBCallback(const CEGUI::EventArgs &e)
{
	if(UnderEditCase == NULL || UnderEditCase->CaseToMove() == false)
		return true;
	UnderEditCase->SetMoveTriggered(IsMoveTriggeredToggleB->isSelected());
	return true;
}

bool LevelEditor::AddMoveStepPushBCallback(const CEGUI::EventArgs &e)
{
	AddMoveStep();
	return true;
}

bool LevelEditor::ApplyToMoveStepPushBCallback(const CEGUI::EventArgs &e)
{
	CEGUI::ListboxTextItem *item = (CEGUI::ListboxTextItem*)ChooseMoveComboB->getSelectedItem();
	if(item == NULL)
		return true;
	Vector3 GoalPos = UnderEditCase->getAbsolutePosition();
	Quaternion GoalAngle = UnderEditCase->getAbsoluteOrientation();
	float TranslationSpeed = CEGUI::PropertyHelper<float>::fromString(MoveTSpeedEditB->getText());
	float RotateSpeed = CEGUI::PropertyHelper<float>::fromString(MoveRSpeedEditB->getText());
	unsigned64 waittime = CEGUI::PropertyHelper<uint64>::fromString(MoveWaitTimeEditB->getText());
	UnderEditCase->UpdateSelectedMove(item->getUserData(), GoalPos, TranslationSpeed, GoalAngle, RotateSpeed, waittime, CorrelateSpeedsToggleB->isSelected());
	return true;
}

void LevelEditor::AddMoveStep(void)
{
	if(UnderEditCase == NULL)
		return;

	BuildRefMove(UnderEditCase);
	Vector3 GoalPos = UnderEditCase->getRelativePosition();
	Quaternion GoalAngle = UnderEditCase->getRelativeOrientation();
	GroupEntity *RefMove = UnderEditCase->getRefMove();
	LOG << "RefMove Pos " << RefMove->getAbsolutePosition() << ", Angle " << RefMove->getAbsoluteOrientation() << std::endl;
	LOG << "Add Move step GoalPos " << GoalPos << ", GoalAngle " << GoalAngle << std::endl;
	float TranslationSpeed = CEGUI::PropertyHelper<float>::fromString(MoveTSpeedEditB->getText());
	float RotateSpeed = CEGUI::PropertyHelper<float>::fromString(MoveRSpeedEditB->getText());
	unsigned64 waittime = CEGUI::PropertyHelper<uint64>::fromString(MoveWaitTimeEditB->getText());
	if(UnderEditCase->CaseToMove() == false)// For the moment no Moves !
	{
		UnderEditCase->ResetToInitial();
		UnderEditCase->AddMovePoint(UnderEditCase->getRelativePosition(), TranslationSpeed, waittime, UnderEditCase->getRelativeOrientation(), RotateSpeed, true);
		UnderEditCase->setRelativePosition(GoalPos);
		UnderEditCase->setRelativeOrientation(GoalAngle);
	}
	if(IsMoveTriggeredToggleB->isSelected() == true)
		UnderEditCase->AddTriggeredMovePoint(GoalPos, TranslationSpeed, waittime, GoalAngle, RotateSpeed);
	else
		UnderEditCase->AddMovePoint(GoalPos, TranslationSpeed, waittime, GoalAngle, RotateSpeed, CorrelateSpeedsToggleB->isSelected());
	UnderEditCase->FillComboboxWithMoves(ChooseMoveComboB);
	ApplyToMoveStepPushB->setEnabled(false);
	DelMoveStepPushB->setEnabled(false);

	if(UnderEditCase->CaseToMove() == true)
		AddCaseToBeMoved(UnderEditCase);
}

bool LevelEditor::DelMoveStepPushBCallback(const CEGUI::EventArgs &e)
{
	DelMoveStep();
	return true;
}

void LevelEditor::DelMoveStep(void)
{
	if(UnderEditCase == NULL)
		return;
	CEGUI::ListboxTextItem *item = (CEGUI::ListboxTextItem*)ChooseMoveComboB->getSelectedItem();
	UnderEditCase->DeletedMove(item->getUserData());
	UnderEditCase->FillComboboxWithMoves(ChooseMoveComboB);
	UnderEditCase->ResetToInitial();
	ApplyToMoveStepPushB->setEnabled(false);
	DelMoveStepPushB->setEnabled(false);

	if(UnderEditCase->CaseToMove() == false)
		DelCaseToBeMoved(UnderEditCase);
}

bool LevelEditor::frameStarted(const Ogre::FrameEvent& fe)
{
	std::list<GroupEntity*>::iterator iter(GroupsToBeEquilibrated.begin());
	while(iter != GroupsToBeEquilibrated.end())
	{
		GroupEntity *G = *iter;
		if(G != NULL)
		{
			LOG << "Pre Frame Computing " << G << std::endl;
			GroupsToEquilibrate.push_back(G);
		}
		iter = GroupsToBeEquilibrated.erase(iter);
	}
    return true;
}

bool LevelEditor::frameEnded(const Ogre::FrameEvent& fe)
{
	// Needed for tool tips ? For the moment doesn't work and if uncommented, all buttons places are broke down !
//	CEGUI::System::getSingleton().getDefaultGUIContext().injectTimePulse( fe.timeSinceLastFrame );

	if(ThumbnailWindow->isVisible() == true && ThumbnailWindow->isDisabled() == false && ogreThumbnailNode != NULL)
		ogreThumbnailNode->roll(Degree(0.01), Node::TS_WORLD);

	GameEngine::frameEnded(fe);

	std::list<GroupEntity*>::iterator iter(GroupsToEquilibrate.begin());
	while(iter != GroupsToEquilibrate.end())
	{
		GroupEntity *G = *iter;
		if(G != NULL)
		{
			LOG << "Post Frame Computing " << G << std::endl;
			G->ComputeAndEquilibrateChilds();
		}
		iter = GroupsToEquilibrate.erase(iter);
	}

    return true;
}

bool LevelEditor::mouseMoved(const OIS::MouseEvent &arg)
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
		if(LastHighlighted != NULL)
		{
			bool HideBoundingBox = true;

			if((UnderEditCase != NULL && UnderEditCase == LastHighlighted)
					|| (UnderEditBall != NULL && UnderEditBall == LastHighlighted)
					|| (ToBePlacedEntity != NULL && ToBePlacedEntity == LastHighlighted)
					|| (ToBeDeletedEntity != NULL && ToBeDeletedEntity == LastHighlighted))
				HideBoundingBox = false;

			if(UnderEditEntites.empty() == false)
			{
				LOG << "Multi selection mode" << std::endl;
				std::list<BaseEntity*>::iterator iter(UnderEditEntites.begin());
				while(iter != UnderEditEntites.end())
				{
					BaseEntity *Entity = *(iter++);
					if(Entity != NULL)
					{
						LOG << "Multiselection check Entity " << Entity->getName() << " @" << Entity << std::endl;
						if(Entity == LastHighlighted)
						{
							HideBoundingBox = false;
							break;
						}
						else if(Entity->getType() == BaseEntity::Types::Group)
						{
							LOG << "Multiselection check Entity has Group" << std::endl;
							GroupEntity *Grp = (GroupEntity*)Entity;
							if(Grp->HasChild(LastHighlighted))
							{
								HideBoundingBox = false;
								break;
							}
						}
					}
				}
			}
			if(HideBoundingBox)
			{
//				LOG << "Entity under mouse not selected" << std::endl;
				LastHighlighted->DisplaySelectedBox(false);
			}
			LastHighlighted = NULL;
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
					LastHighlighted = Ogre::any_cast<Entity*>(((Ogre::Entity*)PickedUpNode->getAttachedObject(0))->getUserObjectBindings().getUserAny());
					LastHighlighted->DisplaySelectedBox(true);
					LastHighlightedGroup = LastHighlighted->getGroup();
					if(LastHighlighted->getType() == BaseEntity::Types::Case && ((CaseEntity*)LastHighlighted)->getRefMove() != NULL)
						LastHighlightedGroup = LastHighlightedGroup->getGroup();
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

bool LevelEditor::ApplyForceChangesToCasePushBCallback(const CEGUI::EventArgs &event)
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

bool LevelEditor::CaseHasForceToggleBCallback(const CEGUI::EventArgs &event)
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

bool LevelEditor::CaseHasForceDirectionToggleBCallback(const CEGUI::EventArgs &event)
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

void LevelEditor::UpdateEditButtons(void)
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

void LevelEditor::UpdateForceArrows(void)
{
	if(ForcesArrows == NULL)
		return;
	LOG << "Scale Force Arrows" << std::endl;
	ForcesArrows->setScale(force_direction.m_x, force_direction.m_y, force_direction.m_z);
}

void LevelEditor::EditCase(CaseEntity *Entity)
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

	ButtonsSetMutedState(EditCaseButtons, true);
	ButtonsSetMutedState(EditMovesButtons, true);
	if(UnderEditCase != NULL)
	{
		if(LevelEditMode == Edit)
		{
			if(EntityEditMode == Caracts)
			{
				const dVector *case_force_direction = UnderEditCase->getForceDirection();

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
			}
			else
				ButtonsSetVisible(EditCaseButtons, false);

			if(EntityEditMode == Moves)
			{
				UnderEditCase->FillComboboxWithMoves(ChooseMoveComboB);
				MoveTSpeedEditB->setText(std::to_string(DEFAULT_ENTITY_MOVE_TRANSLATION_SPEED));
				MoveRSpeedEditB->setText(std::to_string(DEFAULT_ENTITY_MOVE_ROTATION_SPEED));
				MoveWaitTimeEditB->setText(std::to_string(DEFAULT_ENTITY_MOVE_WAIT_TIME));
				IsMoveTriggeredToggleB->setSelected(UnderEditCase->MoveTriggered());
				CorrelateSpeedsToggleB->setSelected(false);
				ButtonsSetVisible(EditMovesButtons, true);
				ApplyToMoveStepPushB->setEnabled(false);
				DelMoveStepPushB->setEnabled(false);
				if(LastHighlightedGroup != NULL)
					AddMoveStepPushB->setEnabled(false);
			}
			else
				ButtonsSetVisible(EditMovesButtons, false);
		}
		else
		{
			ButtonsSetVisible(EditCaseButtons, false);
			ButtonsSetVisible(EditMovesButtons, false);
		}
	}
	else
	{
		ButtonsSetVisible(EditCaseButtons, false);
		ButtonsSetVisible(EditMovesButtons, false);
	}
	ButtonsSetMutedState(EditMovesButtons, false);
	ButtonsSetMutedState(EditCaseButtons, false);
}

bool LevelEditor::ApplyMassChangesToBallPushBCallback(const CEGUI::EventArgs &event)
{
	if(UnderEditBall == NULL)
		return true;
	UnderEditBallMass =	CEGUI::PropertyHelper<float>::fromString(BallMassValueEditB->getText());
	UnderEditBall->setMass(UnderEditBallMass);
	return true;
}

void LevelEditor::EditBall(BallEntity *Entity)
{
	if(mode != Editing)
		return;
	if(UnderEditBall != NULL)
	{
		UnderEditBall->DisplaySelectedBox(false);
		UnderEditBall->ResetToInitial();
	}
	UnderEditBall = Entity;

	ButtonsSetMutedState(EditBallButtons, true);
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
	ButtonsSetMutedState(EditBallButtons, false);
}

void LevelEditor::MultiSelectionSetEmpty(void)
{
	std::list<BaseEntity*>::iterator iter(UnderEditEntites.begin());
	while(iter != UnderEditEntites.end())
	{
		BaseEntity *Entity = *iter;
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

bool LevelEditor::ManageMultiSelectionSet(BaseEntity *entity)
{
	if(entity == NULL)
	{
		MultiSelectionSetEmpty();
		return false;
	}
	bool add_it = true;
	std::list<BaseEntity*>::iterator iter(UnderEditEntites.begin());
	while(iter != UnderEditEntites.end())
	{
		BaseEntity *got = *iter;
		if(got == NULL)
		{
			iter++;
			continue;
		}
		if(entity == got)
		{
			add_it = false;
			LOG << "Remove Entity " << entity->getName() << " @" << entity << " from MultiSelection" << std::endl;
			entity->DisplaySelectedBox(false);
			UnderEditEntites.erase(iter);
			break;
		}
		iter++;
	}
	if(add_it == true)
	{
		LOG << "Add Entity " << entity->getName() << " @" << entity << " to MultiSelection" << std::endl;
		entity->DisplaySelectedBox(true);
		UnderEditEntites.push_back(entity);
	}

	GroupEntity *to_check = NULL;
	bool group_are_identical = true;
	std::list<BaseEntity*>::iterator iter2(UnderEditEntites.begin());
	while(iter2 != UnderEditEntites.end())
	{
		BaseEntity *CheckEnt = *(iter2++);
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

void LevelEditor::FillMultiselectionSetWithGroup(GroupEntity *Grp)
{
	if(Grp == NULL)
	{
		MultiSelectionSetEmpty();
		return;
	}
	std::list<BaseEntity*> to_add;
	Grp->FillListWithChilds(to_add);
	std::list<BaseEntity*>::iterator iter(to_add.begin());
	while(iter != to_add.end())
	{
		BaseEntity *child = *(iter++);
		if(child != NULL)
			ManageMultiSelectionSet(child);
	}
}

bool LevelEditor::mousePressed( const OIS::MouseEvent &arg, OIS::MouseButtonID id )
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
    		if(LastHighlighted != NULL)
			{
				if(MultiSelectionMode == true)
				{
					if(LastHighlightedGroup != NULL)
					{
						std::list<BaseEntity*> to_add;
						LastHighlightedGroup->FillListWithChilds(to_add);
						std::list<BaseEntity*>::iterator iter(to_add.begin());
						while(iter != to_add.end())
						{
							BaseEntity *child = *(iter++);
							if(child != NULL)
								ManageMultiSelectionSet(child);
						}
					}
					else
						ManageMultiSelectionSet(LastHighlighted);
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
					if(LastHighlightedGroup == NULL)
					{
						if(LevelEditMode != Delete)
						{
							switch(LastHighlighted->getType())
							{
							case Entity::Types::Case :
									LOG << "Edit Case by Mouse Pressed" << std::endl;
									EditCase((CaseEntity*)LastHighlighted);
									break;
							case Entity::Types::Ball :
									LOG << "Edit Ball by Mouse Pressed" << std::endl;
									EditBall((BallEntity*)LastHighlighted);
									break;
							}
						}
						else
							PrepareDeleteElement(LastHighlighted);
					}
					else
					{
						if(LevelEditMode == Edit)
						{
							switch(EntityEditMode)
							{
							case EntityEditModes::Caracts :
							case EntityEditModes::Moves :
								switch(LastHighlighted->getType())
								{
								case Entity::Types::Case :
										LOG << "Edit Case by Mouse Pressed" << std::endl;
										EditCase((CaseEntity*)LastHighlighted);
										break;
								case Entity::Types::Ball :
										LOG << "Edit Ball by Mouse Pressed" << std::endl;
										EditBall((BallEntity*)LastHighlighted);
										break;
								}
								break;
							}
						}
						FillMultiselectionSetWithGroup(LastHighlightedGroup);
					}
				}
			}
    		break;
    	}
    }
    return true;
}

bool LevelEditor::mouseReleased( const OIS::MouseEvent &arg, OIS::MouseButtonID id )
{
    CEGUI::GUIContext& context = CEGUI::System::getSingleton().getDefaultGUIContext();
    context.injectMouseButtonUp(convertButton(id));
    //BaseApplication::mouseReleased(arg, id);
    return true;
}

void LevelEditor::MoveEntities(float x, float y, float z)
{
	if(UnderEditCase != NULL && LastHighlightedGroup == NULL)
		UnderEditCase->Move(x, y, z);
	if(UnderEditBall != NULL && LastHighlightedGroup == NULL)
		UnderEditBall->Move(x, y, z);
	std::list<GroupEntity*> ToMoveGroups;
	std::list<BaseEntity*>::iterator iter(UnderEditEntites.begin());
	while(iter != UnderEditEntites.end())
	{
		BaseEntity *Entity = *(iter++);
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

void LevelEditor::RotateEntities(float x, float y, float z)
{
	if(UnderEditCase != NULL && LastHighlightedGroup == NULL)
		UnderEditCase->Rotate(x, y, z);
	if(UnderEditBall != NULL && LastHighlightedGroup == NULL)
		UnderEditBall->Rotate(x, y, z);
	std::list<GroupEntity*> ToRotateGroups;
	std::list<BaseEntity*>::iterator iter(UnderEditEntites.begin());
	while(iter != UnderEditEntites.end())
	{
		BaseEntity *Entity = *(iter++);
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

void LevelEditor::ScaleEntities(float x, float y, float z)
{
	if(UnderEditCase != NULL && LastHighlightedGroup == NULL)
		UnderEditCase->Scale(x, y, z);
	if(UnderEditBall != NULL && LastHighlightedGroup == NULL)
		UnderEditBall->Scale(x, y, z);
	std::list<GroupEntity*> ToScaleGroups;
	std::list<BaseEntity*>::iterator iter(UnderEditEntites.begin());
	while(iter != UnderEditEntites.end())
	{
		BaseEntity *Entity = *(iter++);
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

bool LevelEditor::keyPressed(const OIS::KeyEvent &arg)
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
				Entity *UnderEditEntity = NULL;
				MultiSelectionMode = true;
				LOG << "Activate Multi selection mode" << std::endl;
				if(UnderEditBall != NULL)
					UnderEditEntity = (Entity*)UnderEditBall;
				if(UnderEditCase != NULL)
					UnderEditEntity = (Entity*)UnderEditCase;
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
					AddMoveStep();
					break;
				}
				break;
			}
		}
		break;
	case OIS::KeyCode::KC_DELETE:
		if(mode == Editing && MouseOverButton == false)
		{
			switch(LevelEditMode)
			{
			case Edit :
				switch(EntityEditMode)
				{
				case Moves :
					DelMoveStep();
					break;
				}
				break;
			case Delete :
				DeleteElement();
				break;
			}
		}
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

bool LevelEditor::keyReleased( const OIS::KeyEvent &arg )
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

bool LevelEditor::NewLevelCreateBCallback(const CEGUI::EventArgs &e)
{
	EmptyLevel();
	String level;
	String Filename;
	is_new_level = true;
	level = NewLevelEditB->getText().c_str();
	BuildLevelFilename(level, Filename);
	SetLevel(level, Filename);
	if(mode == Running)
		SwitchEditMode();

	SaveLevelPushB->setEnabled(true);
	EditModePushB->setEnabled(true);
	StatesModePushB->setEnabled(true);
	StopPhysicPushB->setEnabled(true);
	return true;
}

bool LevelEditor::SaveLevelPushBCallback(const CEGUI::EventArgs &e)
{
	String export_str;
	ExportLevelIntoJson(export_str);
	std::ofstream myfile;
	myfile.open (LevelFilename.c_str());
	myfile << export_str;
	myfile.close();

	if(is_new_level == true)
	{
		CEGUI::ListboxTextItem *item = new CEGUI::ListboxTextItem((CEGUI::utf8*)Level.c_str());
		item->setUserData(new String(LevelFilename));
		ChooseLevelComboB->addItem(item);
		ChooseLevelComboB->setEnabled(true);
	    ChooseLevelComboB->setSize(CEGUI::USize(CEGUI::UDim(0, 150), CEGUI::UDim(0, 40 * (ChooseLevelComboB->getItemCount() + 1))));
	}
	if(mode == Running)
		SaveLevelPushB->setEnabled(false);
	return true;
}

bool LevelEditor::ChooseLevelComboBCallback(const CEGUI::EventArgs &e)
{
	CEGUI::ListboxTextItem *item = (CEGUI::ListboxTextItem*)ChooseLevelComboB->getSelectedItem();
	String level(item->getText().c_str()), *filename = (String*)item->getUserData();
	is_new_level = false;
	SetLevel(level, *filename);
	ChangeLevel();
	return true;
}

void LevelEditor::SetLevel(String &level_name, String &levelFilename)
{
	Level = level_name;
	LevelFilename = levelFilename;
	LevelNameBanner->setText((CEGUI::utf8*)Level.c_str());
}

void LevelEditor::EmptyLevel(void)
{
	GameEngine::EmptyLevel();
	EmptyStatesList();
}

void LevelEditor::EmptyLevelsList(void)
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

void LevelEditor::EmptyStatesList(void)
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

void LevelEditor::LoadStatesList(void)
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

void LevelEditor::ChangeLevel(void)
{
	if(mode == Editing)
		SwitchEditMode();
	else
		_StopPhysic();
	EmptyLevel();
	if(Level.empty() == false)
	{
		String nodeNamePrefix;
		ImportLevelFromJson((Node*)mSceneMgr->getRootSceneNode(), nodeNamePrefix);
		LoadStatesList();
	}
	else
	{
		ButtonsSetVisible(MainMenuButtons, true);
		ChooseLevelComboB->setEnabled(false);
		SaveLevelPushB->setEnabled(false);
		EditModePushB->setEnabled(false);
		StatesModePushB->setEnabled(false);
		StopPhysicPushB->setEnabled(false);
	}
}

bool LevelEditor::ExportGroupIntoJson(GroupEntity *G, rapidjson::Value &JGroup, rapidjson::Document::AllocatorType& allocator, std::list<GroupEntity*> &GroupsAlreadyExported)
{
	bool AlreadyHave = false;
	std::list<GroupEntity*>::iterator CheckIter(GroupsAlreadyExported.begin());
	while(CheckIter != GroupsAlreadyExported.end())
	{
		GroupEntity *CheckG = *(CheckIter++);
		if(CheckG == NULL)
			continue;
		if(CheckG == G)
			AlreadyHave = true;
	}
	if(AlreadyHave == false)
	{
		G->ExportToJson(JGroup, allocator);
		GroupsAlreadyExported.push_back(G);
	}
	return AlreadyHave;
}

void LevelEditor::ExportLevelIntoJson(String &export_str)
{
	rapidjson::Document document;
	document.SetObject();

	rapidjson::Document::AllocatorType& allocator = document.GetAllocator();

	document.AddMember(COUNTER_JSON_FIELD, nb_entities, allocator);

	rapidjson::Value groups(rapidjson::kArrayType);

	std::list<GroupEntity*>::iterator Git(Groups.begin());
	std::list<GroupEntity*> GroupsAlreadyExported;
	while(Git != Groups.end())
	{
		GroupEntity *Entity = *(Git++);
		if(Entity == NULL)
			continue;
		rapidjson::Value JGroup(rapidjson::kObjectType);
		GroupEntity *Sup = Entity->getGroup();
		std::list<GroupEntity*> ToExport;
		while(Sup != NULL)
		{
			ToExport.push_front(Sup);
			Sup = Sup->getGroup();
		}
		std::list<GroupEntity*>::iterator ToExportIter(ToExport.begin());
		while(ToExportIter != ToExport.end())
		{
			bool AlreadyHave = false;
			GroupEntity *G = *(ToExportIter++);
			if(G == NULL)
				continue;
			rapidjson::Value JSupGroup(rapidjson::kObjectType);
			if(ExportGroupIntoJson(G, JSupGroup, allocator, GroupsAlreadyExported) == false)
				groups.PushBack(JSupGroup, allocator);
		}

		if(ExportGroupIntoJson(Entity, JGroup, allocator, GroupsAlreadyExported) == false)
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

}
