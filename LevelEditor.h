/*
 * LeveLEditor.h
 *
 *  Created on: 15 nov. 2020
 *      Author: damien
 */

#ifndef LEVELEDITOR_H_
#define LEVELEDITOR_H_

#include "GameEngine.h"

#define LEVELS_FOLDER "./Levels/"
#define LEVELS_EXTENSION "json"
#define STATES_EXTENSION "state"

using namespace Ogre;
using namespace OgreBites;

namespace BallGame {

class LevelEditor : public GameEngine
{
    public :

    LevelEditor();
    ~LevelEditor();

    private :

    class EntityType
	{
	public :

    	EntityType(){ InitialMass = 0.0; Type= Entity::Types::Case; }
    	~EntityType(){}
		String Name;
		enum Entity::Types Type;
		String MeshName;
		Vector3 InitialPos;
		Vector3 InitialScale;
		Quaternion InitialOrientation;
		float InitialMass;
	};

    std::list<class EntityType*> EntityTypes;

    void LoadBallGameEntityTypes(void);

    void _StartPhysic(void);
    void _StopPhysic(void);
    void SwitchEditMode(void);
    void LoadStatesList(void);
    void EmptyStatesList(void);
    void EmptyLevelsList(void);
    void EmptyLevel(void);//Clean all LevelEditor, Newton and Ogre entities to start with new level.
    void ChangeLevel(void);

    String Level;
    bool is_new_level;
    void SetLevel(String &level_name, String &levelFilename);

    //Place New Element
    enum LevelEditModes
	{
    	Place,
    	Edit,
		Delete
	}LevelEditMode;

	enum EntityEditModes
	{
		Simple,
		Moves,
		Caracts
	}EntityEditMode;

	enum EntityEditActions
	{
		Move,
		Rotate,
		Scale,
	}EntityEditAction;

	EntityType *ToBePlacedEntityType;
	SceneNode *ogreThumbnailNode;
    Entity *ToBePlacedEntity;
    Entity *LastPlacedEntity;
    Entity *ToBeDeletedEntity;
    void PlaceNewElement(void);
    void PlaceUnderEditElement(void);
    void PlaceElement(Entity *ToBePlacedEntity);
    void PrepareNewElement(void);
    inline void UnprepareNewElement(void);
    void DeleteElement(void);
    inline void PrepareDeleteElement(Entity *Entity);
    inline void UnprepareDeleteElement(void);

    //Edit Entities
    bool MultiSelectionMode;
    void MoveEntities(float x, float y, float z);
    void RotateEntities(float x, float y, float z);
    void ScaleEntities(float x, float y, float z);
    void MultiSelectionSetEmpty(void);
    bool ManageMultiSelectionSet(BaseEntity *entity);
    void FillMultiselectionSetWithGroup(GroupEntity *Grp);
    inline void ReSetUnderEditEntities(void);
    std::list<class BaseEntity*> UnderEditEntites;
    //Edit Ball
    void EditBall(BallEntity *Entity);
    BallEntity *UnderEditBall;
    dFloat UnderEditBallMass;

    bool CtrlDown;

    //Edit Case
    bool MouseOverButton;
    void EditCase(CaseEntity *Entity);
	CaseEntity *UnderEditCase;
	bool CaseHasForce;
	float UnderEditCaseForce;
	bool force_directed;
	dVector force_direction;
	Ogre::SceneNode *ForcesArrows;
	void UpdateForceArrows(void);
    void UpdateEditButtons(void);

    enum RunningMode
    {
		Running,
		Editing
    }mode;
    /////////////////////  NEWTON ///////////////////
    private :

    inline void ActivateLevelImportInterface(void);
    inline void UnactivateLevelImportInterface(void);

    /////////////////////////////////////////////////


    ////////////////  Ogre ////////////////////////////
    public :

    bool keyPressed( const OIS::KeyEvent &arg );
    bool keyReleased( const OIS::KeyEvent &arg );
	bool mouseMoved( const OIS::MouseEvent &arg );
    bool mousePressed( const OIS::MouseEvent &arg, OIS::MouseButtonID id );
    bool mouseReleased( const OIS::MouseEvent &arg, OIS::MouseButtonID id );

    private :

	void createScene(void);
    void SetupGame(void);
    void chooseSceneManager(void);
    void createCamera(void);
    void createViewports(void);

    //Mouse picking
	Entity *LastHighlighted;
	GroupEntity *LastHighlightedGroup;
	void AddGroupToBeEquilibrated(GroupEntity *Grp);
	std::list<GroupEntity*> GroupsToBeEquilibrated;
	std::list<GroupEntity*> GroupsToEquilibrate;

	Ogre::Camera* mThumbnailCamera;
	Ogre::Camera* mImportLevelCamera;
	Ogre::SceneManager* mThumbnailSceneMgr;
	Ogre::SceneManager* mImportLevelSceneMgr;
	Ogre::TexturePtr pThumbnailtex;
	Ogre::TexturePtr pImportLeveltex;
	Ogre::RenderTexture *rThumbnailtex;
	Ogre::RenderTexture *rImportLeveltex;
    //////////////////////////////////////////////////


	/////////////////// CE GUI ////////////////////////

    private :

    void SetupGUI(void);
    bool EnteringArea(const CEGUI::EventArgs &e);
    bool LeavingArea(const CEGUI::EventArgs &e);
    template<typename T> T*CreateNewGUIComponent(std::string &TypeName, std::string &Name = "");
    template<typename T> T*CreateNewGUIComponent(const char *TypeName, const char *Name = "");

    CEGUI::OgreRenderer* mRenderer;

    CEGUI::LayoutContainer* MainLayout;
    CEGUI::Titlebar *LevelNameBanner;

    // Import Level Buttons
    std::list<CEGUI::Window*> ImportLevelButtons;

    CEGUI::Combobox *ChooseLevelToImportComboB;
    bool ChooseLevelToImportComboBCallback(const CEGUI::EventArgs &e);
    CEGUI::PushButton *ImportLevelPushB;
    bool ImportLevelPushBCallback(const CEGUI::EventArgs &e);
    CEGUI::Window *ImportLevelWindow;
    void BuildImportLevelWindowContent(Node *parent);

    //Main Menu Buttons
    std::list<CEGUI::Window*> MainMenuButtons;

    CEGUI::PushButton *StopPhysicPushB;
    bool StopPhysicPushBCallback(const CEGUI::EventArgs &e);
    CEGUI::PushButton *EditModePushB;
    bool EditModePushBCallback(const CEGUI::EventArgs &e);
    CEGUI::PushButton *StatesModePushB;
    bool StatesModePushBCallback(const CEGUI::EventArgs &e);
    CEGUI::Combobox *ChooseLevelComboB;
    bool ChooseLevelComboBCallback(const CEGUI::EventArgs &e);
    CEGUI::Editbox *NewLevelEditB;
    CEGUI::PushButton *NewLevelCreateB;
    bool NewLevelCreateBCallback(const CEGUI::EventArgs &e);
    CEGUI::PushButton *SaveLevelPushB;
    bool SaveLevelPushBCallback(const CEGUI::EventArgs &e);
    CEGUI::PushButton *QuitPushB;
    bool QuitPushBCallback(const CEGUI::EventArgs &e);

    //Edit buttons

    //States Buttons
    std::list<CEGUI::Window*> StatesButtons;
    CEGUI::Titlebar *StatesBanner;
    CEGUI::PushButton *SaveStatePushB;
    void SetupStatesButtons(void);
    bool SaveStatePushBCallback(const CEGUI::EventArgs &e);
    CEGUI::Combobox *ChooseStateToLoadB;
    bool ChooseStateToLoadBCallback(const CEGUI::EventArgs &e);
    CEGUI::PushButton *LoadStatePushB;
    bool LoadStatePushBCallback(const CEGUI::EventArgs &e);
    CEGUI::PushButton *DelStatePushB;
    bool DelStatePushBCallback(const CEGUI::EventArgs &e);

    //Add new elements Buttons & Callbacks
    std::list<CEGUI::Window*> EditButtons;
    CEGUI::PushButton *ImportLevelActivateInterfacePushB;
    bool ImportLevelActivateInterfacePushBCallback(const CEGUI::EventArgs &e);
    CEGUI::Titlebar *EditingModeTitleBanner;
    CEGUI::Titlebar *AddElementTitleBanner;
    CEGUI::Combobox *ChooseTypeOfElementToAddB;
    bool ChooseTypeOfElementToAddBCallback(const CEGUI::EventArgs &e);
    CEGUI::Window *ThumbnailWindow;
    void CreateThumbnail(String meshname);

    ///Edit Modes
    CEGUI::PushButton *PlaceNewElementB;
    bool PlaceNewElementBCallback(const CEGUI::EventArgs &e);
    CEGUI::PushButton *EditElementB;
    bool EditElementBCallback(const CEGUI::EventArgs &e);
    void EditElementSetupButtons(void);
    CEGUI::PushButton *DeleteElementB;
    bool DeleteElementBCallback(const CEGUI::EventArgs &e);

    ///Edit Entity Modes
    CEGUI::PushButton *SimpleEditElementB;
	bool SimpleEditElementBCallback(const CEGUI::EventArgs &e);
    CEGUI::PushButton *MoveEditElementB;
	bool MoveEditElementBCallback(const CEGUI::EventArgs &e);
    CEGUI::PushButton *CaractsEditElementB;
	bool CaractsEditElementBCallback(const CEGUI::EventArgs &e);

    ///Edit Entity Actions
    CEGUI::PushButton *MoveElementB;
    bool MoveElementBCallback(const CEGUI::EventArgs &e);
    void SetMoveNewElement(void);
    void SetMoveElement(void);
    inline void SetMoveElementAction(void);
    CEGUI::PushButton *RotateElementB;
    bool RotateElementBCallback(const CEGUI::EventArgs &e);
    void SetRotateNewElement(void);
    void SetRotateElement(void);
    inline void SetRotateElementAction(void);
    CEGUI::PushButton *ScaleElementB;
    bool ScaleElementBCallback(const CEGUI::EventArgs &e);
    void SetScaleNewElement(void);
    void SetScaleElement(void);
    inline void SetScaleElementAction(void);
    CEGUI::ToggleButton *GroupElementsB;
    bool GroupElementsBCallback(const CEGUI::EventArgs &e);

    //Edit Entities's moves
    std::list<CEGUI::Window*> EditMovesButtons;
    CEGUI::PushButton *AddMoveStepPushB;
    bool AddMoveStepPushBCallback(const CEGUI::EventArgs &e);
    void AddMoveStep(void);
    CEGUI::PushButton *DelMoveStepPushB;
    bool DelMoveStepPushBCallback(const CEGUI::EventArgs &e);
    void DelMoveStep(void);
    CEGUI::Combobox *ChooseMoveComboB;
    bool ChooseMoveComboBCallback(const CEGUI::EventArgs &e);
    CEGUI::Titlebar *MoveTSpeedTitleB;
    CEGUI::Editbox *MoveTSpeedEditB;
    CEGUI::Titlebar *MoveRSpeedTitleB;
    CEGUI::Editbox *MoveRSpeedEditB;
    CEGUI::ToggleButton *CorrelateSpeedsToggleB;
    bool CorrelateSpeedsToggleBCallback(const CEGUI::EventArgs &e);
    CEGUI::Titlebar *MoveWaitTimeTitleB;
    CEGUI::Editbox *MoveWaitTimeEditB;
    CEGUI::PushButton *ApplyToMoveStepPushB;
    bool ApplyToMoveStepPushBCallback(const CEGUI::EventArgs &e);
    CEGUI::ToggleButton *IsMoveTriggeredToggleB;
    bool IsMoveTriggeredToggleBCallback(const CEGUI::EventArgs &e);

    //Edit Ball Buttons & Callbacks
    std::list<CEGUI::Window*> EditBallButtons;
    CEGUI::Editbox *BallMassValueEditB;
    CEGUI::PushButton *ApplyMassChangesToBallPushB;
    bool ApplyMassChangesToBallPushBCallback(const CEGUI::EventArgs &event);

    //Edit Case Buttons & Callbacks
    std::list<CEGUI::Window*> EditCaseButtons;
    CEGUI::ToggleButton *CaseHasForceToggleB;
    bool CaseHasForceToggleBCallback(const CEGUI::EventArgs &e);
    CEGUI::Editbox *CaseForceValueEditB;
    bool CaseForceValueEditBCallback(const CEGUI::EventArgs &event);
    CEGUI::ToggleButton *CaseHasForceDirectionToggleB;
    bool CaseHasForceDirectionToggleBCallback(const CEGUI::EventArgs &event);
    CEGUI::Editbox *CaseForceDirectionXValueEditB;
    bool CaseForceDirectionXValueEditBMouseWheelCallback(const CEGUI::EventArgs &event);
    CEGUI::Editbox *CaseForceDirectionYValueEditB;
    bool CaseForceDirectionYValueEditBMouseWheelCallback(const CEGUI::EventArgs &event);
    CEGUI::Editbox *CaseForceDirectionZValueEditB;
    bool CaseForceDirectionZValueEditBMouseWheelCallback(const CEGUI::EventArgs &event);
    CEGUI::PushButton *NormalizeCaseForceDirectionPushB;
    bool NormalizeCaseForceDirectionPushBCallback(const CEGUI::EventArgs &e);
    void NormalizeForceDirection(void);
    CEGUI::PushButton *ApplyForceChangesToCasePushB;
    bool ApplyForceChangesToCasePushBCallback(const CEGUI::EventArgs &event);


	//////////////////////////////////////////////////

    /////////////////// RapidJson ////////////////////

    void ExportLevelIntoJson(String &export_str);
    bool ExportGroupIntoJson(GroupEntity *G, rapidjson::Value &JGroup, rapidjson::Document::AllocatorType& allocator, std::list<GroupEntity*> &GroupsAlreadyExported);

    //////////////////////////////////////////////////

    bool frameEnded(const Ogre::FrameEvent& fe);
    bool frameStarted(const Ogre::FrameEvent& fe);
};

}

#endif /* LEVELEDITOR_H_ */
