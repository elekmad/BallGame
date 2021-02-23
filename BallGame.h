/*
 * BallGame.h
 *
 *  Created on: 15 nov. 2020
 *      Author: damien
 */

#ifndef BALLGAME_H_
#define BALLGAME_H_

#include <Ogre.h>
#include <OgreRTShaderSystem.h>
#include <OgreMath.h>
#include <iostream>
#include "BaseApplication.h"
#include "Entity.h"


#include <Newton.h>
#include <dCustomListener.h>
#include <toolbox_stdafx.h>
#include <dHighResolutionTimer.h>
#include <DebugDisplay.h>

#include <dList.h>
#include <dMatrix.h>

//because of Xlib defines (True False Bool None) which must be undef
#undef True
#undef False
#undef None
#undef Bool
#include <rapidjson/document.h>
#include <rapidjson/writer.h>
#include <rapidjson/stringbuffer.h>
#include <CEGUI/CEGUI.h>
#include <CEGUI/RendererModules/Ogre/Renderer.h>

#define LEVELS_FOLDER "./Levels/"
#define LEVELS_EXTENSION "json"
#define STATES_EXTENSION "state"

using namespace Ogre;
using namespace OgreBites;

#define MAX_PHYSICS_FPS				240.0f
#define MAX_PHYSICS_SUB_STEPS		50
//#define PROJECTILE_INITIAL_SPEED	20.0f

#ifdef OGRE_STATIC_LIB
#  define OGRE_STATIC_GL
#  if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
#    define OGRE_STATIC_Direct3D9
// dx10 will only work on vista, so be careful about statically linking
#    if OGRE_USE_D3D10
#      define OGRE_STATIC_Direct3D10
#    endif
#  endif
#  define OGRE_STATIC_BSPSceneManager
#  define OGRE_STATIC_ParticleFX
#  define OGRE_STATIC_CgProgramManager
#  ifdef OGRE_USE_PCZ
#    define OGRE_STATIC_PCZSceneManager
#    define OGRE_STATIC_OctreeZone
#  else
#    define OGRE_STATIC_OctreeSceneManager
#  endif
#  include "OgreStaticPluginLoader.h"
#endif

#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)
#define LOG std::cout << __func__ <<  " (" << __FILENAME__ << "@" << __LINE__ << ") : "

inline float Normalize(float v1, float v2, float v3);
inline double Normalize(double v1, double v2, double v3);

class BallGame : public BaseApplication
{
    public :

    BallGame();
    ~BallGame();
    GroupEntity *findGroup(const char * const name);
    void AddGroup(GroupEntity *Entity);

    private :

    class EntityType
	{
	public :

    	EntityType(){ InitialMass = 0.0; Type= Case; }
    	~EntityType(){}
		String Name;
		enum BallGameEntityType Type;
		String MeshName;
		Vector3 InitialPos;
		Vector3 InitialScale;
		Quaternion InitialOrientation;
		float InitialMass;
	};

    std::list<class EntityType*> EntityTypes;

    void LoadBallGameEntityTypes(void);

    void Append(BallGameEntity *entity);
    void _StartPhysic(void);
    void _StopPhysic(void);
    void SwitchEditMode(void);
    void DeleteCase(CaseEntity *Entity, std::list<CaseEntity*>::iterator *iter = NULL);
    void RemoveCase(CaseEntity *Entity, std::list<CaseEntity*>::iterator *iter = NULL);
    void DeleteBall(BallEntity *Entity, std::list<BallEntity*>::iterator *iter = NULL);
    void RemoveBall(BallEntity *Entity, std::list<BallEntity*>::iterator *iter = NULL);
    void DeleteGroup(GroupEntity *Entity, std::list<GroupEntity*>::iterator *iter = NULL);
    void RemoveGroup(GroupEntity *Entity, std::list<GroupEntity*>::iterator *iter = NULL);
    void LoadStatesList(void);
    void EmptyStatesList(void);
    void EmptyLevelsList(void);
    void EmptyLevel(void);//Clean all BallGame, Newton and Ogre entities to start with new level.
    void ChangeLevel(void);
    void ImportLevelFromJson(Node *parent, String &nodeNamePrefix, bool isForImport = false);

    String Level;
    String LevelFilename;
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
    BallGameEntity *ToBePlacedEntity;
    BallGameEntity *LastPlacedEntity;
    BallGameEntity *ToBeDeletedEntity;
    void PlaceNewElement(void);
    void PlaceUnderEditElement(void);
    void PlaceElement(BallGameEntity *ToBePlacedEntity);
    void PrepareNewElement(void);
    inline void UnprepareNewElement(void);
    void DeleteElement(void);
    inline void PrepareDeleteElement(BallGameEntity *Entity);
    inline void UnprepareDeleteElement(void);

    //Edit Entities
    bool MultiSelectionMode;
    void MoveEntities(float x, float y, float z);
    void RotateEntities(float x, float y, float z);
    void ScaleEntities(float x, float y, float z);
    void MultiSelectionSetEmpty(void);
    bool ManageMultiSelectionSet(BallGameEntity *entity);
    std::list<class BallGameEntity*> UnderEditEntites;
    //Edit Ball
    void EditBall(BallEntity *Entity);
    BallEntity *UnderEditBall;
    dFloat UnderEditBallMass;

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
    public :

    NewtonWorld* m_world;
    static void PostUpdateCallback(const NewtonWorld* const world, dFloat timestep);
    static void OnContactCollision (const NewtonJoint* contactJoint, dFloat timestep, int threadIndex);
    void UpdatePhysics(dFloat timestep);
    NewtonWorld* GetNewton(void);

    void CustomListenerPostUpdateCallback(dFloat timestep);

    private :

    void SetupNewton(void);

	class GameNewtonListener : public dCustomListener
	{
		public :
    	GameNewtonListener(BallGame *engine) : dCustomListener(engine->GetNewton(), "BallGameListener")
    	{
    		Engine = engine;
    	}

		void PostUpdate(dFloat timestep);
		BallGame *Engine;
	};

	GameNewtonListener *listener;

    static void BodySerialization (NewtonBody* const body, void* const bodyUserData, NewtonSerializeCallback serializeCallback, void* const serializeHandle);
    static void BodyDeserialization (NewtonBody* const body, void* const bodyUserData, NewtonDeserializeCallback deserializecallback, void* const serializeHandle);
    void SerializedPhysicScene(const String* const name);
    void DeserializedPhysicScene(const String* const name);
    BallGameEntity *GetEntity(char *name);
    bool CheckIfAlreadyColliding(CaseEntity *ToCheck);
    void AddCaseColliding(CaseEntity *ToAdd);
    inline void _updatePhysic(dFloat timestep);

    bool m_suspendPhysicsUpdate;
    unsigned64 m_microsecunds;
    int m_physicsFramesCount;
    dFloat m_mainThreadPhysicsTime;
    dFloat m_mainThreadPhysicsTimeAcc;
    bool m_asynchronousPhysicsUpdate;

    unsigned long nb_entities;
    std::list<CaseEntity*> Cases;
    std::list<BallEntity*> Balls;
    std::list<GroupEntity*> Groups;
	std::list<CaseEntity*> CasesUnderCollide;
	std::list<CaseEntity*> CasesToBeMoved;
	void AddCaseToBeMoved(CaseEntity *ToAdd);

	String ImportLevelFilename;
	String ImportLevelName;
    std::list<CaseEntity*> ImportLevelCases;
    std::list<BallEntity*> ImportLevelBalls;
    std::list<GroupEntity*> ImportLevelGroups;
    inline void ActivateLevelImportInterface(void);
    inline void UnactivateLevelImportInterface(void);

    void CheckforCollides(void);
    void AddCase(CaseEntity *Entity);
    void AddBall(BallEntity *Entity);

    /////////////////////////////////////////////////


    ////////////////  Ogre ////////////////////////////
    public :

    bool keyPressed( const OIS::KeyEvent &arg );
    bool keyReleased( const OIS::KeyEvent &arg );
	bool mouseMoved( const OIS::MouseEvent &arg );
    bool mousePressed( const OIS::MouseEvent &arg, OIS::MouseButtonID id );
    bool mouseReleased( const OIS::MouseEvent &arg, OIS::MouseButtonID id );
    Ogre::SceneManager *getSceneManager(void) { return mSceneMgr; }

    private :

	void createScene(void);
    void SetupGame(void);

    //Mouse picking
	BallGameEntity *LastHighligted;
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

    //////////////////////////////////////////////////

    virtual bool frameEnded(const Ogre::FrameEvent& fe);
    void SetCam(float x, float y, float z);
    void MoveCam(float x, float y, float z);
};


#endif /* BALLGAME_H_ */
