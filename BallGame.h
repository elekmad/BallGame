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


#include <Newton.h>
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

#define WORLD_LENGTH 20
//#define WORLD_LENGTH 1
//#define WORLD_DEPTH 20
#define WORLD_DEPTH 1

using namespace Ogre;
using namespace OgreBites;

#define MAX_PHYSICS_FPS				60.0f
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

#define Normalize2(a, b) \
	({ typeof(a) __a = (a); \
	typeof(b) __b = (b); \
	sqrt(__a * __a + __b * __b);})

#define Normalize3(a, b, c) \
		({ typeof(a) __a = (a); \
		typeof(b) __b = (b); \
		typeof(c) __c = (c); \
		sqrt(__a * __a + __b * __b + __c * __c);})
#define max(a,b) \
  ({ typeof (a) _a = (a); \
      typeof (b) _b = (b); \
    _a > _b ? _a : _b; })

class BallGame;
enum BallGameEntityType
{
	Case,
	Ball
};
class BallGameEntity
{
	public:
	BallGameEntity(const dMatrix& matrix);
	BallGameEntity();
	~BallGameEntity(){}
    static void TransformCallback(const NewtonBody* body, const dFloat* matrix, int threadIndex);
    void ExportToJson(rapidjson::Value &v, rapidjson::Document::AllocatorType& allocator);
    void ImportFromJson(rapidjson::Value &v);
    void SetOgreNode(SceneNode *node);
    void SetNewtonBody(NewtonBody *body);
	protected:
    enum BallGameEntityType type;
	//mutable dMatrix m_matrix;			// interpolated matrix
	dVector m_curPosition;				// position one physics simulation step in the future
	dVector m_nextPosition;             // position at the current physics simulation step
	dQuaternion m_curRotation;          // rotation one physics simulation step in the future
	dQuaternion m_nextRotation;         // rotation at the current physics simulation step
	SceneNode *OgreEntity;
	NewtonBody *Body;
	Ogre::Vector3 InitialPos;
	Ogre::Vector3 InitialScale;
	Ogre::Quaternion InitialOrientation;

    void SetMatrixUsafe(const dQuaternion& rotation, const dVector& position);



	friend class BallGame;
};

class BallEntity : public BallGameEntity
{
	public:
	BallEntity(const dMatrix& matrix);
	BallEntity();
	void CreateFromJson(rapidjson::Value &v, Ogre::SceneManager* mSceneMgr, NewtonWorld *m_world);
	void AddForceVector(dVector *force);
    void ExportToJson(rapidjson::Value &v, rapidjson::Document::AllocatorType& allocator);
    void ImportFromJson(rapidjson::Value &v);
	dVector *GetForceVector();
	protected:
	dList<dVector*> Forces;
	float InitialMass;

	friend class BallGame;
	friend class CaseEntity;
};

class CaseEntity : public BallGameEntity
{
	public:
	enum CaseType
	{
		typeBox = 0,
		typeRamp = 1
	};
	enum CaseType type;
	CaseEntity(const dMatrix& matrix, enum CaseType _type = typeBox);
	CaseEntity(enum CaseType _type = typeBox);
	void CreateFromJson(rapidjson::Value &v, Ogre::SceneManager* mSceneMgr, NewtonWorld *m_world);
//	void AddBallColliding(NewtonBody *ball);
//	bool CheckIfAlreadyColliding(NewtonBody *ball);
	void SetForceToApply(float force, dVector *direction);
	void ApplyForceOnBall(BallEntity *ball);
    void ExportToJson(rapidjson::Value &v, rapidjson::Document::AllocatorType& allocator);
    void ImportFromJson(rapidjson::Value &v);
	protected:
	dArray<NewtonBody*> BallsUnderCollide;

	float force_to_apply;
	dVector *force_direction;

	friend class BallGame;
	friend class BallEntity;
};

class BallGame : public BaseApplication
{
    public :

    BallGame();
    ~BallGame();

    private :

    void Append(BallGameEntity *entity);
    void _StartPhysic(void);
    void _StopPhysic(void);
    void SwitchEditMode(void);
    void EmptyLevel(void);//Clean all BallGame, Newton and Ogre entities to start with new level.
    void ChangeLevel(void);
    void ImportLevelFromJson(void);

    String Level;
    void SetLevel(String &level_name);

    //Place New Element
    enum PlacementModes
	{
    	Move,
		Rotate,
		Scale
	}PlacementMode;

	enum BallGameEntityType ToBePlacedEntityType;
    BallGameEntity *ToBePlacedEntity;
    BallGameEntity *LastPlacedEntity;
    void PlaceNewElement(void);
    void PrepareNewElement(void);

    //Edit Ball
    void EditBall(BallEntity *Entity);
    BallEntity *UnderEditBall;
    dFloat UnderEditBallMass;

    //Edit Case
    void EditCase(CaseEntity *Entity);
	CaseEntity *UnderEditCase;
	bool CaseHasForce;
	float UnderEditCaseForce;
	bool force_directed;
	dVector force_direction;
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
    void UpdatePhysics(dFloat timestep);

    private :

    void SetupNewton(void);
    NewtonWorld* GetNewton(void);

    bool m_suspendPhysicsUpdate;
    unsigned64 m_microsecunds;
    int m_physicsFramesCount;
    dFloat m_mainThreadPhysicsTime;
    dFloat m_mainThreadPhysicsTimeAcc;
    bool m_asynchronousPhysicsUpdate;

    dArray<CaseEntity*> Cases;
    dArray<BallEntity*> Balls;


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

    private :

	void createScene(void);
    void SetupGame(void);

    //Mouse picking
	Ogre::SceneNode *LastHighligted;
    //////////////////////////////////////////////////


	/////////////////// CE GUI ////////////////////////

    private :

    void SetupGUI(void);

    CEGUI::OgreRenderer* mRenderer;

    CEGUI::LayoutContainer* MainLayout;

    CEGUI::PushButton *StopPhysicPushB;
    bool StopPhysicPushBCallback(const CEGUI::EventArgs &e);
    CEGUI::PushButton *EditModePushB;
    bool EditModePushBCallback(const CEGUI::EventArgs &e);
    CEGUI::Combobox *ChooseLevelComboB;
    bool ChooseLevelComboBCallback(const CEGUI::EventArgs &e);
    CEGUI::Editbox *NewLevelEditB;
//    bool NewLevelEditBCallback(const CEGUI::EventArgs &e);
    CEGUI::PushButton *NewLevelCreateB;
    bool NewLevelCreateBCallback(const CEGUI::EventArgs &e);
    CEGUI::PushButton *SaveLevelPushB;
    bool SaveLevelPushBCallback(const CEGUI::EventArgs &e);
    CEGUI::PushButton *QuitPushB;
    bool QuitPushBCallback(const CEGUI::EventArgs &e);

    //Edit buttons

    CEGUI::Titlebar *EditingModeTitleBanner;
    CEGUI::Titlebar *LevelNameBanner;

    //Add new elements Buttons & Callbacks
    CEGUI::Titlebar *AddElementTitleBanner;
    CEGUI::Combobox *ChooseTypeOfElementToAddB;
    bool ChooseTypeOfElementToAddBCallback(const CEGUI::EventArgs &e);
    CEGUI::PushButton *PlaceNewElementB;
    bool PlaceNewElementBCallback(const CEGUI::EventArgs &e);


    //Edit Ball Buttons & Callbacks
    CEGUI::Editbox *BallMassValueEditB;
    CEGUI::PushButton *ApplyMassChangesToBallPushB;
    bool ApplyMassChangesToBallPushBCallback(const CEGUI::EventArgs &event);

    //Edit Case Buttons & Callbacks
    CEGUI::ToggleButton *CaseHasForceToggleB;
    CEGUI::Editbox *CaseForceValueEditB;
    inline void CaseForceValueEditBSetText(float value);
    inline void CaseForceValueEditBSetText(double value);
    bool CaseForceValueEditBCallback(const CEGUI::EventArgs &event);
    CEGUI::ToggleButton *CaseHasForceDirectionToggleB;
    bool ToggleForceCallback(const CEGUI::EventArgs &e);
    bool ToggleForceDirectedCallback(const CEGUI::EventArgs &event);
    CEGUI::Editbox *CaseForceDirectionXValueEditB;
    inline void CaseForceDirectionXValueEditBSetText(float value);
    inline void CaseForceDirectionXValueEditBSetText(double value);
    CEGUI::Editbox *CaseForceDirectionYValueEditB;
    inline void CaseForceDirectionYValueEditBSetText(float value);
    inline void CaseForceDirectionYValueEditBSetText(double value);
    CEGUI::Editbox *CaseForceDirectionZValueEditB;
    inline void CaseForceDirectionZValueEditBSetText(float value);
    inline void CaseForceDirectionZValueEditBSetText(double value);
    CEGUI::PushButton *NormalizeCaseForceDirectionPushB;
    bool NormalizeCaseForceDirectionPushBCallback(const CEGUI::EventArgs &e);
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
