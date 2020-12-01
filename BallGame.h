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

class BallGame;

class BallGameEntity
{
	public:
	BallGameEntity(const dMatrix& matrix);
	~BallGameEntity(){}
    static void TransformCallback(const NewtonBody* body, const dFloat* matrix, int threadIndex);
    void ExportToJson(rapidjson::Value &v, rapidjson::Document::AllocatorType& allocator);
   	protected:
	mutable dMatrix m_matrix;			// interpolated matrix
	dVector m_curPosition;				// position one physics simulation step in the future
	dVector m_nextPosition;             // position at the current physics simulation step
	dQuaternion m_curRotation;          // rotation one physics simulation step in the future
	dQuaternion m_nextRotation;         // rotation at the current physics simulation step
	SceneNode *OgreEntity;
	int NewtonID;
	Ogre::Vector3 InitialPos;
	Ogre::Vector3 InitialScale;
	Ogre::Quaternion InitialOrientation;

    void SetMatrixUsafe(const dQuaternion& rotation, const dVector& position);



	friend class BallGame;
};

class BallEntity : public BallGameEntity
{
	public:
	BallEntity(const dMatrix& matrix):BallGameEntity(matrix){}
	void AddForceVector(dVector *force);
    void ExportToJson(rapidjson::Value &v, rapidjson::Document::AllocatorType& allocator);
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
	void AddBallColliding(NewtonBody *ball);
	bool CheckIfAlreadyColliding(NewtonBody *ball);
	void SetForceToApply(float force, dVector *direction);
	void ApplyForceOnBall(NewtonBody *ball);
    void ExportToJson(rapidjson::Value &v, rapidjson::Document::AllocatorType& allocator);
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

    String Level;
    void SetLevel(String &level_name);
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

    dArray<NewtonBody*> Cases;
    dArray<NewtonBody*> Balls;


    void CheckforCollides(void);
    void AddCase(NewtonBody *Entity);
    void AddBall(NewtonBody *Entity);

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
    CEGUI::PushButton *SaveLevelPushB;
    bool SaveLevelPushBCallback(const CEGUI::EventArgs &e);
    CEGUI::PushButton *QuitPushB;
    bool QuitPushBCallback(const CEGUI::EventArgs &e);

    //Edit buttons

    CEGUI::Titlebar *EditingModeTitleBanner;
    CEGUI::Titlebar *LevelNameBanner;


    //Edit Ball Buttons & Callbacks
    CEGUI::Editbox *BallMassValueEditB;
    CEGUI::PushButton *ApplyMassChangesToBallPushB;
    bool ApplyMassChangesToBallPushBCallback(const CEGUI::EventArgs &event);

    //Edit Case Buttons & Callbacks
    CEGUI::ToggleButton *CaseHasForceToggleB;
    CEGUI::Editbox *CaseForceValueEditB;
    CEGUI::ToggleButton *CaseHasForceDirectionToggleB;
    bool ToggleForceCallback(const CEGUI::EventArgs &e);
    bool ToggleForceDirectedCallback(const CEGUI::EventArgs &event);
    CEGUI::Editbox *CaseForceDirectionXValueEditB;
    CEGUI::Editbox *CaseForceDirectionYValueEditB;
    CEGUI::Editbox *CaseForceDirectionZValueEditB;
    CEGUI::PushButton *NormalizeCaseForceDirectionPushB;
    bool NormalizeCaseForceDirectionPushBCallback(const CEGUI::EventArgs &e);
    CEGUI::PushButton *ApplyForceChangesToCasePushB;
    bool ApplyForceChangesToCasePushBCallback(const CEGUI::EventArgs &event);


	//////////////////////////////////////////////////

    /////////////////// RapidJson ////////////////////

    void ExportLevelIntoJson(String &export_str);

    //////////////////////////////////////////////////

    virtual bool frameEnded(const Ogre::FrameEvent& fe);
    int camx;
    int camy;
    int camz;
    Radian CamAnglePitch;
    Radian CamAngleYaw;
    Radian CamAngleRoll;
    void SetCam(int x, int y, int z);
    void MoveCam(int x, int y, int z);
    void CamPitch(float angle)
    {
		Degree dangle(angle);
		CamAnglePitch += dangle;
		std::cout << "Pitch " << CamAnglePitch << std::endl;
		mCamera->pitch(CamAnglePitch);//, Ogre::Node::TransformSpace::TS_LOCAL);
		GetCamParams();
    }
    void CamYaw(float angle)
    {
		Degree dangle(angle);
		CamAngleYaw += dangle;
		std::cout << "Yaw " << CamAngleYaw << std::endl;
		mCamera->yaw(CamAngleYaw);//, Ogre::Node::TransformSpace::TS_LOCAL);
		GetCamParams();
    }
    void CamRoll(float angle)
    {
		Degree dangle(angle);
		CamAngleRoll += dangle;
		std::cout << "Roll " << CamAngleRoll << std::endl;
		mCamera->roll(CamAngleRoll);//, Ogre::Node::TransformSpace::TS_LOCAL);
		GetCamParams();
    }
    void GetCamParams(void)
    {
		std::cout << "Cam pos {" << camx << ", " << camy << ", " << camz << "}" << std::endl;
		std::cout << "Orientation = " << mCamera->getOrientation() << std::endl;
    }
};


#endif /* BALLGAME_H_ */
