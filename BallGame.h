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
#include <SdkTrays.h>
#include <SdkCameraMan.h>
#include <OISEvents.h>
#include <OISInputManager.h>
#include <OISKeyboard.h>
#include <OISMouse.h>

#include <Newton.h>
#include <toolbox_stdafx.h>
#include <dHighResolutionTimer.h>
#include <DebugDisplay.h>

#include <dList.h>
#include <dMatrix.h>

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
	protected:
	mutable dMatrix m_matrix;			// interpolated matrix
	dVector m_curPosition;				// position one physics simulation step in the future
	dVector m_nextPosition;             // position at the current physics simulation step
	dQuaternion m_curRotation;          // rotation one physics simulation step in the future
	dQuaternion m_nextRotation;         // rotation at the current physics simulation step
	SceneNode *OgreEntity;

    void SetMatrixUsafe(const dQuaternion& rotation, const dVector& position);



	friend class BallGame;
};

class BallEntity : public BallGameEntity
{
	public:
	BallEntity(const dMatrix& matrix):BallGameEntity(matrix){}
	void AddForceVector(dVector *force);
	dVector *GetForceVector();
	protected:
	dList<dVector*> Forces;

};

class CaseEntity : public BallGameEntity
{
	public:
	enum CaseType
	{
		typeBox,
		typeRamp
	};
	enum CaseType type;
	CaseEntity(const dMatrix& matrix, enum CaseType _type = typeBox);
	void AddBallColliding(NewtonBody *ball);
	bool CheckIfAlreadyColliding(NewtonBody *ball);
	void SetForceToApply(float force, dVector *direction);
	void ApplyForceOnBall(NewtonBody *ball);
	protected:
	dArray<NewtonBody*> BallsUnderCollide;

	float force_to_apply;
	dVector *force_direction;

};

class BallGame : public Ogre::FrameListener, public Ogre::WindowEventListener, public OIS::KeyListener, public OIS::MouseListener, OgreBites::SdkTrayListener
{
    public :
    BallGame();
    virtual ~BallGame();

    void SetupNewton(void);
    void SetupGame(void);


    NewtonWorld* GetNewton(void);
    void Append(BallGameEntity *entity);

    private :
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

    bool configure(void);
    void chooseSceneManager(void);
    void createCamera(void);
    void createViewports(void);
    void loadResources(void);
    void setupResources(void);
    void createFrameListener(void);
    void destroyScene(void);
    bool setup();//Setup callback for Ogre Applications
    void go(void);
	bool keyPressed( const OIS::KeyEvent &arg );
	bool keyReleased( const OIS::KeyEvent &arg );
	bool mouseMoved( const OIS::MouseEvent &arg );
	bool mousePressed( const OIS::MouseEvent &arg, OIS::MouseButtonID id );
	bool mouseReleased( const OIS::MouseEvent &arg, OIS::MouseButtonID id );

    private :

    Ogre::Root *mRoot;
	Ogre::OverlaySystem *mOverlaySystem;
    //Mouse picking
	Ogre::SceneNode *LastHighligted;
	Ogre::SceneManager* scnMgr;
//	Ogre::SceneNode* camNode;
	Ogre::RenderWindow* mWindow;
	Ogre::Camera* mCamera;
	Ogre::String mResourcesCfg;
	Ogre::String mPluginsCfg;
    //////////////////////////////////////////////////

    //////////////////////// Ogre Bites //////////////
    public :

    private :

	OgreBites::InputContext mInputContext;
	OgreBites::SdkTrayManager*	mTrayMgr;
	OgreBites::SdkCameraMan* mCameraMan;     	// basic camera controller
	OgreBites::ParamsPanel* mDetailsPanel;   	// sample details panel
	bool mCursorWasVisible;						// was cursor visible before dialog appeared
	bool mShutDown;


    /////////////////////////////////////////////////


    //////////////////////// OIS /////////////////////
    public :

    private :

	OIS::InputManager* mInputManager;
	OIS::Mouse*    mMouse;
	OIS::Keyboard* mKeyboard;

    //////////////////////////////////////////////////


    virtual bool frameEnded(const Ogre::FrameEvent& fe);
    bool frameRenderingQueued(const Ogre::FrameEvent& evt);
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
