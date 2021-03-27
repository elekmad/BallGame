/*
 * GameEngine.h
 *
 *  Created on: 15 nov. 2020
 *      Author: damien
 */

#ifndef GAMEENGINE_H_
#define GAMEENGINE_H_

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


using namespace Ogre;
using namespace OgreBites;

#define MAX_PHYSICS_FPS				240.0f
#define MAX_PHYSICS_SUB_STEPS		50
//#define PROJECTILE_INITIAL_SPEED	20.0f



#define COUNTER_JSON_FIELD "InternalCounter"
#define GROUPS_JSON_FIELD "Groups"
#define CASES_JSON_FIELD "Cases"
#define BALLS_JSON_FIELD "Balls"

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

namespace BallGame {

class GameEngine : public BaseApplication
{
	public :

	GameEngine();
    ~GameEngine();

    inline GroupEntity *findGroup(const char * const name, bool is_for_import = false);
    virtual GroupEntity *findGroup(String &name, bool is_for_import = false);
    void AddGroup(GroupEntity *Entity, bool recursive = false);

	protected :

    void _StartPhysic(void);
    void _StopPhysic(void);
    void DeleteCase(CaseEntity *Entity, std::list<CaseEntity*>::iterator *iter = NULL);
    void RemoveCase(CaseEntity *Entity, std::list<CaseEntity*>::iterator *iter = NULL);
    void DeleteBall(BallEntity *Entity, std::list<BallEntity*>::iterator *iter = NULL);
    void RemoveBall(BallEntity *Entity, std::list<BallEntity*>::iterator *iter = NULL);
    void DeleteGroup(GroupEntity *Entity);
    inline GroupEntity *DeleteGroup(GroupEntity *Entity, std::list<GroupEntity*>::iterator *iter);
    void RemoveGroup(GroupEntity *Entity, std::list<GroupEntity*>::iterator *iter = NULL);
    void EmptyLevel(void);//Clean all Engine, Newton and Ogre entities to start with new level.
    void ImportLevelFromJson(Node *parent, String &nodeNamePrefix, bool isForImport = false);

    String LevelFilename;

	String ImportLevelFilename;
	String ImportLevelName;
    std::list<CaseEntity*> ImportLevelCases;
    std::list<BallEntity*> ImportLevelBalls;
    std::list<GroupEntity*> ImportLevelGroups;

    /////////////////////  NEWTON ///////////////////
	public :

	NewtonWorld* m_world;
	static void PostUpdateCallback(const NewtonWorld* const world, dFloat timestep);
	static void OnContactCollision (const NewtonJoint* contactJoint, dFloat timestep, int threadIndex);
	void UpdatePhysics(dFloat timestep);
	NewtonWorld* GetNewton(void);

	void CustomListenerPostUpdateCallback(dFloat timestep);

	protected :

	void SetupNewton(void);

	class GameNewtonListener : public dCustomListener
	{
		public :
		GameNewtonListener(GameEngine *engine) : dCustomListener(engine->GetNewton(), "BallGameListener")
		{
			Engine = engine;
		}

		void PostUpdate(dFloat timestep);
		GameEngine *Engine;
	};

	GameNewtonListener *listener;

    static void BodySerialization (NewtonBody* const body, void* const bodyUserData, NewtonSerializeCallback serializeCallback, void* const serializeHandle);
    static void BodyDeserialization (NewtonBody* const body, void* const bodyUserData, NewtonDeserializeCallback deserializecallback, void* const serializeHandle);
    void SerializedPhysicScene(const String* const name);
    void DeserializedPhysicScene(const String* const name);
    Entity *GetEntity(char *name);
    bool CheckIfAlreadyColliding(CaseEntity *ToCheck);
    void AddCaseColliding(CaseEntity *ToAdd);
    inline void _updatePhysic(dFloat timestep);

    bool m_suspendPhysicsUpdate;
    unsigned64 m_microsecunds;
    int m_physicsFramesCount;
    dFloat m_mainThreadPhysicsTime;
    dFloat m_mainThreadPhysicsTimeAcc;
    bool m_asynchronousPhysicsUpdate;

    ///////////////////////////////////////////////////

    /////////////// OGRE /////////////////

	public :

    Ogre::SceneManager *getSceneManager(void) { return mSceneMgr; }

	protected :

    void SetupGame(void);

    //////////////////////////////////

	protected :

    unsigned long nb_entities;
    std::list<CaseEntity*> Cases;
    std::list<BallEntity*> Balls;
    std::list<GroupEntity*> Groups;
	std::list<CaseEntity*> CasesUnderCollide;
	std::list<CaseEntity*> CasesToBeMoved;
	void AddCaseToBeMoved(CaseEntity *ToAdd);
	void BuildRefMove(CaseEntity *ToAdd);
	void DelCaseToBeMoved(CaseEntity *ToDel);

    void CheckforCollides(void);
    void AddCase(CaseEntity *Entity);
    void AddBall(BallEntity *Entity);

    bool frameEnded(const Ogre::FrameEvent& fe);
    void SetCam(float x, float y, float z);
    void MoveCam(float x, float y, float z);
};

}

#endif /* GAMEENGINE_H_ */
