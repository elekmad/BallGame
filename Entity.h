/*
 * BallGame.h
 *
 *  Created on: 15 nov. 2020
 *      Author: damien
 */

#ifndef ENTITY_H_
#define ENTITY_H_

#include <Ogre.h>
#include <OgreRTShaderSystem.h>
#include <OgreMath.h>
#include <iostream>
#include "BaseApplication.h"
#include "BallGame.h"


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

#define DEFAULT_ENTITY_MOVE_TRANSLATION_SPEED 10
#define DEFAULT_ENTITY_MOVE_ROTATION_SPEED 0.1
#define DEFAULT_ENTITY_MOVE_WAIT_TIME 0

using namespace Ogre;
using namespace OgreBites;

class BallGame;
class GroupEntity;

enum BallGameEntityType
{
	Case,
	Ball
};

class BallGameEntity
{
	public :

	BallGameEntity(const dMatrix& matrix);
	BallGameEntity();
	~BallGameEntity(){}
	void Finalize(void);
    static void TransformCallback(const NewtonBody* body, const dFloat* matrix, int threadIndex);
    void ExportToJson(rapidjson::Value &v, rapidjson::Document::AllocatorType& allocator);
    void ImportFromJson(rapidjson::Value &v, BallGame *Game, Node *parent, String &nodeNamePrefix);
    void setOgreNode(SceneNode *node);
    void setNewtonBody(NewtonBody *body);
    const NewtonBody *getNewtonBody(void) const { return Body; }
    const SceneNode *getOgreEntity(void) const { return OgreEntity; }
    dMatrix *PrepareNewtonBody(dVector &NewtonBodyLocation, dVector &NewtonBodySize);
    void DisplaySelectedBox(bool display);
    const Ogre::Vector3 &getInitialPosition(void) const { return InitialPos; }
    const Ogre::Vector3 &getRelativePosition(void) const { return OgreEntity->getPosition(); }
    const Ogre::Vector3 &getAbsolutePosition(void) const { return OgreEntity->_getDerivedPosition(); }
    void setInitialPosition(const Ogre::Vector3 &NewPosition) { InitialPos = NewPosition; }
    void setRelativePosition(const Ogre::Vector3 &NewPosition) { OgreEntity->setPosition(NewPosition); }
    void setAbsolutePosition(const Ogre::Vector3 &NewPosition) { OgreEntity->_setDerivedPosition(NewPosition); }
    const Ogre::Quaternion &getInitialOrientation(void) const { return InitialOrientation; }
    const Ogre::Quaternion &getRelativeOrientation(void) const { return OgreEntity->getOrientation(); }
    const Ogre::Quaternion &getAbsoluteOrientation(void) const { return OgreEntity->_getDerivedOrientation(); }
    void setInitialOrientation(const Ogre::Quaternion &NewOrient) { InitialOrientation = NewOrient; }
    void setRelativeOrientation(const Ogre::Quaternion &NewOrient) { OgreEntity->setOrientation(NewOrient); }
    void setAbsoluteOrientation(const Ogre::Quaternion &NewOrient) { OgreEntity->_setDerivedOrientation(NewOrient); }
    const Ogre::Vector3 &getInitialScale(void) const { return InitialScale; }
    const Ogre::Vector3 &getRelativeScale(void) const { return OgreEntity->getScale(); }
    const Ogre::Vector3 &getAbsoluteScale(void) const { return OgreEntity->_getDerivedScale(); }
    void setInitialScale(const Ogre::Vector3 &NewScale) { InitialScale = NewScale; }
    void setRelativeScale(const Ogre::Vector3 &NewScale) { OgreEntity->setScale(NewScale); }
    const Ogre::String &getName(void) const { return OgreEntity->getName(); }
    void Move(float x, float y, float z);
    void Move(Vector3 &);
    void Rotate(float x, float y, float z);
    void Scale(float x, float y, float z);
    void Scale(Vector3 &);
    void getVelocity(dFloat *Velocity) { NewtonBodyGetVelocity(Body, Velocity); }
    enum BallGameEntityType getType(void) { return type; }
    GroupEntity *getGroup(void) { return Group; }
    void ResetToInitial(void)
    {
    	setRelativeScale(InitialScale);
    	setAbsolutePosition(InitialPos);
    	setAbsoluteOrientation(InitialOrientation);
    }

	protected :

    enum BallGameEntityType type;
	//mutable dMatrix m_matrix;			// interpolated matrix
	dVector m_curPosition;				// position one physics simulation step in the future
	dVector m_nextPosition;             // position at the current physics simulation step
	dQuaternion m_curRotation;          // rotation one physics simulation step in the future
	dQuaternion m_nextRotation;         // rotation at the current physics simulation step
	SceneNode *OgreEntity;
	NewtonBody *Body;
	GroupEntity *Group;
	//We need to have initial pos, scale and orientation appart from ogre's one because we can edit level during physic move, so export level with ogre's one can be impossible !
	Ogre::Vector3 InitialPos;
	Ogre::Vector3 InitialScale;
	Ogre::Quaternion InitialOrientation;

    void SetMatrixUsafe(const dQuaternion& rotation, const dVector& position);

	friend class GroupEntity;
};

class BallEntity : public BallGameEntity
{
	public :

	BallEntity(const dMatrix& matrix);
	BallEntity();
	~BallEntity();
	void CreateFromJson(rapidjson::Value &v, BallGame *Game, NewtonWorld *m_world, Node *parent, String &nodeNamePrefix);
	void AddForceVector(dVector *force);
    void ExportToJson(rapidjson::Value &v, rapidjson::Document::AllocatorType& allocator);
    void ImportFromJson(rapidjson::Value &v, BallGame *Game, Node *parent, String &nodeNamePrefix);
	dVector *GetForceVector();
	void CleanupForces(void);
    void CreateNewtonBody(NewtonWorld *m_world);
    dFloat getMass(void);
    void setMass(dFloat newMass);
    float getInitialMass(void) { return InitialMass; }
    void setInitialMass(float newInitialMass) { InitialMass = newInitialMass; }

	protected :

	dList<dVector*> Forces;
	float InitialMass;
};

class CaseEntity : public BallGameEntity
{
	public :

	enum CaseType
	{
		typeBox = 0,
		typeRamp = 1
	};
	enum CaseType type;
	CaseEntity(const dMatrix& matrix, enum CaseType _type = typeBox);
	CaseEntity(enum CaseType _type = typeBox);
	~CaseEntity();
	void CreateFromJson(rapidjson::Value &v, BallGame *Game, NewtonWorld *m_world, Node *parent, String &nodeNamePrefix);
//	void AddBallColliding(NewtonBody *ball);
//	bool CheckIfAlreadyColliding(NewtonBody *ball);
	void SetForceToApply(float force, dVector *direction);
	void ApplyForceOnBall(BallEntity *ball);
    void ExportToJson(rapidjson::Value &v, rapidjson::Document::AllocatorType& allocator);
    void ImportFromJson(rapidjson::Value &v, BallGame *Game, Node *parent, String &nodeNamePrefix);
    void CreateNewtonBody(NewtonWorld *m_world);
    float getForce(void) { return force_to_apply; }
    const dVector *getForceDirection(void) { return force_direction; }
    void AddBallColliding(BallEntity *ball);
	void ApplyForceOnCollidingBalls(void);
	Ogre::SceneNode *CreateForceArrows(Ogre::SceneManager *Scene);
	void CaseMove(unsigned64 microseconds, dFloat timestep);
	bool CaseToMove(void) { return MovementToDo != NULL; }
	bool MoveTriggered(void) { return MovementToDo != NULL && MovementToDo->is_launched_by_collide; }
	void AddMovePoint(const Vector3 &GoalPos, float speed, unsigned64 waittime, const Quaternion &GoalAngle = Ogre::Quaternion::IDENTITY, float RotateSpeed = NAN);
	void AddTriggeredMovePoint(const Vector3 &GoalPos, float speed, unsigned64 waittime, const Quaternion &GoalAngle = Ogre::Quaternion::IDENTITY, float RotateSpeed = NAN)
	{
		AddMovePoint(GoalPos, speed, waittime, GoalAngle, RotateSpeed);
		SetMoveTriggered(true);
	}
	void SetMoveTriggered(bool trigger);

	void FillComboboxWithMoves(CEGUI::Combobox *box);
	void DisplaySelectedMove(void *step, CEGUI::Editbox *, CEGUI::Editbox *, CEGUI::Editbox *);
	void UpdateSelectedMove(void *step, const Vector3 &GoalPos, float TSpeed, const Quaternion &GoalAngle, float RSpeed, unsigned64 WaitTime);
	void DeletedMove(void *step);

	protected :

    bool CheckIfAlreadyColliding(BallEntity *ball);

	std::list<BallEntity*> BallsUnderCollide;

	struct MovementStep
	{
		Vector3 Position;
		float TranslateSpeed;
		Quaternion Orientation;
		float RotateSpeed;
		unsigned64 waittime;
		MovementStep()
		{
			TranslateSpeed = NAN;
			RotateSpeed = NAN;
			waittime = 0;
		}
	};
	struct Movement
	{
		bool is_launched_by_collide;
		std::list<struct MovementStep*> Moves;
		struct MovementStep *actual;
		unsigned64 foreignedtime;
		Movement()
		{
			actual = NULL;
			is_launched_by_collide = false;
			foreignedtime = 0;
		}
		~Movement()
		{
			std::list<struct MovementStep*>::iterator iter(Moves.begin());
			while(iter != Moves.end())
			{
				struct MovementStep *step = *iter;
				if(step != NULL)
					delete step;
				iter = Moves.erase(iter);
			}
		}
	};

	struct Movement *MovementToDo;

	float force_to_apply;
	dVector *force_direction;
};

class GroupEntity
{
	public :

	GroupEntity(String &name, Ogre::SceneManager* mSceneMgr);
	GroupEntity(){ OgreEntity = NULL; computed = false; equilibrated = false; };
	~GroupEntity(){};
	void Finalize(void);
    void ExportToJson(rapidjson::Value &v, rapidjson::Document::AllocatorType& allocator);
    void ImportFromJson(rapidjson::Value &v, Node *parent, String &nodeNamePrefix);
	void AddChild(BallGameEntity* child);
	bool DelChild(BallGameEntity* child);
	void ComputeChilds(void);
	void ComputeAndEquilibrateChilds(void);
	void FillListWithChilds(std::list<BallGameEntity*> &list);
    void Move(float x, float y, float z);
    void Rotate(float x, float y, float z);
    void Scale(float x, float y, float z);
    const Ogre::String &getName(void) const { return OgreEntity->getName(); }

	private :

	SceneNode *OgreEntity;
	std::list<BallGameEntity*> childs;
	bool computed;
	bool equilibrated;
};


#endif /* ENTITY_H_ */
