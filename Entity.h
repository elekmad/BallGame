/*
 * Entity.h
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
#include "GameEngine.h"


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

inline float Normalize(float v1, float v2, float v3);

namespace BallGame {

class GameEngine;
class GroupEntity;

class BaseEntity
{
	public :

	enum Types
	{
		Case,
		Ball,
		Group
	};

	BaseEntity();
	virtual ~BaseEntity(){}
	virtual void Finalize(void);
	virtual void setOgreNode(SceneNode *node);
    virtual void CreateNewtonBody(NewtonWorld *m_world) = 0;
	virtual void ExportToJson(rapidjson::Value &v, rapidjson::Document::AllocatorType& allocator);
	virtual void ImportFromJson(rapidjson::Value &v, GameEngine *Game, Node *parent, String &nodeNamePrefix);
	virtual const SceneNode *getOgreEntity(void) const { return OgreEntity; }
	virtual const AxisAlignedBox &getWorldAABB(void) const;
	virtual const Ogre::Vector3 &getRelativePosition(void) const { return OgreEntity->getPosition(); }
	virtual const Ogre::Vector3 &getAbsolutePosition(void) const { return OgreEntity->_getDerivedPosition(); }
    virtual void setRelativePosition(const Ogre::Vector3 &NewPosition) { OgreEntity->setPosition(NewPosition); }
    virtual void setAbsolutePosition(const Ogre::Vector3 &NewPosition) { OgreEntity->_setDerivedPosition(NewPosition); }
    virtual const Ogre::Quaternion &getRelativeOrientation(void) const { return OgreEntity->getOrientation(); }
    virtual const Ogre::Quaternion &getAbsoluteOrientation(void) const { return OgreEntity->_getDerivedOrientation(); }
    virtual void setRelativeOrientation(const Ogre::Quaternion &NewOrient) { OgreEntity->setOrientation(NewOrient); }
    virtual void setAbsoluteOrientation(const Ogre::Quaternion &NewOrient) { OgreEntity->_setDerivedOrientation(NewOrient); }
    virtual const Ogre::Vector3 &getRelativeScale(void) const { return OgreEntity->getScale(); }
    virtual const Ogre::Vector3 &getAbsoluteScale(void) const { return OgreEntity->_getDerivedScale(); }
    virtual void setRelativeScale(const Ogre::Vector3 &NewScale) { OgreEntity->setScale(NewScale); }
    virtual const Ogre::Vector3 &getInitialPosition(void) const { return InitialPos; }
    virtual void setInitialPosition(const Ogre::Vector3 &NewPosition) { InitialPos = NewPosition; }
    virtual const Ogre::Quaternion &getInitialOrientation(void) const { return InitialOrientation; }
    virtual void setInitialOrientation(const Ogre::Quaternion &NewOrient) { InitialOrientation = NewOrient; }
    virtual const Ogre::Vector3 &getInitialScale(void) const { return InitialScale; }
    virtual void setInitialScale(const Ogre::Vector3 &NewScale) { InitialScale = NewScale; }
    virtual void Move(float x, float y, float z);
    virtual void Move(const Ogre::Vector3 &);
    virtual void Rotate(float x, float y, float z);
    virtual void Scale(float x, float y, float z);
    virtual void Scale(const Ogre::Vector3 &);
    virtual const Ogre::String &getName(void) const { return OgreEntity->getName(); }
    virtual GroupEntity *getGroup(void) { return GroupPtr; }
    virtual void setGroup(GroupEntity *Grp) { GroupPtr = Grp; }
    virtual void setEngine(GameEngine *E) { Engine = E; }
    virtual const GameEngine *getEngine(void) const { return Engine; }
    virtual void DisplaySelectedBox(bool display);
    virtual enum Types getType(void) { return type; }
    virtual void changeOgreParent(SceneNode *newparent)
    {
		OgreEntity->getParent()->removeChild(OgreEntity);
		newparent->addChild(OgreEntity);
    }
    virtual inline void ResetToInitial(void);
    virtual void copyOgreToInitial(void);

	protected :

    enum Types type;
	SceneNode *OgreEntity;
	GroupEntity *GroupPtr;
	//We need to have initial pos, scale and orientation appart from ogre's one because we can edit level during physic move, so export level with ogre's one can be impossible !
	Ogre::Vector3 InitialPos;
	Ogre::Vector3 InitialScale;
	Ogre::Quaternion InitialOrientation;
	GameEngine *Engine;
};

class Entity : public BaseEntity
{
	public :

	Entity(const dMatrix& matrix);
	Entity();
	void Finalize(void);
    void ExportToJson(rapidjson::Value &v, rapidjson::Document::AllocatorType& allocator);
    void ImportFromJson(rapidjson::Value &v, GameEngine *Game, Node *parent, String &nodeNamePrefix);
    static void TransformCallback(const NewtonBody* body, const dFloat* matrix, int threadIndex);
    void setNewtonBody(NewtonBody *body);
    const NewtonBody *getNewtonBody(void) const { return Body; }
    dMatrix *PrepareNewtonBody(dVector &NewtonBodyLocation, dVector &NewtonBodySize);
    virtual void CreateNewtonBody(NewtonWorld *m_world) = 0;
    void getVelocity(dFloat *Velocity) { NewtonBodyGetVelocity(Body, Velocity); }

	protected :

	//mutable dMatrix m_matrix;			// interpolated matrix
	dVector m_curPosition;				// position one physics simulation step in the future
	dVector m_nextPosition;             // position at the current physics simulation step
	dQuaternion m_curRotation;          // rotation one physics simulation step in the future
	dQuaternion m_nextRotation;         // rotation at the current physics simulation step
	NewtonBody *Body;

    void SetMatrixUsafe(const dQuaternion& rotation, const dVector& position);
};

class BallEntity : public Entity
{
	public :

	BallEntity(const dMatrix& matrix);
	BallEntity();
	~BallEntity();
	void CreateFromJson(rapidjson::Value &v, GameEngine *Game, NewtonWorld *m_world, Node *parent, String &nodeNamePrefix);
	void AddForceVector(dVector *force);
    void ExportToJson(rapidjson::Value &v, rapidjson::Document::AllocatorType& allocator);
    void ImportFromJson(rapidjson::Value &v, GameEngine *Game, Node *parent, String &nodeNamePrefix);
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

class CaseEntity : public Entity
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
	void CreateFromJson(rapidjson::Value &v, GameEngine *Game, NewtonWorld *m_world, Node *parent, String &nodeNamePrefix);
//	void AddBallColliding(NewtonBody *ball);
//	bool CheckIfAlreadyColliding(NewtonBody *ball);
	void SetForceToApply(float force, dVector *direction);
	void ApplyForceOnBall(BallEntity *ball);
    void ExportToJson(rapidjson::Value &v, rapidjson::Document::AllocatorType& allocator);
    void ImportFromJson(rapidjson::Value &v, GameEngine *Game, Node *parent, String &nodeNamePrefix);
    void CreateNewtonBody(NewtonWorld *m_world);
    float getForce(void) { return force_to_apply; }
    const dVector *getForceDirection(void) { return force_direction; }
    void AddBallColliding(BallEntity *ball);
	void ApplyForceOnCollidingBalls(void);
	Ogre::SceneNode *CreateForceArrows(Ogre::SceneManager *Scene);
	bool CaseMove(unsigned64 microseconds, dFloat timestep);
	bool CaseToMove(void) { return MovementToDo != NULL; }
	bool MoveTriggered(void) { return MovementToDo != NULL && MovementToDo->is_launched_by_collide; }
	void AddMovePoint(const Vector3 &GoalPos, float speed, unsigned64 waittime, const Quaternion &GoalAngle = Ogre::Quaternion::IDENTITY, float RotateSpeed = NAN, bool correlated_speed = false);
	void AddTriggeredMovePoint(const Vector3 &GoalPos, float speed, unsigned64 waittime, const Quaternion &GoalAngle = Ogre::Quaternion::IDENTITY, float RotateSpeed = NAN, bool correlated_speed = false)
	{
		AddMovePoint(GoalPos, speed, waittime, GoalAngle, RotateSpeed, correlated_speed);
		SetMoveTriggered(true);
	}
	void SetMoveTriggered(bool trigger);

	void FillComboboxWithMoves(CEGUI::Combobox *box);
	void DisplaySelectedMove(void *step, CEGUI::Editbox *, CEGUI::Editbox *, CEGUI::Editbox *, CEGUI::ToggleButton *);
	void UpdateSelectedMove(void *step, const Vector3 &GoalPos, float TSpeed, const Quaternion &GoalAngle, float RSpeed, unsigned64 WaitTime, bool CorrelateSpeeds);
	void DeletedMove(void *step);
	inline void setRefMove(GroupEntity *Grp);
	GroupEntity *getRefMove(void) { return RefMove; }
	void ComputeMove(void);
	void *getActualMoveStep(void)
	{
		if(MovementToDo == NULL) return NULL;
		return (void*)MovementToDo->actual;
	}
	void setActualMoveStep(void*);

	protected :

    bool CheckIfAlreadyColliding(BallEntity *ball);

	std::list<BallEntity*> BallsUnderCollide;

	struct MovementStep
	{
		//Relative members are present because we must have a reference when we move Group
		Vector3 RelativePosition;
		Quaternion RelativeOrientation;
		//Absolute members are present because when computing the move, relative move depend of group orientation and so move can be inverted !
		Vector3 AbsolutePosition;
		Quaternion AbsoluteOrientation;
		float TranslateSpeed;
		float RotateSpeed;
		bool correlated_speed;
		unsigned64 waittime;
		MovementStep()
		{
			TranslateSpeed = NAN;
			RotateSpeed = NAN;
			waittime = 0;
			correlated_speed = false;
		}
	};
	struct Movement
	{
		bool is_launched_by_collide;
		bool is_computed;
		std::list<struct MovementStep*> Moves;
		struct MovementStep *actual;
		unsigned64 foreignedtime;
		Movement()
		{
			actual = NULL;
			is_launched_by_collide = false;
			foreignedtime = 0;
			is_computed = false;
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
	GroupEntity *RefMove;

	float force_to_apply;
	dVector *force_direction;
};

class GroupEntity : public BaseEntity
{
	public :

	GroupEntity(String &name, Ogre::SceneManager* mSceneMgr);
	GroupEntity(){ type = Group; computed = false; equilibrated = false; isRefMove = false; };
    void CreateNewtonBody(NewtonWorld *m_world);
	void Finalize(void);
	void AddChild(BaseEntity* child);
	bool DelChild(BaseEntity* child);
	bool HasChild(BaseEntity* child);
	bool getisRefMove(void) { return isRefMove; }
	void setisRefMove(bool ref) { isRefMove = ref; }
	const AxisAlignedBox &getWorldAABB(void) const;
	void computeWorldAABB(void);
	void setForceRecomputeChilds(bool force_compute = false);
	void ComputeChilds(void);
	void ComputeAndEquilibrateChilds(void);
	void EquilibrateAABBAroundOrigin(void);
	void FillListWithChilds(std::list<BaseEntity*> &list);
    void Scale(float x, float y, float z);
    void DisplayChilds(void);
    virtual void DisplaySelectedBox(bool display);
    virtual void ResetToInitial(void);
    virtual void copyOgreToInitial(void);
    virtual void ResetToInitial(bool with_childs);
    virtual void copyOgreToInitial(bool with_childs);

	private :

    Ogre::AxisAlignedBox AABB;

	std::list<BaseEntity*> childs;
	bool computed;
	bool equilibrated;
	bool isRefMove;
};

}

#endif /* ENTITY_H_ */
