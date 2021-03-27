/*
 * LevelEditor.cpp
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
//Put LevelEditor.h in last because of Xlib defines (True False Bool None) which must be undef
#include "Entity.h"

#define CASE_MOVE_TRANSLATION_PRECISION 0.05
#define CASE_MOVE_ROTATION_PRECISION 0.01

inline float Normalize(float v1, float v2, float v3)
{
	return sqrtf(v1 * v1 + v2 * v2 + v3 * v3);
}

namespace BallGame {

BaseEntity::BaseEntity()
{
	Engine = NULL;
	OgreEntity = NULL;
	GroupPtr = NULL;
	type = Case;
}

BaseEntity::BaseEntity(const BaseEntity &other, const String &NamePrefix)
{
	LOG << "Copying " << other.getName() << std::endl;
	Engine = other.Engine;
	GroupPtr = NULL;
	String name(NamePrefix);
	name += "-";
	name += other.getName();
	type = other.type;
	InitialPos = other.InitialPos;
	InitialOrientation = other.InitialOrientation;
	InitialScale = other.InitialScale;
	Ogre::SceneNode *Node = (Ogre::SceneNode*)other.Engine->getSceneManager()->getRootSceneNode()->createChild(name, InitialPos, InitialOrientation);
	OgreEntity = NULL;
	setOgreNode(Node);
}

Entity::Entity(const dMatrix& matrix) :
	m_curPosition (matrix.m_posit),
	m_nextPosition (matrix.m_posit),
	m_curRotation (dQuaternion (matrix)),
	m_nextRotation (dQuaternion (matrix))
{
	Body = NULL;
}

Entity::Entity()
{
	Body = NULL;
}

Entity::Entity(const Entity &other, const String &NamePrefix) : BaseEntity(other, NamePrefix)
{
	LOG << "Copying " << other.getName() << std::endl;
	Body = NULL;
	Ogre::SceneManager *mSceneMgr = other.Engine->getSceneManager();
	Ogre::MeshPtr Mesh = ((Ogre::Entity*)other.OgreEntity->getAttachedObject(0))->getMesh();
	Ogre::Entity* ogreEntity = mSceneMgr->createEntity(Mesh);
	OgreEntity->attachObject(ogreEntity);
	((Ogre::Entity*)OgreEntity->getAttachedObject(0))->getUserObjectBindings().setUserAny(Ogre::Any(this));
}

void BaseEntity::DisplaySelectedBox(bool display)
{
	OgreEntity->showBoundingBox(display);
}

void BaseEntity::Finalize(void)
{
	setOgreNode(NULL);
}

void Entity::Finalize(void)
{
	setNewtonBody(NULL);
	BaseEntity::Finalize();
}

const AxisAlignedBox &BaseEntity::getWorldAABB(void) const
{
	LOG << "AABB Base" << std::endl;
	return OgreEntity->_getWorldAABB();
}

void BaseEntity::setOgreNode(SceneNode *node)
{
	if(OgreEntity != NULL)
	{
		LOG << "Remove Ogre " << OgreEntity->getName() << std::endl;
		SceneNode *parent = (SceneNode*)OgreEntity->getParent();
		parent->removeAndDestroyChild(OgreEntity->getName());
	}
	OgreEntity = node;
	if(node != NULL)
	{
		node->_setDerivedPosition(InitialPos);
//		LOG << "Entity " << node->getName() << " (" << node << ") set position {" << InitialPos.x << ", " << InitialPos.y << ", " << InitialPos.z << "}" << std::endl;
		node->setScale(InitialScale);
//		LOG << "Entity" << this << "set scale {" << InitialScale.x << ", " << InitialScale.y << ", " << InitialScale.z << "}" << std::endl;
		node->_setDerivedOrientation(InitialOrientation);
//		LOG << "Entity" << this << "set orientation {" << InitialOrientation.x << ", " << InitialOrientation.y << ", " << InitialOrientation.z << ", " << InitialOrientation.w << "}" << std::endl;
	}
}

inline void BaseEntity::ResetToInitial(void)
{
	if(OgreEntity != NULL)
	{
		LOG << "Reset " << getName() << " To " << InitialPos << " / " << InitialOrientation << " / " << InitialScale << std::endl;
		setRelativeScale(InitialScale);
		setAbsolutePosition(InitialPos);
		setAbsoluteOrientation(InitialOrientation);
	}
}

void Entity::setNewtonBody(NewtonBody *body)
{
	if(Body != NULL)
		NewtonDestroyBody(Body);
	Body = body;
	if(body != NULL)
	{
		dMatrix matrix;
		NewtonBodySetUserData(body, this);
		NewtonBodyGetMatrix(body, &matrix[0][0]);
		m_curPosition = matrix.m_posit;
		m_nextPosition = matrix.m_posit;
		m_curRotation = dQuaternion(matrix);
		m_nextRotation = dQuaternion(matrix);
	}
}

void Entity::SetMatrixUsafe(const dQuaternion& rotation, const dVector& position)
{
	m_curPosition = m_nextPosition;
	m_curRotation = m_nextRotation;

	m_nextPosition = position;
	m_nextRotation = rotation;

	dFloat angle = m_curRotation.DotProduct(m_nextRotation);
	if (angle < 0.0f) {
		m_curRotation.Scale(-1.0f);
		LOG << "angle < 0 : " << angle << std::endl;
	}
}

void Entity::TransformCallback(const NewtonBody* body, const dFloat* matrix, int threadIndex)
{
//	LOG << "TransformCallback" << std::endl;
	Entity* const Ent = (Entity*) NewtonBodyGetUserData(body);
	if (Ent)
	{
		dMatrix transform(matrix);
		dQuaternion rot;
		NewtonBodyGetRotation(body, &rot.m_x);
		Ent->SetMatrixUsafe(rot, transform.m_posit);


		Vector3 NewPosition(Ent->m_curPosition.m_x, Ent->m_curPosition.m_y, Ent->m_curPosition.m_z);
		Quaternion NewOrientation(Ent->m_curRotation.m_w, Ent->m_curRotation.m_x, Ent->m_curRotation.m_y, Ent->m_curRotation.m_z);

		//scene->Lock(Entity->m_lock);
		//scene->Unlock(Entity->m_lock);
//		LOG << "Entity " << Entity->getName() << " transform " << "position {" << NewPosition.x << ", " << NewPosition.y << ", " << NewPosition.z << "}" << std::endl;
//		LOG << "Entity " << Entity->getName() << " RelativeOrientation {" << NewOrientation.w << ", " << NewOrientation.x << ", " << NewOrientation.y << ", " << NewOrientation.z << "}" << std::endl;
		Ent->OgreEntity->_setDerivedPosition(NewPosition);
		Ent->OgreEntity->_setDerivedOrientation(NewOrientation);
	}
}

void BaseEntity::copyOgreToInitial(void)
{
	if(OgreEntity != NULL)
	{
		InitialPos = getAbsolutePosition();
		InitialOrientation = getAbsoluteOrientation();
		InitialScale = getAbsoluteScale();
		LOG << getName() << " InitialPos=" << InitialPos << ", InitialOrientation=" << InitialOrientation << ", InitialScale=" << InitialScale << std::endl;
	}
}

dMatrix * Entity::PrepareNewtonBody(dVector &NewtonBodyLocation, dVector &NewtonBodySize)
{
	copyOgreToInitial();
	NewtonBodyLocation.m_x = InitialPos.x;
	NewtonBodyLocation.m_y = InitialPos.y;
	NewtonBodyLocation.m_z = InitialPos.z;
	NewtonBodyLocation.m_w = 1;
	Ogre::Entity *ogreEntity = (Ogre::Entity*)OgreEntity->getAttachedObject(0);
	Vector3 AABB(ogreEntity->getBoundingBox().getSize());
	NewtonBodySize.m_x = AABB.x * InitialScale.x;
	NewtonBodySize.m_y = AABB.y * InitialScale.y;
	NewtonBodySize.m_z = AABB.z * InitialScale.z;
	NewtonBodySize.m_w = 0.0f;

	return new dMatrix(InitialOrientation.getPitch(false).valueRadians(), InitialOrientation.getYaw(false).valueRadians(), InitialOrientation.getRoll(false).valueRadians(), NewtonBodyLocation);
}

BallEntity::BallEntity(const dMatrix& matrix):Entity(matrix)
{
	InitialMass = 1;
	type = Ball;
}

BallEntity::BallEntity()
{
	InitialMass = 1;
	type = Ball;
}

BallEntity::BallEntity(const BallEntity &other, const String &NamePrefix) : Entity(other, NamePrefix)
{
	InitialMass = other.InitialMass;
	type = Ball;
}

BallEntity::~BallEntity()
{
	CleanupForces();
}

void BallEntity::AddForceVector(dVector *force)
{
//	LOG << "Add Force {" << (*force)[0] << ", " << (*force)[1] << ", " << (*force)[2] << "} On ball" << std::endl;
	Forces.Append(force);
}

void BallEntity::CleanupForces(void)
{
	dVector *f;
	do
	{
		f = GetForceVector();
		if(f != NULL)
			delete f;
	}
	while(f != NULL);
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

void BallEntity::CreateNewtonBody(NewtonWorld *m_world)
{
	dVector NewtonBodyLocation;
	dVector NewtonBodySize;
	NewtonBody *newtonBody;
	dMatrix *bodymatrix = PrepareNewtonBody(NewtonBodyLocation, NewtonBodySize);

	LOG << "Place a Ball" << std::endl;

	int defaultMaterialID = NewtonMaterialGetDefaultGroupID (m_world);
	newtonBody = WorldAddBall(m_world, ((BallEntity*)this)->InitialMass, NewtonBodySize, defaultMaterialID, *bodymatrix);
	setNewtonBody(newtonBody);

	delete bodymatrix;
}

dFloat BallEntity::getMass(void)
{
	dFloat inertx, inerty, inertz, ret;
	NewtonBodyGetMass(Body, &ret, &inertx, &inerty, &inertz);
	return ret;
}

void BallEntity::setMass(dFloat newMass)
{
	LOG << "Ball new mass : " << this << std::endl;
	NewtonCollision *collision = NewtonBodyGetCollision(Body);
	NewtonBodySetMassProperties(Body, newMass, collision);
}

CaseEntity::CaseEntity(const dMatrix& matrix, enum CaseType _type):Entity(matrix)
{
	type = _type;
	this->BaseEntity::type = Case;
	force_to_apply = NAN;
	force_direction = NULL;
	MovementToDo = NULL;
	RefMove = NULL;
}

CaseEntity::CaseEntity(enum CaseType _type)
{
	type = _type;
	this->BaseEntity::type = Case;
	force_to_apply = NAN;
	force_direction = NULL;
	MovementToDo = NULL;
	RefMove = NULL;
}

CaseEntity::CaseEntity(const CaseEntity &other, const String &NamePrefix) : Entity(other, NamePrefix)
{
	type = other.type;
	this->BaseEntity::type = Case;
	force_to_apply = other.force_to_apply;
	if(other.force_direction != NULL)
		force_direction = new dVector(*(other.force_direction));
	else
		force_direction = NULL;
	MovementToDo = NULL;
	if(other.MovementToDo != NULL)
	{
		SetMoveTriggered(other.MovementToDo->is_launched_by_collide);
		auto iter(other.MovementToDo->Moves.begin());
		while(iter != other.MovementToDo->Moves.end())
		{
			struct MovementStep *step = *(iter++);
			if(step == NULL)
				continue;
			AddMovePoint(step->RelativePosition, step->TranslateSpeed, step->waittime, step->RelativeOrientation, step->RotateSpeed, step->correlated_speed);
		}
	}
	RefMove = NULL;
	CreateNewtonBody(Engine->GetNewton());
}

CaseEntity::~CaseEntity()
{
	BallsUnderCollide.clear();
	SetForceToApply(NAN,  NULL);
	if(MovementToDo != NULL)
		delete MovementToDo;
}

void CaseEntity::SetForceToApply(float force, dVector *direction)
{
	force_to_apply = force;
	if(force_direction != NULL)
		delete force_direction;
	force_direction = direction;
}

void CaseEntity::AddMovePoint(const Vector3 &GoalPos, float speed, unsigned64 waittime, const Quaternion &GoalAngle, float RotateSpeed, bool correlated_speed)
{
	if(MovementToDo == NULL)
		MovementToDo = new struct Movement;
	MovementToDo->is_computed = false;
	struct MovementStep *step = new struct MovementStep;
	step->RelativePosition = GoalPos;
	step->TranslateSpeed = speed;
	step->waittime = waittime;
	step->RelativeOrientation = GoalAngle;
	step->RotateSpeed = RotateSpeed;
	step->correlated_speed = correlated_speed;
	MovementToDo->Moves.push_back(step);
	LOG << "Case " << getName() << " Add Move Point" << std::endl;
}

void CaseEntity::SetMoveTriggered(bool trigger)
{
	if(MovementToDo != NULL)
	{
		MovementToDo->is_launched_by_collide = trigger;
		LOG << "Move is " << String(trigger == false ? "not " : "") << "triggered" << std::endl;
	}
}

void CaseEntity::FillComboboxWithMoves(CEGUI::Combobox *box)
{
	while (box->getItemCount())
	{
		CEGUI::ListboxTextItem *item = (CEGUI::ListboxTextItem*)box->getListboxItemFromIndex(0);
		box->removeItem(item);
	}
	box->setText("");
	if(MovementToDo != NULL)
	{
		int cmpt = 0;
		std::list<struct MovementStep*>::iterator iter(MovementToDo->Moves.begin());
		while(iter != MovementToDo->Moves.end())
		{
			struct MovementStep *step = *(iter++);
			if(step == NULL)
				continue;
			CEGUI::ListboxTextItem *item = new CEGUI::ListboxTextItem(String("Step - ") + std::to_string(cmpt++));
			item->setUserData(step);
			box->addItem(item);
		}
	}

	box->setSize(CEGUI::USize(CEGUI::UDim(0, 150), CEGUI::UDim(0, 40 * (box->getItemCount() + 1))));
}

void CaseEntity::DisplaySelectedMove(void *vstep, CEGUI::Editbox *TSpeed, CEGUI::Editbox *RSpeed, CEGUI::Editbox *WaitTime, CEGUI::ToggleButton *CorrelateSpeeds)
{
	struct MovementStep *step = (struct MovementStep*)vstep;
	setRelativePosition(step->RelativePosition);
	setRelativeOrientation(step->RelativeOrientation);

	TSpeed->setText(std::to_string(step->TranslateSpeed));
	WaitTime->setText(std::to_string(step->waittime));
	CorrelateSpeeds->setSelected(step->correlated_speed);
	if(step->correlated_speed == true)
		RSpeed->setEnabled(false);
	RSpeed->setText(std::to_string(step->RotateSpeed));
}

void CaseEntity::UpdateSelectedMove(void *vstep, const Vector3 &GoalPos, float TSpeed, const Quaternion &GoalAngle, float RSpeed, unsigned64 WaitTime, bool CorrelateSpeeds)
{
	struct MovementStep *step = (struct MovementStep*)vstep;
	step->RelativePosition = GoalPos;
	step->TranslateSpeed = TSpeed;
	step->RelativeOrientation = GoalAngle;
	step->RotateSpeed = RSpeed;
	step->waittime = WaitTime;
	step->correlated_speed = CorrelateSpeeds;
	MovementToDo->is_computed = false;
}

void CaseEntity::DeletedMove(void *vstep)
{
	struct MovementStep *step_to_del = (struct MovementStep*)vstep;
	if(MovementToDo != NULL)
	{
		std::list<struct MovementStep*>::iterator iter(MovementToDo->Moves.begin());
		while(iter != MovementToDo->Moves.end())
		{
			struct MovementStep *step = *iter;
			if(step != NULL)
			{
				if(step == step_to_del)
				{
					iter = MovementToDo->Moves.erase(iter);
					break;
				}
			}
			iter++;
		}
		if(MovementToDo->Moves.empty() == true)
		{
			delete MovementToDo;
			MovementToDo = NULL;
		}
	}
}

inline void CaseEntity::setRefMove(GroupEntity *Grp)
{
	RefMove = Grp;
	RefMove->setisRefMove(true);
}

void CaseEntity::ComputeMove(void)
{
	if(MovementToDo == NULL || MovementToDo->is_computed == true)
		return;

	Quaternion ActuOri = getAbsoluteOrientation();
	Vector3 ActuPos = getAbsolutePosition();



	//First pass to computes Absolutes coords for each Moves.
	//As Correlate Speeds depends of the previous, the first needs the last !
	std::list<struct MovementStep*>::iterator iterRelToAbs(MovementToDo->Moves.begin());
	while(iterRelToAbs != MovementToDo->Moves.end())
	{
		struct MovementStep *step = *(iterRelToAbs++);
		if(step != NULL)
		{
			setRelativePosition(step->RelativePosition);
			setRelativeOrientation(step->RelativeOrientation);
			step->AbsolutePosition = getAbsolutePosition();
			step->AbsoluteOrientation = getAbsoluteOrientation();

			LOG << "Compute Step, RelOri = " << step->RelativeOrientation << ", AbsOri = " << step->AbsoluteOrientation << std::endl;
			LOG << "Compute Step, RelPos = " << step->RelativePosition << ", AbsPos = " << step->AbsolutePosition << std::endl;
		}
	}

	//Second pass to computes Correlated Speeds.
	std::list<struct MovementStep*>::iterator iter(MovementToDo->Moves.begin());
	while(iter != MovementToDo->Moves.end())
	{
		struct MovementStep *step = *iter;
		if(step != NULL && step->correlated_speed == true && MovementToDo->Moves.size() > 1)
		{
			if(step->TranslateSpeed == 0 || isnanf(step->TranslateSpeed))
			{
				step->correlated_speed = false;
				iter++;
				continue;
			}
			Vector3 prevpos;
			Quaternion prevori;
			double NormDistanceT, NormDistanceR;
			std::list<struct MovementStep*>::reverse_iterator previter(MovementToDo->Moves.end());
			if(iter != MovementToDo->Moves.begin())
			{
				while(*previter != step)
					previter++;
				previter++;
			}

			struct MovementStep *prevstep = *previter;
			prevpos = prevstep->AbsolutePosition;
			prevori = prevstep->AbsoluteOrientation;

			LOG << "Pos " << step->AbsolutePosition << ", Ori " << step->AbsoluteOrientation << std::endl;
			LOG << "PrevPos " << prevpos << ", PrevOri " << prevori << std::endl;
			NormDistanceT = Normalize(step->AbsolutePosition.x - prevpos.x,
					step->AbsolutePosition.y - prevpos.y,
					step->AbsolutePosition.z - prevpos.z);
			NormDistanceR = Normalize(step->AbsoluteOrientation.getPitch().valueRadians() - prevori.getPitch().valueRadians(),
					step->AbsoluteOrientation.getYaw().valueRadians() - prevori.getYaw().valueRadians(),
					step->AbsoluteOrientation.getRoll().valueRadians() - prevori.getRoll().valueRadians());

			if(isinf(NormDistanceT) || isnan(NormDistanceT))
				NormDistanceT = 0;
			if(isinf(NormDistanceR) || isnan(NormDistanceR))
				NormDistanceR = 0;
			if(NormDistanceT == 0 || NormDistanceR == 0)
			{
				step->correlated_speed = false;
				iter++;
				continue;
			}

			step->RotateSpeed = step->TranslateSpeed * NormDistanceR / NormDistanceT;
			LOG << "NormDistanceR = " << NormDistanceR << ", NormDistanceT = " << NormDistanceT << ", TranslateSpeed = " << step->TranslateSpeed << ", RotateSpeed = " << step->RotateSpeed << std::endl;
		}

		iter++;
	}

	setAbsolutePosition(ActuPos);
	setAbsoluteOrientation(ActuOri);

	MovementToDo->is_computed = true;
}

bool CaseEntity::CaseMove(unsigned64 microseconds, dFloat timestep)
{
	if(MovementToDo == NULL)
		return false;

	if(MovementToDo->is_computed == false)
		return false;

	bool MoveToNextPoint = false;
	bool MustTranslate = false;
	bool MustRotate = false;
	Vector3 *ToReachPos = NULL;
	Quaternion *ToReachRot = NULL;
	if(MovementToDo->is_launched_by_collide == false)
		MoveToNextPoint = true;
	else
	{
		if(BallsUnderCollide.empty() == false)
			MoveToNextPoint = true;
		else if(MovementToDo->actual != NULL)
			MoveToNextPoint = true;
	}

	if(MoveToNextPoint)
	{
		if(MovementToDo->actual == NULL)
		{
			MovementToDo->actual = *(MovementToDo->Moves.begin());
			MovementToDo->foreignedtime = 0;
		}
		ToReachPos = &MovementToDo->actual->AbsolutePosition;
		ToReachRot = &MovementToDo->actual->AbsoluteOrientation;
	}
	else
		return false;


	if(ToReachPos != NULL)
	{
		dFloat DiffPos[3], MovePos[3];
		dFloat DiffAngle[3], RotatePos[3];
		if(!isnanf(MovementToDo->actual->TranslateSpeed))
		{
			DiffPos[0] = ToReachPos->x - getAbsolutePosition().x;
			DiffPos[1] = ToReachPos->y - getAbsolutePosition().y;
			DiffPos[2] = ToReachPos->z - getAbsolutePosition().z;
			dFloat Norm = Normalize(DiffPos[0], DiffPos[1], DiffPos[2]);
			LOG << "TranslatePos = {" << MovePos[0] << ", " << MovePos[1] << ", " << MovePos[2] << "}"
					<< " From " << getAbsolutePosition() << " To " << *ToReachPos
					<< std::endl;
			if (DiffPos[0] > CASE_MOVE_TRANSLATION_PRECISION)
			{
				MovePos[0] = MovementToDo->actual->TranslateSpeed;
				MustTranslate = true;
			}
			else if (DiffPos[0] < -CASE_MOVE_TRANSLATION_PRECISION)
			{
				MovePos[0] = -1 * MovementToDo->actual->TranslateSpeed;
				MustTranslate = true;
			}
			else
				MovePos[0] = 0;
			if (DiffPos[1] > CASE_MOVE_TRANSLATION_PRECISION)
			{
				MovePos[1] = MovementToDo->actual->TranslateSpeed;
				MustTranslate = true;
			}
			else if (DiffPos[1] < -CASE_MOVE_TRANSLATION_PRECISION)
			{
				MovePos[1] = -1 * MovementToDo->actual->TranslateSpeed;
				MustTranslate = true;
			}
			else
				MovePos[1] = 0;
			if (DiffPos[2] > CASE_MOVE_TRANSLATION_PRECISION)
			{
				MovePos[2] = MovementToDo->actual->TranslateSpeed;
				MustTranslate = true;
			}
			else if (DiffPos[2] < -CASE_MOVE_TRANSLATION_PRECISION)
			{
				MovePos[2] = -1 * MovementToDo->actual->TranslateSpeed;
				MustTranslate = true;
			}
			else
				MovePos[2] = 0;
			if(MustTranslate && Norm != 0)
			{
				LOG << "Must Translate = {" << MovePos[0] << ", " << MovePos[1] << ", " << MovePos[2] << "}" << std::endl;
				MovePos[0] *= fabs(DiffPos[0] / Norm);
				MovePos[1] *= fabs(DiffPos[1] / Norm);
				MovePos[2] *= fabs(DiffPos[2] / Norm);
				LOG << "Translate Proportionned = {" << MovePos[0] << ", " << MovePos[1] << ", " << MovePos[2] << "}" << std::endl;
			}
		}
		if(!isnanf(MovementToDo->actual->RotateSpeed))
		{
			DiffAngle[0] = ToReachRot->getPitch().valueRadians() - getAbsoluteOrientation().getPitch().valueRadians();
			DiffAngle[1] = ToReachRot->getYaw().valueRadians() - getAbsoluteOrientation().getYaw().valueRadians();
			DiffAngle[2] = ToReachRot->getRoll().valueRadians() - getAbsoluteOrientation().getRoll().valueRadians();
			while(DiffAngle[0] >= M_PI)
				DiffAngle[0] -= 2*M_PI;
			while(DiffAngle[0] <= -M_PI)
				DiffAngle[0] += 2*M_PI;

			while(DiffAngle[1] >= M_PI)
				DiffAngle[1] -= 2*M_PI;
			while(DiffAngle[1] <= -M_PI)
				DiffAngle[1] += 2*M_PI;

			while(DiffAngle[2] >= M_PI)
				DiffAngle[2] -= 2*M_PI;
			while(DiffAngle[2] <= -M_PI)
				DiffAngle[2] += 2*M_PI;
			dFloat Norm = Normalize(DiffAngle[0], DiffAngle[1], DiffAngle[2]);
			LOG << "DiffAngle = {" << DiffAngle[0] << ", " << DiffAngle[1] << ", " << DiffAngle[2] << "}"
					<< " From " << getAbsoluteOrientation() << " To " << *ToReachRot
					<< std::endl;
			LOG << "Rotate from {" << getAbsoluteOrientation().getRoll(false).valueDegrees()
					<< ", " << getAbsoluteOrientation().getPitch(false).valueDegrees()
					<< ", " << getAbsoluteOrientation().getYaw(false).valueDegrees()
					<< "} To {"	<< ToReachRot->getRoll(false).valueDegrees()
					<< ", " << ToReachRot->getPitch(false).valueDegrees()
					<< ", " << ToReachRot->getYaw(false).valueDegrees()
					<< "}" << std::endl;
			LOG << "Rotate from {" << getAbsoluteOrientation().getRoll().valueDegrees()
					<< ", " << getAbsoluteOrientation().getPitch().valueDegrees()
					<< ", " << getAbsoluteOrientation().getYaw().valueDegrees()
					<< "} To {"	<< ToReachRot->getRoll().valueDegrees()
					<< ", " << ToReachRot->getPitch().valueDegrees()
					<< ", " << ToReachRot->getYaw().valueDegrees()
					<< "}" << std::endl;
			if (DiffAngle[0] > CASE_MOVE_ROTATION_PRECISION)
			{
				RotatePos[0] = MovementToDo->actual->RotateSpeed;
				MustRotate = true;
			}
			else if (DiffAngle[0] < -CASE_MOVE_ROTATION_PRECISION)
			{
				RotatePos[0] = -1 * MovementToDo->actual->RotateSpeed;
				MustRotate = true;
			}
			else
				RotatePos[0] = 0;
			if (DiffAngle[1] > CASE_MOVE_ROTATION_PRECISION)
			{
				RotatePos[1] = MovementToDo->actual->RotateSpeed;
				MustRotate = true;
			}
			else if (DiffAngle[1] < -CASE_MOVE_ROTATION_PRECISION)
			{
				RotatePos[1] = -1 * MovementToDo->actual->RotateSpeed;
				MustRotate = true;
			}
			else
				RotatePos[1] = 0;
			if (DiffAngle[2] > CASE_MOVE_ROTATION_PRECISION)
			{
				RotatePos[2] = MovementToDo->actual->RotateSpeed;
				MustRotate = true;
			}
			else if (DiffAngle[2] < -CASE_MOVE_ROTATION_PRECISION)
			{
				RotatePos[2] = -1 * MovementToDo->actual->RotateSpeed;
				MustRotate = true;
			}
			else
				RotatePos[2] = 0;
			if(MustRotate && Norm != 0)
			{
				LOG << "Must Rotate = {" << RotatePos[0] << ", " << RotatePos[1] << ", " << RotatePos[2] << "}" << std::endl;
				RotatePos[0] *= fabs(DiffAngle[0] / Norm);
				RotatePos[1] *= fabs(DiffAngle[1] / Norm);
				RotatePos[2] *= fabs(DiffAngle[2] / Norm);
				LOG << "Rotate Proportionned = {" << RotatePos[0] << ", " << RotatePos[1] << ", " << RotatePos[2] << "}" << std::endl;
			}
		}
		if(MustRotate == false && MustTranslate == false)// So we have reached the position
		{
			if(MovementToDo->foreignedtime == 0)
				MovementToDo->foreignedtime = microseconds;
			else if (MovementToDo->foreignedtime + MovementToDo->actual->waittime < microseconds)
			{
				LOG << "Change Current Move" << std::endl;
				//Move has been reached. Place the following !
				std::list<struct MovementStep*>::iterator iter(MovementToDo->Moves.begin());
				while(iter != MovementToDo->Moves.end())
				{
					struct MovementStep *step = *iter;
					if(step != NULL)
					{
						if(step == MovementToDo->actual)
						{
							iter++;
							if(iter == MovementToDo->Moves.end())
							{
								if(MovementToDo->is_launched_by_collide == false || BallsUnderCollide.empty() == false)
								{
									LOG << "Cycling" << std::endl;
									MovementToDo->actual = *(MovementToDo->Moves.begin());
								}
								else
								{
									LOG << "Launched by collide " << MovementToDo->is_launched_by_collide
											<< " && Balls under collide " << BallsUnderCollide.empty()
											<< std::endl;
									MovementToDo->actual = NULL;
								}
								MovementToDo->foreignedtime = 0;
							}
							else
							{
								MovementToDo->actual = *iter;
								MovementToDo->foreignedtime = 0;
							}
							break;
						}
					}
					iter++;
				}
			}
			else
				LOG << "Time is " << microseconds << " : Waiting until " << MovementToDo->foreignedtime + MovementToDo->actual->waittime << " before Changing Current Move" << std::endl;
		}
		//Apply Current Move.
		if(MustTranslate)
		{
//			LOG << "Move to next point : from " << getAbsolutePosition() << " to " << *(ToReachPos) << std::endl;
			NewtonBodySetVelocity(Body, MovePos);
		}
		if(MustRotate)
		{
//			LOG << "Rotate to next Orientation : from " << getAbsoluteOrientation() << " to " << *(ToReachRot) << std::endl;
			NewtonBodySetOmega(Body, RotatePos);
		}
		if(MustTranslate || MustRotate)
		{
			NewtonBodyIntegrateVelocity(Body, timestep);

			//For forcing transformation !
			dMatrix matrix;
			NewtonBodyGetMatrix(Body, &matrix[0][0]);
			NewtonBodySetMatrix(Body, &matrix[0][0]);
		}
	}
	return true;
}

void CaseEntity::setActualMoveStep(void *ptr)
{
	if(MovementToDo == NULL)
		return;
	struct MovementStep *steptoput = (struct MovementStep*)ptr;
	std::list<struct MovementStep*>::iterator iter(MovementToDo->Moves.begin());
	while(iter != MovementToDo->Moves.end())
	{
		struct MovementStep *step = *(iter++);
		if(step == steptoput)
		{
			MovementToDo->actual = step;
			return;
		}
	}
}

void CaseEntity::AddBallColliding(BallEntity *ball)
{
	if(ball == NULL)
		return;
	if(CheckIfAlreadyColliding(ball) == false)
		BallsUnderCollide.push_back(ball);
}

bool CaseEntity::CheckIfAlreadyColliding(BallEntity *ToCheck)
{
	bool ret = false;
	if(ToCheck == NULL)
		return ret;
	std::list<BallEntity*>::iterator iter(BallsUnderCollide.begin());
	while(iter != BallsUnderCollide.end())
	{
		BallEntity *Ball = *(iter++);
		if(Ball == ToCheck)
			return true;
	}
	return ret;
}

void CaseEntity::ApplyForceOnCollidingBalls(void)
{
	std::list<BallEntity*>::iterator iter(BallsUnderCollide.begin());
	while(iter != BallsUnderCollide.end())
	{
		BallEntity *Ball = *iter;
		if(Ball != NULL)
			ApplyForceOnBall(Ball);
		iter = BallsUnderCollide.erase(iter);
	}
}

void CaseEntity::ApplyForceOnBall(BallEntity *ball)
{
//	LOG << "Case " << this << " apply force on ball " << ball << std::endl;
	if(!isnanf(force_to_apply))
	{
		if(force_direction != NULL)
		{
			dVector *force = new dVector();
			*force = force_direction->Scale(force_to_apply);
			ball->AddForceVector(force);
		}
		else
		{
			dFloat velocityf[3];
			double Velocity[3], sum;
			ball->getVelocity(velocityf);
			Velocity[0] = (double)velocityf[0];
			Velocity[1] = (double)velocityf[1];
			Velocity[2] = (double)velocityf[2];
			sum = Normalize(Velocity[0], Velocity[1], Velocity[2]);
			Velocity[0] /= sum;
			Velocity[1] /= sum;
			Velocity[2] /= sum;//Like that we have normalization of velocity into percents, we can use it to scale force.
			dVector *force = new dVector(Velocity[0], Velocity[1], Velocity[2]);
			*force = force->Scale(force_to_apply);
			ball->AddForceVector(force);
		}
	}
	//AddBallColliding(ball);
}

void CaseEntity::CreateNewtonBody(NewtonWorld *m_world)
{
	dVector NewtonBodyLocation;
	dVector NewtonBodySize;
	NewtonBody *newtonBody;

	Ogre::Entity *ogreEntity = (Ogre::Entity*)OgreEntity->getAttachedObject(0);
	dMatrix *bodymatrix = PrepareNewtonBody(NewtonBodyLocation, NewtonBodySize);

	if(MovementToDo != NULL && RefMove != NULL)//Case will move, so Group is the relative move's ref
	{
		RefMove->setInitialPosition(InitialPos);
		setAbsolutePosition(InitialPos);
		RefMove->setInitialOrientation(InitialOrientation);
		setAbsoluteOrientation(InitialOrientation);
		MovementToDo->is_computed = false;
	}

	NewtonCollision *collision_tree = NULL;
	Matrix4 ogre_matrix;

	ogre_matrix.makeTransform(Vector3::ZERO, InitialScale, Quaternion::IDENTITY);
	const MeshPtr ptr = ogreEntity->getMesh();
	collision_tree = ParseEntity(m_world, ptr, ogre_matrix);

	int defaultMaterialID = NewtonMaterialGetDefaultGroupID (m_world);
	newtonBody = WorldAddCase(m_world, NewtonBodySize, defaultMaterialID, *bodymatrix, collision_tree);
	setNewtonBody(newtonBody);
}


GroupEntity::GroupEntity(String &name, Ogre::SceneManager* mSceneMgr)
{
	OgreEntity = (SceneNode*)mSceneMgr->getRootSceneNode()->createChild(name);
	type = Group;
	computed = false;
	equilibrated = false;
	isRefMove = false;
}

GroupEntity::GroupEntity(const GroupEntity &other, const String &NamePrefix) : BaseEntity(other, NamePrefix)
{
	type = Group;
	computed = false;
	equilibrated = false;

	isRefMove = other.isRefMove;
	if(isRefMove)
	{
		CaseEntity *childC = (CaseEntity*)*(other.childs.begin()), *DupC;
		DupC = new CaseEntity(*childC, NamePrefix);
		DupC->setRefMove(this);
		//Now RefMove is filled, we can call CreateNewtonBody
		DupC->CreateNewtonBody(Engine->GetNewton());
		AddChild((BaseEntity*)DupC);
	}
	else
	{
		auto iter(other.childs.begin());
		while(iter != other.childs.end())
		{
			BaseEntity *Ent = *(iter++), *DupEnt = NULL;
			if(Ent == NULL)
				continue;
			switch(Ent->getType())
			{
				case BaseEntity::Types::Ball :
					DupEnt = new BallEntity(*(BallEntity*)Ent, NamePrefix);
					break;
				case BaseEntity::Types::Case :
					DupEnt = new CaseEntity(*(CaseEntity*)Ent, NamePrefix);
					break;
				case BaseEntity::Types::Group :
					DupEnt = new GroupEntity(*(GroupEntity*)Ent, NamePrefix);
					break;
			}
			if(DupEnt != NULL)
				AddChild(DupEnt);
		}
	}
}

void GroupEntity::CreateNewtonBody(NewtonWorld *m_world)
{
	std::list<BaseEntity*>::iterator iter(childs.begin());
	while(iter != childs.end())
	{
		BaseEntity *Entity = *(iter++);
		if(Entity != NULL)
			Entity->CreateNewtonBody(m_world);
	}
}

void GroupEntity::Finalize(void)
{
	std::list<BaseEntity*>::iterator iter(childs.begin());
	while(iter != childs.end())
	{
		BaseEntity *Entity = *iter;
		if(Entity != NULL)
			Entity->changeOgreParent((SceneNode*)OgreEntity->getParent());
		iter = childs.erase(iter);
	}
	BaseEntity::Finalize();
}

void GroupEntity::AddChild(BaseEntity* child)
{
	LOG << "Add " << child->getName() << " to " << getName() << std::endl;
	childs.push_back(child);
	child->setGroup(this);
	computed = false;
	equilibrated = false;
}

bool GroupEntity::DelChild(BaseEntity* child)
{
	LOG << "Child " << child->getName() << " Removed from Group" << std::endl;
	Vector3 childpos = child->getAbsolutePosition();
	Vector3 childscale = child->getAbsoluteScale();
	Quaternion childori = child->getAbsoluteOrientation();
	child->setGroup(NULL);

//	Vector3 pos = child->getRelativePosition(), abspos = child->getAbsolutePosition();
//	LOG << "Child " << child->getName() << " position {" << pos.x << ", " << pos.y << ", " << pos.z << "}" << std::endl;
//	LOG << "Child " << child->getName() << " absposition {" << abspos.x << ", " << abspos.y << ", " << abspos.z << "}" << std::endl;

	std::list<BaseEntity*>::iterator iter(childs.begin());
	while(iter != childs.end())
	{
		BaseEntity *Entity = *iter;
		if(Entity == child)
		{
			Entity->changeOgreParent((SceneNode*)OgreEntity->getParent());
			childs.erase(iter);
			break;
		}
		iter++;
	}
	child->setAbsoluteOrientation(childori);
	child->setAbsolutePosition(childpos);
	child->setRelativeScale(childscale);
	return childs.empty();
}

bool GroupEntity::HasChild(BaseEntity* child)
{
	std::list<BaseEntity*>::iterator iter(childs.begin());
	while(iter != childs.end())
	{
		BaseEntity *Entity = *(iter++);
		if(Entity == child)
			return true;
	}
	return false;
}

void GroupEntity::FillListWithChilds(std::list<BaseEntity*> &list, bool recursive)
{
	std::list<BaseEntity*>::iterator iter(childs.begin());
	while(iter != childs.end())
	{
		BaseEntity *Entity = *(iter++);
		if(Entity == NULL)
			continue;
		LOG << "Add " << Entity->getName() << " to list" << std::endl;
		list.push_back(Entity);
		if(Entity->getType() == BaseEntity::Types::Group && recursive == true)
			((GroupEntity*)Entity)->FillListWithChilds(list, true);
	}
}

void GroupEntity::ResetToInitial(void)
{
	BaseEntity::ResetToInitial();
	std::list<BaseEntity*>::iterator iter(childs.begin());
	while(iter != childs.end())
	{
		BaseEntity *Entity = *(iter++);
		if(Entity != NULL)
			Entity->ResetToInitial();
	}
}

void GroupEntity::copyOgreToInitial(void)
{
	BaseEntity::copyOgreToInitial();
	std::list<BaseEntity*>::iterator iter(childs.begin());
	while(iter != childs.end())
	{
		BaseEntity *Entity = *(iter++);
		if(Entity != NULL)
			Entity->copyOgreToInitial();
	}
}

void GroupEntity::ResetToInitial(bool with_childs)
{
	BaseEntity::ResetToInitial();
	if(with_childs == false)
		return;
	std::list<BaseEntity*>::iterator iter(childs.begin());
	while(iter != childs.end())
	{
		BaseEntity *Entity = *(iter++);
		if(Entity != NULL)
			Entity->ResetToInitial();
	}
}

void GroupEntity::copyOgreToInitial(bool with_childs)
{
	BaseEntity::copyOgreToInitial();
	if(with_childs == false)
		return;
	std::list<BaseEntity*>::iterator iter(childs.begin());
	while(iter != childs.end())
	{
		BaseEntity *Entity = *(iter++);
		if(Entity != NULL)
			Entity->copyOgreToInitial();
	}
}

void _DisplayChilds(SceneNode *node)
{
	Node::ChildNodeIterator ite(node->getChildIterator());
	if(ite.hasMoreElements())
	{
		LOG << "Display childs of " << node->getName() << std::endl;
		while ( ite.hasMoreElements() )
		{
			SceneNode* child = static_cast<SceneNode*>(ite.getNext());
			if(child != NULL)
			{
				LOG << child->getName() << " Parent is " << child->getParent()->getName() << std::endl;
				_DisplayChilds(child);
			}
		}
		LOG << "End Display for " << node->getName() << std::endl;
	}
	else
		LOG << node->getName() << " Has no childs !" << std::endl;
}

void GroupEntity::ComputeChilds(void)
{
	if(computed == true)
		return;
	std::list<BaseEntity*>::iterator iter(childs.begin());
	while(iter != childs.end())
	{
		BaseEntity *Entity = *(iter++);
		if(Entity == NULL)
			continue;
		if(Entity->getType() == BaseEntity::Types::Group)
			((GroupEntity*)Entity)->ComputeChilds();
		LOG << "Child " << Entity->getName() << " (" << Entity << ") Added to Group" << std::endl;
		Entity->changeOgreParent(OgreEntity);
		Entity->ResetToInitial();
	}
//	_DisplayChilds(OgreEntity);
	computed = true;
}

void GroupEntity::setForceRecomputeChilds(bool force_compute)
{
	if(force_compute)
		computed = false;
	equilibrated = false;
	std::list<BaseEntity*>::iterator iter(childs.begin());
	while(iter != childs.end())
	{
		BaseEntity *Ent = *(iter++);
		if(Ent != NULL && Ent->getType() == BaseEntity::Types::Group)
		{
			GroupEntity *Grp =(GroupEntity*)Ent;
			Grp->setForceRecomputeChilds(force_compute);
		}
	}
}

const AxisAlignedBox &GroupEntity::getWorldAABB(void) const
{
	LOG << "AABB Group" << std::endl;
	return AABB;
}

void GroupEntity::computeWorldAABB(void)
{
	LOG << "Group " << getName() << " Compute AABB" << std::endl;
	std::list<BaseEntity*>::iterator iter(childs.begin());
	AABB.setMinimum(INFINITY, INFINITY, INFINITY);
	AABB.setMaximum(-INFINITY, -INFINITY, -INFINITY);
	while(iter != childs.end())
	{
		BaseEntity *Ent = *(iter++);
		if(Ent != NULL)
		{
			const Ogre::AxisAlignedBox &childAABB = Ent->getWorldAABB();
			if(childAABB.getMinimum().x < AABB.getMinimum().x)
			{
				AABB.setMinimumX(childAABB.getMinimum().x);
				LOG << "Set MinX to " << childAABB.getMinimum().x << std::endl;
			}
			if(childAABB.getMinimum().y < AABB.getMinimum().y)
			{
				AABB.setMinimumY(childAABB.getMinimum().y);
				LOG << "Set MinY to " << childAABB.getMinimum().y << std::endl;
			}
			if(childAABB.getMinimum().z < AABB.getMinimum().z)
			{
				AABB.setMinimumZ(childAABB.getMinimum().z);
				LOG << "Set MinZ to " << childAABB.getMinimum().z << std::endl;
			}
			if(childAABB.getMaximum().x > AABB.getMaximum().x)
			{
				AABB.setMaximumX(childAABB.getMaximum().x);
				LOG << "Set MaxX to " << childAABB.getMaximum().x << std::endl;
			}
			if(childAABB.getMaximum().y > AABB.getMaximum().y)
			{
				AABB.setMaximumY(childAABB.getMaximum().y);
				LOG << "Set MaxY to " << childAABB.getMaximum().y << std::endl;
			}
			if(childAABB.getMaximum().z > AABB.getMaximum().z)
			{
				AABB.setMaximumZ(childAABB.getMaximum().z);
				LOG << "Set MaxZ to " << childAABB.getMaximum().z << std::endl;
			}
		}
	}
}

void GroupEntity::ComputeAndEquilibrateChilds(void)
{
	ComputeChilds();
	EquilibrateAABBAroundOrigin();
}

void GroupEntity::EquilibrateAABBAroundOrigin(void)
{
	if(equilibrated == true)
		return;

	if(isRefMove == true)
	{
		BaseEntity *child = *(childs.begin());
		LOG << "Is a RefMove, set Position to " << child->getInitialPosition() << " and Orientation to " << child->getInitialOrientation() << std::endl;
		setInitialPosition(child->getInitialPosition());
		setInitialOrientation(child->getInitialOrientation());
		Vector3 scale(1.0, 1.0, 1.0);
		setInitialScale(scale);
		BaseEntity::ResetToInitial();
		child->ResetToInitial();
		computeWorldAABB();
		equilibrated = true;
		return;
	}

	Vector3 min(NAN, NAN, NAN), max(NAN, NAN, NAN);
	std::list<BaseEntity*>::iterator Eiter(childs.begin());
	while(Eiter != childs.end())
	{
		BaseEntity *Ent = *(Eiter++);
		if(Ent == NULL)
			continue;
		if(Ent->getType() == BaseEntity::Types::Group)
			((GroupEntity*)Ent)->EquilibrateAABBAroundOrigin();
		Ent->ResetToInitial();
		const AxisAlignedBox &EntAABB = Ent->getWorldAABB();
		Vector3 childmin = EntAABB.getMinimum();
		Vector3 childmax = EntAABB.getMaximum();

		if(min.isNaN())
		   min = childmin;
		if(max.isNaN())
		   max = childmax;

		min.x = std::min<Ogre::Real>(min.x, childmin.x);
		min.y = std::min<Ogre::Real>(min.y, childmin.y);
		min.z = std::min<Ogre::Real>(min.z, childmin.z);

		max.x = std::max<Ogre::Real>(max.x, childmax.x);
		max.y = std::max<Ogre::Real>(max.y, childmax.y);
		max.z = std::max<Ogre::Real>(max.z, childmax.z);
//		LOG << Ent->getName() << " child min " << childmin.x << ", " << childmin.y << ", " << childmin.z << std::endl;
//		LOG << Ent->getName() << " child max " << childmax.x << ", " << childmax.y << ", " << childmax.z << std::endl;
//		LOG << "=> min " << min.x << ", " << min.y << ", " << min.z << std::endl;
//		LOG << "=> max " << max.x << ", " << max.y << ", " << max.z << std::endl;
	}

	Vector3 pos = OgreEntity->getPosition();
//	LOG << "Node " << pos.x << ", " << pos.y << ", " << pos.z << std::endl;
	OgreEntity->translate((min + max) / 2);
	pos = OgreEntity->getPosition();
//	LOG << "After Compute Node " << pos.x << ", " << pos.y << ", " << pos.z << std::endl;

	std::list<BaseEntity*>::iterator Eiter2(childs.begin());
	while(Eiter2 != childs.end())
	{
		BaseEntity *Ent = *(Eiter2++);
		if(Ent == NULL)
			continue;
		Ent->ResetToInitial();
	}
	Vector3 GrpPos = OgreEntity->getPosition();
//	LOG << "After Equilibrate : Group(" << GrpPos.x << ", " << GrpPos.y << ", " << GrpPos.z << ")" << std::endl;
	copyOgreToInitial(false);
	computeWorldAABB();
	equilibrated = true;
}

void GroupEntity::DisplaySelectedBox(bool display)
{
	std::list<BaseEntity*>::iterator iter(childs.begin());
	while(iter != childs.end())
	{
		BaseEntity *Ent = *(iter++);
		if(Ent != NULL)
			Ent->DisplaySelectedBox(display);
	}
}

Ogre::SceneNode *CaseEntity::CreateForceArrows(Ogre::SceneManager *Scene)
{
	Ogre::ManualObject *ForcesArrows = new ManualObject("Arrows");
	Vector3 coords = OgreEntity->_getWorldAABB().getCenter();
	coords.z = OgreEntity->_getWorldAABB().getMaximum().z;
	ForcesArrows->begin("Material.001", Ogre::RenderOperation::OT_LINE_STRIP);
	ForcesArrows->position (0.0, 0.0, 0.0);
	ForcesArrows->position (40.0, 0.0, 0.0);
	ForcesArrows->position (0.0, 0.0, 0.0);
	ForcesArrows->position (0.0, 40.0, 0.0);
	ForcesArrows->position (0.0, 0.0, 0.0);
	ForcesArrows->position (0.0, 0.0, 40.0);
	ForcesArrows->end();
	LOG << "Create Arrows child" << std::endl;
	Ogre::SceneNode *node = (Ogre::SceneNode*)Scene->getRootSceneNode()->createChildSceneNode("Arrows");
	node->attachObject(ForcesArrows);
	node->setPosition(coords);
	if(force_direction != NULL && !isnanf(force_direction->m_x) && !isnanf(force_direction->m_y) && !isnanf(force_direction->m_z))
		node->setScale(force_direction->m_x, force_direction->m_y, force_direction->m_z);

	return node;
}

inline void MoveNode(Node *node, const Ogre::Vector3 &addPos)
{
	Vector3 pos = node->getPosition();
	pos += addPos;
	node->setPosition(pos);
}

inline void MoveNode(Node *node, float x, float y, float z)
{
	Vector3 addPos(x, y, z);
	MoveNode(node, addPos);
}

void GroupEntity::DisplayChilds(void)
{
	LOG << getName() << " Pos " << getAbsolutePosition() << " / " << getRelativePosition() << std::endl;
	std::list<BaseEntity*>::iterator iter(childs.begin());
	while(iter != childs.end())
	{
		BaseEntity *E = *(iter++);
		if(E != NULL)
		{
			LOG << E->getName() << " Pos " << E->getAbsolutePosition() << " / " << E->getRelativePosition() << std::endl;
			if(E->getType() == BaseEntity::Types::Group)
				((GroupEntity*)E)->DisplayChilds();
		}
	}
}

void BaseEntity::Move(float x, float y, float z)
{
	MoveNode(OgreEntity, x, y, z);
}

void BaseEntity::Move(const Vector3 &addPos)
{
	MoveNode(OgreEntity, addPos);
}

inline void ScaleNode(Node *node, const Ogre::Vector3 &addScale)
{
	Vector3 sc = node->getScale();
	sc += addScale;
	node->setScale(sc);

//	sc = node->getScale();
//	Vector3 dsc = node->_getDerivedScale();
//	LOG << "=> Scale = " << sc.x << ", " << sc.y << ", " << sc.z << std::endl;
//	LOG << "=> DerivatedScale = " << dsc.x << ", " << dsc.y << ", " << dsc.z << std::endl;
}

inline void ScaleNode(Node *node, float x, float y, float z)
{
	Vector3 addScale(x, y, z);
	ScaleNode(node, addScale);
}

void BaseEntity::Scale(float x, float y, float z)
{
	ScaleNode(OgreEntity, x, y, z);
}

void BaseEntity::Scale(const Vector3 &addScale)
{
	ScaleNode(OgreEntity, addScale);
}

void GroupEntity::Scale(float x, float y, float z)
{
	std::list<BaseEntity*>::iterator iter(childs.begin());
	while(iter != childs.end())
	{
		BaseEntity *Entity = *(iter++);
		if(Entity == NULL)
			continue;
		Vector3 childPos = Entity->getRelativePosition();
		Vector3 childScale = Entity->getRelativeScale();
		childPos.x *= x / childScale.x;
		childPos.y *= y / childScale.y;
		childPos.z *= z / childScale.z;
		if(Entity->getType() != BaseEntity::Types::Group)
			Entity->Scale(x, y, z);
		else
			((GroupEntity*)Entity)->Scale(x, y, z);
		//As scale parent is a factor and here we add scaling, not multiply it, we must do it ourself by scaling child and moving it consequently by the same factor (newscale / odlscale) !
		Entity->Move(childPos);
	}
}

inline void RotateNode(Node *node, float x, float y, float z)
{
	node->pitch(Degree(x));
	node->roll(Degree(z));
	node->yaw(Degree(y));
}

void BaseEntity::Rotate(float x, float y, float z)
{
	RotateNode(OgreEntity, x, y, z);
}

#define MASS_JSON_FIELD "Mass"

void BallEntity::ExportToJson(rapidjson::Value &v, rapidjson::Document::AllocatorType& allocator)
{
	Entity::ExportToJson(v, allocator);
	v.AddMember(MASS_JSON_FIELD, InitialMass, allocator);
}

#define CASETYPE_JSON_FIELD "Type"
#define FORCEPRESENT_JSON_FIELD "ForcePresent"
#define FORCEVALUE_JSON_FIELD "ForceValue"
#define FORCEDIRECTIONPRESENT_JSON_FIELD "ForceDirectionPresent"
#define FORCEDIRECTIONX_JSON_FIELD "ForceDirectionX"
#define FORCEDIRECTIONY_JSON_FIELD "ForceDirectionY"
#define FORCEDIRECTIONZ_JSON_FIELD "ForceDirectionZ"
#define FORCEDIRECTIONW_JSON_FIELD "ForceDirectionW"
#define MOVEPRESENT_JSON_FIELD "MovePresent"
#define MOVEISTRIGGERED_JSON_FIELD "MoveTriggered"
#define MOVES_JSON_FIELD "Moves"
#define MOVESTEPPOSITIONPRESENT_JSON_FIELD "MoveTransactionPresent"
#define MOVESTEPPOSITIONX_JSON_FIELD "MoveGoalPosX"
#define MOVESTEPPOSITIONY_JSON_FIELD "MoveGoalPosY"
#define MOVESTEPPOSITIONZ_JSON_FIELD "MoveGoalPosZ"
#define MOVESTEPTRANSLATIONSPEED_JSON_FIELD "MoveTranslateSpeed"
#define MOVESTEPROTATIONPRESENT_JSON_FIELD "MoveRotationPresent"
#define MOVESTEPROTATIONX_JSON_FIELD "MoveGoalRotX"
#define MOVESTEPROTATIONY_JSON_FIELD "MoveGoalRotY"
#define MOVESTEPROTATIONZ_JSON_FIELD "MoveGoalRotZ"
#define MOVESTEPROTATIONW_JSON_FIELD "MoveGoalRotW"
#define MOVESTEPROTATIONSPEED_JSON_FIELD "MoveRotationSpeed"
#define MOVESTEPROTATIONSPEEDCORRELATED_JSON_FIELD "MoveRotationSpeedCorrelated"
#define MOVESTEPWAITTIME_JSON_FIELD "MovePauseDuration"

void CaseEntity::ExportToJson(rapidjson::Value &v, rapidjson::Document::AllocatorType& allocator)
{
	Entity::ExportToJson(v, allocator);
	v.AddMember(CASETYPE_JSON_FIELD, (int)type, allocator);
	if(isnan(force_to_apply))
		v.AddMember(FORCEPRESENT_JSON_FIELD, false, allocator);
	else
	{
		v.AddMember(FORCEPRESENT_JSON_FIELD, true, allocator);
		v.AddMember(FORCEVALUE_JSON_FIELD, force_to_apply, allocator);
		if(force_direction == NULL)
			v.AddMember(FORCEDIRECTIONPRESENT_JSON_FIELD, false, allocator);
		else
		{
			dVector *force_dir = force_direction;
			v.AddMember(FORCEDIRECTIONPRESENT_JSON_FIELD, true, allocator);
			v.AddMember(FORCEDIRECTIONX_JSON_FIELD, force_dir->m_x, allocator);
			v.AddMember(FORCEDIRECTIONY_JSON_FIELD, force_dir->m_y, allocator);
			v.AddMember(FORCEDIRECTIONZ_JSON_FIELD, force_dir->m_z, allocator);
			v.AddMember(FORCEDIRECTIONW_JSON_FIELD, force_dir->m_w, allocator);
		}
	}
	if(MovementToDo == NULL)
		v.AddMember(MOVEPRESENT_JSON_FIELD, false, allocator);
	else
	{
		v.AddMember(MOVEPRESENT_JSON_FIELD, true, allocator);
		v.AddMember(MOVEISTRIGGERED_JSON_FIELD, MovementToDo->is_launched_by_collide, allocator);
		rapidjson::Value Moves(rapidjson::kArrayType);
		std::list<struct MovementStep*>::iterator iter(MovementToDo->Moves.begin());
		while(iter != MovementToDo->Moves.end())
		{
			struct MovementStep *step = *(iter++);
			if(step == NULL)
				continue;
			rapidjson::Value Step(rapidjson::kObjectType);
			if(!isnanf(step->TranslateSpeed))
			{
				Step.AddMember(MOVESTEPPOSITIONPRESENT_JSON_FIELD, true, allocator);
				Step.AddMember(MOVESTEPPOSITIONX_JSON_FIELD, step->RelativePosition.x, allocator);
				Step.AddMember(MOVESTEPPOSITIONY_JSON_FIELD, step->RelativePosition.y, allocator);
				Step.AddMember(MOVESTEPPOSITIONZ_JSON_FIELD, step->RelativePosition.z, allocator);
				Step.AddMember(MOVESTEPTRANSLATIONSPEED_JSON_FIELD, step->TranslateSpeed, allocator);
			}
			else
				Step.AddMember(MOVESTEPPOSITIONPRESENT_JSON_FIELD, false, allocator);
			if(!isnanf(step->RotateSpeed))
			{
				Step.AddMember(MOVESTEPROTATIONPRESENT_JSON_FIELD, true, allocator);
				Step.AddMember(MOVESTEPROTATIONX_JSON_FIELD, step->RelativeOrientation.x, allocator);
				Step.AddMember(MOVESTEPROTATIONY_JSON_FIELD, step->RelativeOrientation.y, allocator);
				Step.AddMember(MOVESTEPROTATIONZ_JSON_FIELD, step->RelativeOrientation.z, allocator);
				Step.AddMember(MOVESTEPROTATIONW_JSON_FIELD, step->RelativeOrientation.w, allocator);
				Step.AddMember(MOVESTEPROTATIONSPEED_JSON_FIELD, step->RotateSpeed, allocator);
				Step.AddMember(MOVESTEPROTATIONSPEEDCORRELATED_JSON_FIELD, step->correlated_speed, allocator);
			}
			else
				Step.AddMember(MOVESTEPROTATIONPRESENT_JSON_FIELD, false, allocator);
			Step.AddMember(MOVESTEPWAITTIME_JSON_FIELD, (uint64_t)step->waittime, allocator);

			Moves.PushBack(Step, allocator);
		}
		v.AddMember(MOVES_JSON_FIELD, Moves, allocator);
	}
}

void CaseEntity::CreateFromJson(rapidjson::Value &v, GameEngine *Game, NewtonWorld *m_world, Node *parent, String &nodeNamePrefix)
{
	ImportFromJson(v, Game, parent, nodeNamePrefix);

//	if(type == CaseEntity::CaseType::typeRamp)
//		LOG << "Place a Ramp" << std::endl;
//	else
//		LOG << "Place a Box" << std::endl;

	if(m_world)
		CreateNewtonBody(m_world);
}

void CaseEntity::ImportFromJson(rapidjson::Value &v, GameEngine *Game, Node *parent, String &nodeNamePrefix)
{
	Entity::ImportFromJson(v, Game, parent, nodeNamePrefix);
	float force_json =  NAN;
	dVector *direction_json = NULL;
	type = (CaseEntity::CaseType)v[CASETYPE_JSON_FIELD].GetInt();
	if(v[FORCEPRESENT_JSON_FIELD].GetBool() == true)
	{
		force_json = v[FORCEVALUE_JSON_FIELD].GetFloat();
		if(v[FORCEDIRECTIONPRESENT_JSON_FIELD].GetBool() == true)
		{
			float xjson, yjson, zjson, wjson;
			xjson = v[FORCEDIRECTIONX_JSON_FIELD].GetFloat();
			yjson = v[FORCEDIRECTIONY_JSON_FIELD].GetFloat();
			zjson = v[FORCEDIRECTIONZ_JSON_FIELD].GetFloat();
			wjson = v[FORCEDIRECTIONW_JSON_FIELD].GetFloat();
			direction_json = new dVector(xjson, yjson, zjson, wjson);
		}
	}
	SetForceToApply(force_json, direction_json);


	//Parsing Moves
	if(MovementToDo != NULL)
		delete MovementToDo;
	if(v[MOVEPRESENT_JSON_FIELD].GetBool() == true)
	{
		setRefMove(GroupPtr);
		LOG << "Entity " << getName() << " Moves is present, set RefMove to " << GroupPtr << std::endl;
		assert(GroupPtr != NULL);
		struct Movement *Movement = new struct Movement;
		Movement->is_launched_by_collide = v[MOVEISTRIGGERED_JSON_FIELD].GetBool();
		for(int cmpt = 0; cmpt < v[MOVES_JSON_FIELD].GetArray().Size(); cmpt++)
		{
			rapidjson::Value &Stepjson = v[MOVES_JSON_FIELD].GetArray()[cmpt];
			struct MovementStep *Step = new struct MovementStep;
			Step->waittime = (unsigned64)Stepjson[MOVESTEPWAITTIME_JSON_FIELD].GetUint64();
			if(Stepjson[MOVESTEPPOSITIONPRESENT_JSON_FIELD].GetBool() == true)
			{
				Step->RelativePosition.x = Stepjson[MOVESTEPPOSITIONX_JSON_FIELD].GetFloat();
				Step->RelativePosition.y = Stepjson[MOVESTEPPOSITIONY_JSON_FIELD].GetFloat();
				Step->RelativePosition.z = Stepjson[MOVESTEPPOSITIONZ_JSON_FIELD].GetFloat();
				Step->TranslateSpeed = Stepjson[MOVESTEPTRANSLATIONSPEED_JSON_FIELD].GetFloat();
			}
			else
				Step->TranslateSpeed = NAN;
			if(Stepjson[MOVESTEPROTATIONPRESENT_JSON_FIELD].GetBool() == true)
			{
				Step->RelativeOrientation.x = Stepjson[MOVESTEPROTATIONX_JSON_FIELD].GetFloat();
				Step->RelativeOrientation.y = Stepjson[MOVESTEPROTATIONY_JSON_FIELD].GetFloat();
				Step->RelativeOrientation.z = Stepjson[MOVESTEPROTATIONZ_JSON_FIELD].GetFloat();
				Step->RelativeOrientation.w = Stepjson[MOVESTEPROTATIONW_JSON_FIELD].GetFloat();
				Step->RotateSpeed = Stepjson[MOVESTEPROTATIONSPEED_JSON_FIELD].GetFloat();
				Step->correlated_speed = Stepjson[MOVESTEPROTATIONSPEEDCORRELATED_JSON_FIELD].GetBool();
			}
			else
				Step->RotateSpeed = NAN;
			Movement->Moves.push_back(Step);
		}
		MovementToDo = Movement;
	}
	else
		MovementToDo = NULL;
}

#define MESH_JSON_FIELD "Mesh"
#define NODENAME_JSON_FIELD "NodeName"
#define GROUPNAME_JSON_FIELD "GroupName"
#define POSITIONX_JSON_FIELD "PosX"
#define POSITIONY_JSON_FIELD "PosY"
#define POSITIONZ_JSON_FIELD "PosZ"
#define SCALEX_JSON_FIELD "ScaleX"
#define SCALEY_JSON_FIELD "ScaleY"
#define SCALEZ_JSON_FIELD "ScaleZ"
#define ORIENTATIONX_JSON_FIELD "OrientationX"
#define ORIENTATIONY_JSON_FIELD "OrientationY"
#define ORIENTATIONZ_JSON_FIELD "OrientationZ"
#define ORIENTATIONW_JSON_FIELD "OrientationW"

void BaseEntity::ExportToJson(rapidjson::Value &v, rapidjson::Document::AllocatorType& allocator)
{
	rapidjson::Value name;
	const char *ogrename = (const char*)OgreEntity->getName().c_str();
	name.SetString(ogrename, allocator);
	v.AddMember(NODENAME_JSON_FIELD, name, allocator);
	if(GroupPtr != NULL)
	{
		rapidjson::Value gname;
		const char *groupname = (const char*)GroupPtr->getName().c_str();
		gname.SetString(groupname, allocator);
		v.AddMember(GROUPNAME_JSON_FIELD, gname, allocator);
	}
	else
		v.AddMember(GROUPNAME_JSON_FIELD, "", allocator);
	LOG << getName() << " InitialPos=" << InitialPos << ", InitialOrientation=" << InitialOrientation << ", InitialScale=" << InitialScale << std::endl;
	v.AddMember(POSITIONX_JSON_FIELD, InitialPos.x, allocator);
	v.AddMember(POSITIONY_JSON_FIELD, InitialPos.y, allocator);
	v.AddMember(POSITIONZ_JSON_FIELD, InitialPos.z, allocator);
	v.AddMember(SCALEX_JSON_FIELD, InitialScale.x, allocator);
	v.AddMember(SCALEY_JSON_FIELD, InitialScale.y, allocator);
	v.AddMember(SCALEZ_JSON_FIELD, InitialScale.z, allocator);
	v.AddMember(ORIENTATIONX_JSON_FIELD, InitialOrientation.x, allocator);
	v.AddMember(ORIENTATIONY_JSON_FIELD, InitialOrientation.y, allocator);
	v.AddMember(ORIENTATIONZ_JSON_FIELD, InitialOrientation.z, allocator);
	v.AddMember(ORIENTATIONW_JSON_FIELD, InitialOrientation.w, allocator);
}

void Entity::ExportToJson(rapidjson::Value &v, rapidjson::Document::AllocatorType& allocator)
{
	Ogre::Entity *entity = (Ogre::Entity*)OgreEntity->getAttachedObject(0);
	const char *cname = (const char*)entity->getMesh().get()->getName().c_str();
	rapidjson::Value name;
	name.SetString(cname, allocator);
	v.AddMember(MESH_JSON_FIELD, name, allocator);
	BaseEntity::ExportToJson(v, allocator);
}

void BallEntity::CreateFromJson(rapidjson::Value &v, GameEngine *Game, NewtonWorld *m_world, Node *parent, String &nodeNamePrefix)
{
	ImportFromJson(v, Game, parent, nodeNamePrefix);

	if(m_world != NULL)
		CreateNewtonBody(m_world);
}

void BallEntity::ImportFromJson(rapidjson::Value &v, GameEngine *Game, Node *parent, String &nodeNamePrefix)
{
	Entity::ImportFromJson(v, Game, parent, nodeNamePrefix);
	InitialMass = v[MASS_JSON_FIELD].GetFloat();
}

void Entity::ImportFromJson(rapidjson::Value &v, GameEngine *Game, Node *parent, String &nodeNamePrefix)
{
	BaseEntity::ImportFromJson(v, Game, parent, nodeNamePrefix);
	const char *meshname = v[MESH_JSON_FIELD].GetString();
	Ogre::SceneManager *mSceneMgr = Game->getSceneManager();
	Ogre::Entity* ogreEntity = mSceneMgr->createEntity(meshname);
	OgreEntity->attachObject(ogreEntity);
	((Ogre::Entity*)OgreEntity->getAttachedObject(0))->getUserObjectBindings().setUserAny(Ogre::Any(this));
}

void BaseEntity::ImportFromJson(rapidjson::Value &v, GameEngine *Game, Node *parent, String &nodeNamePrefix)
{
//	LOG << "Mesh Entity Name : '" << ogreEntity->getName() << "'" << std::endl;
	InitialPos.x = v[POSITIONX_JSON_FIELD].GetFloat();
	InitialPos.y = v[POSITIONY_JSON_FIELD].GetFloat();
	InitialPos.z = v[POSITIONZ_JSON_FIELD].GetFloat();
	InitialScale.x = v[SCALEX_JSON_FIELD].GetFloat();
	InitialScale.y = v[SCALEY_JSON_FIELD].GetFloat();
	InitialScale.z = v[SCALEZ_JSON_FIELD].GetFloat();
	InitialOrientation.x = v[ORIENTATIONX_JSON_FIELD].GetFloat();
	InitialOrientation.y = v[ORIENTATIONY_JSON_FIELD].GetFloat();
	InitialOrientation.z = v[ORIENTATIONZ_JSON_FIELD].GetFloat();
	InitialOrientation.w = v[ORIENTATIONW_JSON_FIELD].GetFloat();

	SceneNode* ogreNode;
	const char *nodename_c = v[NODENAME_JSON_FIELD].GetString();
	String nodeName = nodeNamePrefix;
	nodeName += nodename_c;
	ogreNode = (SceneNode*)parent->createChild(nodeName, InitialPos);
	setOgreNode(ogreNode);
	const char *Groupname = v[GROUPNAME_JSON_FIELD].GetString();
	if(strcmp(Groupname, "") != 0)
	{
		String GrpName = nodeNamePrefix;
		GrpName += Groupname;
		bool is_for_import = nodeNamePrefix.empty() == false;
		GroupEntity *Grp = Game->findGroup(GrpName, is_for_import);
		if(Grp != NULL)
		{
			LOG << "Add Child " << nodeName << " (" << nodename_c << ") to Group " << GrpName << std::endl;
			Grp->AddChild(this);
		}
		else
			LOG << "Pb : Group " << GrpName << " not found !!" << std::endl;
	}
}

}
