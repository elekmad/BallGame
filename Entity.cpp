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
#include "Entity.h"

void EquilibrateAABBAroundOrigin(Node *node)
{
	Vector3 min(NAN, NAN, NAN), max(NAN, NAN, NAN);
	Node::ChildNodeIterator ite(node->getChildIterator());
	while ( ite.hasMoreElements() )
	{
	       SceneNode* child = static_cast<SceneNode*>(ite.getNext());
	       Vector3 childmin = child->_getWorldAABB().getMinimum();
	       Vector3 childmax = child->_getWorldAABB().getMaximum();

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
//	       LOG << "child min " << childmin.x << ", " << childmin.y << ", " << childmin.z << std::endl;
//	       LOG << "child max " << childmax.x << ", " << childmax.y << ", " << childmax.z << std::endl;
//	       LOG << "=> min " << min.x << ", " << min.y << ", " << min.z << std::endl;
//	       LOG << "=> max " << max.x << ", " << max.y << ", " << max.z << std::endl;
	}

//	Vector3 pos = node->getPosition();
//	LOG << "Node " << pos.x << ", " << pos.y << ", " << pos.z << std::endl;
	node->translate((min + max) / 2);
//	pos = node->getPosition();
//	LOG << "After Compute Node " << pos.x << ", " << pos.y << ", " << pos.z << std::endl;

	Node::ChildNodeIterator ite2(node->getChildIterator());
	while ( ite2.hasMoreElements() )
	{
		   SceneNode* child = static_cast<SceneNode*>(ite2.getNext());
		   child->translate(-1 * (min + max) / 2);
		   Vector3 pos = child->getPosition(), abspos = child->_getDerivedPosition();
		   LOG << "child " << child->getName() << " (" << child << ") Position : {" << pos.x << ", " << pos.y << ", " << pos.z << "}" << std::endl;
		   LOG << "child " << child->getName() << " (" << child << ") AbsPosition : {" << abspos.x << ", " << abspos.y << ", " << abspos.z << "}" << std::endl;
//	       Vector3 childmin = child->_getWorldAABB().getMinimum();
//	       Vector3 childmax = child->_getWorldAABB().getMaximum();
//	       LOG << "child min " << childmin.x << ", " << childmin.y << ", " << childmin.z << std::endl;
//	       LOG << "child max " << childmax.x << ", " << childmax.y << ", " << childmax.z << std::endl;
	}
	Vector3 GrpPos = node->getPosition();
	LOG << "After Equilibrate : Group(" << GrpPos.x << ", " << GrpPos.y << ", " << GrpPos.z << ")" << std::endl;
}

BallGameEntity::BallGameEntity(const dMatrix& matrix) :// m_matrix(matrix),
	m_curPosition (matrix.m_posit),
	m_nextPosition (matrix.m_posit),
	m_curRotation (dQuaternion (matrix)),
	m_nextRotation (dQuaternion (matrix))
{
	OgreEntity = NULL;
	Group = NULL;
	Body = NULL;
	type = Case;
}

BallGameEntity::BallGameEntity()
{
	OgreEntity = NULL;
	Body = NULL;
	Group = NULL;
	type = Case;
}

void BallGameEntity::DisplaySelectedBox(bool display)
{
	OgreEntity->showBoundingBox(display);
}

void BallGameEntity::Finalize(void)
{
	setNewtonBody(NULL);
	if(OgreEntity != NULL)
	{
		LOG << "Remove Ogre " << OgreEntity->getName() << std::endl;
		SceneNode *parent = (SceneNode*)OgreEntity->getParent();
		parent->removeAndDestroyChild(OgreEntity->getName());
	}
}

void BallGameEntity::setOgreNode(SceneNode *node)
{
	OgreEntity = node;
	if(node != NULL)
	{
		node->_setDerivedPosition(InitialPos);
//		LOG << "Entity " << node->getName() << " (" << node << ") set position {" << InitialPos.x << ", " << InitialPos.y << ", " << InitialPos.z << "}" << std::endl;
		node->setScale(InitialScale);
//		LOG << "Entity" << this << "set scale {" << InitialScale.x << ", " << InitialScale.y << ", " << InitialScale.z << "}" << std::endl;
		node->_setDerivedOrientation(InitialOrientation);
//		LOG << "Entity" << this << "set orientation {" << InitialOrientation.x << ", " << InitialOrientation.y << ", " << InitialOrientation.z << ", " << InitialOrientation.w << "}" << std::endl;
		((Ogre::Entity*)node->getAttachedObject(0))->getUserObjectBindings().setUserAny(Ogre::Any(this));
	}
}

void BallGameEntity::setNewtonBody(NewtonBody *body)
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

void BallGameEntity::SetMatrixUsafe(const dQuaternion& rotation, const dVector& position)
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

void BallGameEntity::TransformCallback(const NewtonBody* body, const dFloat* matrix, int threadIndex)
{
//	LOG << "TransformCallback" << std::endl;
	BallGameEntity* const Entity = (BallGameEntity*) NewtonBodyGetUserData(body);
	if (Entity)
	{
		BallGame* const scene = (BallGame*)NewtonWorldGetUserData(NewtonBodyGetWorld(body));
		dMatrix transform(matrix);
		dQuaternion rot;
		NewtonBodyGetRotation(body, &rot.m_x);
		Entity->SetMatrixUsafe(rot, transform.m_posit);


		Vector3 NewPosition(Entity->m_curPosition.m_x, Entity->m_curPosition.m_y, Entity->m_curPosition.m_z);
		Quaternion NewOrientation(Entity->m_curRotation.m_w, Entity->m_curRotation.m_x, Entity->m_curRotation.m_y, Entity->m_curRotation.m_z);

		//scene->Lock(Entity->m_lock);
		//scene->Unlock(Entity->m_lock);
//		LOG << "Entity " << Entity->getName() << " transform " << "position {" << NewPosition.x << ", " << NewPosition.y << ", " << NewPosition.z << "}" << std::endl;
//		LOG << "Entity " << Entity->getName() << " Orientation {" << NewOrientation.w << ", " << NewOrientation.x << ", " << NewOrientation.y << ", " << NewOrientation.z << "}" << std::endl;
		Entity->OgreEntity->_setDerivedPosition(NewPosition);
		Entity->OgreEntity->_setDerivedOrientation(NewOrientation);
	}
}

dMatrix * BallGameEntity::PrepareNewtonBody(dVector &NewtonBodyLocation, dVector &NewtonBodySize)
{
	InitialPos = OgreEntity->_getDerivedPosition();
	InitialScale = OgreEntity->_getDerivedScale();
	InitialOrientation = OgreEntity->_getDerivedOrientation();
	NewtonBodyLocation.m_x = InitialPos.x;
	NewtonBodyLocation.m_y = InitialPos.y;
	NewtonBodyLocation.m_z = InitialPos.z;
	NewtonBodyLocation.m_w = 1;
	Entity *ogreEntity = (Ogre::Entity*)OgreEntity->getAttachedObject(0);
	Vector3 AABB(ogreEntity->getBoundingBox().getSize());
	NewtonBodySize.m_x = AABB.x * InitialScale.x;
	NewtonBodySize.m_y = AABB.y * InitialScale.y;
	NewtonBodySize.m_z = AABB.z * InitialScale.z;
	NewtonBodySize.m_w = 0.0f;

	return new dMatrix(InitialOrientation.getPitch(false).valueRadians(), InitialOrientation.getYaw(false).valueRadians(), InitialOrientation.getRoll(false).valueRadians(), NewtonBodyLocation);
}

BallEntity::BallEntity(const dMatrix& matrix):BallGameEntity(matrix)
{
	InitialMass = 1;
	type = Ball;
}

BallEntity::BallEntity()
{
	InitialMass = 1;
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

CaseEntity::CaseEntity(const dMatrix& matrix, enum CaseType _type):BallGameEntity(matrix)
{
	type = _type;
	this->BallGameEntity::type = Case;
	force_to_apply = NAN;
	force_direction = NULL;
	MovementToDo = NULL;
}

CaseEntity::CaseEntity(enum CaseType _type)
{
	type = _type;
	this->BallGameEntity::type = Case;
	force_to_apply = NAN;
	force_direction = NULL;
	MovementToDo = NULL;
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

void CaseEntity::AddMovePoint(const Vector3 &GoalPos, float speed, unsigned64 waittime, const Quaternion &GoalAngle, float RotateSpeed)
{
	if(MovementToDo == NULL)
		MovementToDo = new struct Movement;
	struct MovementStep *step = new struct MovementStep;
	step->Position = GoalPos;
	step->TranslateSpeed = speed;
	step->waittime = waittime;
	step->Orientation = GoalAngle;
	step->RotateSpeed = RotateSpeed;
	MovementToDo->Moves.push_back(step);
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

void CaseEntity::DisplaySelectedMove(void *vstep, CEGUI::Editbox *TSpeed, CEGUI::Editbox *RSpeed, CEGUI::Editbox *WaitTime)
{
	struct MovementStep *step = (struct MovementStep*)vstep;
	setAbsolutePosition(step->Position);
	setAbsoluteOrientation(step->Orientation);

	TSpeed->setText(std::to_string(step->TranslateSpeed));
	RSpeed->setText(std::to_string(step->RotateSpeed));
	WaitTime->setText(std::to_string(step->waittime));
}

void CaseEntity::UpdateSelectedMove(void *vstep, const Vector3 &GoalPos, float TSpeed, const Quaternion &GoalAngle, float RSpeed, unsigned64 WaitTime)
{
	struct MovementStep *step = (struct MovementStep*)vstep;
	step->Position = GoalPos;
	step->TranslateSpeed = TSpeed;
	step->Orientation = GoalAngle;
	step->RotateSpeed = RSpeed;
	step->waittime = WaitTime;
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

void CaseEntity::CaseMove(unsigned64 microseconds, dFloat timestep)
{
	if(MovementToDo == NULL)
		return;

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
		ToReachPos = &MovementToDo->actual->Position;
		ToReachRot = &MovementToDo->actual->Orientation;
	}
	else
		return;

	if(ToReachPos != NULL)
	{
		dFloat MovePos[3];
		dFloat RotatePos[3];
		if(!isnanf(MovementToDo->actual->TranslateSpeed))
		{
			MovePos[0] = ToReachPos->x - getAbsolutePosition().x;
			MovePos[1] = ToReachPos->y - getAbsolutePosition().y;
			MovePos[2] = ToReachPos->z - getAbsolutePosition().z;
			if (MovePos[0] > 0.01)
			{
				MovePos[0] = MovementToDo->actual->TranslateSpeed;
				MustTranslate = true;
			}
			else if (MovePos[0] < -0.01)
			{
				MovePos[0] = -1 * MovementToDo->actual->TranslateSpeed;
				MustTranslate = true;
			}
			else
				MovePos[0] = 0;
			if (MovePos[1] > 0.01)
			{
				MovePos[1] = MovementToDo->actual->TranslateSpeed;
				MustTranslate = true;
			}
			else if (MovePos[1] < -0.01)
			{
				MovePos[1] = -1 * MovementToDo->actual->TranslateSpeed;
				MustTranslate = true;
			}
			else
				MovePos[1] = 0;
			if (MovePos[2] > 0.01)
			{
				MovePos[2] = MovementToDo->actual->TranslateSpeed;
				MustTranslate = true;
			}
			else if (MovePos[2] < -0.01)
			{
				MovePos[2] = -1 * MovementToDo->actual->TranslateSpeed;
				MustTranslate = true;
			}
			else
				MovePos[2] = 0;
		}
		if(!isnanf(MovementToDo->actual->RotateSpeed))
		{
			RotatePos[0] = ToReachRot->x - getAbsoluteOrientation().x;
			RotatePos[1] = ToReachRot->y - getAbsoluteOrientation().y;
			RotatePos[2] = ToReachRot->z - getAbsoluteOrientation().z;
//			LOG << "RotatePos = {" << RotatePos[0] << ", " << RotatePos[1] << ", " << RotatePos[2] << "}" << std::endl;
			if (RotatePos[0] > 0.01)
			{
				RotatePos[0] = MovementToDo->actual->RotateSpeed;
				MustRotate = true;
			}
			else if (RotatePos[0] < -0.01)
			{
				RotatePos[0] = -1 * MovementToDo->actual->RotateSpeed;
				MustRotate = true;
			}
			else
				RotatePos[0] = 0;
			if (RotatePos[1] > 0.01)
			{
				RotatePos[1] = MovementToDo->actual->RotateSpeed;
				MustRotate = true;
			}
			else if (RotatePos[1] < -0.01)
			{
				RotatePos[1] = -1 * MovementToDo->actual->RotateSpeed;
				MustRotate = true;
			}
			else
				RotatePos[1] = 0;
			if (RotatePos[2] > 0.01)
			{
				RotatePos[2] = MovementToDo->actual->RotateSpeed;
				MustRotate = true;
			}
			else if (RotatePos[2] < -0.01)
			{
				RotatePos[2] = -1 * MovementToDo->actual->RotateSpeed;
				MustRotate = true;
			}
			else
				RotatePos[2] = 0;
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

	Entity *ogreEntity = (Ogre::Entity*)OgreEntity->getAttachedObject(0);
	dMatrix *bodymatrix = PrepareNewtonBody(NewtonBodyLocation, NewtonBodySize);

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
	computed = false;
	equilibrated = false;
}

void GroupEntity::Finalize(void)
{
	std::list<BallGameEntity*>::iterator iter(childs.begin());
	while(iter != childs.end())
	{
		BallGameEntity *Entity = *iter;
		if(Entity != NULL)
		{
			Entity->OgreEntity->getParent()->removeChild(Entity->OgreEntity);
			OgreEntity->getParent()->addChild(Entity->OgreEntity);
		}
		iter = childs.erase(iter);
	}
	SceneNode *parent = (SceneNode*)OgreEntity->getParent();
	parent->removeAndDestroyChild(OgreEntity->getName());
}

void GroupEntity::AddChild(BallGameEntity* child)
{
	childs.push_back(child);
	child->Group = this;
	computed = false;
	equilibrated = false;
}

bool GroupEntity::DelChild(BallGameEntity* child)
{
	LOG << "Child " << child->getName() << " Removed from Group" << std::endl;
	child->Group = NULL;
	Quaternion ChildOrientation = child->getAbsoluteOrientation();
	child->OgreEntity->setOrientation(ChildOrientation);
	Vector3 ChildPosition = child->getAbsolutePosition();
	child->OgreEntity->setPosition(ChildPosition);
	Vector3 ChildScale = child->getAbsoluteScale();
	child->OgreEntity->setScale(ChildScale);

//	Vector3 pos = child->getRelativePosition(), abspos = child->getAbsolutePosition();
//	LOG << "Child " << child->getName() << " position {" << pos.x << ", " << pos.y << ", " << pos.z << "}" << std::endl;
//	LOG << "Child " << child->getName() << " absposition {" << abspos.x << ", " << abspos.y << ", " << abspos.z << "}" << std::endl;

	std::list<BallGameEntity*>::iterator iter(childs.begin());
	while(iter != childs.end())
	{
		BallGameEntity *Entity = *iter;
		if(Entity == child)
		{
			child->OgreEntity->getParent()->removeChild(child->OgreEntity);
			OgreEntity->getParent()->addChild(child->OgreEntity);
			childs.erase(iter);
			break;
		}
		iter++;
	}
	return childs.empty();
}

void GroupEntity::FillListWithChilds(std::list<BallGameEntity*> &list)
{
	std::list<BallGameEntity*>::iterator iter(childs.begin());
	while(iter != childs.end())
	{
		BallGameEntity *Entity = *(iter++);
		if(Entity == NULL)
			continue;
		list.push_back(Entity);
	}
}

void GroupEntity::ComputeChilds(void)
{
	if(computed)
		return;
	std::list<BallGameEntity*>::iterator iter(childs.begin());
	while(iter != childs.end())
	{
		BallGameEntity *Entity = *(iter++);
		if(Entity == NULL)
			continue;
		Vector3 ChildPos = Entity->getAbsolutePosition();
		LOG << "Child " << Entity->getName() << " (" << Entity << ") Added to Group" << std::endl;
		Entity->OgreEntity->getParentSceneNode()->removeChild(Entity->OgreEntity);
		OgreEntity->addChild(Entity->OgreEntity);
		Entity->setAbsolutePosition(ChildPos);//Must conserve position will adding into group !!!!
	}
	computed = true;
}

void GroupEntity::ComputeAndEquilibrateChilds(void)
{
	ComputeChilds();
	if(equilibrated == false)
	{
		EquilibrateAABBAroundOrigin((Node*)OgreEntity);
		equilibrated = true;
	}
}

void GroupEntity::ExportToJson(rapidjson::Value &v, rapidjson::Document::AllocatorType& allocator)
{
	rapidjson::Value name;
	const char *ogrename = (const char*)OgreEntity->getName().c_str();
	name.SetString(ogrename, allocator);
	v.AddMember("NodeName", name, allocator);
	const Vector3 &InitialPos = OgreEntity->getPosition();
	const Vector3 &InitialScale = OgreEntity->getScale();
	const Quaternion &InitialOrientation = OgreEntity->getOrientation();

	v.AddMember("PosX", InitialPos.x, allocator);
	v.AddMember("PosY", InitialPos.y, allocator);
	v.AddMember("PosZ", InitialPos.z, allocator);
	v.AddMember("ScaleX", InitialScale.x, allocator);
	v.AddMember("ScaleY", InitialScale.y, allocator);
	v.AddMember("ScaleZ", InitialScale.z, allocator);
	v.AddMember("OrientationX", InitialOrientation.x, allocator);
	v.AddMember("OrientationY", InitialOrientation.y, allocator);
	v.AddMember("OrientationZ", InitialOrientation.z, allocator);
	v.AddMember("OrientationW", InitialOrientation.w, allocator);
}

void GroupEntity::ImportFromJson(rapidjson::Value &v, Node *parent, String &nodeNamePrefix)
{
	Vector3 InitialPos, InitialScale;
	Quaternion InitialOrientation;
	InitialPos.x = v["PosX"].GetFloat();
	InitialPos.y = v["PosY"].GetFloat();
	InitialPos.z = v["PosZ"].GetFloat();
	InitialScale.x = v["ScaleX"].GetFloat();
	InitialScale.y = v["ScaleY"].GetFloat();
	InitialScale.z = v["ScaleZ"].GetFloat();
	InitialOrientation.x = v["OrientationX"].GetFloat();
	InitialOrientation.y = v["OrientationY"].GetFloat();
	InitialOrientation.z = v["OrientationZ"].GetFloat();
	InitialOrientation.w = v["OrientationW"].GetFloat();

	const char *nodename_c = v["NodeName"].GetString();
	String nodeName = nodeNamePrefix;
	nodeName += nodename_c;
	OgreEntity = (Ogre::SceneNode*)parent->createChild(nodeName, InitialPos);
//	LOG << "Group " << OgreEntity->getName() << " (" << OgreEntity << ") set Position {" << InitialPos.x << ", " << InitialPos.y << ", " << InitialPos.z << "}" << std::endl;
	OgreEntity->setPosition(InitialPos);
	OgreEntity->setScale(InitialScale);
	OgreEntity->setOrientation(InitialOrientation);
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

inline void MoveNode(Node *node, Vector3 &addPos)
{
	LOG << "Move Node " << node->getName() << " by " << addPos.x << ", " << addPos.y << ", " << addPos.z << std::endl;
	Vector3 pos = node->getPosition();
	pos += addPos;
	node->setPosition(pos);
}

inline void MoveNode(Node *node, float x, float y, float z)
{
	Vector3 addPos(x, y, z);
	MoveNode(node, addPos);
}

void BallGameEntity::Move(float x, float y, float z)
{
	MoveNode(OgreEntity, x, y, z);
}

void BallGameEntity::Move(Vector3 &addPos)
{
	MoveNode(OgreEntity, addPos);
}

void GroupEntity::Move(float x, float y, float z)
{
	MoveNode(OgreEntity, x, y, z);
}

inline void ScaleNode(Node *node, Vector3 &addScale)
{
	LOG << "Scale Node " << node << " by " << addScale.x << ", " << addScale.y << ", " << addScale.z << std::endl;
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

void BallGameEntity::Scale(float x, float y, float z)
{
	ScaleNode(OgreEntity, x, y, z);
}

void BallGameEntity::Scale(Vector3 &addScale)
{
	ScaleNode(OgreEntity, addScale);
}

void GroupEntity::Scale(float x, float y, float z)
{
	std::list<BallGameEntity*>::iterator iter(childs.begin());
	while(iter != childs.end())
	{
		BallGameEntity *Entity = *(iter++);
		if(Entity == NULL)
			continue;
		Vector3 childPos = Entity->getRelativePosition();
		Vector3 childScale = Entity->getRelativeScale();
		childPos.x *= x / childScale.x;
		childPos.y *= y / childScale.y;
		childPos.z *= z / childScale.z;
		Entity->Scale(x, y, z);
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

void BallGameEntity::Rotate(float x, float y, float z)
{
	RotateNode(OgreEntity, x, y, z);
}

void GroupEntity::Rotate(float x, float y, float z)
{
	RotateNode(OgreEntity, x, y, z);
}

#define MASS_JSON_FIELD "Mass"

void BallEntity::ExportToJson(rapidjson::Value &v, rapidjson::Document::AllocatorType& allocator)
{
	BallGameEntity::ExportToJson(v, allocator);
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
#define MOVESTEPWAITTIME_JSON_FIELD "MovePauseDuration"

void CaseEntity::ExportToJson(rapidjson::Value &v, rapidjson::Document::AllocatorType& allocator)
{
	BallGameEntity::ExportToJson(v, allocator);
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
				Step.AddMember(MOVESTEPPOSITIONX_JSON_FIELD, step->Position.x, allocator);
				Step.AddMember(MOVESTEPPOSITIONY_JSON_FIELD, step->Position.y, allocator);
				Step.AddMember(MOVESTEPPOSITIONZ_JSON_FIELD, step->Position.z, allocator);
				Step.AddMember(MOVESTEPTRANSLATIONSPEED_JSON_FIELD, step->TranslateSpeed, allocator);
			}
			else
				Step.AddMember(MOVESTEPPOSITIONPRESENT_JSON_FIELD, false, allocator);
			if(!isnanf(step->RotateSpeed))
			{
				Step.AddMember(MOVESTEPROTATIONPRESENT_JSON_FIELD, true, allocator);
				Step.AddMember(MOVESTEPROTATIONX_JSON_FIELD, step->Orientation.x, allocator);
				Step.AddMember(MOVESTEPROTATIONY_JSON_FIELD, step->Orientation.y, allocator);
				Step.AddMember(MOVESTEPROTATIONZ_JSON_FIELD, step->Orientation.z, allocator);
				Step.AddMember(MOVESTEPROTATIONW_JSON_FIELD, step->Orientation.w, allocator);
				Step.AddMember(MOVESTEPROTATIONSPEED_JSON_FIELD, step->RotateSpeed, allocator);
			}
			else
				Step.AddMember(MOVESTEPROTATIONPRESENT_JSON_FIELD, false, allocator);
			Step.AddMember(MOVESTEPWAITTIME_JSON_FIELD, (uint64_t)step->waittime, allocator);

			Moves.PushBack(Step, allocator);
		}
		v.AddMember(MOVES_JSON_FIELD, Moves, allocator);
	}
}

void CaseEntity::CreateFromJson(rapidjson::Value &v, BallGame *Game, NewtonWorld *m_world, Node *parent, String &nodeNamePrefix)
{
	ImportFromJson(v, Game, parent, nodeNamePrefix);

//	if(type == CaseEntity::CaseType::typeRamp)
//		LOG << "Place a Ramp" << std::endl;
//	else
//		LOG << "Place a Box" << std::endl;

	if(m_world)
		CreateNewtonBody(m_world);
}

void CaseEntity::ImportFromJson(rapidjson::Value &v, BallGame *Game, Node *parent, String &nodeNamePrefix)
{
	BallGameEntity::ImportFromJson(v, Game, parent, nodeNamePrefix);
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
		struct Movement *Movement = new struct Movement;
		Movement->is_launched_by_collide = v[MOVEISTRIGGERED_JSON_FIELD].GetBool();
		for(int cmpt = 0; cmpt < v[MOVES_JSON_FIELD].GetArray().Size(); cmpt++)
		{
			rapidjson::Value &Stepjson = v[MOVES_JSON_FIELD].GetArray()[cmpt];
			struct MovementStep *Step = new struct MovementStep;
			Step->waittime = (unsigned64)Stepjson[MOVESTEPWAITTIME_JSON_FIELD].GetUint64();
			if(Stepjson[MOVESTEPPOSITIONPRESENT_JSON_FIELD].GetBool() == true)
			{
				Step->Position.x = Stepjson[MOVESTEPPOSITIONX_JSON_FIELD].GetFloat();
				Step->Position.y = Stepjson[MOVESTEPPOSITIONY_JSON_FIELD].GetFloat();
				Step->Position.z = Stepjson[MOVESTEPPOSITIONZ_JSON_FIELD].GetFloat();
				Step->TranslateSpeed = Stepjson[MOVESTEPTRANSLATIONSPEED_JSON_FIELD].GetFloat();
			}
			else
				Step->TranslateSpeed = NAN;
			if(Stepjson[MOVESTEPROTATIONPRESENT_JSON_FIELD].GetBool() == true)
			{
				Step->Orientation.x = Stepjson[MOVESTEPROTATIONX_JSON_FIELD].GetFloat();
				Step->Orientation.y = Stepjson[MOVESTEPROTATIONY_JSON_FIELD].GetFloat();
				Step->Orientation.z = Stepjson[MOVESTEPROTATIONZ_JSON_FIELD].GetFloat();
				Step->Orientation.w = Stepjson[MOVESTEPROTATIONW_JSON_FIELD].GetFloat();
				Step->RotateSpeed = Stepjson[MOVESTEPROTATIONSPEED_JSON_FIELD].GetFloat();
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

void BallGameEntity::ExportToJson(rapidjson::Value &v, rapidjson::Document::AllocatorType& allocator)
{
	Ogre::Entity *entity = (Ogre::Entity*)OgreEntity->getAttachedObject(0);
	const char *cname = (const char*)entity->getMesh().get()->getName().c_str();
	rapidjson::Value name;
	name.SetString(cname, allocator);
	v.AddMember(MESH_JSON_FIELD, name, allocator);
	const char *ogrename = (const char*)OgreEntity->getName().c_str();
	name.SetString(ogrename, allocator);
	v.AddMember(NODENAME_JSON_FIELD, name, allocator);
	if(Group != NULL)
	{
		rapidjson::Value gname;
		const char *groupname = (const char*)Group->getName().c_str();
		gname.SetString(groupname, allocator);
		v.AddMember(GROUPNAME_JSON_FIELD, gname, allocator);
	}
	else
		v.AddMember(GROUPNAME_JSON_FIELD, "", allocator);
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

void BallEntity::CreateFromJson(rapidjson::Value &v, BallGame *Game, NewtonWorld *m_world, Node *parent, String &nodeNamePrefix)
{
	ImportFromJson(v, Game, parent, nodeNamePrefix);

	if(m_world != NULL)
		CreateNewtonBody(m_world);
}

void BallEntity::ImportFromJson(rapidjson::Value &v, BallGame *Game, Node *parent, String &nodeNamePrefix)
{
	BallGameEntity::ImportFromJson(v, Game, parent, nodeNamePrefix);
	InitialMass = v[MASS_JSON_FIELD].GetFloat();
}

void BallGameEntity::ImportFromJson(rapidjson::Value &v, BallGame *Game, Node *parent, String &nodeNamePrefix)
{
	const char *meshname = v[MESH_JSON_FIELD].GetString();
	Ogre::SceneManager *mSceneMgr = Game->getSceneManager();
	Entity* ogreEntity = mSceneMgr->createEntity(meshname);
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
	ogreNode->attachObject(ogreEntity);
	setOgreNode(ogreNode);
	const char *Groupname = v[GROUPNAME_JSON_FIELD].GetString();
	if(strcmp(Groupname, "") != 0)
	{
		GroupEntity *Grp = Game->findGroup(Groupname);
		if(Grp != NULL)
		{
			LOG << "Add Child " << nodeName << " (" << nodename_c << ") to Group " << Groupname << std::endl;
			Grp->AddChild(this);
		}
		else
			LOG << "Pb : Group " << Groupname << " not found !!" << std::endl;
	}
}
