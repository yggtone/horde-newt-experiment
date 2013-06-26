
#ifndef _PlayerEntity_H_
#define _PlayerEntity_H_

#include "newton.h"
#include <math.h>
#include <iomanip>
#include <string>
#include <vector>
#include "utMath.h"
#include "HordeEntity.h"
#include "global.h"
#include "JointLibrary.h"
#include "HordeEntity.h"
#include "PhysicsManager.h"



class PlayerEntity : public HordeEntity
{
	public:
	PlayerEntity(int hordeID, std::string contentDir);
	~PlayerEntity();
	
	void update(float timestep);

	bool grounded;


	void setMoveVector(Horde3D::Vec3f moveVec){ moveVector = moveVec; };
	Horde3D::Vec3f getMoveVector(){return moveVector; };

	

	void jump();

	
	private:

	Horde3D::Vec3f moveVector;

	float jumpDelay;

	//Horde3D::Vec3f pickingPosition;
	
	Horde3D::Quaternion camRotTarget;
	Horde3D::Quaternion camRotCurrent;


};


#endif // _playerentity_H_



