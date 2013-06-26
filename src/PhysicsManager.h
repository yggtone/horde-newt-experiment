
#ifndef _PhysicsManager_H_
#define _PhysicsManager_H_


#include "newton.h"
#include <math.h>
#include <iomanip>
#include <string>
#include <vector>
#include "utMath.h"
#include "HordeEntity.h"
#include "PlayerEntity.h"
#include "NPCEntity.h"
#include "global.h"
#include "JointLibrary.h"

class PhysicsManager
{
public:
	PhysicsManager(float desiredFps = 100.0f, int maxUpdatesPerFrames = 5);
	~PhysicsManager();

	
	//return real updates
	int update(float t_step);

	void createBodyFromPoints( HordeEntity *newEnt, float mass  );
	void createTreeFromPoints( HordeEntity *newEnt, float mass  );


	void createHinge(Horde3D::Vec3f axis, Horde3D::Vec3f pos, HordeEntity *entChild, HordeEntity *entParent, bool limits = false, float limitLowDeg = 0, float limitHighDeg = 0);


	NewtonWorld* getWorld(){ return g_world; };


	float getDefaultLinDamping(){ return m_defaultLinearDamping; };



	void setBodyDefaultCallback(NewtonBody* bod);
	void setBodyPickingCallback(NewtonBody* bod);
private:

	NewtonBody*  createNewtonBody(NewtonCollision* collision, dFloat mass,const float* transMatrix, HordeEntity *newEnt = NULL );
	
	NewtonWorld* g_world;


	int m_maxTicksPerFrames;
	float m_timestep;
	float m_invTimestep;
	float m_timeAcumulator;
	float m_updateFPS;


	float m_defaultAngularDamping;
	float m_defaultLinearDamping;


};

#endif



