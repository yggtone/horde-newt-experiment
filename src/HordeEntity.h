
#ifndef _HordeEntity_H_
#define _HordeEntity_H_

#include "newton.h"
#include <math.h>
#include <iomanip>
#include <vector>
#include <string>
#include "Horde3D.h"
#include "Horde3DUtils.h"
#include "utMath.h"
#include "dVector.h"
#include "dMatrix.h"
#include "dQuaternion.h"
#include "utMath.h"
#include "JointLibrary.h"
#include "global.h"



class HordeEntity
{
public:
	HordeEntity(int hordeID, std::string contentDir);
	~HordeEntity();

	

	std::vector<float> getMeshVertices();

	const float* getTransformationMat();
	void setTransformationMat(const float* mat);

	void GetNodeTransform( float *tx, float *ty, float *tz,
                              float *rx, float *ry, float *rz, float *sx, float *sy, float *sz );


	// these are the element to represent the position and orientation state of a graphics object in the world 
	dMatrix m_matrix;					// current interpolated visual matrix
	dVector m_curPosition;				// position one physics simulation step in the future
	dVector m_prevPosition;             // position at the current physics simulation step
	dQuaternion m_curRotation;          // rotation one physics simulation step in the future  
	dQuaternion m_prevRotation;         // rotation at the current physics simulation step  

	const dFloat* tmatrix;


	dVector m_scale;

	bool matrixUpdated;
	Horde3D::Matrix4f scaleMatrix;

	void updatePosOri();

	virtual void update(float timestep);

	void setBody(NewtonBody* body) 
	{ 
		this->m_body = body; 
		//dMatrix tmat;
		//NewtonBodyGetMatrix(body, &tmat[0][0] );
		//this->tmatrix = &tmat[0][0];

		physicsEnabled = true;
	};
	NewtonBody* getBody(){return m_body;};

	void setBodyPosition(float tx, float ty, float tz);
	void setBodyVelocity(float x, float y, float z);
	Horde3D::Vec3f getBodyVelocity();

	void setJoint(NewtonUserJoint* joint) { m_joint = joint; };
	NewtonUserJoint* getJoint(){return m_joint;};

	Horde3D::Vec3f getPosition();

	bool isPlayer(){return is_player;};
	bool isNPC(){return is_npc;};

	void setType(int ntype){ type = ntype; };
	int getType(){return type;};

	
	void setPickable(int ntype){ pickable = ntype; };
	int isPickable(){return pickable;};

	int getHordeID(){return m_hordeID;};

	void releaseNode();
	
	void setPickingPos(Horde3D::Vec3f pickingPos){ pickingPosition = dVector(pickingPos.x,pickingPos.y,pickingPos.z); };
	dVector getPickingPos(){return pickingPosition; };


	float defaultLinDampening;

	
	void updateAnimation(float elapsed);


	bool planetMode;
protected:

	bool staticBody;
	bool physicsEnabled;
	float animTime;
	bool animationSet;

	std::string m_contentDir;

	void extractMeshData();
	bool is_player;
	bool is_npc;

	NewtonBody* m_body;
	NewtonUserJoint* m_joint;
	int m_hordeID;
	
	//position of picking hand, updated by factory when picking, accessed in forces callback
	dVector pickingPosition;

	int type;
	bool pickable;

private:

	unsigned int NumVertices, NumTriangleIndices;

	float* VertexBase;

	unsigned int VertRStart;

	unsigned int* TriangleBase32;
	unsigned short* TriangleBase16;

	unsigned int TriangleMode;


	
	
	
};

#endif // _HordeEntity_H_



