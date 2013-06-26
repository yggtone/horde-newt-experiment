

#include "PhysicsManager.h"

#include "newton.h"
#include "dVector.h"
#include "dMatrix.h"
#include "dQuaternion.h"



#define GRAVITY	-10.0f

static void DestroyBodyCallback (const NewtonBody* body);
static void SetTransformCallback (const NewtonBody* body, const dFloat* matrix, int threadIndex);
static void ApplyForceAndTorqueCallback (const NewtonBody* body, dFloat timestep, int threadIndex);
static void BodyCallbackPlanet (const NewtonBody* body, dFloat timestep, int threadIndex);


static void CharControllerCallback (const NewtonBody* body, dFloat timestep, int threadIndex);
static void CharControllerCallbackPlanet(const NewtonBody* body, dFloat timestep, int threadIndex);
static void CharControllerCallbackNPC (const NewtonBody* body, dFloat timestep, int threadIndex);
static void CharControllerCallbackNPCPlanet (const NewtonBody* body, dFloat timestep, int threadIndex);
static void PickingCallback (const NewtonBody* body, dFloat timestep, int threadIndex);


static dFloat UserMeshCollisionCallback (const NewtonBody* const body, const NewtonCollision* const collisionTree, dFloat interception, dFloat* normal, int faceId, void* usedData);




PhysicsManager::PhysicsManager(float desiredFps, int maxUpdatesPerFrames): g_world(NULL), m_defaultAngularDamping(0.f), m_defaultLinearDamping(0.f)
{

	// create the Newton World
	g_world = NewtonCreate ();		
	NewtonInvalidateCache(g_world);
	printf("newt:%i\n",NewtonWorldGetVersion() );
	NewtonSetSolverModel(g_world,3);//1,2 are more intensive accuracy, 3--n is fast linear, up to n passes
	NewtonSetFrictionModel(g_world,1);//0-exact 1-adaptive(faster)

	float wsize = 100.f;
	float minpoint[3] = { -wsize,-wsize,-wsize};
	float maxpoint[3] = { wsize,wsize,wsize};


	if (maxUpdatesPerFrames < 1) {
		maxUpdatesPerFrames = 1;
	}

	if (maxUpdatesPerFrames > 10) {
		maxUpdatesPerFrames = 10;
	}

	m_maxTicksPerFrames = maxUpdatesPerFrames;
	m_updateFPS = desiredFps;

	m_timestep = 1.0f / desiredFps;

	if (m_timestep > 1.0f / 10.0f) {
		// recalculate the iteration count to met the desire fps 
		m_maxTicksPerFrames += (int)ceilf (m_timestep / (1.0f / 10.0f));
		m_timestep = 1.0 / 10.0f;
		m_updateFPS = 10;
	}

	if (m_timestep < 1.0f / 1000.0f) {
		m_timestep = 1.0 / 1000.0f;
		m_updateFPS = 1000;
	}

	m_invTimestep = 1.0f / m_timestep;
	m_timeAcumulator = m_timestep;





	//.1f * (60.f / desiredFps);
	m_defaultAngularDamping = .01f * (60.f / desiredFps);
	m_defaultLinearDamping = .01f * (60.f / desiredFps);


}


PhysicsManager::~PhysicsManager()
{
	//#ifdef USE_VISUAL_DEBUGGER
	// destroy the debugger server
	//	NewtonDebuggerDestroyServer (g_newtonDebugger);
	//#endif

	// destroy all rigid bodies, this is no necessary because Newton Destroy world will also destroy all bodies
	// but if you want to change level and restart you can call this function to clean the world without destroying the world.
	//NewtonDestroyAllBodies (g_world);

	// finally destroy the newton world 
	NewtonDestroy (g_world);
}



void GetBBox (dVector& minBox, dVector& maxBox, std::vector<float> vertices)
{
	// initialize the Box bound to ridicules values
	minBox = dVector ( 1.0e10f,  1.0e10f,  1.0e10f, 1.0f);
	maxBox = dVector (-1.0e10f, -1.0e10f, -1.0e10f, 1.0f);
	for (int i = 0; i < vertices.size() - 2; i += 3) {
		dFloat val;

		// adjust to a better bound for x
		val = vertices[i + 0];
		minBox.m_x = (val < minBox.m_x) ? val : minBox.m_x;
		maxBox.m_x = (val > maxBox.m_x) ? val : maxBox.m_x;

		// adjust to a better bound for y
		val = vertices[i + 1];
		minBox.m_y = (val < minBox.m_y) ? val : minBox.m_y;
		maxBox.m_y = (val > maxBox.m_y) ? val : maxBox.m_y;

		// adjust to a better bound for z
		val = vertices[i + 2];
		minBox.m_z = (val < minBox.m_z) ? val : minBox.m_z;
		maxBox.m_z = (val > maxBox.m_z) ? val : maxBox.m_z;
	}
}


void PhysicsManager::createBodyFromPoints( HordeEntity *newEnt, float mass )
{
	if(!newEnt)
		return;
	
	std::vector<float> vertices = newEnt->getMeshVertices();
	if( vertices.size() < 3)
		return;

	NewtonCollision* collision;

	int shapeId = 0;
	int pcount = ((int)vertices.size())/(int)3;
	//dVector minBox;
	//dVector* tmpArray = new dVector [pcount];
	float* tmpArray = new float[vertices.size() * 4];




	float tx,ty,tz,rx,ry,rz,sx,sy,sz;
	//void h3dGetNodeTransform( H3DNode node, float *tx, float *ty, float *tz,
    //                          float *rx, float *ry, float *rz, float *sx, float *sy, float *sz );
	newEnt->GetNodeTransform(&tx,&ty,&tz,&rx,&ry,&rz,&sx,&sy,&sz);

	//dVector origin = dVector(tx,ty,tz,1);

	Horde3D::Matrix4f scaleMat = Horde3D::Matrix4f::ScaleMat(sx,sy,sz);
	//scaleMat = Horde3D::Matrix4f(newEnt->getTransformationMat() );
	//Horde3D::Vec3f origin = Horde3D::Vec3f(tx,ty,tz);
	//origin = scaleMat.mult33Vec(origin);

	int arrayIndex = 0;

	for (int i = 0; i < vertices.size() - 2; i += 3) {

		Horde3D::Vec3f tmp =  Horde3D::Vec3f(vertices[i], vertices[i+1], vertices[i+2]);
		tmp = scaleMat.mult33Vec(tmp);
		//tmp = tmp - Horde3D::Vec3f(origin.m_x,origin.m_y,origin.m_z);

		//dVector vecp = dVector(tmp.x,tmp.y,tmp.z,0);

		//tmpArray[i/3] = vecp;
				
		tmpArray[arrayIndex++] = tmp.x;
		tmpArray[arrayIndex++] = tmp.y;
		tmpArray[arrayIndex++] = tmp.z;
		tmpArray[arrayIndex++] = 1.0f;
	}
	//printf("t:%f\n",  tmpArray[9] );
	printf("pcount:%i\n",  pcount );

	

	Horde3D::Matrix4f offset; //offset.c[3][0] = origin.m_x;	offset.c[3][1] = origin.m_y;	offset.c[3][2] = origin.m_z;
	
	
	//collision = NewtonCreateConvexHull(g_world, pcount, &tmpArray[0][0], sizeof (dVector), 0.01f, shapeId, &offset[0][0]);
	collision = NewtonCreateConvexHull(g_world, pcount, &tmpArray[0], sizeof (float) * 4, 0.01f, shapeId, &offset.c[0][0]);
	


	delete tmpArray;


	
	createNewtonBody(collision, mass, newEnt->getTransformationMat(), newEnt);


	//don't need collision shape anymore after creating body
	//NewtonReleaseCollision (g_world, collision);
	NewtonDestroyCollision(collision);

}


void PhysicsManager::createTreeFromPoints( HordeEntity *newEnt, float mass )
{
	if(!newEnt)
		return;

	std::vector<float> vertices = newEnt->getMeshVertices();
	if( vertices.size() < 3)
		return;

	NewtonCollision* collision;


	// now create and empty collision tree
	collision = NewtonCreateTreeCollision (g_world, 0);

	// start adding faces to the collision tree 
	NewtonTreeCollisionBeginBuild (collision);





	int shapeId = 0;
	int pcount = ((int)vertices.size())/(int)3;
	


	float tx,ty,tz,rx,ry,rz,sx,sy,sz;
	//void h3dGetNodeTransform( H3DNode node, float *tx, float *ty, float *tz,
    //                          float *rx, float *ry, float *rz, float *sx, float *sy, float *sz );
	newEnt->GetNodeTransform(&tx,&ty,&tz,&rx,&ry,&rz,&sx,&sy,&sz);

	//dVector origin = dVector(tx,ty,tz,1);

	Horde3D::Matrix4f scaleMat = Horde3D::Matrix4f::ScaleMat(sx,sy,sz);
	//scaleMat = Horde3D::Matrix4f(newEnt->getTransformationMat() );


	Horde3D::Vec3f origin = Horde3D::Vec3f(tx,ty,tz);
	origin = scaleMat.mult33Vec(origin);



	for (int i = 0; i < vertices.size() - 8 ; i += 9) 
	{
		dVector face[3];


		//face[0] = dVector (ent->m_vertex[index + 0], ent->m_vertex[index + 1], ent->m_vertex[index + 2]);
		Horde3D::Vec3f tmp =  Horde3D::Vec3f(vertices[i], vertices[i+1], vertices[i+2]);
		tmp = scaleMat.mult33Vec(tmp);
		face[0] = dVector(tmp.x,tmp.y,tmp.z,0);

		tmp =  Horde3D::Vec3f(vertices[i+3], vertices[i+4], vertices[i+5]);
		tmp = scaleMat.mult33Vec(tmp);
		face[1] = dVector(tmp.x,tmp.y,tmp.z,0);

		tmp =  Horde3D::Vec3f(vertices[i+6], vertices[i+7], vertices[i+8]);
		tmp = scaleMat.mult33Vec(tmp);
		face[2] = dVector(tmp.x,tmp.y,tmp.z,0);


		//NewtonTreeCollisionAddFace(collision, 3, &face[0].m_x, sizeof (dVector), i + 1);
		NewtonTreeCollisionAddFace(collision, 3, &face[0].m_x, sizeof (dVector), shapeId);

	

	}



		


	dMatrix offset (GetIdentityMatrix());
	
	//collision = NewtonCreateConvexHull(g_world, pcount, &tmpArray[0][0], sizeof (dVector), 0.1f, shapeId, &offset[0][0]);
	
	// end adding faces to the collision tree, also optimize the mesh for best performance
	NewtonTreeCollisionEndBuild (collision, 0);


	// look here, this is how you add the user callback to the collision tree mesh,
	// not to be confused with the filter callback which is called on each collision shape that the ray hit.
	// this function is called on each face of this collision tree that ray hit. 
	NewtonTreeCollisionSetUserRayCastCallback(collision, UserMeshCollisionCallback);

	createNewtonBody(collision, mass, newEnt->getTransformationMat(), newEnt);


	//don't need collision shape anymore after creating body
	//NewtonReleaseCollision (g_world, collision);
	NewtonDestroyCollision(collision);
}



NewtonBody*  PhysicsManager::createNewtonBody(NewtonCollision* collision, dFloat mass,const float* transMatrix, HordeEntity *newEnt )
{
	
	//(NewtonWorld* world, Entity* ent, NewtonCollision* collision, dFloat mass)

	//dFloat mass = 2.0f;

	dVector minBox;
	dVector maxBox;
	dVector origin;
	dVector inertia;
	NewtonBody* body;

	// we need to set physics properties to this body
	//dMatrix matrix (ent->m_curRotation, ent->m_curPosition);
	//NewtonBodySetMatrix (body, &matrix[0][0]);
	dMatrix matrix (GetIdentityMatrix());





	//__________Create Newton Transformation Matrix   to pass into NewtonCreateBody()
	//note: The scale of the transformation matrix will be ignored by Newton, so only
	//      rotation and translation will used to initialize.  Scale must be saved and applied to the new transformation
	//      matrices that newton returns, in order to get correct visuals in horde. Scale also must be applied to the actual
	//      geo vertices when creating the Newton collision out of a horde mesh.(in createBodyFromPoints(), etc.)
	Horde3D::Vec3f tr,rotation,sc;
	Horde3D::Matrix4f(transMatrix).decompose(tr,rotation,sc);	
	//newton transformation matrix - position is last column, upper left is rotation matrix
	//.  .  .  x
	//.  .  .  y
	//.  .  .  z
	//0  0  0  1
	Horde3D::Matrix4f rotMat = Horde3D::Matrix4f::RotMat(rotation.x,rotation.y,rotation.z);
	//rotMat.c[3][0] = tr.x;
	//rotMat.c[3][1] = tr.y;
	//rotMat.c[3][2] = tr.z;	
	const float matT[] = {1.f, 0, 0, 0,
		0, 1.f, 0, 0,
		0, 0, 1.f, 0,
		tr.x, tr.y, tr.z, 1};
	Horde3D::Matrix4f matTransl(matT);
	/*
	const float matS[] = {sc.x, 0, 0, 0,
							0, sc.y, 0, 0,
							0, 0, sc.z, 0,
							0, 0, 0, 1.f};
	Horde3D::Matrix4f matScale(matS);
	*/
	//trs
	Horde3D::Matrix4f finalTmat = matTransl * rotMat;//scale already applied during collision creation

	body = NewtonCreateBody (g_world, collision, &finalTmat.c[0][0]);
	//body = NewtonCreateBody (g_world, collision, &finalMatB[0][0]);


	
	//NewtonBodySetCollision(body,collision);
	//NewtonBodySetCollisionScale(body,sc.x,sc.y,sc.z);

	// bodies can have a destructor. 
	// this is a function callback that can be used to destroy any local data stored 
	// and that need to be destroyed before the body is destroyed. 
	NewtonBodySetDestructorCallback (body, DestroyBodyCallback);

	// save the entity as the user data for this body
	if(newEnt)
		NewtonBodySetUserData (body, newEnt);


	// we need to set the proper center of mass and inertia matrix for this body
	// the inertia matrix calculated by this function does not include the mass.
	// therefore it needs to be multiplied by the mass of the body before it is used.
	NewtonConvexCollisionCalculateInertialMatrix (collision, &inertia[0], &origin[0]);	

	// set the body mass matrix
	//NewtonBodySetMassMatrix (body, mass, mass * inertia.m_x, mass * inertia.m_y, mass * inertia.m_z); //deprecated? in newton3

	//origin = dVector(0,0,0,0);
	// set the body origin
	//NewtonBodySetCentreOfMass (body, &origin[0]);//deprecated? in newton3

	//NewtonBodySetMassProperties (const NewtonBody* const body, dFloat mass, const NewtonCollision* const collision);
	NewtonBodySetMassProperties(body,mass, collision);





	//also sets default dampening
	setBodyDefaultCallback(body);

	if(newEnt->isPlayer() )
	{

		//
		if(newEnt->planetMode)
			NewtonBodySetForceAndTorqueCallback (body, CharControllerCallbackPlanet);
		else
			NewtonBodySetForceAndTorqueCallback (body, CharControllerCallback);

		newEnt->defaultLinDampening = m_defaultLinearDamping;
	}
	else if(newEnt->isNPC() )
	{
		//
		if(newEnt->planetMode)
			NewtonBodySetForceAndTorqueCallback (body, CharControllerCallbackNPCPlanet);
		else
			NewtonBodySetForceAndTorqueCallback (body, CharControllerCallbackNPC);

		newEnt->defaultLinDampening = m_defaultLinearDamping;
	}
	//else
	//	NewtonBodySetForceAndTorqueCallback (body, ApplyForceAndTorqueCallback);
	

		// set the function callback to set the transformation state of the graphic entity associated with this body 
		// each time the body change position and orientation in the physics world (note: scale is not saved within transformation)
	NewtonBodySetTransformCallback (body, SetTransformCallback);



	newEnt->setBody(body);

	return body;

}


//return real updates
int PhysicsManager::update(float t_step)
{
	//void NewtonUpdate (const NewtonWorld* const newtonWorld, dFloat timestep, int concurrent);
	//NewtonUpdate(g_world,timestep,0);

	int realUpdates = 0;

	/*
	// clean up all pending bodies for update
	for( BodyVectorVector::iterator it = m_bodyUpdateNodeRequests.begin(); it != m_bodyUpdateNodeRequests.end(); it++ )
	{
		for( BodyVector::iterator body = it->begin(); body != it->end(); body++ )
		{
			(*body)->setNodeUpdateNeeded (false);
		}

		it->clear();
	}
	*/


	// clamp the step if necessary
	if (t_step > (m_timestep * m_maxTicksPerFrames)) {
		t_step = m_timestep * m_maxTicksPerFrames;
	}

	// advance the accumulator;
	m_timeAcumulator += t_step;

	while (m_timeAcumulator >= m_timestep) {
		NewtonUpdate (g_world, m_timestep);//todo concurrent param?
		m_timeAcumulator -= m_timestep;
		realUpdates++;
	}




	float param = m_timeAcumulator * m_invTimestep;


	return realUpdates;
}





void PhysicsManager::createHinge(Horde3D::Vec3f axis, Horde3D::Vec3f pos, HordeEntity *entChild, HordeEntity *entParent, bool limits, float limitLowDeg, float limitHighDeg)
{

	if(entChild == NULL)
		return;

	NewtonBody *bodC = entChild->getBody();
	if(bodC == NULL)
		return;
	NewtonBody *bodP = NULL;
	if(entParent)
		bodP = entParent->getBody();

	//Horde3D::Matrix4f transMat;

	// make the joint matrix 
	dVector dir (axis.x, axis.y, axis.z, 0.0f);
	dMatrix pinsAndPivoFrame (dgGrammSchmidt(dir));
	pinsAndPivoFrame.m_posit = dVector (pos.x, pos.y, pos.z, 1.0f);

	NewtonUserJoint* newHinge;
	
	newHinge = CreateCustomHinge (&pinsAndPivoFrame[0][0], bodC, bodP);
	HingeEnableLimits (newHinge, limits);
	HingeSetLimits (newHinge, Horde3D::degToRad(limitLowDeg), Horde3D::degToRad(limitHighDeg));


	entChild->setJoint(newHinge);
	if(entParent && bodP )
		entParent->setJoint(newHinge);

	// deprecated, not longer continue collision is set on the material  	
//	NEWTON_API void NewtonMaterialSetContinuousCollisionMode (const NewtonWorld* const newtonWorld, int id0, int id1, int state);
	//NEWTON_API void NewtonMaterialSetCollisionCallback (const NewtonWorld* const newtonWorld, int id0, int id1, void* const userData,
	//													NewtonOnAABBOverlap aabbOverlap, NewtonContactsProcess process);
}


void PhysicsManager::setBodyDefaultCallback(NewtonBody* bod)
{
	

	NewtonBodySetLinearDamping(bod,m_defaultLinearDamping);
	float adamp[3] = {m_defaultAngularDamping,m_defaultAngularDamping,m_defaultAngularDamping};
	NewtonBodySetAngularDamping(bod,adamp);

	//
	HordeEntity *ent = (HordeEntity*) NewtonBodyGetUserData(bod);
	if(ent->planetMode)
		NewtonBodySetForceAndTorqueCallback (bod, BodyCallbackPlanet);
	else
		NewtonBodySetForceAndTorqueCallback (bod, ApplyForceAndTorqueCallback);
	
}

void PhysicsManager::setBodyPickingCallback(NewtonBody* bod)
{
	NewtonBodySetForceAndTorqueCallback (bod, PickingCallback);
	
	//NewtonBodySetLinearDamping(bod,0.29);
	//float adamp[3] = {0.85,0.85,0.85};
	//NewtonBodySetAngularDamping(bod,adamp);
}





static dFloat UserMeshCollisionCallback (const NewtonBody* const body, const NewtonCollision* const collisionTree, dFloat interception, dFloat* normal, int faceId, void* usedData)
{
	//printf("tree\n");
	return 1.0f;
}

void DestroyBodyCallback (const NewtonBody* body)
{
	// for now there is nothing to destroy
}

// Transform callback to set the matrix of a the visual entity
void SetTransformCallback (const NewtonBody* body, const dFloat* matrix, int threadIndex)
{


	HordeEntity *ent;
	ent = (HordeEntity*) NewtonBodyGetUserData(body);

	
	// Get the position from the matrix
	dVector posit (matrix[12], matrix[13], matrix[14], 1.0f);
	dQuaternion rotation;

	// we will ignore the Rotation part of matrix and use the quaternion rotation stored in the body
	NewtonBodyGetRotation(body, &rotation.m_q0);
	
	
	// since this tutorial run the physics and a different fps than the Graphics
	// we need to save the entity current transformation state before updating the new state.
	ent->m_prevPosition = ent->m_curPosition;
	ent->m_prevRotation = ent->m_curRotation;
	if (ent->m_curRotation.DotProduct (rotation) < 0.0f) {		ent->m_prevRotation.Scale(-1.0f);	}

	// set the new position and orientation for this entity
	ent->m_curPosition = posit;
	ent->m_curRotation = rotation;

	//dMatrix tmat;
	//NewtonBodyGetMatrix(body, &tmat[0][0] );
	//ent->m_matrix = tmat;
	


	ent->tmatrix = matrix;
	ent->matrixUpdated = true;
	//ent->updatePosOri();

	//if(ent->isPlayer() )
	//printf("bupdate\n");
	
}


// callback to apply external forces to body
void ApplyForceAndTorqueCallback (const NewtonBody* body, dFloat timestep, int threadIndex)
{
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	dFloat mass;

	NewtonBodyGetMassMatrix (body, &mass, &Ixx, &Iyy, &Izz);

	dVector gravityForce  (0.0f, mass * GRAVITY, 0.0f, 1.0f);
	NewtonBodySetForce(body, &gravityForce[0]);
}

void BodyCallbackPlanet (const NewtonBody* body, dFloat timestep, int threadIndex)
{
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	dFloat mass;

	dMatrix bodyMatrix;
	NewtonBodyGetMatrix(body, &bodyMatrix[0][0] );	
	dVector bodyPos = bodyMatrix.m_posit;

	NewtonBodyGetMassMatrix (body, &mass, &Ixx, &Iyy, &Izz);
	//dVector gravityForce  (0.0f, mass * GRAVITY, 0.0f, 1.0f);
	Horde3D::Vec3f gravForce(bodyPos.m_x,bodyPos.m_y,bodyPos.m_z);
	gravForce.normalize();
	gravForce *= mass * GRAVITY;
	dVector gravityForce = dVector(gravForce.x, gravForce.y , gravForce.z  );
		
	dVector totalForce = gravityForce;// + moveForce;
		

	NewtonBodySetForce(body, &totalForce[0]);
}



// callback to apply external forces to body
void CharControllerCallback (const NewtonBody* body, dFloat timestep, int threadIndex)
{
	

	PlayerEntity *ent;
	ent = (PlayerEntity*) NewtonBodyGetUserData(body);

	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	dFloat mass;
	NewtonBodyGetMassMatrix (body, &mass, &Ixx, &Iyy, &Izz);

	dMatrix matID(GetIdentityMatrix() );

	//set body upright orientation but same position
	dMatrix tmat;
	NewtonBodyGetMatrix(body, &tmat[0][0] );
	matID[3][0] = tmat[3][0];
	matID[3][1] = tmat[3][1];
	matID[3][2] = tmat[3][2];
	NewtonBodySetMatrix(body,&matID[0][0]);
	
	if(ent->grounded)
	{
		//dVector nvel = dVector(0,0,0);
		//NewtonBodySetVelocity(body,&nvel[0]);


	
		//dVector impPos = dVector(0,0,0);
		//dVector impPos = dVector(tmat[3][0],tmat[3][1],tmat[3][2]);

		//Horde3D::Vec3f impulse = ent->getMoveVector();
		//impulse *= 0.075f;
		//dVector moveImpulse(impulse.x,impulse.y,impulse.z);
		//printf("\nimpulse: %f %f %f \n",impulse.x,impulse.y,impulse.z );
		//NewtonBodyAddImpulse (const NewtonBody* const body, const dFloat* const pointDeltaVeloc, const dFloat* const pointPosit);
		//NewtonBodyAddImpulse (body, &moveImpulse[0], &impPos[0]);

		Horde3D::Vec3f moveVec = ent->getMoveVector();
		//moveVec *= 500.f;
		
		dVector moveForce(moveVec.x,moveVec.y,moveVec.z);
		NewtonBodySetForce(body, &moveForce[0]);
		//printf("\moveVec: %f %f %f \n",moveVec.x,moveVec.y,moveVec.z );
		dVector zero(0,0,0);
		NewtonBodySetOmega(body,&zero[0]);
		
		
		if(moveVec.x == 0 && moveVec.z == 0)
		{
			//dVector gravityForce  (0.0f, -100.f, 0.0f, 1.0f);
			//NewtonBodySetForce(body, &gravityForce[0]);

			//extra damp/break
			dVector curVelocity;
			NewtonBodyGetVelocity(body,&curVelocity[0] );
			curVelocity = curVelocity.Scale(-5.99f * mass);
			NewtonBodySetForce(body, &curVelocity[0]);
		}


		//if(moveVec.x == 0 && moveVec.z == 0)
		//	NewtonBodySetVelocity(body,&omeg[0] );

		//dVector damp(0.99f,0.99f,0.99f);
		//if(moveVec.x == 0 && moveVec.z == 0)
		//	NewtonBodySetAngularDamping (body,&damp[0]);
		//else
		//	NewtonBodySetAngularDamping (body,&zero[0]);
	}
	else
	{
		NewtonBodyGetMassMatrix (body, &mass, &Ixx, &Iyy, &Izz);
		dVector gravityForce  (0.0f, mass * GRAVITY, 0.0f, 1.0f);
		NewtonBodySetForce(body, &gravityForce[0]);
	}

	//printf("timestep: %f\n",timestep);
}



void CharControllerCallbackPlanet(const NewtonBody* body, dFloat timestep, int threadIndex)
{
	

	PlayerEntity *ent;
	ent = (PlayerEntity*) NewtonBodyGetUserData(body);

	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	dFloat mass;
	NewtonBodyGetMassMatrix (body, &mass, &Ixx, &Iyy, &Izz);

	//dMatrix matID(GetIdentityMatrix() );
	//set body upright orientation but same position
	//dMatrix tmat;
	//NewtonBodyGetMatrix(body, &tmat[0][0] );
	//matID[3][0] = tmat[3][0];
	//matID[3][1] = tmat[3][1];
	//matID[3][2] = tmat[3][2];
	//NewtonBodySetMatrix(body,&matID[0][0]);
		

	dMatrix bodyMatrix;
	NewtonBodyGetMatrix(body, &bodyMatrix[0][0] );

	
	dVector bodyPos = bodyMatrix.m_posit;
	//get rotation from current body-up, to the planet's normal(desired target)
	//Vector3 bodyVec = body->getOrientation() * Vector3(0,1,0);
	
	//dVector bodyUp = bodyMatrix.RotateVector(dVector(0,1.f,0) );
	//dVector bodyUp = bodyMatrix.UnrotateVector(dVector(0,1.f,0) );
	
	dQuaternion rotation;
	// we will ignore the Rotation part of matrix and use the quaternion rotation stored in the body
	NewtonBodyGetRotation(body, &rotation.m_q0);
	dVector bodyUp = dVector(0,1.f,0);//rotation.RotateVector(dVector(0,1.f,0) );

	/*
	Horde3D::Matrix4f bodyRot = Horde3D::Matrix4f(&bodyMatrix[0][0]);
	Horde3D::Vec3f trans, rot, scale;
	bodyRot.decompose(trans,rot,scale);
	//headRotTarget = Horde3D::Quaternion(rot.x,rot.y,rot.z);
	bodyRot = bodyRot.RotMat(Horde3D::degToRad(rot.x),Horde3D::degToRad(rot.z),Horde3D::degToRad(rot.y));
	Horde3D::Vec3f bodyUp(0,1.f,0);	
	bodyUp = bodyRot * bodyUp;
	*/
	Horde3D::Vec3f bodPosition = ent->getPosition();

	Horde3D::Quaternion planetOri;
	planetOri = planetOri.getRotationTo(Horde3D::Vec3f(bodyUp.m_x,bodyUp.m_y,bodyUp.m_z),Horde3D::Vec3f(bodyPos.m_x,bodyPos.m_y,bodyPos.m_z) );
	//planetOri = planetOri.getRotationTo(Horde3D::Vec3f(bodyUp.m_x,bodyUp.m_y,bodyUp.m_z),bodPosition, Horde3D::Vec3f(0,1.f,0) );


	dMatrix planetMatrix(dQuaternion(planetOri.x,planetOri.y,planetOri.z,planetOri.w), bodyPos );
	//dMatrix planetMatrix(dQuaternion(planetOri.w,planetOri.z,planetOri.y,planetOri.x), bodyPos );
	//dMatrix planetMatrix(dQuaternion(bodyPos.m_x,bodyPos.m_y,bodyPos.m_z,0.f), bodyPos );
	//planetMatrix[3][0] = bodyMatrix[3][0];
	//planetMatrix[3][1] = bodyMatrix[3][1];
	//planetMatrix[3][2] = bodyMatrix[3][2];
	//NewtonBodySetMatrix(body,&planetMatrix[0][0]);
	
	

	


	

	/*
	Horde3D::Vec3f up = Horde3D::Vec3f(bodyPos.m_x,bodyPos.m_y,bodyPos.m_z);
	up.normalize();
	Horde3D::Vec3f right = Horde3D::Vec3f(0,1.f,0);
	Horde3D::Vec3f forward = up.cross(right);
	forward.normalize();
	right = forward.cross(up);
	right.normalize();
	*/
	Horde3D::Vec3f up = Horde3D::Vec3f(bodyPos.m_x,bodyPos.m_y,bodyPos.m_z);
	up.normalize();	
	Horde3D::Vec3f forward = Horde3D::Vec3f(bodyMatrix.m_front.m_x,bodyMatrix.m_front.m_y,bodyMatrix.m_front.m_z);
	forward.normalize();
	Horde3D::Vec3f right = forward.cross(up);
	right.normalize();
	forward = up.cross(right);
	forward.normalize();
	//up = Horde3D::Vec3f(0,1,0);bodyMatrix
	//right = Horde3D::Vec3f(1,0,0);
	//forward =Horde3D::Vec3f(0,0,1.f);

	// rotation matrix, column major format
	//Horde3D::Matrix4f m1 = [right.x, up.x, -forward.x, 0,
	//	right.y, up.y, -forward.y, 0,
	//	right.z, up.z, -forward.z, 0,
	//	0, 0, 0, 1]
	//horde format
	const float mA[] = { 
		right.x, up.x, forward.x, 0,
		right.y, up.y, forward.y, 0,
		right.z, up.z, forward.z, 0,
		bodyPos.m_x,bodyPos.m_y,bodyPos.m_z, 1 };
	//newton format
	const float mAb[] = { 
		forward.x, forward.y, forward.z, 0,
		 up.x, up.y, up.z, 0,
		right.x, right.y, right.z, 0,
		bodyPos.m_x,bodyPos.m_y,bodyPos.m_z, 1 };
	Horde3D::Matrix4f planetRotation(mAb);
	//matrixA = matrixA.transposed();
	NewtonBodySetMatrix(body,&planetRotation.c[0][0]);
	

	/*
	Horde3D::Vec3f trans, rot, scale;
	planetRotation.decompose(trans,rot,scale);
	//headRotTarget = Horde3D::Quaternion(rot.x,rot.y,rot.z);
	dQuaternion targetRotation(rot.x,rot.y,rot.z,1.0f);
	dQuaternion currentRotation;
	NewtonBodyGetRotation(body, &currentRotation.m_q0);	
	currentRotation = currentRotation.Slerp(targetRotation,timestep);
	*/




	


	//ent->tmatrix = &bodyMatrix[0][0];
	//ent->matrixUpdated = true;




	if(ent->grounded)
	{
		

		
		Horde3D::Vec3f moveVec = ent->getMoveVector();
		//moveVec *= 500.f;
		
		dVector moveForce(moveVec.x,moveVec.y,moveVec.z);
		NewtonBodySetForce(body, &moveForce[0]);
		//printf("\moveVec: %f %f %f \n",moveVec.x,moveVec.y,moveVec.z );
		
		
		
		if(moveVec.x == 0 && moveVec.z == 0)
		{
			//dVector gravityForce  (0.0f, -100.f, 0.0f, 1.0f);
			//NewtonBodySetForce(body, &gravityForce[0]);

			//extra damp/break
			dVector curVelocity;
			NewtonBodyGetVelocity(body,&curVelocity[0] );
			curVelocity = curVelocity.Scale(-5.99f * mass);
			NewtonBodySetForce(body, &curVelocity[0]);
		}
		

		
	}
	else
	{
		//move slightly in mid-air, may cause orbit exploit flying
		//Horde3D::Vec3f moveVec = ent->getMoveVector();
		//moveVec *= 0.2f;		
		//dVector moveForce(moveVec.x,moveVec.y,moveVec.z);

		NewtonBodyGetMassMatrix (body, &mass, &Ixx, &Iyy, &Izz);
		//dVector gravityForce  (0.0f, mass * GRAVITY, 0.0f, 1.0f);
		Horde3D::Vec3f gravForce(bodyPos.m_x,bodyPos.m_y,bodyPos.m_z);
		gravForce.normalize();
		gravForce *= mass * GRAVITY;
		dVector gravityForce = dVector(gravForce.x, gravForce.y , gravForce.z  );
		
		dVector totalForce = gravityForce;// + moveForce;
		

		NewtonBodySetForce(body, &totalForce[0]);




	}

	dVector zero(0,0,0);
	NewtonBodySetOmega(body,&zero[0]);

	//printf("timestep: %f\n",timestep);
}


void CharControllerCallbackNPC (const NewtonBody* body, dFloat timestep, int threadIndex)
{
	

	NPCEntity *ent;
	ent = (NPCEntity*) NewtonBodyGetUserData(body);

	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	dFloat mass;
	NewtonBodyGetMassMatrix (body, &mass, &Ixx, &Iyy, &Izz);

	dMatrix matID(GetIdentityMatrix() );

	//set body upright orientation but same position
	dMatrix tmat;
	NewtonBodyGetMatrix(body, &tmat[0][0] );
	matID[3][0] = tmat[3][0];
	matID[3][1] = tmat[3][1];
	matID[3][2] = tmat[3][2];
	NewtonBodySetMatrix(body,&matID[0][0]);
	//printf("setste\n");
	if(ent->grounded)
	{
		//dVector nvel = dVector(0,0,0);
		//NewtonBodySetVelocity(body,&nvel[0]);


	
		//dVector impPos = dVector(0,0,0);
		//dVector impPos = dVector(tmat[3][0],tmat[3][1],tmat[3][2]);

		//Horde3D::Vec3f impulse = ent->getMoveVector();
		//impulse *= 0.075f;
		//dVector moveImpulse(impulse.x,impulse.y,impulse.z);
		//printf("\nimpulse: %f %f %f \n",impulse.x,impulse.y,impulse.z );
		//NewtonBodyAddImpulse (const NewtonBody* const body, const dFloat* const pointDeltaVeloc, const dFloat* const pointPosit);
		//NewtonBodyAddImpulse (body, &moveImpulse[0], &impPos[0]);

		Horde3D::Vec3f moveVec = ent->getMoveVector();
		//moveVec *= 500.f;
		

		//extra damp/break
		dVector curVelocity;
		NewtonBodyGetVelocity(body,&curVelocity[0] );
		dVector extraBreak = curVelocity.Scale(-0.99f * mass);// 5.99

		float gravityOffset = mass * GRAVITY * .5f;
		dVector moveForce(moveVec.x,moveVec.y + gravityOffset,moveVec.z);
		moveForce += extraBreak;
		NewtonBodySetForce(body, &moveForce[0]);
		//printf("\moveVec: %f %f %f \n",moveVec.x,moveVec.y,moveVec.z );
		dVector zero(0,0,0);
		NewtonBodySetOmega(body,&zero[0]);
		
		
		if(moveVec.x == 0 && moveVec.z == 0)
		{
			//dVector gravityForce  (0.0f, -100.f, 0.0f, 1.0f);
			//NewtonBodySetForce(body, &gravityForce[0]);

			//extra damp/break
			//dVector curVelocity;
			//NewtonBodyGetVelocity(body,&curVelocity[0] );
			//curVelocity = curVelocity.Scale(-5.99f * mass);
			//NewtonBodySetForce(body, &curVelocity[0]);
		}

		
		//if(moveVec.x == 0 && moveVec.z == 0)
		//	NewtonBodySetVelocity(body,&omeg[0] );

		//dVector damp(0.99f,0.99f,0.99f);
		//if(moveVec.x == 0 && moveVec.z == 0)
		//	NewtonBodySetAngularDamping (body,&damp[0]);
		//else
		//	NewtonBodySetAngularDamping (body,&zero[0]);
	}
	else
	{
		NewtonBodyGetMassMatrix (body, &mass, &Ixx, &Iyy, &Izz);
		dVector gravityForce  (0.0f, mass * GRAVITY, 0.0f, 1.0f);
		NewtonBodySetForce(body, &gravityForce[0]);
	}

	//printf("timestep: %f\n",timestep);
}



void CharControllerCallbackNPCPlanet (const NewtonBody* body, dFloat timestep, int threadIndex)
{
	
	NPCEntity *ent;
	ent = (NPCEntity*) NewtonBodyGetUserData(body);

	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	dFloat mass;
	NewtonBodyGetMassMatrix (body, &mass, &Ixx, &Iyy, &Izz);


	dMatrix bodyMatrix;
	NewtonBodyGetMatrix(body, &bodyMatrix[0][0] );

	
	dVector bodyPos = bodyMatrix.m_posit;
		
	Horde3D::Vec3f up = Horde3D::Vec3f(bodyPos.m_x,bodyPos.m_y,bodyPos.m_z);
	up.normalize();	
	Horde3D::Vec3f forward = Horde3D::Vec3f(bodyMatrix.m_front.m_x,bodyMatrix.m_front.m_y,bodyMatrix.m_front.m_z);
	forward.normalize();
	Horde3D::Vec3f right = forward.cross(up);
	right.normalize();
	forward = up.cross(right);
	forward.normalize();
	
	//newton format
	const float mAb[] = { 
		forward.x, forward.y, forward.z, 0,
		 up.x, up.y, up.z, 0,
		right.x, right.y, right.z, 0,
		bodyPos.m_x,bodyPos.m_y,bodyPos.m_z, 1 };
	Horde3D::Matrix4f planetRotation(mAb);
	//matrixA = matrixA.transposed();
	NewtonBodySetMatrix(body,&planetRotation.c[0][0]);
	



	//if(ent->grounded)
	//{
		

		
		Horde3D::Vec3f moveVec = ent->getMoveVector();
		//moveVec *= 500.f;
		
		dVector moveForce(moveVec.x,moveVec.y,moveVec.z);

		Horde3D::Vec3f gravForce(bodyPos.m_x,bodyPos.m_y,bodyPos.m_z);
		gravForce.normalize();
		gravForce *= mass * GRAVITY * 1.1f;
		dVector gravityForce = dVector(gravForce.x, gravForce.y , gravForce.z  );
		
		moveForce += gravityForce;

		NewtonBodySetForce(body, &moveForce[0]);
		//printf("\moveVec: %f %f %f \n",moveVec.x,moveVec.y,moveVec.z );
		dVector zero(0,0,0);
		NewtonBodySetOmega(body,&zero[0]);
		
		

}


static void PickingCallback (const NewtonBody* body, dFloat timestep, int threadIndex)
{
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	dFloat mass;

	NewtonBodyGetMassMatrix (body, &mass, &Ixx, &Iyy, &Izz);

	//dVector gravityForce  (0.0f, mass * GRAVITY, 0.0f, 1.0f);
	//NewtonBodySetForce(body, &gravityForce[0]);


	HordeEntity *ent;
	ent = (HordeEntity*) NewtonBodyGetUserData(body);
		
	dVector pickingPos = ent->getPickingPos();


	dMatrix tmat;
	NewtonBodyGetMatrix(body, &tmat[0][0] );
	
	dVector bodyPos(tmat[3][0],tmat[3][1],tmat[3][2]);
	
	//printf("pickpos:%f %f %f\n", pickingPos[0],pickingPos[1],pickingPos[2] );

	dVector distanceDiff = (pickingPos - bodyPos);

	dVector bodVelocity;
	NewtonBodyGetVelocity(body, &bodVelocity[0]);
	

	//dVector springForce =  distanceDiff.Scale(100.f) ;
	//dVector dampening = bodVelocity.Scale(20.f);

	//dVector pickingForce = springForce - dampening;
	//pickingForce.Scale( 0.00001f);


	dFloat forceScale =  15.f - bodVelocity%bodVelocity;
	forceScale = dClampValue(forceScale,0.01f,15.0f);

	//printf("velsq: %f\n", bodVelocity%bodVelocity );

	//pickingforce = springforce - dampening
	dVector pickingForce = distanceDiff.Scale(9.f) - bodVelocity.Scale(0.9f);//(distanceDiff).Scale(forceScale); // 9. .7
	pickingForce = pickingForce.Scale(mass * 30.f);//40, too large moves player

	NewtonBodyAddForce(body, &pickingForce[0]);


	//stop rotational velocity
	dVector omeg(0,0,0);
	NewtonBodySetOmega(body,&omeg[0]);
}
