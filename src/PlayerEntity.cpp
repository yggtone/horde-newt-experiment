

#include "PlayerEntity.h"


PlayerEntity::PlayerEntity(int hordeID, std::string contentDir): HordeEntity(hordeID, contentDir)
{
	//m_hordeID =hordeID;
	is_player = true;
	grounded = false;

	jumpDelay = 0.f;
}

PlayerEntity::~PlayerEntity()
{
	

}

static unsigned ConvexCastCallback (const NewtonBody* body, const NewtonCollision* collision, void* userData)
{
	// this convex cast have to skip the casting body
	NewtonBody* me = (NewtonBody*) userData;
	return (me == body) ? 0 : 1;
}


// implement a ray cast pre-filter
static unsigned RayCastPrefilter (const NewtonBody* body,  const NewtonCollision* collision, void* userData)
{
	// ray cannot pick trigger volumes

	dFloat mass;
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	NewtonBodyGetMassMatrix (body, &mass, &Ixx, &Iyy, &Izz);

	// do not pick static bodies
	//return (mass > 0.0f) ? 1 : 0;
	return 1;
}
static dFloat pickedParam;
static NewtonBody* pickedBody;
static bool isPickedBodyDynamics;
// implement a ray cast filter
static dFloat RayCastFilter (const NewtonBody* body, const dFloat* normal, int collisionID, void* userData, dFloat intersetParam)
{
	dFloat mass;
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;

	NewtonBodyGetMassMatrix (body, &mass, &Ixx, &Iyy, &Izz);
	if (intersetParam <  pickedParam) {
		isPickedBodyDynamics = (mass > 0.0f);
		pickedParam = intersetParam;
		pickedBody = (NewtonBody*)body;
//		rayLocalNormal = dVector (normal[0], normal[1], normal[2]);
	}
	return intersetParam;
}

void PlayerEntity::jump()
{
	//if(!grounded)
	//	return;

	if(jumpDelay > 0.f)
		return;

	if(this->planetMode)
	{
		//use separate raycast for jump in case char contrller is slightly above grounded
		NewtonWorld* world;
		world = NewtonBodyGetWorld(m_body);

		Horde3D::Vec3f playerPos = this->getPosition();	
		dVector p0 (playerPos.x, playerPos.y,playerPos.z);	
		dVector p1 = p0.Scale(0.95f); //.98



		pickedBody = NULL;
		pickedParam = 1.1f;
		NewtonWorldRayCast(world, &p0[0], &p1[0], RayCastFilter, NULL, RayCastPrefilter);
		if (!pickedBody) 
		{
			return;
		}

		jumpDelay = 0.2f;

		//
	

		dVector impPos = dVector(playerPos.x,playerPos.y,playerPos.z);

	
		//dVector moveImpulse(0,10.0,0);
		//planet vers
		Horde3D::Vec3f jumpImpulse = playerPos.normalized() * 20.0f;
		dVector moveImpulse = dVector(jumpImpulse.x,jumpImpulse.y,jumpImpulse.z);
		NewtonBodyAddImpulse (m_body, &moveImpulse[0], &impPos[0]);
	}
	else
	{


		Horde3D::Vec3f playerPos = this->getPosition();

		dVector impPos = dVector(playerPos.x,playerPos.y,playerPos.z);

	
		//dVector moveImpulse(0,10.0,0);
		//planet vers
		Horde3D::Vec3f jumpImpulse = Horde3D::Vec3f(0,10.f,0);
		dVector moveImpulse = dVector(jumpImpulse.x,jumpImpulse.y,jumpImpulse.z);
		NewtonBodyAddImpulse (m_body, &moveImpulse[0], &impPos[0]);
	}
}


void PlayerEntity::update(float timestep)
{
	if(jumpDelay > 0.f)
		jumpDelay -= timestep;

	updatePosOri();
	NewtonWorld* world;
	world = NewtonBodyGetWorld(m_body);

	Horde3D::Vec3f playerPos = this->getPosition();
	
	//dVector p0 (ScreenToWorld(dVector (cursorPosit1.m_x, cursorPosit1.m_y, 0.0f, 0.0f)));
	//dVector p1 (ScreenToWorld(dVector (cursorPosit1.m_x, cursorPosit1.m_y, 1.0f, 0.0f)));
	dVector p0 (playerPos.x, playerPos.y,playerPos.z);
	
	dVector p1;
	//flat version
	if(this->planetMode)
	{
		//planet version, slightly smaller vector, closer to origin of planet(0,0,0)
		p1 =  p0.Scale(0.95f); //.98
	}
	else
	{
		p1 =  dVector(playerPos.x, playerPos.y - (float)PLAYERHEIGHT - 0.2f,playerPos.z);
	}

	//falling ungrounded, low dampening
	NewtonBodySetLinearDamping(m_body,defaultLinDampening);
	//float adamp[3] = {.09f,.09f,.09f};
	//NewtonBodySetAngularDamping(m_body,adamp);


	grounded = false;
	pickedBody = NULL;
	pickedParam = 1.1f;
	isPickedBodyDynamics = false;
	NewtonWorldRayCast(world, &p0[0], &p1[0], RayCastFilter, NULL, RayCastPrefilter);
	if (pickedBody) 
	{
		//need to make feed grounded when standing on non-static tables, etc. or can't move
		grounded = true;
		NewtonBodySetLinearDamping(m_body,0.9999f);
		

	}





	return;



}
