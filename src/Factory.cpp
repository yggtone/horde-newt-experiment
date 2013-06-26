

#include "factory.h"

static void GenericContactProcess (const NewtonJoint* contactJoint, dFloat timestep, int threadIndex);

Factory::Factory(): player(NULL), currentPickedEnt(NULL), m_soundManager(NULL), m_physicsManager(NULL)
{
	

	cam = 0;

	running = false;
	
	_statMode = 0;
	_freezeMode = 0; 
	_debugViewMode = false; 
	_wireframeMode = false;

	

	loadScreenAlpha = 0;
}

Factory::~Factory()
{
	release();
	/*
	h3dRelease();

	
	if(m_physicsManager);
		delete m_physicsManager;
	if(m_soundManager)
		delete m_soundManager;
	m_soundManager = NULL;
	m_physicsManager = NULL;
	*/

	
}

void Factory::release()
{
	h3dRelease();

	
	
	if(m_physicsManager)
		delete m_physicsManager;
	m_physicsManager = NULL;
	if(m_soundManager)
		delete m_soundManager;
	m_soundManager = NULL;


	//h3dReleaseUnusedResources();
	cam = 0;
	player = NULL;
	currentPickedEnt = NULL;

	for(int i=0; i < entities.size(); i++)
	{
		delete entities[i];
	}
	entities.clear();

	

}

void Factory::reset()
{
	
	for(int i=0; i < entities.size(); i++)
	{
		//entities[i]->releaseNode();
		delete entities[i];
	}
	entities.clear();

	if(m_physicsManager)
		delete m_physicsManager;
	m_physicsManager = NULL;
	if(m_soundManager)
		delete m_soundManager;
	m_soundManager = NULL;

	player = NULL;
	currentPickedEnt = NULL;

	

	h3dClear();

	
	
}


bool Factory::init(std::string contentDir)
{
	m_soundManager = new SoundManager();
	m_physicsManager = new PhysicsManager(200,5);

	_contentDir = contentDir;

	
	
	
	if(!running)
	{
		// Initialize engine
		if( !h3dInit() )
		{	
			h3dutDumpMessages();
			return false;
		}
		running = true;
	}
	
	// Set options
	h3dSetOption( H3DOptions::LoadTextures, 1 );
	h3dSetOption( H3DOptions::TexCompression, 1 );
	h3dSetOption( H3DOptions::MaxAnisotropy, 16 );
	h3dSetOption( H3DOptions::ShadowMapSize, 1024 );
	h3dSetOption( H3DOptions::FastAnimation, 0 );





	//load overlay
	// Overlays
	_fontMatRes = h3dAddResource( H3DResTypes::Material, "overlays/font.material.xml", 0 );
	_panelMatRes = h3dAddResource( H3DResTypes::Material, "overlays/panel.material.xml", 0 );
	_logoMatRes = h3dAddResource( H3DResTypes::Material, "overlays/logo.material.xml", 0 );
	h3dutLoadResourcesFromDisk( _contentDir.c_str() );


	return true;
}


//struct object needed to pass in class pointer to newton collision callback userdata
struct SoundEffect
{
	void * m_sound;
	SoundManager * m_manager;

	int soundEffect;
};
SoundEffect soundTest;






NPCEntity* Factory::createNPC(std::string fileName,float tx,float ty,float tz,float rx,float ry,float rz,float sx ,float sy ,float sz ,
		 float mass , int materialID , int type, bool planetMode )
{

	H3DRes modelRes = h3dAddResource( H3DResTypes::SceneGraph,fileName.c_str(), 0 );
	//H3DRes charControllerRes = h3dAddResource( H3DResTypes::SceneGraph,"models/ellipsoid.scene.xml", 0 );
	H3DRes charControllerRes = h3dAddResource( H3DResTypes::SceneGraph,"models/collisionD/collisionD.scene.xml", 0 );
	h3dutLoadResourcesFromDisk( _contentDir.c_str() );
	
	H3DNode newNode;
	//HordeEntity *ent;

	//character controller, scale Y for tall ellipsoid
	newNode = h3dAddNodes( H3DRootNode, charControllerRes );
	float cscale = 1.1f;//12.7f;
	//h3dSetNodeTransform(newNode,tx,ty,tz,rx,ry,rz,sx * cscale * .5f,sy *cscale,sz * cscale * .5f);		
	h3dSetNodeTransform(newNode,tx,ty,tz,rx,ry,rz,sx * cscale,sy *cscale,sz * cscale );		
	
	//character mesh
	H3DNode meshID = h3dAddNodes( H3DRootNode, modelRes );
	h3dSetNodeTransform(meshID,tx,ty,tz,rx,ry,rz,sx,sy,sz);	

	NPCEntity* newNPC = new NPCEntity(meshID,newNode, _contentDir);

	newNPC->planetMode = planetMode;
	//ent->setType(type);
	//ent->staticBody = staticBody;
	if(m_physicsManager  )
	{
		
		m_physicsManager->createBodyFromPoints( newNPC,mass);
		
		NewtonBodySetMaterialGroupID(newNPC->getBody(),materialID);
	}
	
	
	//ent->setType(11);
	entities.push_back(newNPC);


	
	return newNPC;


}



PlayerEntity* Factory::createPlayer(float mass, int materialID, bool planetMode)
{
	H3DRes modelRes = h3dAddResource( H3DResTypes::SceneGraph,"models/ellipsoid.scene.xml", 0 );
	h3dutLoadResourcesFromDisk( _contentDir.c_str() );
	

	//player
	H3DNode playerNode = h3dAddNodes( H3DRootNode, modelRes );
	h3dSetNodeTransform(playerNode,2,1.0,0,   0.f,0,0,   0.35,PLAYERHEIGHT,0.35);	
	//h3dSetNodeTransform(playerNode,0,50,0,   0.f,0,0,   0.5,0.5,0.5);	



	PlayerEntity *pent = new PlayerEntity(playerNode, _contentDir);
	pent->planetMode = planetMode;

	if(m_physicsManager)
	{
		m_physicsManager->createBodyFromPoints( pent,mass);//50	
		NewtonBodySetMaterialGroupID(pent->getBody(),materialID);	
	}
	
	entities.push_back(pent);
	player = pent;
	
	return pent;
}


int Factory::createPhysicsMaterial()
{
	return NewtonMaterialCreateGroupID (m_physicsManager->getWorld());
}


void Factory::setPhysicsMaterialPair(int matA, int matB, int soundEffect, MaterialCallbackType callbackType, float elasticCoef, float staticFric, float kineticFriction)
{

	//TODO various callback function types, add option elasticity,friction params

	//struct to pass pointers to contact callbacks
	soundTest.m_manager = m_soundManager;
	//soundTest.soundeffect = soundEffect;

	if(callbackType == characterCallback)
	{
		
	}
	else
	{
		NewtonMaterialSetCollisionCallback (m_physicsManager->getWorld(), matA,matB, &soundTest, NULL,  GenericContactProcess);
	}

	//NewtonMaterialSetDefaultElasticity(const NewtonWorld* const newtonWorld, int id0, int id1, dFloat elasticCoef)
	//test
	NewtonMaterialSetDefaultElasticity(m_physicsManager->getWorld(), matA,matB, elasticCoef);
	
	//void NewtonMaterialSetDefaultFriction(const NewtonWorld* const newtonWorld, int id0, int id1, dFloat staticFriction, dFloat kineticFriction)
	NewtonMaterialSetDefaultFriction(m_physicsManager->getWorld(),  matA,matB, staticFric,kineticFriction);
	
}

int Factory::createCamera(std::string pipelineFileName)
{
	//if(cam)
	//	return;


	H3DRes pipelineResource = h3dAddResource( H3DResTypes::Pipeline, pipelineFileName.c_str(), 0 );
	
	_currentPipeRes = pipelineResource;

	// Load resources
	h3dutLoadResourcesFromDisk( _contentDir.c_str() );
	

	// Add camera, save reference
	cam = h3dAddCameraNode( H3DRootNode, "Camera", pipelineResource );

	return cam;
}



HordeEntity* Factory::createLight(std::string fileName, std::string name, float tx,float ty,float tz,float rx,float ry,float rz,float sx,float sy,float sz,
		float red, float green, float blue, float radius,float fov, float bias, int shadowMaps )
{


	H3DRes lightMatRes = h3dAddResource( H3DResTypes::Material, fileName.c_str(), 0 );	
	// Load resources
	h3dutLoadResourcesFromDisk( _contentDir.c_str() );
	// Add light source
	H3DNode light = h3dAddLightNode( H3DRootNode, name.c_str(), lightMatRes, "LIGHTING", "SHADOWMAP" );
	h3dSetNodeTransform( light, tx,ty,tz,rx,ry,rz,sx,sy,sz );
	h3dSetNodeParamF( light, H3DLight::RadiusF, 0, radius );
	h3dSetNodeParamF( light, H3DLight::FovF, 0, fov );
	h3dSetNodeParamI( light, H3DLight::ShadowMapCountI, shadowMaps );
	h3dSetNodeParamF( light, H3DLight::ShadowSplitLambdaF, 0, 0.6f );
	h3dSetNodeParamF( light, H3DLight::ShadowMapBiasF, 0, bias );
	//h3dSetNodeParamF( light, H3DLight::shadowMaps, 0, 0.0009f );
	h3dSetNodeParamF( light, H3DLight::ColorF3, 0, red );
	h3dSetNodeParamF( light, H3DLight::ColorF3, 1, green );
	h3dSetNodeParamF( light, H3DLight::ColorF3, 2, blue );


	HordeEntity * lightEnt = new HordeEntity(light, _contentDir);
	entities.push_back(lightEnt);

	return lightEnt;
}

HordeEntity* Factory::createEntity(std::string fileName,float tx,float ty,float tz,float rx,float ry,float rz,float sx,float sy,float sz,
	 bool enablePhysics, bool staticBody, float mass, int materialID, int type, bool pickable, bool planetMode)
{

	H3DRes modelRes = h3dAddResource( H3DResTypes::SceneGraph,fileName.c_str(), 0 );
	h3dutLoadResourcesFromDisk( _contentDir.c_str() );
	
	H3DNode newNode;
	HordeEntity *ent;

	newNode = h3dAddNodes( H3DRootNode, modelRes );
	h3dSetNodeTransform(newNode,tx,ty,tz,rx,ry,rz,sx,sy,sz);		
	ent = new HordeEntity(newNode, _contentDir);

	ent->setType(type);
	ent->setPickable(pickable);
	//ent->staticBody = staticBody;
	ent->planetMode = planetMode;
	if(m_physicsManager && enablePhysics )
	{
		if(staticBody)
			m_physicsManager->createTreeFromPoints( ent,0.0f);
		else
			m_physicsManager->createBodyFromPoints( ent,mass);


		NewtonBodySetMaterialGroupID(ent->getBody(),materialID);
	}
	
	//ent->setType(11);
	
	entities.push_back(ent);

	return ent;
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
	return (mass > 0.0f) ? 1 : 0;
	//return 1;
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





void Factory::pickingRelease()
{
	
	if(currentPickedEnt)
	{

		m_physicsManager->setBodyDefaultCallback(currentPickedEnt->getBody() );

		currentPickedEnt = NULL;

	}
}




void Factory::pickingCast()
{
	
	if(!player)
		return;


	NewtonWorld* world;
	world = NewtonBodyGetWorld(player->getBody());

	Horde3D::Vec3f playerPos = player->getPosition();
	Horde3D::Vec3f camPos;
	
	
	Horde3D::Vec3f pickRay(0,0,-PICKLENGTH);
	const float *camOri;
	h3dGetNodeTransMats(cam,&camOri,NULL);
	Horde3D::Matrix4f camOriRot(camOri);
	camPos.x = camOriRot.c[3][0]; 
	camPos.y = camOriRot.c[3][1]; 
	camPos.z = camOriRot.c[3][2]; 
	camOriRot.c[3][0] = 0;
	camOriRot.c[3][1] = 0;
	camOriRot.c[3][2] = 0;
	pickRay = camOriRot * pickRay;
	



	Horde3D::Vec3f pickEnd = camPos + pickRay;

	dVector p0 (camPos.x, camPos.y,camPos.z);
	dVector p1 (pickEnd.x, pickEnd.y,pickEnd.z);
	
	
	pickedBody = NULL;
	pickedParam = 1.1f;
	isPickedBodyDynamics = false;
	NewtonWorldRayCast(world, &p0[0], &p1[0], RayCastFilter, NULL, RayCastPrefilter);
	if (pickedBody) 
	{
		
		HordeEntity * bEnt = (HordeEntity*) NewtonBodyGetUserData(pickedBody);
		if(bEnt)
		{
			if(bEnt->isPickable() )
			{
				printf("picked\n");
				//todo if pickable
				currentPickedEnt = bEnt;			
				m_physicsManager->setBodyPickingCallback(pickedBody);
				//bEnt->setPickingPos(pickEnd);
				bEnt->setPickingPos(bEnt->getPosition());

				pickRayCurrentLength = (camPos - bEnt->getPosition() ).length();
			}
		}
		

	}



}


void Factory::setMoveVector(Horde3D::Vec3f moveVector, bool run)
{
	if(!player)
		return;

	if(!player->planetMode)
	{
	
		const float *camOri;
		h3dGetNodeTransMats(cam,&camOri,NULL);
		Horde3D::Matrix4f camOriRot(camOri);
		camOriRot.c[3][0] = 0;
		camOriRot.c[3][1] = 0;
		camOriRot.c[3][2] = 0;
		moveVector = camOriRot * moveVector;
	
		moveVector.y = 0;
		moveVector.normalize();
		moveVector *= (float)PLAYERSPEED;
		if(run)
			moveVector *= 2.0;
		player->setMoveVector(moveVector);

	}
	else
	{
		//planet version
		Horde3D::Matrix4f horizlookRot;
		horizlookRot = horizlookRot.RotMat(0,Horde3D::degToRad(_ry),0);
		moveVector = (camMatrix *  horizlookRot) * moveVector;

		//moveVector.y = 0;
		moveVector.normalize();
		moveVector *= (float)PLAYERSPEED;
		if(run)
			moveVector *= 2.0;
		//printf("\nvelvec: %f %f %f \n",moveVector.x,moveVector.y,moveVector.z );
	
		player->setMoveVector(moveVector);

	}
}

void Factory::update(float timestep)
{
	//in case init hasn't been called
	//if(!m_physicsManager)
	//	return;


	
	
	if(m_physicsManager)
		m_physicsManager->update(timestep);	



	

	
	for(int i=0; i < entities.size() ; i++)
	{
		entities[i]->update(timestep);


	}


	
	Horde3D::Vec3f playerPos;

	//set camera rotations from mouse
	h3dSetNodeTransform( cam, _x, _y, _z, _rx ,_ry, 0, 1, 1, 1 );
	
	
	//update camera to first person of player
	if(player)
	{

		if(!player->planetMode)
		{
			//flatland version
		
			//get camera orientation
			const float* cOrigCam = 0x0;
			h3dGetNodeTransMats(cam, &cOrigCam, NULL );
			playerPos = player->getPosition();
			//set position of transf matrix to the player
			Horde3D::Matrix4f camFinalTran(cOrigCam);
			camFinalTran.c[3][0] = playerPos.x;
			camFinalTran.c[3][1] = playerPos.y + (float)(PLAYERHEIGHT * .5f);
			camFinalTran.c[3][2] = playerPos.z;
			h3dSetNodeTransMat( cam, &camFinalTran.c[0][0] );
		
		}
		else
		{


			//planetary version-====================
			//character controller orientation calculated in callback function (CharControllerCallbackPlanet)
			//read from player here for final camera transformation
			playerPos = player->getPosition() * 1.01f;//slightly further from planet center to get head height
			//float tx,ty,tz,rx,ry,rz,sx,sy,sz;		
			//h3dSetNodeTransMat( cam, player->getTransformationMat() );
			//h3dGetNodeTransform(cam,&tx,&ty,&tz,&rx,&ry,&rz,&sx,&sy,&sz);
			//h3dSetNodeTransform(cam,tx,ty,tz,    rx ,ry,rz,   1, 1, 1);
			//h3dSetNodeTransform(cam,tx,ty,tz, rx +  _rx ,ry +_ry,rz,   1, 1, 1);

			//rotation matrix from player look movement
			Horde3D::Matrix4f mouselookRot;
			mouselookRot = mouselookRot.RotMat(Horde3D::degToRad(_rx),Horde3D::degToRad(_ry),0);
			//from transformation matrix of player, get character controller body orientation
			Horde3D::Matrix4f playerMat(player->getTransformationMat() );
			Horde3D::Vec3f trans, rot, scale;
			playerMat.decompose(trans,rot,scale);
			camRotTarget = Horde3D::Quaternion(rot.x,rot.y,rot.z);
			//interpolate from last orientation (smooth movement, also necessary for some matrix building anomalies)
			camRotCurrent = camRotCurrent.slerp(camRotTarget,timestep * 4.f);
			//build final cam matrix, rotation with translation offset
			Horde3D::Matrix4f finalcamMatrix(camRotCurrent);	
			//save for use with moveVector
			camMatrix = finalcamMatrix;
			finalcamMatrix.c[3][0] = playerPos.x; finalcamMatrix.c[3][1] = playerPos.y; finalcamMatrix.c[3][2] = playerPos.z;
		

			finalcamMatrix = finalcamMatrix * mouselookRot;//apply mouse rotation directly to final matrix to avoid lag
			h3dSetNodeTransMat(cam,&finalcamMatrix.c[0][0] );

		}

	
	}
	
	//camlook, override player cam
	if(_camlook)
		h3dSetNodeTransform( cam, _x, _y, _z, _rx ,_ry, 0, 1, 1, 1 );
	

	Horde3D::Vec3f camPos;
	const float *camOri;
	h3dGetNodeTransMats(cam,&camOri,NULL);
	Horde3D::Matrix4f camOriRot(camOri);
	camPos.x = camOriRot.c[3][0]; 
	camPos.y = camOriRot.c[3][1]; 
	camPos.z = camOriRot.c[3][2]; 

	if(player && currentPickedEnt)
	{		
		//update player picking position, to be read by picking callbacks
		Horde3D::Vec3f pickRay(0,0,-pickRayCurrentLength);		
		camOriRot.c[3][0] = 0;
		camOriRot.c[3][1] = 0;
		camOriRot.c[3][2] = 0;
		pickRay = camOriRot * pickRay;	
		Horde3D::Vec3f pickEnd = camPos + pickRay;

		
		currentPickedEnt->setPickingPos(pickEnd);

	}

	std::string printpos = "pos " + toString(camPos.x) + " " + toString(camPos.y) + " " + toString(camPos.z) + " \n";
	h3dutShowText( printpos.c_str()  , 0.03f, 0.042f, 0.03f, 1, 1, 1, _fontMatRes );
	



	if(m_soundManager)
		m_soundManager->update();






	// Show stats
	h3dutShowFrameStats( _fontMatRes, _panelMatRes, _statMode );
	if( _statMode > 0 )
	{
		if( h3dGetNodeParamI( cam, H3DCamera::PipeResI ) == _forwardPipeRes )
			h3dutShowText( "Pipeline: forward", 0.03f, 0.24f, 0.026f, 1, 1, 1, _fontMatRes );
		else
			h3dutShowText( "Pipeline: deferred", 0.03f, 0.24f, 0.026f, 1, 1, 1, _fontMatRes );
	}
	h3dSetOption( H3DOptions::DebugViewMode, _debugViewMode ? 1.0f : 0.0f );
	h3dSetOption( H3DOptions::WireframeMode, _wireframeMode ? 1.0f : 0.0f );
	

	
	//render


	//show cursor	
	const float ww = (float)h3dGetNodeParamI( cam, H3DCamera::ViewportWidthI ) /
	                 (float)h3dGetNodeParamI( cam, H3DCamera::ViewportHeightI );
	//const float ww = 1920.f / 1080.f;
	//const float ovLogo[] = { ww-0.4f, 0.8f, 0, 1,  ww-0.4f, 1, 0, 0,  ww, 1, 1, 0,  ww, 0.8f, 1, 1 };
	float cursorWidthHalf = .005f;
	const float ovLogo[] = { ww * .5 - cursorWidthHalf, 0.49f, 0, 1,    ww * .5  - cursorWidthHalf,.51f, 0, 0,
		ww * .5  + cursorWidthHalf,.51f, 1, 0,      ww * .5  + cursorWidthHalf,.49f, 1, 1 };
	h3dShowOverlays( ovLogo, 4, 1.f, 1.f, 1.f, 1.f, _logoMatRes, 0 );

	


	if(loadScreenAlpha > 0.0f)
	{
		const float ww = (float)h3dGetNodeParamI( cam, H3DCamera::ViewportWidthI ) /
						 (float)h3dGetNodeParamI( cam, H3DCamera::ViewportHeightI );
		const float ovLogo[] = { ww-0.4f, 0.8f, 0, 1,  ww-0.4f, 1, 0, 0,  ww, 1, 1, 0,  ww, 0.8f, 1, 1 };
		h3dShowOverlays( ovLogo, 4, 1.f, 1.f, 1.f, loadScreenAlpha, _logoMatRes, 0 );
	}







}

void Factory::render()
{
	// Render scene
	h3dRender( cam );

	// Finish rendering of frame
	h3dFinalizeFrame();

	// Remove all overlays
	h3dClearOverlays();

	// Write all messages to log file
	//h3dutDumpMessages();

}

void Factory::camResize( int width, int height )
{
	// Resize viewport
	h3dSetNodeParamI( cam, H3DCamera::ViewportXI, 0 );
	h3dSetNodeParamI( cam, H3DCamera::ViewportYI, 0 );
	h3dSetNodeParamI( cam, H3DCamera::ViewportWidthI, width );
	h3dSetNodeParamI( cam, H3DCamera::ViewportHeightI, height );
	
	// Set virtual camera parameters
	//h3dSetupCameraView( NodeHandle cameraNode, float fov, float aspect, float nearDist, float farDist )
	h3dSetupCameraView( cam, 80.0f, (float)width / height, 0.1f, 1230.0f );
	//h3dResizePipelineBuffers( _deferredPipeRes, width, height );
	h3dResizePipelineBuffers( _currentPipeRes, width, height );
}

static void GenericContactProcess (const NewtonJoint* contactJoint, dFloat timestep, int threadIndex)
{
	dFloat contactBestSpeed;
	SoundEffect* bestSound = NULL;
	dVector contactPosit;
	dFloat contactHighestForce = 0;

	bestSound = NULL;
	

	//void NewtonMaterialGetContactForce (const NewtonMaterial* const material, NewtonBody* const body, dFloat* const force);

	contactBestSpeed = 0.5f;
	NewtonBody* const body0 = NewtonJointGetBody0(contactJoint);
	for (void* contact = NewtonContactJointGetFirstContact (contactJoint); contact; contact = NewtonContactJointGetNextContact (contactJoint, contact)) {
		dFloat contactNormalSpeed;
		NewtonMaterial* material;

		// get the material for this contact;
		material = NewtonContactGetMaterial (contact);


		dFloat contactForce;
		NewtonMaterialGetContactForce(material,body0,&contactForce);

		if(contactForce >contactHighestForce)
			contactHighestForce = contactForce;

		contactNormalSpeed = NewtonMaterialGetContactNormalSpeed (material);
		if (contactNormalSpeed > contactBestSpeed){
			contactBestSpeed = contactNormalSpeed;
			dVector normal;
			contactBestSpeed = contactNormalSpeed;
			NewtonMaterialGetContactPositionAndNormal (material, body0, &contactPosit[0], &normal[0]);
				

			bestSound = (SoundEffect *)NewtonMaterialGetMaterialPairUserData (material);
		}
	}


	HordeEntity *ent0;
	ent0 = (HordeEntity*) NewtonBodyGetUserData(body0);

	NewtonBody* const body1 = NewtonJointGetBody1(contactJoint);
	HordeEntity *ent1;
	ent1 = (HordeEntity*) NewtonBodyGetUserData(body1);

	
	if (bestSound != NULL)// && contactHighestForce > 1.0f) 
	{
		
		
		bestSound->m_manager->playBump(contactPosit[0],contactPosit[1],contactPosit[2],contactBestSpeed);
		
		
		
	}
}
