
#ifndef _factory_H_
#define _factory_H_

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
#include "PlayerEntity.h"
#include "NPCEntity.h"

#include "PhysicsManager.h"
#include "SoundManager.h"



using namespace std;

class Factory
{
	public:
	Factory();
	~Factory();


	enum MaterialCallbackType
	{		
		standardCallback,
		characterCallback
	};
	
	int GAMESTATE, NEXTGAMESTATE;

	bool init(std::string contentDir);
	void release();
	void reset();
	bool running;	
	

	void update(float timestep);
	void render();
	void camResize( int width, int height );





	int createCamera(std::string pipelineFileName);
	HordeEntity* createEntity(std::string fileName,float tx,float ty,float tz,float rx,float ry,float rz,float sx = 1.0f,float sy = 1.0f,float sz = 1.0f,
		bool enablePhysics = false, bool staticBody = false, float mass = 1.0f, int materialID = 0, int type = 0, bool pickable = false, bool planetMode = false);
	PlayerEntity* createPlayer(float mass, int materialID = 0, bool planetMode = false);

	NPCEntity* createNPC(std::string fileName,float tx,float ty,float tz,float rx,float ry,float rz,float sx = 1.0f,float sy = 1.0f,float sz = 1.0f,
		 float mass = 1.0f, int materialID = 0, int type = 0, bool planetMode = false);

	HordeEntity* createLight(std::string fileName, std::string name, float tx,float ty,float tz,float rx,float ry,float rz,float sx = 1.0f,float sy = 1.0f,float sz = 1.0f,
		float red = 1.0f, float green = 1.0f, float blue = 1.0f, float radius = 30.0f,float fov = 90.f, float bias = 0.0009f, int shadowMaps = 1 );

	int createPhysicsMaterial();
	//todo, create enum for callback
	void setPhysicsMaterialPair(int matA, int matB, int soundEffect, MaterialCallbackType callbackType, float elasticCoef = 0.1f, float staticFric = 0.5f, float kineticFriction = 0.45f);

		


	//player and control functions
	PlayerEntity *getPlayer(){return player; };
	void pickingCast();
	void pickingRelease();
	void setMoveVector(Horde3D::Vec3f moveVector, bool run);
	void setMouseXY( float dX, float dY )
	{
		_rx = dX;
		_ry = dY;
	};






	int          _statMode;
	int          _freezeMode;
	bool         _debugViewMode, _wireframeMode;

	bool		_camlook;

	float        _x, _y, _z;  // Viewer pos
	float        _velocity;  // Velocity for movement

	float getLoadScreenAlpha(){return loadScreenAlpha;};
	void setLoadScreenAlpha(float alpha){ loadScreenAlpha = alpha; };

private:

	float	loadScreenAlpha;

	PhysicsManager* m_physicsManager;
	SoundManager* m_soundManager;


	vector<HordeEntity*> entities;
	


	PlayerEntity *player;
	HordeEntity *currentPickedEnt;
	float pickRayCurrentLength;





	H3DNode cam;
	float       _rx, _ry;	
	float        _curFPS;
	// Engine objects
	H3DRes       _fontMatRes, _panelMatRes;
	H3DRes       _logoMatRes, _forwardPipeRes, _deferredPipeRes;
	H3DRes	_currentPipeRes;
	//H3DNode      _cam;
	std::string  _contentDir;

	Horde3D::Quaternion camRotTarget;
	Horde3D::Quaternion camRotCurrent;
	Horde3D::Matrix4f camMatrix;







};


#endif // _factory_H_



