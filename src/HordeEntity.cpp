

#include "HordeEntity.h"




HordeEntity::HordeEntity(int hordeID, std::string contentDir) :
	m_matrix (GetIdentityMatrix()),
	m_curPosition (0.0f, 0.0f, 0.0f, 1.0f),
	m_prevPosition (0.0f, 0.0f, 0.0f, 1.0f),
	m_curRotation (1.0f, 0.0f, 0.0f, 0.0f),
	m_prevRotation (1.0f, 0.0f, 0.0f, 0.0f)
{
	animationSet = false;
	m_contentDir = contentDir;
	
	planetMode = false;

	matrixUpdated = false;
	m_body = NULL;
	m_joint = NULL;
	m_hordeID = hordeID;
	is_player = false;
	is_npc = false;

	physicsEnabled = false;

	tmatrix = NULL;

	pickingPosition = dVector(0,0,0);

	float tx,ty,tz,rx,ry,rz,sx,sy,sz;
	//void h3dGetNodeTransform( H3DNode node, float *tx, float *ty, float *tz,
    //                          float *rx, float *ry, float *rz, float *sx, float *sy, float *sz );
	h3dGetNodeTransform(m_hordeID,&tx,&ty,&tz,&rx,&ry,&rz,&sx,&sy,&sz);

	m_scale = dVector(sx,sy,sz,1);


	pickable = false;

	/*
	const float* transMatrix = 0x0;
	h3dGetNodeTransMats(m_hordeID,NULL,&transMatrix);
	//m_matrix = dMatrix(&transMatrix[0]);
	m_matrix.m_front = dVector(transMatrix[0],transMatrix[4],transMatrix[8],transMatrix[12]);
	m_matrix.m_up = dVector(transMatrix[1],transMatrix[5],transMatrix[9],transMatrix[13]);
	m_matrix.m_right = dVector(transMatrix[2],transMatrix[6],transMatrix[10],transMatrix[14]);
	m_matrix.m_posit = dVector(transMatrix[3],transMatrix[7],transMatrix[11],transMatrix[15]);
	*/


	NumVertices = 0;
	NumTriangleIndices = 0;

	VertexBase = NULL;

	VertRStart = 0;

	TriangleBase32 = 0;
	TriangleBase16 = 0;

	TriangleMode = 0;


	
	tmatrix = getTransformationMat();


	const float matS[] = {m_scale.m_x, 0, 0, 0,
							0, m_scale.m_y, 0, 0,
							0, 0, m_scale.m_z, 0,
							0, 0, 0, 1};	
	Horde3D::Matrix4f matScale(matS);
	scaleMatrix = Horde3D::Matrix4f(matS);
	//tmatrix = &(Horde3D::Matrix4f(this->getTransformationMat()) * scaleMatrix).c[0][0];

	extractMeshData();

}

HordeEntity::~HordeEntity()
{
	//if(m_body)
	//	delete m_body;
	m_body = NULL;
	//if(m_joint)
	//	delete m_joint;
	m_joint = NULL;
	delete VertexBase;
	delete TriangleBase32;
	delete TriangleBase16;

	//delete tmatrix;

	//printf("des\n");

}

void HordeEntity::setBodyPosition(float tx, float ty, float tz)
{
	dMatrix bodyMatrix;
	NewtonBodyGetMatrix(m_body, &bodyMatrix[0][0] );
	bodyMatrix[3][0] = tx;
	bodyMatrix[3][1] = ty;
	bodyMatrix[3][2] = tz;

	NewtonBodySetMatrix(m_body,&bodyMatrix[0][0]);
	
	
}

Horde3D::Vec3f HordeEntity::getBodyVelocity()
{
	dVector vel;
	NewtonBodyGetVelocity(m_body,&vel[0]);

	return Horde3D::Vec3f(vel.m_x,vel.m_y,vel.m_z);

}

void HordeEntity::setBodyVelocity(float x, float y, float z)
{
	dVector vel(x,y,z);
	NewtonBodySetVelocity(m_body,&vel[0]);
}

void HordeEntity::releaseNode()
{
	
	h3dRemoveNode(m_hordeID);

}

void HordeEntity::updateAnimation(float elapsed)
{
	animTime += elapsed * 100.f;
	h3dSetModelAnimParams( m_hordeID, 0, animTime, 1.0f );
	
	
		//h3dSetModelMorpher( H3DNode modelNode, const char *target, float weight )
	
		// h3dUpdateModel( H3DNode modelNode, int flags )
	h3dUpdateModel(m_hordeID,1);
}

void HordeEntity::update(float timestep)
{


	



	if(physicsEnabled)
		updatePosOri();

	if(animationSet)
	{
		updateAnimation(timestep);
	}
	
}

void HordeEntity::updatePosOri()
{
	if(!matrixUpdated)
		return;



	//h3dSetNodeTransform( m_hordeID, m_curPosition.m_x, m_curPosition.m_y, m_curPosition.m_z,  Horde3D::radToDeg(m_curRotation.GetXYZ_EulerAngles().m_x), Horde3D::radToDeg(m_curRotation.GetXYZ_EulerAngles().m_y), Horde3D::radToDeg(m_curRotation.GetXYZ_EulerAngles().m_z),	  m_scale.m_x, m_scale.m_y, m_scale.m_z );
	//h3dSetNodeTransform( m_hordeID, m_curPosition.m_x, m_curPosition.m_y, m_curPosition.m_z,  -Horde3D::radToDeg(m_curRotation.GetXYZ_EulerAngles().m_x), -Horde3D::radToDeg(m_curRotation.GetXYZ_EulerAngles().m_y), -Horde3D::radToDeg(m_curRotation.GetXYZ_EulerAngles().m_z),	  m_scale.m_x, m_scale.m_y, m_scale.m_z );




	

	Horde3D::Matrix4f matTR(tmatrix);
	
	//Horde3D::Matrix4f preveMat(getTransformationMat() );
	//combine for transformation matRotB matTR
	//Horde3D::Matrix4f finalTransMat =  matTransl * matRot * matScale;
	Horde3D::Matrix4f finalTransMat =  matTR * scaleMatrix;

	// finalTransMat =  matTR;

	h3dSetNodeTransMat( m_hordeID, &finalTransMat.c[0][0] );



	
}

void HordeEntity::setTransformationMat(const float* mat)
{
	h3dSetNodeTransMat(m_hordeID,mat);

}

const float* HordeEntity::getTransformationMat()
{
	const float* transMatrix = 0x0;
	h3dGetNodeTransMats(m_hordeID,&transMatrix,NULL);
	//h3dGetNodeTransMats(m_hordeID,NULL,&transMatrix);


	return transMatrix;


}

void HordeEntity::GetNodeTransform( float *tx, float *ty, float *tz,
                              float *rx, float *ry, float *rz, float *sx, float *sy, float *sz )
{
	h3dGetNodeTransform(m_hordeID,tx,ty,tz,rx,ry,rz,sx,sy,sz);


}

Horde3D::Vec3f  HordeEntity::getPosition()
{
	float tx,ty,tz;
	h3dGetNodeTransform(m_hordeID,&tx,&ty,&tz, NULL, NULL, NULL, NULL, NULL, NULL);
	return Horde3D::Vec3f(tx,ty,tz);

}

std::vector<float>  HordeEntity::getMeshVertices()
{
	std::vector<float> vertices;

	if(	VertexBase && (TriangleBase32 || TriangleBase16))
	{

		int offset = 3;
		if (TriangleMode == 5) // Triangle Strip //TODO find out whats up with this below
			offset = 1;

		//btAlignedObjectArray<btVector3> cvertices;

		// copy mesh from graphics to physics
		bool index16 = false;
		if (TriangleBase16)
			index16 = true;


	
		printf("horde-triangles: %i \n", NumTriangleIndices );
	
		for (unsigned int i = 0; i < NumTriangleIndices - 2; i+=offset)
		{
			unsigned int index1 = index16 ? (TriangleBase16[i]   - VertRStart) * 3 : (TriangleBase32[i]   - VertRStart) * 3;
			unsigned int index2 = index16 ? (TriangleBase16[i+1] - VertRStart) * 3 : (TriangleBase32[i+1] - VertRStart) * 3;
			unsigned int index3 = index16 ? (TriangleBase16[i+2] - VertRStart) * 3 : (TriangleBase32[i+2] - VertRStart) * 3;
			
			vertices.push_back(VertexBase[index1] );
			vertices.push_back(VertexBase[index1+1] );
			vertices.push_back(VertexBase[index1+2] );

			vertices.push_back(VertexBase[index2] );
			vertices.push_back(VertexBase[index2+1] );
			vertices.push_back(VertexBase[index2+2] );

			vertices.push_back(VertexBase[index3] );
			vertices.push_back(VertexBase[index3+1] );
			vertices.push_back(VertexBase[index3+2] );

			
		
		}			

	}

	return vertices;
}


void HordeEntity::extractMeshData()
{
	

	if (m_hordeID > 0)
	{
		H3DRes geoResource = 0;
		int vertexOffset = 0;
		int indexOffset = 0;
		switch(h3dGetNodeType(m_hordeID))
		{
		case H3DNodeTypes::Mesh:
			geoResource = h3dGetNodeParamI(h3dGetNodeParent(m_hordeID), H3DModel::GeoResI);
			NumVertices = h3dGetNodeParamI(m_hordeID, H3DMesh::VertREndI) - h3dGetNodeParamI(m_hordeID, H3DMesh::VertRStartI) + 1;
			NumTriangleIndices = h3dGetNodeParamI(m_hordeID, H3DMesh::BatchCountI);		
			VertRStart = h3dGetNodeParamI(m_hordeID, H3DMesh::VertRStartI);
			vertexOffset = h3dGetNodeParamI(m_hordeID, H3DMesh::VertRStartI) * 3;
			indexOffset = h3dGetNodeParamI(m_hordeID, H3DMesh::BatchStartI);
			break;
		case H3DNodeTypes::Model:
			geoResource = h3dGetNodeParamI(m_hordeID, H3DModel::GeoResI);
			NumVertices = h3dGetResParamI(geoResource, H3DGeoRes::GeometryElem, 0, H3DGeoRes::GeoVertexCountI);
			NumTriangleIndices = h3dGetResParamI(geoResource, H3DGeoRes::GeometryElem, 0, H3DGeoRes::GeoIndexCountI);		
			break;
			/*
		case H3DEXT_NodeType_Terrain:
			unloadTerrainGeoRes();
			geoResource = h3dextCreateTerrainGeoRes( 
				m_hordeID, 
				h3dGetNodeParamStr( m_hordeID, H3DNodeParams::NameStr ), 
				h3dGetNodeParamF( m_hordeID, H3DEXTTerrain::MeshQualityF, 0) );		
			NumVertices = h3dGetResParamI(geoResource, H3DGeoRes::GeometryElem, 0, H3DGeoRes::GeoVertexCountI);
			NumTriangleIndices = h3dGetResParamI(geoResource, H3DGeoRes::GeometryElem, 0, H3DGeoRes::GeoIndexCountI);		
			m_terrainGeoRes = geoResource;
			break;*/
		}
		float* vb = (float*)h3dMapResStream(geoResource, H3DGeoRes::GeometryElem, 0, H3DGeoRes::GeoVertPosStream, true, false);
		VertexBase = new float[(NumVertices*3)];
		memcpy(VertexBase, vb+vertexOffset, sizeof(float)*NumVertices*3);
		h3dUnmapResStream(geoResource);

		//Triangle indices, must cope with 16 bit and 32 bit
		if (h3dGetResParamI(geoResource, H3DGeoRes::GeometryElem, 0, H3DGeoRes::GeoIndices16I)) {
			unsigned short* tb = (unsigned short*)h3dMapResStream(geoResource, H3DGeoRes::GeometryElem, 0, H3DGeoRes::GeoIndexStream, true, false);
			TriangleBase16 = new unsigned short[NumTriangleIndices];
			memcpy(TriangleBase16, tb+indexOffset, sizeof(unsigned short)*NumTriangleIndices);
			h3dUnmapResStream(geoResource);
		} else {
			unsigned int* tb = (unsigned int*)h3dMapResStream(geoResource, H3DGeoRes::GeometryElem, 0, H3DGeoRes::GeoIndexStream, true, false);
			TriangleBase32 = new unsigned int[NumTriangleIndices];
			memcpy(TriangleBase32, tb+indexOffset, sizeof(unsigned int)*NumTriangleIndices);
			h3dUnmapResStream(geoResource);
		}
		//data->TriangleMode = Horde3D::getResourceParami(geoResource, GeometryResParams::TriangleMode);
	}
	

	printf("tri:%i\n", NumTriangleIndices );
	
}
