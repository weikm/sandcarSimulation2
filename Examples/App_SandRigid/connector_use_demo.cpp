#include "demoParticleSand2.h"

#include "sandRigidCommon.h"//这俩头文件没加进来？cy:头文件物理上要在同一个文件夹里，才可以
#include "Dynamics/Sand/SandSimulator.h"//
#include "Framework/Framework/SceneGraph.h"
#include "Rendering/PointRenderModule.h"
#include "Dynamics/Sand/PBDSandSolver.h"//
#include "Dynamics/Sand/PBDSandRigidInteraction.h"//
#include "Dynamics/RigidBody/PBDRigid/PBDSolverNode.h"
#include "Rendering/RigidMeshRender.h"
#include "Dynamics/RigidBody/RigidUtil.h"
#include "Dynamics/Sand/ParticleSandRigidInteraction.h"//
#include "Dynamics/Sand/HeightFieldSandRigidInteraction.h"
#include "Dynamics/RigidBody/Vehicle/HeightFieldTerrainRigidInteractionNode.h"//碰撞加的
#include "Dynamics/HeightField/HeightFieldMesh.h"
#include "IO/Surface_Mesh_IO/ObjFileLoader.h"
#include "Dynamics/RigidBody/PBDRigid/HeightFieldPBDInteractionNode.h"

#include "Dynamics/Sand/SandVisualPointSampleModule.h"
#include "IO/Image_IO/HeightFieldLoader.h"

#include "Core/Utility/Function1Pt.h"

#include <random>
#include <iostream>

//----------------------
#include "Dynamics/ParticleSystem/StaticBoundary.h"
#include "Dynamics/RigidBody/FreeJoint.h"
//#include "Rendering/RigidMeshRender.h"
#include "Dynamics/RigidBody/RigidUtil.h"
#include "Dynamics/RigidBody/FixedJoint.h"
#include "Framework/Action/Action.h"
#include <string>
#include <functional>
#include <cmath>

#include"connector_use_demo.h"
#include "connector.h"

using namespace std;

connector_use_demo* connector_use_demo::m_instance = 0;
void connector_use_demo::createScene(const VPE::SandSimulationRegionCreateInfo& info)//可能是这个问题：set和add把参数导进去了，但是create没有用之前的region对象。
{
	m_region = VPE::SandSimulationRegion::Create(info);
	if (info.cars.size() > 0) {
		m_car = m_region->GetCar(0);//这里获取到车的智能指针
	}
}


