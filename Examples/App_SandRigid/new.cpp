#include "new.h"

#include "sandRigidCommon.h"
#include "Dynamics/Sand/SandSimulator.h"
#include "Framework/Framework/SceneGraph.h"
#include "Rendering/PointRenderModule.h"
#include "Dynamics/Sand/PBDSandSolver.h"
#include "Dynamics/Sand/PBDSandRigidInteraction.h"
#include "Dynamics/RigidBody/PBDRigid/PBDSolverNode.h"
#include "Rendering/RigidMeshRender.h"
#include "Dynamics/RigidBody/RigidUtil.h"
#include "Dynamics/Sand/ParticleSandRigidInteraction.h"
#include "Dynamics/Sand/HeightFieldSandRigidInteraction.h"
#include "Dynamics/HeightField/HeightFieldMesh.h"
#include "IO/Surface_Mesh_IO/ObjFileLoader.h"

#include "Dynamics/Sand/SandVisualPointSampleModule.h"

#include "IO/Image_IO/HeightFieldLoader.h"
#include "Dynamics/Sand/SSESandSolver.h"
#include "Dynamics/Sand/SandSimulator.h"

#include "sandRigidCommon.h"
#include <random>
//有这么多createScene()函数，每个都可以连在demo上来，看看输出咋样

new1* new1::m_instance = 0;
void                             new1::createScene()
{
    SandGridInfo sandinfo;
    sandinfo.nx               = 64*2 /**1.5*/;
    sandinfo.ny               = 64*2;
    sandinfo.griddl           = 0.04;
    sandinfo.mu               = 0.7;
    sandinfo.drag             = 0.95;
    sandinfo.slide            = 10 * sandinfo.griddl;
    sandinfo.sandRho          = 1000.0;
    double sandParticleHeight = 0.1;

    SceneGraph& scene = SceneGraph::getInstance();
    scene.setUpperBound(Vector3f(10, 10, 10));
    scene.setLowerBound(Vector3f(-10, -5, -10));

    // Root node. Also the simulator.
    std::shared_ptr<HeightFieldSandRigidInteraction> root = scene.createNewScene<HeightFieldSandRigidInteraction>();
    root->setActive(true);
    root->setDt(0.02);
    root->m_subStep        = 1;
    auto interactionSolver = root->getInteractionSolver();

    root->varCHorizontal()->setValue(0.5);
    root->varCVertical()->setValue(1.3);
    root->varBouyancyFactor()->setValue(300);
    root->varDragFactor()->setValue(1.0);

    // Sand simulator.
    std::shared_ptr<SandSimulator> sandSim     = std::make_shared<SandSimulator>();
    std::shared_ptr<SSESandSolver> psandSolver = std::make_shared<SSESandSolver>();
    sandSim->needForward(false);
    sandSim->setSandSolver(psandSolver);
    root->setSandSolver(psandSolver);
    root->addChild(sandSim);

    // Initialize sand grid data.
    SandGrid& sandGrid = psandSolver->getSandGrid();  //sandSim->getSandGrid();
    sandGrid.setSandInfo(sandinfo);
    root->setSandGrid(sandGrid.m_sandHeight, sandGrid.m_landHeight);

    // Height
    std::vector<float> landHeight(sandinfo.nx * sandinfo.ny);
    std::vector<float> surfaceHeight(sandinfo.nx * sandinfo.ny);
    std::vector<int>   humpBlock = { 0, 20, 5, 25 };
    fillGrid2D(&(landHeight[0]), sandinfo.nx, sandinfo.ny, 0.0f);
    fillGrid2D(&(surfaceHeight[0]), sandinfo.nx, sandinfo.ny, 0.2f);
    //fillGrid2D(&(surfaceHeight[0]), sandinfo.nx, sandinfo.ny, humpBlock, 0.5f);
    sandGrid.initialize(&(landHeight[0]), &(surfaceHeight[0]));

    // Rendering module of simulator.
    auto pRenderModule = std::make_shared<PointRenderModule>();
    //pRenderModule->varRenderMode()->getValue().currentKey() = PointRenderModule::RenderModeEnum::SPRITE;
    pRenderModule->setSphereInstaceSize(sandinfo.griddl * 0.3);
    //pRenderModule->setColor(Vector3f(1.0f, 1.0f, 122.0f / 255.0f));
    pRenderModule->setColor(Vector3f(1.0f * 0.9, 0.9f * 0.9, 122.0f / 255.0f * 0.9));
    //pRenderModule->setColor(Vector3f(254.0f/255.f, 204.0f/255.f, 153.0f / 255.0f));
    //pRenderModule->setColor(Vector3f(211.0f/255.f, 198.0f/255.f, 166.0f / 255.0f));

    sandSim->addVisualModule(pRenderModule);

    // topology
    auto topology = std::make_shared<PointSet<DataType3f>>();
    sandSim->setTopologyModule(topology);
    topology->getPoints().resize(1);

    // Render point sampler (module).
    auto psampler = std::make_shared<SandHeightRenderParticleSampler>();
    sandSim->addCustomModule(psampler);
    psampler->m_sandHeight = &sandGrid.m_sandHeight;
    psampler->m_landHeight = &sandGrid.m_landHeight;
    psampler->Initalize(sandinfo.nx, sandinfo.ny, 3, 2, sandinfo.griddl);

	/// ------  Rigid ------------
	 //13PBD刚体模拟节点
	std::shared_ptr<PBDSolverNode> rigidSim = std::make_shared<PBDSolverNode>();//PBD刚体模拟节点
	rigidSim->getSolver()->setUseGPU(true);
	rigidSim->needForward(false);
	auto rigidSolver = rigidSim->getSolver();

	root->setRigidSolver(rigidSolver);
	root->addChild(rigidSim);
	//下面是车-----------------------------------------------------------------------------------------------------------
	// Car.14小车内部数据类型
	double   scale1d = 1.;
	Vector3d scale3d(scale1d, scale1d, scale1d);//没用到
	Vector3f scale3f(scale1d, scale1d, scale1d);//(1,1,1)//用在网格里

	Vector3f chassisCenter;//三维数组，初始都是000
	Vector3f wheelCenter[4];
	Vector3f chassisSize;
	Vector3f wheelSize[4];

	std::shared_ptr<TriangleSet<DataType3f>> chassisTri;//底盘和轮子都设置为三角网格
	std::shared_ptr<TriangleSet<DataType3f>> wheelTri[4];

	DistanceField3D<DataType3f> chassisSDF;
	DistanceField3D<DataType3f> wheelSDF[4];

	// Load car mesh.15载入底盘和轮子的网格和SDF
	{
		//Vector3f boundingsize;//这个是死代码吧，后面再也没有了，魏注释掉了

		// Chassis mesh.底盘网格和包围盒
		ObjFileLoader chassisLoader("../../Media/car2/chassis_cube.obj");//载入底盘文件

		chassisTri = std::make_shared<TriangleSet<DataType3f>>();//底盘上设置三角网格
		chassisTri->setPoints(chassisLoader.getVertexList());
		chassisTri->setTriangles(chassisLoader.getFaceList());
		computeBoundingBox(chassisCenter, chassisSize, chassisLoader.getVertexList());//计算包围盒，点进去就是算法//问一下啥是计算包围盒？？？
		//std::cout<<chassisCenter[0]<<' '<<chassisCenter[1]<<' '<<chassisCenter[2]<<std::endl;//这两行是常悦帮忙写的，嘱咐我要多练多谢多改代码！
		//chassisCenter *= scale3f;//这里可以实验一下啊，输出一下，初值时是多少，乘完是多少
		//std::cout<<chassisCenter[0]<<' '<<chassisCenter[1]<<' '<<chassisCenter[2]<<std::endl;//与我所想完全一致，这个三维数组未赋初值，开始是0，乘完还是0
		//chassisSize *= scale3f;
		chassisTri->scale(scale3f);//这俩行不懂，scale函数和translate函数是啥意思？？？
		chassisTri->translate(-chassisCenter);//大概是网格节点都需要带上这俩，那具体是啥意思？？

		// Chassis sdf.
		chassisSDF.loadSDF("../../Media/car2/chassis_cube.sdf");//载入底盘SDF文件
		chassisSDF.scale(scale1d);//这俩行不懂，scale函数和translate函数是啥意思？
		chassisSDF.translate(-chassisCenter);//所以SDF节点也要带上这俩？？？
		//interactionSolver->addSDF(sdf);

		for (int i = 0; i < 4; ++i)//四个轮子
		{
			string objfile("../../Media/car2/wheel.obj");
			string sdffile("../../Media/car2/wheel.sdf");

			// Wheel mesh.轮子设置上网格
			ObjFileLoader wheelLoader(objfile);
			wheelTri[i] = std::make_shared<TriangleSet<DataType3f>>();
			wheelTri[i]->setPoints(wheelLoader.getVertexList());//这两行是啥？
			wheelTri[i]->setTriangles(wheelLoader.getFaceList());
			computeBoundingBox(wheelCenter[i], wheelSize[i], wheelLoader.getVertexList());//计算包围盒？？？
			//wheelCenter[i] *= scale3f;//乘完还是000
			//wheelSize[i] *= scale3f;//乘完还是000
			wheelTri[i]->scale(scale3f);
			wheelTri[i]->translate(-wheelCenter[i]);

			// Wheel sdf.
			DistanceField3D<DataType3f>& sdf = wheelSDF[i];
			sdf.loadSDF(sdffile);
			sdf.scale(scale1d);
			sdf.translate(-wheelCenter[i]);
			//interactionSolver->addSDF(sdf);
		}
	}

	//16车的系列参数设置,m_car登场
	m_car = std::make_shared<PBDCar>();//点进去能看见PBDCar这个类，包含下面一系列成员和函数
	rigidSim->addChild(m_car);
	m_car->m_rigidSolver = rigidSolver;
	//---------------------------------------------------------------------------------------
	m_car2 = std::make_shared<PBDCar>();//点进去能看见PBDCar这个类，包含下面一系列成员和函数
	rigidSim->addChild(m_car2);
	m_car2->m_rigidSolver = rigidSolver;

	m_car->carPosition = Vector3f(0.35, 0.7, 1.5) + chassisCenter;//设置车子初始位置
	m_car2->carPosition = Vector3f(0.35, 0.7, 0.1) + chassisCenter;
	//std::printf("wkm%f %f %f %f", m_car->wheelRelRotation[0], m_car->wheelRelRotation[1], m_car->wheelRelRotation[2], m_car->wheelRelRotation[3]);//输出0000.rotation变量根本在这个demo里面没参与运算。
	//四个轮子的相对位置和相对旋转，要粘在接口函数GetWheelPositionRotation里
	m_car->wheelRelPosition[0] = Vector3f(-0.3f, -0.2, -0.4f) * scale1d + wheelCenter[0] - chassisCenter;
	m_car->wheelRelPosition[1] = Vector3f(+0.3f, -0.2, -0.4f) * scale1d + wheelCenter[1] - chassisCenter;
	m_car->wheelRelPosition[2] = Vector3f(-0.3f, -0.2, 0.4f) * scale1d + wheelCenter[2] - chassisCenter;
	m_car->wheelRelPosition[3] = Vector3f(+0.3f, -0.2, 0.4f) * scale1d + wheelCenter[3] - chassisCenter;

	//m_car->wheelRelRotation[0] = Quaternion<float>(0, 0, 0, 1);  // (0, 0.5, 0, 0.5).normalize();
	//m_car->wheelRelRotation[1] = Quaternion<float>(0, 0, 0, 1);  //(0, 0.5, 0, 0.5).normalize();
	//m_car->wheelRelRotation[2] = Quaternion<float>(0, 0, 0, 1);  //(0, 0.5, 0, 0.5).normalize();
	//m_car->wheelRelRotation[3] = Quaternion<float>(0, 0, 0, 1);  //(0, 0.5, 0, 0.5).normalize();
	//没有底盘的位姿，但是下面这两行，体现了底盘和轮子的相对位置！
	m_car->wheelupDirection = Vector3f(0, 0.5, 0);//轮子在z轴方向上和底盘的相对位置，原来是(0, 1, 0)
	m_car->wheelRightDirection = Vector3f(1, 0, 0);//啥意思，反正去掉之后车就动不了了

	m_car->chassisMass = 1500;  //设置底盘质量，去掉的话，车刚落地就飞了
	m_car->chassisInertia = RigidUtil::calculateCubeLocalInertia(m_car->chassisMass, chassisSize);//计算底盘惯性并设置
	//m_car->chassisFile = "../../Media/standard/standard_cube.obj";//这部分是cy加的
	//m_car->wheelFile[0] = "../../Media/Cylinder/cylinder2.obj";
	//m_car->wheelFile[1] = "../../Media/Cylinder/cylinder2.obj";
	//m_car->wheelFile[2] = "../../Media/Cylinder/cylinder2.obj";
	//m_car->wheelFile[3] = "../../Media/Cylinder/cylinder2.obj";


	m_car2->wheelRelPosition[0] = Vector3f(-0.3f, -0.2, -0.4f /* -0.01*/) * scale1d + wheelCenter[0] - chassisCenter;
	m_car2->wheelRelPosition[1] = Vector3f(+0.3f /*+0.01*/, -0.2, -0.4f /* +0.02*/) * scale1d + wheelCenter[1] - chassisCenter;
	m_car2->wheelRelPosition[2] = Vector3f(-0.3f, -0.2, 0.4f) * scale1d + wheelCenter[2] - chassisCenter;
	m_car2->wheelRelPosition[3] = Vector3f(+0.3f, -0.2, 0.4f) * scale1d + wheelCenter[3] - chassisCenter;

	//m_car2->wheelRelRotation[0] = Quaternion<float>(0, 0, 0, 1);  // (0, 0.5, 0, 0.5).normalize();
	//m_car2->wheelRelRotation[1] = Quaternion<float>(0, 0, 0, 1);  //(0, 0.5, 0, 0.5).normalize();
	//m_car2->wheelRelRotation[2] = Quaternion<float>(0, 0, 0, 1);  //(0, 0.5, 0, 0.5).normalize();
	//m_car2->wheelRelRotation[3] = Quaternion<float>(0, 0, 0, 1);  //(0, 0.5, 0, 0.5).normalize();
	//没有底盘的位姿，但是下面这两行，体现了底盘和轮子的相对位置！
	m_car2->wheelupDirection = Vector3f(0, 1, 0);//轮子在z轴方向上和底盘的相对位置
	m_car2->wheelRightDirection = Vector3f(1, 0, 0);//啥意思，反正去掉之后车就动不了了

	m_car2->chassisMass = 1500;  //设置底盘质量
	m_car2->chassisInertia = RigidUtil::calculateCubeLocalInertia(m_car->chassisMass, chassisSize);//计算底盘惯性并设置
	//m_car2->chassisFile = "../../Media/standard/standard_cube.obj";//这部分是cy加的,要做碰撞，需要补全democar2里面参数
	//m_car2->wheelFile[0] = "../../Media/Cylinder/cylinder2.obj";
	//m_car2->wheelFile[1] = "../../Media/Cylinder/cylinder2.obj";
	//m_car2->wheelFile[2] = "../../Media/Cylinder/cylinder2.obj";
	//m_car2->wheelFile[3] = "../../Media/Cylinder/cylinder2.obj";
	//
	//m_car->chassisMeshScale = Vector3f(0.3, 0.2, 0.5) * 0.5;//这部分是cy加的
	//m_car->wheelMeshScale[0] = Vector3f(0.002, 0.01, 0.01) * 0.5;
	//m_car->wheelMeshScale[1] = Vector3f(0.002, 0.01, 0.01) * 0.5;
	//m_car->wheelMeshScale[2] = Vector3f(0.002, 0.01, 0.01) * 0.5;
	//m_car->wheelMeshScale[3] = Vector3f(0.002, 0.01, 0.01) * 0.5;
	//m_car->chassisMeshTranslate = Vector3f(0, 0, 0);
	//m_car->wheelMeshTranslate[0] = Vector3f(0, 0, 0);  // 0.075);
	//m_car->wheelMeshTranslate[1] = Vector3f(0, 0, 0);  // 0.075);
	//m_car->wheelMeshTranslate[2] = Vector3f(0, 0, 0);  // 0.075);
	//m_car->wheelMeshTranslate[3] = Vector3f(0, 0, 0);  // 0.075);

	//m_car2->chassisMeshScale = Vector3f(0.3, 0.2, 0.5) * 0.5;//这部分是cy加的
	//m_car2->wheelMeshScale[0] = Vector3f(0.002, 0.01, 0.01) * 0.5;
	//m_car2->wheelMeshScale[1] = Vector3f(0.002, 0.01, 0.01) * 0.5;
	//m_car2->wheelMeshScale[2] = Vector3f(0.002, 0.01, 0.01) * 0.5;
	//m_car2->wheelMeshScale[3] = Vector3f(0.002, 0.01, 0.01) * 0.5;
	//m_car2->chassisMeshTranslate = Vector3f(0, 0, 0);
	//m_car2->wheelMeshTranslate[0] = Vector3f(0, 0, 0);  // 0.075);
	//m_car2->wheelMeshTranslate[1] = Vector3f(0, 0, 0);  // 0.075);
	//m_car2->wheelMeshTranslate[2] = Vector3f(0, 0, 0);  // 0.075);
	//m_car2->wheelMeshTranslate[3] = Vector3f(0, 0, 0);  // 0.075);

	//wheelm和wheelI代表啥？质量和惯性
	float wheelm = 30;//单个轮子质量，原本是50
	//float wheelRad = wheelTri[0][1]
	Vector3f wheelI = RigidUtil::calculateCylinderLocalInertia(wheelm,//计算圆柱体局部惯性参数
		(wheelSize[0][1] + wheelSize[0][2]) / 2.0,
		wheelSize[0][0],
		0);
	m_car->wheelMass[0] = wheelm;//轮质量50
	m_car->wheelInertia[0] = wheelI;
	m_car->wheelMass[1] = wheelm;
	m_car->wheelInertia[1] = wheelI;
	m_car->wheelMass[2] = wheelm;
	m_car->wheelInertia[2] = wheelI;
	m_car->wheelMass[3] = wheelm;
	m_car->wheelInertia[3] = wheelI;

	m_car->steeringLowerBound = -0.5;//旋转角下边界，啥意思？？？？
	m_car->steeringUpperBound = 0.5;

	m_car->forwardForceAcc = 10000;//前向牵引力增加量
	m_car->maxVel = 2.5;//最大速度
	//-----------------------------------------
	m_car2->wheelMass[0] = wheelm;//轮质量50
	m_car2->wheelInertia[0] = wheelI;
	m_car2->wheelMass[1] = wheelm;
	m_car2->wheelInertia[1] = wheelI;
	m_car2->wheelMass[2] = wheelm;
	m_car2->wheelInertia[2] = wheelI;
	m_car2->wheelMass[3] = wheelm;
	m_car2->wheelInertia[3] = wheelI;

	m_car2->steeringLowerBound = -0.5;//旋转角下边界，啥意思？？？？
	m_car2->steeringUpperBound = 0.5;

	m_car2->forwardForceAcc = 10000;//前向牵引力增加量
	m_car2->maxVel = 2.5;//最大速度


	// Build.组装
	m_car->build();//build是MultiWheelCar这个类的函数，不是PBDCar的函数呀//应该是PBDCar的函数ya
	m_car2->build();//删了多轮车，还是连不上PBDCar





	// Add visualization module and topology module.添加可视化模块和拓扑模块。先是底盘，再是轮子//注意这个部分要放在碰撞模块后面，要不然会发生vector越界
	m_car->m_chassis->setTopologyModule(chassisTri);//拓扑模块。啥用？啥意思？？？？
	auto chassisRender = std::make_shared<RigidMeshRender>(m_car->m_chassis->getTransformationFrame());
	chassisRender->setColor(Vector3f(0.8, std::rand() % 1000 / (double)1000, 0.8));
	m_car->m_chassis->addVisualModule(chassisRender);
	interactionSolver->addSDF(chassisSDF, m_car->m_chassis->getId());
	//-----------------------------------------------------------------------------------------
	m_car2->m_chassis->setTopologyModule(chassisTri);//拓扑模块。啥用？啥意思？？？？
	auto chassisRender2 = std::make_shared<RigidMeshRender>(m_car2->m_chassis->getTransformationFrame());//这里加了2
	//chassisRender->setColor(Vector3f(0.8, std::rand() % 1000 / (double)1000, 0.8));
	m_car2->m_chassis->addVisualModule(chassisRender2);
	interactionSolver->addSDF(chassisSDF, m_car2->m_chassis->getId());

	// Bounding radius of chassis.底盘的边界半径？啥意思？？点进去这个函数看看
	float chassisRadius = chassisTri->computeBoundingRadius();
	m_car->m_chassis->setRadius(chassisRadius);

	//float chassisRadius = chassisTri->computeBoundingRadius();
	m_car2->m_chassis->setRadius(chassisRadius);

	m_rigids.push_back(m_car->m_chassis);//这俩行 啥意思
	//m_rigidRenders.push_back(chassisRender);

	m_rigids.push_back(m_car2->m_chassis);//这俩行 啥意思
	m_rigidRenders.push_back(chassisRender);


	for (int i = 0; i < 4; ++i)//这循环，给轮子，先添加可视化模块和拓扑模块，再设置底盘的边界半径
	{
		m_car->m_wheels[i]->setTopologyModule(wheelTri[i]);
		auto renderModule = std::make_shared<RigidMeshRender>(m_car->m_wheels[i]->getTransformationFrame());
		renderModule->setColor(Vector3f(0.8, std::rand() % 1000 / (double)1000, 0.8));
		m_car->m_wheels[i]->addVisualModule(renderModule);
		interactionSolver->addSDF(wheelSDF[i], m_car->m_wheels[i]->getId());

		// Bounding radius of chassis.
		float wheelRadius = wheelTri[i]->computeBoundingRadius();
		m_car->m_wheels[i]->setRadius(wheelRadius);

		m_rigids.push_back(m_car->m_wheels[i]);
		m_rigidRenders.push_back(renderModule);
	}
	//------------------------------------------------------------------------------------------------------------
	for (int i = 0; i < 4; ++i)//这循环，给轮子，先添加可视化模块和拓扑模块，再设置底盘的边界半径
	{
		m_car2->m_wheels[i]->setTopologyModule(wheelTri[i]);
		auto renderModule = std::make_shared<RigidMeshRender>(m_car2->m_wheels[i]->getTransformationFrame());
		renderModule->setColor(Vector3f(0.8, std::rand() % 1000 / (double)1000, 0.8));
		m_car2->m_wheels[i]->addVisualModule(renderModule);
		interactionSolver->addSDF(wheelSDF[i], m_car2->m_wheels[i]->getId());

		// Bounding radius of chassis.
		float wheelRadius = wheelTri[i]->computeBoundingRadius();
		m_car2->m_wheels[i]->setRadius(wheelRadius);

		m_rigids.push_back(m_car2->m_wheels[i]);
		m_rigidRenders.push_back(renderModule);
	}


	interactionSolver->m_prigids = &(rigidSolver->getRigidBodys());
    // Translate camera position
    auto& camera_ = this->activeCamera();
    //camera_.translate(Vector3f(0, 1.5, 3));
    //camera_.setEyePostion(Vector3f(1.5, 1.5, 6));
    Vector3f camPos(0, 3, 3.5);
    camera_.lookAt(camPos, Vector3f(0, 0, 0.), Vector3f(0, 1, 0));

    //this->disableDisplayFrameRate();
    this->enableDisplayFrameRate();
    //this->enableDisplayFrame();
}

