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
//����ô��createScene()������ÿ������������demo�������������զ��

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
	 //13PBD����ģ��ڵ�
	std::shared_ptr<PBDSolverNode> rigidSim = std::make_shared<PBDSolverNode>();//PBD����ģ��ڵ�
	rigidSim->getSolver()->setUseGPU(true);
	rigidSim->needForward(false);
	auto rigidSolver = rigidSim->getSolver();

	root->setRigidSolver(rigidSolver);
	root->addChild(rigidSim);
	//�����ǳ�-----------------------------------------------------------------------------------------------------------
	// Car.14С���ڲ���������
	double   scale1d = 1.;
	Vector3d scale3d(scale1d, scale1d, scale1d);//û�õ�
	Vector3f scale3f(scale1d, scale1d, scale1d);//(1,1,1)//����������

	Vector3f chassisCenter;//��ά���飬��ʼ����000
	Vector3f wheelCenter[4];
	Vector3f chassisSize;
	Vector3f wheelSize[4];

	std::shared_ptr<TriangleSet<DataType3f>> chassisTri;//���̺����Ӷ�����Ϊ��������
	std::shared_ptr<TriangleSet<DataType3f>> wheelTri[4];

	DistanceField3D<DataType3f> chassisSDF;
	DistanceField3D<DataType3f> wheelSDF[4];

	// Load car mesh.15������̺����ӵ������SDF
	{
		//Vector3f boundingsize;//�����������ɣ�������Ҳû���ˣ�κע�͵���

		// Chassis mesh.��������Ͱ�Χ��
		ObjFileLoader chassisLoader("../../Media/car2/chassis_cube.obj");//��������ļ�

		chassisTri = std::make_shared<TriangleSet<DataType3f>>();//������������������
		chassisTri->setPoints(chassisLoader.getVertexList());
		chassisTri->setTriangles(chassisLoader.getFaceList());
		computeBoundingBox(chassisCenter, chassisSize, chassisLoader.getVertexList());//�����Χ�У����ȥ�����㷨//��һ��ɶ�Ǽ����Χ�У�����
		//std::cout<<chassisCenter[0]<<' '<<chassisCenter[1]<<' '<<chassisCenter[2]<<std::endl;//�������ǳ��ð�æд�ģ�������Ҫ������л��Ĵ��룡
		//chassisCenter *= scale3f;//�������ʵ��һ�°������һ�£���ֵʱ�Ƕ��٣������Ƕ���
		//std::cout<<chassisCenter[0]<<' '<<chassisCenter[1]<<' '<<chassisCenter[2]<<std::endl;//����������ȫһ�£������ά����δ����ֵ����ʼ��0�����껹��0
		//chassisSize *= scale3f;
		chassisTri->scale(scale3f);//�����в�����scale������translate������ɶ��˼������
		chassisTri->translate(-chassisCenter);//���������ڵ㶼��Ҫ�����������Ǿ�����ɶ��˼����

		// Chassis sdf.
		chassisSDF.loadSDF("../../Media/car2/chassis_cube.sdf");//�������SDF�ļ�
		chassisSDF.scale(scale1d);//�����в�����scale������translate������ɶ��˼��
		chassisSDF.translate(-chassisCenter);//����SDF�ڵ�ҲҪ��������������
		//interactionSolver->addSDF(sdf);

		for (int i = 0; i < 4; ++i)//�ĸ�����
		{
			string objfile("../../Media/car2/wheel.obj");
			string sdffile("../../Media/car2/wheel.sdf");

			// Wheel mesh.��������������
			ObjFileLoader wheelLoader(objfile);
			wheelTri[i] = std::make_shared<TriangleSet<DataType3f>>();
			wheelTri[i]->setPoints(wheelLoader.getVertexList());//��������ɶ��
			wheelTri[i]->setTriangles(wheelLoader.getFaceList());
			computeBoundingBox(wheelCenter[i], wheelSize[i], wheelLoader.getVertexList());//�����Χ�У�����
			//wheelCenter[i] *= scale3f;//���껹��000
			//wheelSize[i] *= scale3f;//���껹��000
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

	//16����ϵ�в�������,m_car�ǳ�
	m_car = std::make_shared<PBDCar>();//���ȥ�ܿ���PBDCar����࣬��������һϵ�г�Ա�ͺ���
	rigidSim->addChild(m_car);
	m_car->m_rigidSolver = rigidSolver;
	//---------------------------------------------------------------------------------------
	m_car2 = std::make_shared<PBDCar>();//���ȥ�ܿ���PBDCar����࣬��������һϵ�г�Ա�ͺ���
	rigidSim->addChild(m_car2);
	m_car2->m_rigidSolver = rigidSolver;

	m_car->carPosition = Vector3f(0.35, 0.7, 1.5) + chassisCenter;//���ó��ӳ�ʼλ��
	m_car2->carPosition = Vector3f(0.35, 0.7, 0.1) + chassisCenter;
	//std::printf("wkm%f %f %f %f", m_car->wheelRelRotation[0], m_car->wheelRelRotation[1], m_car->wheelRelRotation[2], m_car->wheelRelRotation[3]);//���0000.rotation�������������demo����û�������㡣
	//�ĸ����ӵ����λ�ú������ת��Ҫճ�ڽӿں���GetWheelPositionRotation��
	m_car->wheelRelPosition[0] = Vector3f(-0.3f, -0.2, -0.4f) * scale1d + wheelCenter[0] - chassisCenter;
	m_car->wheelRelPosition[1] = Vector3f(+0.3f, -0.2, -0.4f) * scale1d + wheelCenter[1] - chassisCenter;
	m_car->wheelRelPosition[2] = Vector3f(-0.3f, -0.2, 0.4f) * scale1d + wheelCenter[2] - chassisCenter;
	m_car->wheelRelPosition[3] = Vector3f(+0.3f, -0.2, 0.4f) * scale1d + wheelCenter[3] - chassisCenter;

	//m_car->wheelRelRotation[0] = Quaternion<float>(0, 0, 0, 1);  // (0, 0.5, 0, 0.5).normalize();
	//m_car->wheelRelRotation[1] = Quaternion<float>(0, 0, 0, 1);  //(0, 0.5, 0, 0.5).normalize();
	//m_car->wheelRelRotation[2] = Quaternion<float>(0, 0, 0, 1);  //(0, 0.5, 0, 0.5).normalize();
	//m_car->wheelRelRotation[3] = Quaternion<float>(0, 0, 0, 1);  //(0, 0.5, 0, 0.5).normalize();
	//û�е��̵�λ�ˣ��������������У������˵��̺����ӵ����λ�ã�
	m_car->wheelupDirection = Vector3f(0, 0.5, 0);//������z�᷽���Ϻ͵��̵����λ�ã�ԭ����(0, 1, 0)
	m_car->wheelRightDirection = Vector3f(1, 0, 0);//ɶ��˼������ȥ��֮�󳵾Ͷ�������

	m_car->chassisMass = 1500;  //���õ���������ȥ���Ļ���������ؾͷ���
	m_car->chassisInertia = RigidUtil::calculateCubeLocalInertia(m_car->chassisMass, chassisSize);//������̹��Բ�����
	//m_car->chassisFile = "../../Media/standard/standard_cube.obj";//�ⲿ����cy�ӵ�
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
	//û�е��̵�λ�ˣ��������������У������˵��̺����ӵ����λ�ã�
	m_car2->wheelupDirection = Vector3f(0, 1, 0);//������z�᷽���Ϻ͵��̵����λ��
	m_car2->wheelRightDirection = Vector3f(1, 0, 0);//ɶ��˼������ȥ��֮�󳵾Ͷ�������

	m_car2->chassisMass = 1500;  //���õ�������
	m_car2->chassisInertia = RigidUtil::calculateCubeLocalInertia(m_car->chassisMass, chassisSize);//������̹��Բ�����
	//m_car2->chassisFile = "../../Media/standard/standard_cube.obj";//�ⲿ����cy�ӵ�,Ҫ����ײ����Ҫ��ȫdemocar2�������
	//m_car2->wheelFile[0] = "../../Media/Cylinder/cylinder2.obj";
	//m_car2->wheelFile[1] = "../../Media/Cylinder/cylinder2.obj";
	//m_car2->wheelFile[2] = "../../Media/Cylinder/cylinder2.obj";
	//m_car2->wheelFile[3] = "../../Media/Cylinder/cylinder2.obj";
	//
	//m_car->chassisMeshScale = Vector3f(0.3, 0.2, 0.5) * 0.5;//�ⲿ����cy�ӵ�
	//m_car->wheelMeshScale[0] = Vector3f(0.002, 0.01, 0.01) * 0.5;
	//m_car->wheelMeshScale[1] = Vector3f(0.002, 0.01, 0.01) * 0.5;
	//m_car->wheelMeshScale[2] = Vector3f(0.002, 0.01, 0.01) * 0.5;
	//m_car->wheelMeshScale[3] = Vector3f(0.002, 0.01, 0.01) * 0.5;
	//m_car->chassisMeshTranslate = Vector3f(0, 0, 0);
	//m_car->wheelMeshTranslate[0] = Vector3f(0, 0, 0);  // 0.075);
	//m_car->wheelMeshTranslate[1] = Vector3f(0, 0, 0);  // 0.075);
	//m_car->wheelMeshTranslate[2] = Vector3f(0, 0, 0);  // 0.075);
	//m_car->wheelMeshTranslate[3] = Vector3f(0, 0, 0);  // 0.075);

	//m_car2->chassisMeshScale = Vector3f(0.3, 0.2, 0.5) * 0.5;//�ⲿ����cy�ӵ�
	//m_car2->wheelMeshScale[0] = Vector3f(0.002, 0.01, 0.01) * 0.5;
	//m_car2->wheelMeshScale[1] = Vector3f(0.002, 0.01, 0.01) * 0.5;
	//m_car2->wheelMeshScale[2] = Vector3f(0.002, 0.01, 0.01) * 0.5;
	//m_car2->wheelMeshScale[3] = Vector3f(0.002, 0.01, 0.01) * 0.5;
	//m_car2->chassisMeshTranslate = Vector3f(0, 0, 0);
	//m_car2->wheelMeshTranslate[0] = Vector3f(0, 0, 0);  // 0.075);
	//m_car2->wheelMeshTranslate[1] = Vector3f(0, 0, 0);  // 0.075);
	//m_car2->wheelMeshTranslate[2] = Vector3f(0, 0, 0);  // 0.075);
	//m_car2->wheelMeshTranslate[3] = Vector3f(0, 0, 0);  // 0.075);

	//wheelm��wheelI����ɶ�������͹���
	float wheelm = 30;//��������������ԭ����50
	//float wheelRad = wheelTri[0][1]
	Vector3f wheelI = RigidUtil::calculateCylinderLocalInertia(wheelm,//����Բ����ֲ����Բ���
		(wheelSize[0][1] + wheelSize[0][2]) / 2.0,
		wheelSize[0][0],
		0);
	m_car->wheelMass[0] = wheelm;//������50
	m_car->wheelInertia[0] = wheelI;
	m_car->wheelMass[1] = wheelm;
	m_car->wheelInertia[1] = wheelI;
	m_car->wheelMass[2] = wheelm;
	m_car->wheelInertia[2] = wheelI;
	m_car->wheelMass[3] = wheelm;
	m_car->wheelInertia[3] = wheelI;

	m_car->steeringLowerBound = -0.5;//��ת���±߽磬ɶ��˼��������
	m_car->steeringUpperBound = 0.5;

	m_car->forwardForceAcc = 10000;//ǰ��ǣ����������
	m_car->maxVel = 2.5;//����ٶ�
	//-----------------------------------------
	m_car2->wheelMass[0] = wheelm;//������50
	m_car2->wheelInertia[0] = wheelI;
	m_car2->wheelMass[1] = wheelm;
	m_car2->wheelInertia[1] = wheelI;
	m_car2->wheelMass[2] = wheelm;
	m_car2->wheelInertia[2] = wheelI;
	m_car2->wheelMass[3] = wheelm;
	m_car2->wheelInertia[3] = wheelI;

	m_car2->steeringLowerBound = -0.5;//��ת���±߽磬ɶ��˼��������
	m_car2->steeringUpperBound = 0.5;

	m_car2->forwardForceAcc = 10000;//ǰ��ǣ����������
	m_car2->maxVel = 2.5;//����ٶ�


	// Build.��װ
	m_car->build();//build��MultiWheelCar�����ĺ���������PBDCar�ĺ���ѽ//Ӧ����PBDCar�ĺ���ya
	m_car2->build();//ɾ�˶��ֳ�������������PBDCar





	// Add visualization module and topology module.��ӿ��ӻ�ģ�������ģ�顣���ǵ��̣���������//ע���������Ҫ������ײģ����棬Ҫ��Ȼ�ᷢ��vectorԽ��
	m_car->m_chassis->setTopologyModule(chassisTri);//����ģ�顣ɶ�ã�ɶ��˼��������
	auto chassisRender = std::make_shared<RigidMeshRender>(m_car->m_chassis->getTransformationFrame());
	chassisRender->setColor(Vector3f(0.8, std::rand() % 1000 / (double)1000, 0.8));
	m_car->m_chassis->addVisualModule(chassisRender);
	interactionSolver->addSDF(chassisSDF, m_car->m_chassis->getId());
	//-----------------------------------------------------------------------------------------
	m_car2->m_chassis->setTopologyModule(chassisTri);//����ģ�顣ɶ�ã�ɶ��˼��������
	auto chassisRender2 = std::make_shared<RigidMeshRender>(m_car2->m_chassis->getTransformationFrame());//�������2
	//chassisRender->setColor(Vector3f(0.8, std::rand() % 1000 / (double)1000, 0.8));
	m_car2->m_chassis->addVisualModule(chassisRender2);
	interactionSolver->addSDF(chassisSDF, m_car2->m_chassis->getId());

	// Bounding radius of chassis.���̵ı߽�뾶��ɶ��˼�������ȥ�����������
	float chassisRadius = chassisTri->computeBoundingRadius();
	m_car->m_chassis->setRadius(chassisRadius);

	//float chassisRadius = chassisTri->computeBoundingRadius();
	m_car2->m_chassis->setRadius(chassisRadius);

	m_rigids.push_back(m_car->m_chassis);//������ ɶ��˼
	//m_rigidRenders.push_back(chassisRender);

	m_rigids.push_back(m_car2->m_chassis);//������ ɶ��˼
	m_rigidRenders.push_back(chassisRender);


	for (int i = 0; i < 4; ++i)//��ѭ���������ӣ�����ӿ��ӻ�ģ�������ģ�飬�����õ��̵ı߽�뾶
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
	for (int i = 0; i < 4; ++i)//��ѭ���������ӣ�����ӿ��ӻ�ģ�������ģ�飬�����õ��̵ı߽�뾶
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

