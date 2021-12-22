#include "demoParticleSand2.h"

#include "sandRigidCommon.h"//����ͷ�ļ�û�ӽ�����cy:ͷ�ļ�������Ҫ��ͬһ���ļ�����ſ���
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
#include "Dynamics/RigidBody/Vehicle/HeightFieldTerrainRigidInteractionNode.h"//��ײ�ӵ�
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


//ˮ��ʦ�磺����õ������ӷ�����άSPH�����Ǹ߶ȳ���
//�����б�Ӳ����߶ȳ��ĳ��������񳤿�ɳ�ӵ���ظ߶ȣ���ɳ�߶ȣ�����λ�ã��������������̺������ļ���
using namespace std;

DemoParticleSandMultiRigid2* DemoParticleSandMultiRigid2::m_instance = 0;
void                        DemoParticleSandMultiRigid2::createScene(/*double *data,int sandinfo_length,int sandinfo_wide*/)
{
    SandGridInfo sandinfo;
    sandinfo.nx               = 64*4/*sandinfo_length*/;//��䳡����С�����Ըı����4�����ᵼ�³������б������������뵽���ܲ��ܰѳ���С�أ����ܣ�ɳ�Ӻܴ��������ɳ��Ҳ��С���ǣ����൱��û���ˣ�һ������
    sandinfo.ny               = 64*4/*sandinfo_wide*/;//�����Ǹ߶ȳ����ݸ�������
    sandinfo.griddl           = 0.03;//���Ӽ�࣬Ҳ��ɳ����С��
    sandinfo.mu               = 0.7;//�ĸĿ���,Ӧ����Ħ������������֮��
    sandinfo.drag             = 0.95;//�ĸĿ�����ק����û�ио�Ҳһ��
    sandinfo.slide            = 10 * sandinfo.griddl;//10�ĳ�100�������仯
    sandinfo.sandRho          = 1000.0;////�ĸĿ�����û����

    double sandParticleHeight = 0.05;//ע�⣺0.05��Ϊ��ظ߶ȣ���ɳ���Ӿ������ĸ߶ȣ�������������ɳ������Ĳ���

    SceneGraph& scene = SceneGraph::getInstance();// �������������ӣ�������һ������ͼ���󡣲���һЩ����
    scene.setUpperBound(Vector3f(10, 10, 10));//��Щ��ĸĿ�Ч������֪��������
    scene.setLowerBound(Vector3f(-10, -5, -10));

    // Root node. Also the simulator.1���ڵ�
    std::shared_ptr<ParticleSandRigidInteraction> root = scene.createNewScene<ParticleSandRigidInteraction>();//����ParticleSandRigidInteraction.h.cu//�����о�һ��root������
    root->setActive(true);//���Ҫ�Ǹĳ�false�ᵼ��С���޷����£����޷���ʼ//�������Ҳ������ParticleSandRigidInteraction����������ѽ����զ���¡���node�������
    root->setDt(0.02);//Ҳ��node�����
    auto interactionSolver = root->getInteractionSolver();//ParticleSandRigidInteraction�����
	//���ȥ������root���ڵ���.h��public������ж�Ӧ��������������ע�͵Ľ���//���ڳ���ɳ����ϲ��ֵĲ�������ϣ�������ϣ���ˮ�����ģ�
    root->varBouyancyFactor()->setValue(50);//�������أ��⼸���ʶ���ɶ��˼//�������Ҳ��ParticleSandRigidInteraction����������
    root->varDragFactor()->setValue(1.0);//�����һֱ���⣬��ק������ˮ������
    root->varCHorizontal()->setValue(1.);//ˮƽ����û����
    root->varCVertical()->setValue(2.);//��ֱ
    root->varCprobability()->setValue(100);//���ʣ������ԣ�
	//������5��������޸ı�����������ȷ����

    // Sand simulator.2����ɳ�غ�Ӳ�ط���׼�������������������ڵ�
	//���������࣬������Ҫ�ģ�����GPU���������ģ��ֱ���ɶ��
    std::shared_ptr<SandSimulator> sandSim     = std::make_shared<SandSimulator>();//��������SandSimulator.cu//���ڶ���ѧ���������sand����
    std::shared_ptr<PBDSandSolver> psandSolver = std::make_shared<PBDSandSolver>();//��������PBDSandSolver.cu//PBDɳ�������������ģ���������һ�ѽڵ�//������������ô�࣬��˵��������ܵ�����ô��API�����Ժ��ľ��Ǹ㶮����࣬����Щ��Ա�����ԣ���������
    psandSolver->setSandGridInfo(sandinfo);//�������setSandGridInfo������PBDSandSolver��Ľӿ��а���
    sandSim->needForward(false);//ȷʵ��SandSimulator���еĽӿ�
    sandSim->setSandSolver(psandSolver);//ͬ��
    root->setSandSolver(psandSolver);//����������û�������һ����Ķ���ȥָ������Լ��������β���������Ķ���
    root->addChild(sandSim);//����

    auto sandinitfun = [](PBDSandSolver* solver) { solver->freeFlow(1); };//�����������������ݣ�
    psandSolver->setPostInitializeFun(std::bind(sandinitfun, std::placeholders::_1));//���ȷʵ��PBDSandSolver���еĽӿ�
	//std::bind(function, std::placeholders::_1),�����sandinitfun��������setPostInitializeFun

    // landӲ����߶ȳ���ʼ�� 3Ӳ�������ò���������
	//HostHeightField1d�Ǹ߶ȳ�����HeightFieldGrid<double, double, DeviceType::CPU>�ı���
	//���￴���߶ȳ������õ���CPU
    HostHeightField1d landHeight;//landHeight�����漸�е�λ���ǵ�������ļ�
    landHeight.resize(sandinfo.nx, sandinfo.ny);//����Ӳ�����С
    landHeight.setSpace(sandinfo.griddl, sandinfo.griddl);//����Ӳ����߶ȳ����񳤿�
	//printf("wkm%d\n%d ", landHeight.Nx(), landHeight.Ny());//�������256 256 
	//png����߶ȳ�����
    HeightFieldLoader hfloader;//����HeightFieldLoader.h//������Ǹ߶ȳ�����
    double            maxh = 1;//�߶����ֵ
    hfloader.setRange(0, maxh);//���õ���߶ȳ���Χ
    hfloader.load(landHeight, "../../Media/HeightFieldImg/terrain_lying2.png");//���⵼������ļ�
	
    for (int i = 0; i < sandinfo.nx; ++i){//����߶����ã�����ѭ��
        for (int j = 0; j < sandinfo.ny; ++j){

            double curh = 0.5 * maxh - landHeight(i, j);//landHeight��ֵ�Ǹ��ݵ���png�ļ�����
            if (curh < 0.2) curh = 0.2;

            landHeight(i, j) = curh;//���������С0.2�����0.5
        }
    }

	////�����data����Ӳ����߶ȳ�����
	//for (int i = 0; i < sandinfo.nx; ++i) {//����߶����ã�����ѭ��
	//	for (int j = 0; j < sandinfo.ny; ++j) {
	//		landHeight(i, j) = *data;
	//		data++;
	//	}
	//}

	//printf("wkm%d\n%d ", landHeight.Nx(), landHeight.Ny());//��256
    psandSolver->setLand(landHeight);//�����ϣ�����ѭ�����ϵĵ���߶���Ϣ

    // Land mesh.4Ӳ�����������ɲ���Ⱦ
    {
        auto landrigid = std::make_shared<RigidBody2<DataType3f>>("Land");
        root->addChild(landrigid);//�����ϸ������

        // Mesh triangles.��������
        auto triset = std::make_shared<TriangleSet<DataType3f>>();
        landrigid->setTopologyModule(triset);//��������������ģ��

        // Generate mesh.����
        auto&           hfland = psandSolver->getLand();
        HeightFieldMesh hfmesh;
        hfmesh.generate(triset, hfland);

        // Mesh renderer.������Ⱦ��Ϳ��ɫ
        auto renderModule = std::make_shared<RigidMeshRender>(landrigid->getTransformationFrame());
        renderModule->setColor(Vector3f(210.0 / 255.0, 180.0 / 255.0, 140.0 / 255.0));
        landrigid->addVisualModule(renderModule);
    }

    // Sand height 5ɳ�Ӹ߶ȳ�����δʵ�����ã�
    //std::vector<int>  humpBlock = { 31 - 10, 31 + 9, 31 - 8, 31 + 8 };//�շ�飬4ά
    HostHeightField1d sandHeight;//ɳ�Ӹ߶�//HeightFieldGrid.h�߶ȳ�����
    sandHeight.resize(sandinfo.nx, sandinfo.ny);
    //sandHeight.setSpace(sandinfo.griddl, sandinfo.griddl);//���У�����ֵҲһ����κ������ע�͵���
    //fillGrid2D(sandHeight, 0.2);//�߶ȳ���ֵ,�����Ǹ߶ȣ��������еĻ�����Ҳһ����κ������ע�͵���
    //fillGrid2D(sandHeight, humpBlock, 0.4);//�߶ȳ��շ�츳ֵ������Ҳһ����κע�͵���
    psandSolver->setHeight(sandHeight);//����ɳ�����������棬����ע�͵�����

    // Sand particles.6ɳ�����ӳ�ʼ��
    std::vector<Vector3d>                 particlePos;//����λ��
    std::vector<ParticleType>             particleType;//ɳ�����ͣ������岻�������ɶ��˼
    std::vector<double>                   particleMass;//��������
	//������ɶ
    std::default_random_engine            e(31);//�����������,31�����������
    std::uniform_real_distribution<float> u(-0.3, 0.3);//���ȷֲ���ģ�壬u�Ǹ����ȷֲ�

    // Sand plane.7ɳ��ƽ��
    double spacing = sandinfo.griddl / 2.0;//��񳤶�
    double m0      = sandParticleHeight * sandinfo.sandRho * spacing * spacing;//50*0.015^2//�������ѭ���и�����ɳ������
    double mb      = m0 * 5;//�߽紦ɳ����������
    //ɳ��ƽ�棿�����ȥ���Ļ�����ɳ�ӾͶ�������
    double spacing2 = spacing;  // / 2.0;//��񳤶�

	//8��������ɳ����
    for (int i = 0; i < sandinfo.nx; ++i){
        for (int j = 0; j < sandinfo.ny; ++j){
			//���˫ѭ�����ɳ�Ӹ߶ȳ�ÿһ������
            if (landHeight(i, j) < 0.25)//ע�⣺Ӳ����߶�С��0.25�Ļ�����������˫ѭ�������򲻷�ɳ�ӡ�0.25��Ϊ��ɳ�߶�
            {
                Vector3d centerij = landHeight.gridCenterPosition(i, j);//�����Ȱ�Ӳ����������Ϊɳ������
                for (int ii = 0; ii < 2; ++ii)
                    for (int jj = 0; jj < 2; ++jj)
					//����ɳ��λ��˫ѭ�������˫ѭ����ʹ��ÿ�������з��������������������ĸ�ɳ����ÿ�����ӵ�λ����һ����Χ�ڵ�����ԡ�
					//����������ԵĻ�����һ�������η�������պ�ƽ�����ĸ����ӣ�����λ�ü�ɳ������λ�á�����ͼ��
						//////
						//**//
						//**//
						//////
                    {
                        Vector3d curp = centerij;//ɳ������
                        curp[0] -= sandinfo.griddl / 2.0;//������ȥ�����У�����ֻ���ٲ�����ɳ�ӡ�����һ��ɳ������
                        curp[2] -= sandinfo.griddl / 2.0;
                        curp[0] += ii * spacing2 + spacing2 / 2.0 * (1.0 + u(e));//u(e)����(-0.3, 0.3)�������
                        curp[2] += jj * spacing2 + spacing2 / 2.0 * (1.0 + u(e));
                        particlePos.push_back(curp);//push_back������vectorĩβ�ټ�һ��Ԫ�أ�������˫��ѭ���൱�ڸ�vector��ֵ
                        particleType.push_back(ParticleType::SAND);
                        particleMass.push_back(m0);//����vector������С���Ǹ߶ȳ���������
                    }
            }
        }
    }

    // Boundary particle.9�߽�����ɳ���ӣ�ûɶ�ã�ע�͵���
    //for (int i = -5; i < sandinfo.nx + 5; ++i)
    //{
    //    for (int j = -5; j < sandinfo.ny + 5; ++j)
    //    {
    //        if (i >= 0 && i < sandinfo.nx && j >= 0 && j < sandinfo.ny) continue;//ԭ������ֱ���������˿����ֻ����߽�
    //        Vector3d centerij = landHeight.gridCenterPosition(i, j);
    //        for (int ii = 0; ii < 2; ++ii)
    //            for (int jj = 0; jj < 2; ++jj)
    //            {
    //                Vector3d curp = centerij;
    //                curp[0] -= sandinfo.griddl / 2.0;
    //                curp[2] -= sandinfo.griddl / 2.0;
    //                curp[0] += ii * spacing + spacing / 2.0 * (1.0 + u(e));
    //                curp[2] += jj * spacing + spacing / 2.0 * (1.0 + u(e));
    //                particlePos.push_back(curp);
    //                particleType.push_back(ParticleType::BOUNDARY);
    //                particleMass.push_back(mb);
    //            }
    //    }
    //}

    std::vector<Vector3d> particleVel(particlePos.size());//ÿ�����񴦶�Ӧ���и������ٶȣ������Ǹ�vector���ٶ�����ά����
	//�������������Ҫ����������ȫ����Ϣ����0����ʼ��ַ������PBDSandSolver.cu
    psandSolver->setParticles(&particlePos[0], &particleVel[0], &particleMass[0], &particleType[0], particlePos.size(), sandinfo.sandRho, m0, sandinfo.griddl * 1.0, 0.85, sandParticleHeight * 0.5);

    //���������Ŀ��195064����ʵ�Ǹ߶ȳ�������Ŀ�����˱�0.25�ߵ�Ӳ���棩��ע��������ʾ����ʵ����һ������������һ������
    std::cout << "Particle number:   " << particlePos.size() << std::endl;

    // Rendering module of simulator.10.ɳ����Ⱦ
    auto pRenderModule = std::make_shared<PointRenderModule>();
    //pRenderModule->varRenderMode()->getValue().currentKey() = PointRenderModule::RenderModeEnum::SPRITE;
    pRenderModule->setSphereInstaceSize(sandinfo.griddl * 0.5);//��������ɣ�����ɳ�Ӵ�С
    pRenderModule->setColor(Vector3f(1.0f, 1.0f, 122.0f / 255.0f));//����ɳ����ɫ���ĸĿ���
    sandSim->addVisualModule(pRenderModule);//�����ǲ���ɳ�ӿ��ӻ��Ĵ��룿

    // topology����ѧ 11.ɳ������
    auto topology = std::make_shared<PointSet<DataType3f>>();
    sandSim->setTopologyModule(topology);//��������ģ�飬��ģ�鵽�׸�ɶ�õģ�
    topology->getPoints().resize(1);

    // Render point sampler (module). 12.ɳ����Ⱦ������������ǻ�����Ⱦ��֮ǰ�Ĳ�������һ����ϡ��Ȳɳ��㣬Ȼ�󻭣�
    auto psampler = std::make_shared<ParticleSandRenderSampler>();//ɳ������Ⱦ�����ڵ�
    psampler->Initialize(psandSolver);
    sandSim->addCustomModule(psampler);//����Զ���ģ�飬��ɶ�õģ�

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
    m_car->wheelRelPosition[2] = Vector3f(-0.3f, -0.2,  0.4f) * scale1d + wheelCenter[2] - chassisCenter;
    m_car->wheelRelPosition[3] = Vector3f(+0.3f, -0.2,  0.4f) * scale1d + wheelCenter[3] - chassisCenter;

    //m_car->wheelRelRotation[0] = Quaternion<float>(0, 0, 0, 1);  // (0, 0.5, 0, 0.5).normalize();
    //m_car->wheelRelRotation[1] = Quaternion<float>(0, 0, 0, 1);  //(0, 0.5, 0, 0.5).normalize();
    //m_car->wheelRelRotation[2] = Quaternion<float>(0, 0, 0, 1);  //(0, 0.5, 0, 0.5).normalize();
    //m_car->wheelRelRotation[3] = Quaternion<float>(0, 0, 0, 1);  //(0, 0.5, 0, 0.5).normalize();
	//û�е��̵�λ�ˣ��������������У������˵��̺����ӵ����λ�ã�
    m_car->wheelupDirection    = Vector3f(0, 0.5, 0);//������z�᷽���Ϻ͵��̵����λ�ã�ԭ����(0, 1, 0)
    m_car->wheelRightDirection = Vector3f(1, 0, 0);//ɶ��˼������ȥ��֮�󳵾Ͷ�������

    m_car->chassisMass    = 1500;  //���õ���������ȥ���Ļ���������ؾͷ���
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
    Vector3f wheelI        = RigidUtil::calculateCylinderLocalInertia(wheelm,//����Բ����ֲ����Բ���
                                                               (wheelSize[0][1] + wheelSize[0][2]) / 2.0,
                                                               wheelSize[0][0],
                                                               0);
    m_car->wheelMass[0]    = wheelm;//������50
    m_car->wheelInertia[0] = wheelI;
    m_car->wheelMass[1]    = wheelm;
    m_car->wheelInertia[1] = wheelI;
    m_car->wheelMass[2]    = wheelm;
    m_car->wheelInertia[2] = wheelI;
    m_car->wheelMass[3]    = wheelm;
    m_car->wheelInertia[3] = wheelI;

    m_car->steeringLowerBound = -0.5;//��ת���±߽磬ɶ��˼��������
    m_car->steeringUpperBound = 0.5;

    m_car->forwardForceAcc = 10000;//ǰ��ǣ����������
    m_car->maxVel        = 2.5;//����ٶ�
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
    m_car->build();//build��MultiWheelCar�����ĺ���������PBDCar�ĺ���ѽ//Ӧ����PBDCar�ĺ���
	m_car2->build();


	////-------------------------------------------------------
	////��ײ֮ǰ׼����Ŀǰ���ɴ�ģ
	//
	//printf("initialize OK\n");
	//m_groundRigidInteractor = std::make_shared<HeightFieldPBDInteractionNode>();
 //   //m_groundRigidInteractor->setRigidBodySystem(m_car->m_rigidSystem);
 //   //m_groundRigidInteractor->setSize(sandinfo.nx, sandinfo.ny, sandinfo.griddl, sandinfo.griddl);
 //   //m_groundRigidInteractor->getSolver()->m_numSubstep          = 5;
 //   //m_groundRigidInteractor->getSolver()->m_numContactSolveIter = 20;

 //   m_groundRigidInteractor->getSolver()->setUseGPU(false);
 //   root->addChild(m_groundRigidInteractor);//ע�͵�����ȫ��������ײ��
 //   m_groundRigidInteractor->setDt(0.016);


	////DeviceHeightField1d& terrain  = m_groundRigidInteractor->getHeightField();//����ע�͵�����
 ////   DeviceHeightField1d* terrain_ = &terrain;
 ////   Function1Pt::copy(*terrain_, landHeight);
 //   m_groundRigidInteractor->setDetectionMethod(HeightFieldPBDInteractionNode::HFDETECTION::POINTVISE);//ע�͵�����
 //   //m_groundRigidInteractor->setDetectionMethod(HeightFieldTerrainRigidInteractionNode::HFDETECTION::FACEVISE);
 //   //terrain.setOrigin(0, 0, 0);
	//
	////��������ײ֮ǰ׼��


	////���Լ���ײ
	//{//m_car����
 //       auto pointset = TypeInfo::cast<PointSet<DataType3f>>(m_car->getChassis()->getTopologyModule());//ԭ����getChassis()
	//	if (pointset)
	//	{
	//		std::shared_ptr<TOrientedBox3D<float>> pobb = std::make_shared<TOrientedBox3D<float>>();
	//		pobb->u = Vector3f(1, 0, 0);
	//		pobb->v = Vector3f(0, 1, 0);
	//		pobb->w = Vector3f(0, 0, 1);

	//		//DeviceArray<Vector3f>& vertices = pointset->getPoints();
	//		this->computeAABB(pointset, pobb->center, pobb->extent);

	//		if (m_groundRigidInteractor == NULL)
	//			m_groundRigidInteractor = std::make_shared<HeightFieldPBDInteractionNode>();//cy����������Ǹ���ָ�룬������2��
	//		auto pdetector = m_groundRigidInteractor->getRigidContactDetector();//��������ע�͵�����û�쳣�ˣ�����ģ
	//		printf("&&&&&&&&&&&&&&&&&&& call ADD HERE!\n");
	//		pdetector->addCollidableObject(m_car->m_chassis, pobb);//bug�����⣡pobbûpush��ȥ��

	//		//wkm���ǲ�����Ϊû�е���doCollision֮���
	//	}//�쳣��cy�޺�֮���Դ�ģ����Ϊ��©��Դ�����й���m_groundRigidInteractor��һϵ����䣬��������root��
	//	else
	//		printf("PointSet ERROR!\n");
	//}
	//
	////m_car����
	//for (int i = 0; i < 4; ++i){
 //       auto pwheel   = m_car->getWheels(i);
 //       auto pointset = TypeInfo::cast<PointSet<DataType3f>>(pwheel->getTopologyModule());
 //       if (pointset)
 //       {
 //           std::shared_ptr<TOrientedBox3D<float>> pobb = std::make_shared<TOrientedBox3D<float>>();
 //           //pobb->center = chaCenter;
 //           //pobb->extent = chaSize;
 //           pobb->u = Vector3f(1, 0, 0);
 //           pobb->v = Vector3f(0, 1, 0);
 //           pobb->w = Vector3f(0, 0, 1);

 //           //DeviceArray<Vector3f>& vertices = pointset->getPoints();
 //           this->computeAABB(pointset, pobb->center, pobb->extent);

	//		if (m_groundRigidInteractor == NULL)
	//			m_groundRigidInteractor = std::make_shared<HeightFieldPBDInteractionNode>();//cy

 //           auto pdetector = m_groundRigidInteractor->getRigidContactDetector();
 //           pdetector->addCollidableObject(pwheel, pobb);
 //       }
	//}
	//m_groundRigidInteractor->addChild(m_car);//���������У�֮ǰ©��

	//{//m_car2����
 //       auto pointset = TypeInfo::cast<PointSet<DataType3f>>(m_car2->getChassis()->getTopologyModule());//ԭ����getChassis()
 //       if (pointset)
 //       {
 //           std::shared_ptr<TOrientedBox3D<float>> pobb = std::make_shared<TOrientedBox3D<float>>();
 //           pobb->u = Vector3f(1, 0, 0);
 //           pobb->v = Vector3f(0, 1, 0);
 //           pobb->w = Vector3f(0, 0, 1);

 //           //DeviceArray<Vector3f>& vertices = pointset->getPoints();
 //           this->computeAABB(pointset, pobb->center, pobb->extent);

	//		if (m_groundRigidInteractor == NULL)
	//			m_groundRigidInteractor = std::make_shared<HeightFieldPBDInteractionNode>();//cy
 //           auto pdetector = m_groundRigidInteractor->getRigidContactDetector();//��������ע�͵�����û�쳣�ˣ�����ģ
 //           pdetector->addCollidableObject(m_car2->m_chassis, pobb);//�����������������push_back
 //       }
 //   }

	////m_car2����
	//for (int i = 0; i < 4; ++i)
 //   {
 //       auto pwheel   = m_car2->getWheels(i);
 //       auto pointset = TypeInfo::cast<PointSet<DataType3f>>(pwheel->getTopologyModule());
 //       if (pointset)
 //       {
 //           std::shared_ptr<TOrientedBox3D<float>> pobb = std::make_shared<TOrientedBox3D<float>>();
 //           //pobb->center = chaCenter;
 //           //pobb->extent = chaSize;
 //           pobb->u = Vector3f(1, 0, 0);
 //           pobb->v = Vector3f(0, 1, 0);
 //           pobb->w = Vector3f(0, 0, 1);

 //           //DeviceArray<Vector3f>& vertices = pointset->getPoints();
 //           this->computeAABB(pointset, pobb->center, pobb->extent);

	//		if (m_groundRigidInteractor == NULL)
	//			m_groundRigidInteractor = std::make_shared<HeightFieldPBDInteractionNode>();//cy

 //           auto pdetector = m_groundRigidInteractor->getRigidContactDetector();
 //           pdetector->addCollidableObject(pwheel, pobb);
 //       }
 //   }
	//
	//m_groundRigidInteractor->addChild(m_car2);
	////�����Ǽ���ײ--------------------------------------------------------------------------------------------------




    // Add visualization module and topology module.��ӿ��ӻ�ģ�������ģ�顣���ǵ��̣���������//ע���������Ҫ������ײģ����棬Ҫ��Ȼ�ᷢ��vectorԽ��
    m_car->m_chassis->setTopologyModule(chassisTri);//����ģ�顣ɶ�ã�ɶ��˼��������
    auto chassisRender = std::make_shared<RigidMeshRender>(m_car->m_chassis->getTransformationFrame());
    chassisRender->setColor(Vector3f(0.8, std::rand() % 1000 / ( double )1000, 0.8));
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
        renderModule->setColor(Vector3f(0.8, std::rand() % 1000 / ( double )1000, 0.8));
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

    // Translate camera position   ���λ�ã�û��Ҳ�У��������λ�ÿ�����������
    auto& camera_ = this->activeCamera();
    Vector3f camPos(0, 1.5, 5);
    camera_.lookAt(camPos, Vector3f(0, 0, 0), Vector3f(0, 1, 0));
}
