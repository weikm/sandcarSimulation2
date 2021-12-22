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


//水生师哥：这个用的是粒子法，二维SPH！不是高度场！
//参数列表：硬地面高度场的长宽，单网格长宽，沙子的离地高度，放沙高度，车子位置，车子质量，底盘和轮子文件，
using namespace std;

DemoParticleSandMultiRigid2* DemoParticleSandMultiRigid2::m_instance = 0;
void                        DemoParticleSandMultiRigid2::createScene(/*double *data,int sandinfo_length,int sandinfo_wide*/)
{
    SandGridInfo sandinfo;
    sandinfo.nx               = 64*4/*sandinfo_length*/;//想变场景大小，可以改变这个4，变大会导致程序运行变慢（但我又想到，能不能把车变小呢？不能，沙子很大。那如果把沙子也变小，那，就相当于没改了，一样慢。
    sandinfo.ny               = 64*4/*sandinfo_wide*/;//这俩是高度场横纵格子数，
    sandinfo.griddl           = 0.03;//格子间距，也是沙粒大小！
    sandinfo.mu               = 0.7;//改改看看,应该是摩擦，或者阻尼之类
    sandinfo.drag             = 0.95;//改改看看拖拽力，没有感觉也一样
    sandinfo.slide            = 10 * sandinfo.griddl;//10改成100看不出变化
    sandinfo.sandRho          = 1000.0;////改改看看，没明白

    double sandParticleHeight = 0.05;//注意：0.05称为离地高度，是沙粒子距离地面的高度！不是设置设置沙子区域的参数

    SceneGraph& scene = SceneGraph::getInstance();// 生成这个类的例子，并赋给一个场景图对象。并做一些设置
    scene.setUpperBound(Vector3f(10, 10, 10));//这些多改改看效果，就知道含义了
    scene.setLowerBound(Vector3f(-10, -5, -10));

    // Root node. Also the simulator.1根节点
    std::shared_ptr<ParticleSandRigidInteraction> root = scene.createNewScene<ParticleSandRigidInteraction>();//来自ParticleSandRigidInteraction.h.cu//后续研究一下root的类型
    root->setActive(true);//这个要是改成false会导致小车无法落下，即无法开始//这个函数也并不是ParticleSandRigidInteraction这个类里面的呀？！咋回事。是node类里面的
    root->setDt(0.02);//也是node类里的
    auto interactionSolver = root->getInteractionSolver();//ParticleSandRigidInteraction类里的
	//点进去，对象root所在的类.h的public里面就有对应出处，包含类似注释的解释//属于车和沙子耦合部分的参数，耦合，刚体耦合，看水生论文！
    root->varBouyancyFactor()->setValue(50);//浮力因素，这几个词都是啥意思//这个函数也是ParticleSandRigidInteraction这个类里面的
    root->varDragFactor()->setValue(1.0);//这个词一直不解，拖拽力，见水生论文
    root->varCHorizontal()->setValue(1.);//水平，，没明白
    root->varCVertical()->setValue(2.);//垂直
    root->varCprobability()->setValue(100);//概率，可能性？
	//上面这5大参数，修改编译运行以明确含义

    // Sand simulator.2设置沙地和硬地仿真准备，声明对象，连至根节点
	//下面这俩类，是最重要的，都是GPU。区别在哪？分别负责啥？
    std::shared_ptr<SandSimulator> sandSim     = std::make_shared<SandSimulator>();//这下面是SandSimulator.cu//都在动力学引擎下面的sand下面
    std::shared_ptr<PBDSandSolver> psandSolver = std::make_shared<PBDSandSolver>();//这下面是PBDSandSolver.cu//PBD沙粒解算器，核心，后面连了一堆节点//这个结点连了那么多，就说明这个类能调用这么多API，所以核心就是搞懂这个类，有哪些成员和特性！！！！！
    psandSolver->setSandGridInfo(sandinfo);//可是这个setSandGridInfo并不在PBDSandSolver类的接口中啊？
    sandSim->needForward(false);//确实是SandSimulator类中的接口
    sandSim->setSandSolver(psandSolver);//同上
    root->setSandSolver(psandSolver);//！！！发现没，这个是一个类的对象去指针调用自己函数，形参是其他类的对象。
    root->addChild(sandSim);//连上

    auto sandinitfun = [](PBDSandSolver* solver) { solver->freeFlow(1); };//这个函数里面很有内容！
    psandSolver->setPostInitializeFun(std::bind(sandinitfun, std::placeholders::_1));//这个确实是PBDSandSolver类中的接口
	//std::bind(function, std::placeholders::_1),代表把sandinitfun函数绑定在setPostInitializeFun

    // land硬地面高度场初始化 3硬地面设置参数并导入
	//HostHeightField1d是高度场网格HeightFieldGrid<double, double, DeviceType::CPU>的别名
	//这里看出高度场网格用的是CPU
    HostHeightField1d landHeight;//landHeight，下面几行的位置是导入地面文件
    landHeight.resize(sandinfo.nx, sandinfo.ny);//设置硬地面大小
    landHeight.setSpace(sandinfo.griddl, sandinfo.griddl);//设置硬地面高度场网格长宽
	//printf("wkm%d\n%d ", landHeight.Nx(), landHeight.Ny());//这里就是256 256 
	//png导入高度场数据
    HeightFieldLoader hfloader;//来自HeightFieldLoader.h//这个类是高度场导入
    double            maxh = 1;//高度最大值
    hfloader.setRange(0, maxh);//设置地面高度场范围
    hfloader.load(landHeight, "../../Media/HeightFieldImg/terrain_lying2.png");//从这导入地面文件
	
    for (int i = 0; i < sandinfo.nx; ++i){//地面高度设置，两层循环
        for (int j = 0; j < sandinfo.ny; ++j){

            double curh = 0.5 * maxh - landHeight(i, j);//landHeight初值是根据地面png文件来的
            if (curh < 0.2) curh = 0.2;

            landHeight(i, j) = curh;//所以这个最小0.2，最大0.5
        }
    }

	////下面从data导入硬地面高度场数据
	//for (int i = 0; i < sandinfo.nx; ++i) {//地面高度设置，两层循环
	//	for (int j = 0; j < sandinfo.ny; ++j) {
	//		landHeight(i, j) = *data;
	//		data++;
	//	}
	//}

	//printf("wkm%d\n%d ", landHeight.Nx(), landHeight.Ny());//是256
    psandSolver->setLand(landHeight);//挂载上，上面循环赋上的地面高度信息

    // Land mesh.4硬地面网格生成并渲染
    {
        auto landrigid = std::make_shared<RigidBody2<DataType3f>>("Land");
        root->addChild(landrigid);//挂载上刚体地面

        // Mesh triangles.三角网格
        auto triset = std::make_shared<TriangleSet<DataType3f>>();
        landrigid->setTopologyModule(triset);//刚体地面挂上拓扑模块

        // Generate mesh.生成
        auto&           hfland = psandSolver->getLand();
        HeightFieldMesh hfmesh;
        hfmesh.generate(triset, hfland);

        // Mesh renderer.网格渲染，涂颜色
        auto renderModule = std::make_shared<RigidMeshRender>(landrigid->getTransformationFrame());
        renderModule->setColor(Vector3f(210.0 / 255.0, 180.0 / 255.0, 140.0 / 255.0));
        landrigid->addVisualModule(renderModule);
    }

    // Sand height 5沙子高度场（并未实质运用）
    //std::vector<int>  humpBlock = { 31 - 10, 31 + 9, 31 - 8, 31 + 8 };//驼峰块，4维
    HostHeightField1d sandHeight;//沙子高度//HeightFieldGrid.h高度场网格，
    sandHeight.resize(sandinfo.nx, sandinfo.ny);
    //sandHeight.setSpace(sandinfo.griddl, sandinfo.griddl);//这行，换个值也一样，魏把这行注释掉了
    //fillGrid2D(sandHeight, 0.2);//高度场赋值,赋的是高度，不加这行的话，，也一样。魏把这行注释掉了
    //fillGrid2D(sandHeight, humpBlock, 0.4);//高度场驼峰快赋值，不加也一样，魏注释掉了
    psandSolver->setHeight(sandHeight);//连在沙粒解算器下面，这行注释掉不行

    // Sand particles.6沙子粒子初始化
    std::vector<Vector3d>                 particlePos;//粒子位置
    std::vector<ParticleType>             particleType;//沙子类型，，具体不清楚里面啥意思
    std::vector<double>                   particleMass;//粒子质量
	//这俩是啥
    std::default_random_engine            e(31);//随机数生成器,31是随机数种子
    std::uniform_real_distribution<float> u(-0.3, 0.3);//均匀分布类模板，u是个均匀分布

    // Sand plane.7沙子平面
    double spacing = sandinfo.griddl / 2.0;//半格长度
    double m0      = sandParticleHeight * sandinfo.sandRho * spacing * spacing;//50*0.015^2//在下面的循环中赋给了沙子质量
    double mb      = m0 * 5;//边界处沙子粒子质量
    //沙子平面？把这块去掉的话所有沙子就都不见了
    double spacing2 = spacing;  // / 2.0;//半格长度

	//8中心区域沙粒子
    for (int i = 0; i < sandinfo.nx; ++i){
        for (int j = 0; j < sandinfo.ny; ++j){
			//这个双循环针对沙子高度场每一个格子
            if (landHeight(i, j) < 0.25)//注意：硬地面高度小于0.25的话，进行如下双循环，否则不放沙子。0.25称为放沙高度
            {
                Vector3d centerij = landHeight.gridCenterPosition(i, j);//这里先把硬地面中心作为沙地中心
                for (int ii = 0; ii < 2; ++ii)
                    for (int jj = 0; jj < 2; ++jj)
					//设置沙子位置双循环，这个双循环，使得每个方格中放置左上右上左下右下四个沙粒，每个粒子的位置有一定范围内的随机性。
					//不考虑随机性的话就是一个正方形饭盒里面刚好平放了四个包子，包子位置即沙子粒子位置。如下图。
						//////
						//**//
						//**//
						//////
                    {
                        Vector3d curp = centerij;//沙地中心
                        curp[0] -= sandinfo.griddl / 2.0;//这四行去掉不行，将会只有少部分有沙子。调了一下沙地中心
                        curp[2] -= sandinfo.griddl / 2.0;
                        curp[0] += ii * spacing2 + spacing2 / 2.0 * (1.0 + u(e));//u(e)就是(-0.3, 0.3)的随机数
                        curp[2] += jj * spacing2 + spacing2 / 2.0 * (1.0 + u(e));
                        particlePos.push_back(curp);//push_back代表在vector末尾再加一个元素，所以这双层循环相当于给vector赋值
                        particleType.push_back(ParticleType::SAND);
                        particleMass.push_back(m0);//这仨vector容量大小都是高度场网格数量
                    }
            }
        }
    }

    // Boundary particle.9边界区域沙粒子，没啥用，注释掉了
    //for (int i = -5; i < sandinfo.nx + 5; ++i)
    //{
    //    for (int j = -5; j < sandinfo.ny + 5; ++j)
    //    {
    //        if (i >= 0 && i < sandinfo.nx && j >= 0 && j < sandinfo.ny) continue;//原区域内直接跳过，此块代码只处理边界
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

    std::vector<Vector3d> particleVel(particlePos.size());//每个网格处都应该有个粒子速度，所以是个vector，速度是三维向量
	//下面这个结点很重要，设置粒子全部信息，从0处开始传址。来自PBDSandSolver.cu
    psandSolver->setParticles(&particlePos[0], &particleVel[0], &particleMass[0], &particleType[0], particlePos.size(), sandinfo.sandRho, m0, sandinfo.griddl * 1.0, 0.85, sandParticleHeight * 0.5);

    //输出粒子数目，195064，其实是高度场网格数目（除了比0.25高的硬地面）。注意这里显示，其实就是一个网格上最多放一个粒子
    std::cout << "Particle number:   " << particlePos.size() << std::endl;

    // Rendering module of simulator.10.沙子渲染
    auto pRenderModule = std::make_shared<PointRenderModule>();
    //pRenderModule->varRenderMode()->getValue().currentKey() = PointRenderModule::RenderModeEnum::SPRITE;
    pRenderModule->setSphereInstaceSize(sandinfo.griddl * 0.5);//就是这里吧，设置沙子大小
    pRenderModule->setColor(Vector3f(1.0f, 1.0f, 122.0f / 255.0f));//设置沙子颜色，改改看！
    sandSim->addVisualModule(pRenderModule);//这里是不是沙子可视化的代码？

    // topology拓扑学 11.沙子拓扑
    auto topology = std::make_shared<PointSet<DataType3f>>();
    sandSim->setTopologyModule(topology);//设置拓扑模块，这模块到底干啥用的？
    topology->getPoints().resize(1);

    // Render point sampler (module). 12.沙子渲染点采样（采样是画（渲染）之前的操作，以一定的稀疏度采出点，然后画）
    auto psampler = std::make_shared<ParticleSandRenderSampler>();//沙粒子渲染采样节点
    psampler->Initialize(psandSolver);
    sandSim->addCustomModule(psampler);//添加自定义模块，干啥用的？

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
    m_car->wheelRelPosition[2] = Vector3f(-0.3f, -0.2,  0.4f) * scale1d + wheelCenter[2] - chassisCenter;
    m_car->wheelRelPosition[3] = Vector3f(+0.3f, -0.2,  0.4f) * scale1d + wheelCenter[3] - chassisCenter;

    //m_car->wheelRelRotation[0] = Quaternion<float>(0, 0, 0, 1);  // (0, 0.5, 0, 0.5).normalize();
    //m_car->wheelRelRotation[1] = Quaternion<float>(0, 0, 0, 1);  //(0, 0.5, 0, 0.5).normalize();
    //m_car->wheelRelRotation[2] = Quaternion<float>(0, 0, 0, 1);  //(0, 0.5, 0, 0.5).normalize();
    //m_car->wheelRelRotation[3] = Quaternion<float>(0, 0, 0, 1);  //(0, 0.5, 0, 0.5).normalize();
	//没有底盘的位姿，但是下面这两行，体现了底盘和轮子的相对位置！
    m_car->wheelupDirection    = Vector3f(0, 0.5, 0);//轮子在z轴方向上和底盘的相对位置，原来是(0, 1, 0)
    m_car->wheelRightDirection = Vector3f(1, 0, 0);//啥意思，反正去掉之后车就动不了了

    m_car->chassisMass    = 1500;  //设置底盘质量，去掉的话，车刚落地就飞了
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
    Vector3f wheelI        = RigidUtil::calculateCylinderLocalInertia(wheelm,//计算圆柱体局部惯性参数
                                                               (wheelSize[0][1] + wheelSize[0][2]) / 2.0,
                                                               wheelSize[0][0],
                                                               0);
    m_car->wheelMass[0]    = wheelm;//轮质量50
    m_car->wheelInertia[0] = wheelI;
    m_car->wheelMass[1]    = wheelm;
    m_car->wheelInertia[1] = wheelI;
    m_car->wheelMass[2]    = wheelm;
    m_car->wheelInertia[2] = wheelI;
    m_car->wheelMass[3]    = wheelm;
    m_car->wheelInertia[3] = wheelI;

    m_car->steeringLowerBound = -0.5;//旋转角下边界，啥意思？？？？
    m_car->steeringUpperBound = 0.5;

    m_car->forwardForceAcc = 10000;//前向牵引力增加量
    m_car->maxVel        = 2.5;//最大速度
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
    m_car->build();//build是MultiWheelCar这个类的函数，不是PBDCar的函数呀//应该是PBDCar的函数
	m_car2->build();


	////-------------------------------------------------------
	////碰撞之前准备，目前依旧穿模
	//
	//printf("initialize OK\n");
	//m_groundRigidInteractor = std::make_shared<HeightFieldPBDInteractionNode>();
 //   //m_groundRigidInteractor->setRigidBodySystem(m_car->m_rigidSystem);
 //   //m_groundRigidInteractor->setSize(sandinfo.nx, sandinfo.ny, sandinfo.griddl, sandinfo.griddl);
 //   //m_groundRigidInteractor->getSolver()->m_numSubstep          = 5;
 //   //m_groundRigidInteractor->getSolver()->m_numContactSolveIter = 20;

 //   m_groundRigidInteractor->getSolver()->setUseGPU(false);
 //   root->addChild(m_groundRigidInteractor);//注释掉就完全不调用碰撞了
 //   m_groundRigidInteractor->setDt(0.016);


	////DeviceHeightField1d& terrain  = m_groundRigidInteractor->getHeightField();//这仨注释掉试试
 ////   DeviceHeightField1d* terrain_ = &terrain;
 ////   Function1Pt::copy(*terrain_, landHeight);
 //   m_groundRigidInteractor->setDetectionMethod(HeightFieldPBDInteractionNode::HFDETECTION::POINTVISE);//注释掉看看
 //   //m_groundRigidInteractor->setDetectionMethod(HeightFieldTerrainRigidInteractionNode::HFDETECTION::FACEVISE);
 //   //terrain.setOrigin(0, 0, 0);
	//
	////上面是碰撞之前准备


	////尝试加碰撞
	//{//m_car底盘
 //       auto pointset = TypeInfo::cast<PointSet<DataType3f>>(m_car->getChassis()->getTopologyModule());//原来是getChassis()
	//	if (pointset)
	//	{
	//		std::shared_ptr<TOrientedBox3D<float>> pobb = std::make_shared<TOrientedBox3D<float>>();
	//		pobb->u = Vector3f(1, 0, 0);
	//		pobb->v = Vector3f(0, 1, 0);
	//		pobb->w = Vector3f(0, 0, 1);

	//		//DeviceArray<Vector3f>& vertices = pointset->getPoints();
	//		this->computeAABB(pointset, pobb->center, pobb->extent);

	//		if (m_groundRigidInteractor == NULL)
	//			m_groundRigidInteractor = std::make_shared<HeightFieldPBDInteractionNode>();//cy，发现这个是个空指针，加了这2行
	//		auto pdetector = m_groundRigidInteractor->getRigidContactDetector();//把这两行注释掉，就没异常了，但穿模
	//		printf("&&&&&&&&&&&&&&&&&&& call ADD HERE!\n");
	//		pdetector->addCollidableObject(m_car->m_chassis, pobb);//bug就在这！pobb没push进去！

	//		//wkm：是不是因为没有调用doCollision之类的
	//	}//异常再cy修好之后，仍穿模，因为遗漏了源代码中关于m_groundRigidInteractor的一系列语句，包括连在root上
	//	else
	//		printf("PointSet ERROR!\n");
	//}
	//
	////m_car轮子
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
	//m_groundRigidInteractor->addChild(m_car);//加了这两行，之前漏了

	//{//m_car2底盘
 //       auto pointset = TypeInfo::cast<PointSet<DataType3f>>(m_car2->getChassis()->getTopologyModule());//原来是getChassis()
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
 //           auto pdetector = m_groundRigidInteractor->getRigidContactDetector();//把这两行注释掉，就没异常了，但穿模
 //           pdetector->addCollidableObject(m_car2->m_chassis, pobb);//这个函数里面有两个push_back
 //       }
 //   }

	////m_car2轮子
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
	////上面是加碰撞--------------------------------------------------------------------------------------------------




    // Add visualization module and topology module.添加可视化模块和拓扑模块。先是底盘，再是轮子//注意这个部分要放在碰撞模块后面，要不然会发生vector越界
    m_car->m_chassis->setTopologyModule(chassisTri);//拓扑模块。啥用？啥意思？？？？
    auto chassisRender = std::make_shared<RigidMeshRender>(m_car->m_chassis->getTransformationFrame());
    chassisRender->setColor(Vector3f(0.8, std::rand() % 1000 / ( double )1000, 0.8));
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

    // Translate camera position   相机位置，没有也行，反正相机位置可以用鼠标调整
    auto& camera_ = this->activeCamera();
    Vector3f camPos(0, 1.5, 5);
    camera_.lookAt(camPos, Vector3f(0, 0, 0), Vector3f(0, 1, 0));
}
