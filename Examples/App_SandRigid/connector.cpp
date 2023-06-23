#include "connector.h"
#include <vector>
#include <Dynamics/RigidBody/RigidUtil.h>
#include <Dynamics/RigidBody/PBDRigid/PBDSolverNode.h>
#include <Dynamics/RigidBody/Vehicle/MultiWheelCar.h>
#include <Dynamics/RigidBody/Vehicle/PBDCar.h>
#include <Dynamics/Sand/ParticleSandRigidInteraction.h>
#include <Dynamics/Sand/SandGrid.h>
#include <Dynamics/Sand/SandSimulator.h>
#include <Framework/Framework/SceneGraph.h>
//#include "DataTypes.h"
#include "GUI/GlutGUI/GLApp.h"
#include "Rendering/PointRenderModule.h"
#include "Dynamics/Sand/PBDSandSolver.h"
#include "Dynamics/Sand/PBDSandRigidInteraction.h"
#include "Rendering/RigidMeshRender.h"
#include "Dynamics/RigidBody/RigidUtil.h"
#include "Dynamics/HeightField/HeightFieldMesh.h"
#include "IO/Surface_Mesh_IO/ObjFileLoader.h"
#include "Dynamics\Sand\ParticleSandRigidInteraction.h"
#include "Dynamics/Sand/SandVisualPointSampleModule.h"
#include "IO/Image_IO/HeightFieldLoader.h"

#include "GUI/GlutGUI/GLApp.h"


#include "sandRigidCommon.h"
#include "math.h"
//目前bug：vector越界
namespace VPE {
	using namespace PhysIKA;

	namespace {
		Vector3f ToPhysIKA(const Vec3 &v) {
			return { v.x, v.y, v.z };
		}

		Quaternion<float> ToPhysIKA(const Quat &v) {
			return { v.x, v.y, v.z, v.w };
		}
	}


	struct SandSimulationRegion::Impl {//原来那个类都不要了，改成留一个这个impl
	public:
		std::shared_ptr<PhysIKA::Node> node;
		float _sand_layer_thickness;
		float _delta;
		double _total_width_in_meter;//这东西在哪输入？setHeight
		double _total_height_in_meter;
		float    chassisMass = 1.0;//need corrected
		int newcarnumber = 0;
		std::vector<VPE::PhysIKACarCreateInfo> car_cache;

		//bianliang of create function
		std::shared_ptr<ParticleSandRigidInteraction> root;
		std::shared_ptr<PhysIKA::PBDSolverNode> rigidSim;
		std::vector<shared_ptr<PhysIKA::PBDCar>> m_car;
		std::vector<shared_ptr<VPE::PhysIKACar>> m_PhysIKACar;//这个类包含小车的位姿和轮子位姿
		std::shared_ptr<PhysIKA::SandInteractionForceSolver> interactionSolver;
		std::shared_ptr<PhysIKA::RigidBody2<PhysIKA::DataType3f>> m_chassis;
		std::string chassisFile = "";
		PhysIKA::Vector3f chassisInertia;
		PhysIKA::Vector3f          carPosition;
		PhysIKA::Quaternion<float> carRotation;
		std::vector<PhysIKA::RigidBody2_ptr>                   m_rigids;
		std::vector<std::shared_ptr<PhysIKA::RigidMeshRender>> m_rigidRenders;

		PhysIKA::SandGridInfo sandinfo;//网格
		PhysIKA::HostHeightField1d landHeight;//高度
		std::vector<PhysIKA::Vector3d>  particlePos;//粒子位置
		shared_ptr<SandSimulationRegion>m_instance;
		std::shared_ptr<PhysIKA::PBDSolver> rigidSolver;


		std::shared_ptr<PhysIKA::Node> GetRoot()//这个root是属于哪个派生类？ParticleSandRigidInteraction
		{
			return root;
		}

		//void SetHeight(double *data, int resolution_x, int resolution_y)
		//{

		//	//这里前面的两行就设置成员变量缓存吧，后面剪切到build里
		//	sandinfo.nx = resolution_x;
		//	sandinfo.ny = resolution_y;
		//	sandinfo.griddl = _total_width_in_meter / resolution_x;//格子间距，也是沙粒大小！
		//	sandinfo.mu = 0.7;//改改看看,应该是摩擦，或者阻尼之类//这几个，应该是都没用到。
		//	sandinfo.drag = 0.95;//改改看看拖拽力，没有感觉也一样
		//	sandinfo.slide = 10 * sandinfo.griddl;//10改成100看不出变化
		//	sandinfo.sandRho = 1000.0;////改改看看，没明白
		//	double sandParticleHeight = 0.05;//0.05是沙粒子距离地面高度！

		//	//1这边要导入data

		//	landHeight.resize(sandinfo.nx, sandinfo.ny);//设置硬地面大小
		//	landHeight.setSpace(sandinfo.griddl, sandinfo.griddl);//设置硬地面高度场网格长宽
		//	printf("%d\n%d\n", sandinfo.nx, sandinfo.ny);//256256没错
		//	printf("%d\n%d\n", landHeight.Nx(), landHeight.Ny());//256256

		//	//下面从data导入硬地面高度场数据
		//	for (int i = 0; i < sandinfo.nx; ++i) {//地面高度设置，两层循环
		//		for (int j = 0; j < sandinfo.ny; ++j) {
		//			double aa = *data;
		//			landHeight(i, j) = aa;
		//			data++;
		//		}
		//	}
		//}

		std::shared_ptr<VPE::PhysIKACar> GetCar(uint64_t car_handle)//car_handle是啥？小车个数
		{
			if (car_handle < newcarnumber) {//我懂啦，一个函数想调用另一个函数的对象，就把这个对象设置为静态，然后调用时带上原来这个类的域就好啦
				//car_handle = Builder->m_car.size();//我知道了，这些画红线的，是因为没有列出对象，对象+->才对。
				return m_PhysIKACar[car_handle];//一问：如何拿到这个车的智能指针呢？答：在build中加用一个函数，来输出车的指针。不行呀，加一个缓存vector吗，那还是过不了.h那关呀
			}//m_car不是PhysIKACar类，这里要改
			else return nullptr;
		}

		std::vector< VPE::Vec3> GetSandParticles()//由粒子序号获得粒子信息//VPE::Vec3*原来要的是这个输出
		{
			int particle_num = particlePos.size();//先这种也是在build里面做一个缓存变量，就在输出粒子那个地方
			vector<VPE::Vec3> a;
			VPE::Vec3 a_cache;
			a_cache.x = 0;
			a_cache.y = 0;
			a_cache.z = 0;
			// TODO over
			for (int i = 0; i < particle_num; i++) {
				a.push_back(a_cache);
				a[i].x = particlePos[i][0];//像这种，在build类里面设置一个缓存二维数组，在build函数里面把二维数组赋值，然后在这拿着这个二维数组来输出
				a[i].y = particlePos[i][1];//also put in impl-chengyuanbianliang,, 
				a[i].z = particlePos[i][2];
			}

			return a;
		}

		//uint64_t AddCar(const VPE::PhysIKACarCreateInfo& carinfo)//返回车辆编号，也是car_vector.size
		//{
		//	newcarnumber++;
		//	car_cache.push_back(carinfo);
		//	return newcarnumber;
		//}



		std::shared_ptr<SandSimulationRegion> Init(const SandSimulationRegionCreateInfo& info);

	};//create

	std::shared_ptr<PhysIKA::Node> SandSimulationRegion::GetRoot() {//因为用了impl所以要有这些
		return _impl->GetRoot();
	}


	std::shared_ptr<VPE::PhysIKACar>  SandSimulationRegion::GetCar(uint64_t car_handle) {
		return _impl->GetCar(car_handle);
	}

	std::vector< VPE::Vec3> SandSimulationRegion::GetSandParticles() {
		return _impl->GetSandParticles();
	}

	//SandSimulationRegion* SandSimulationRegion::region;
	std::shared_ptr<SandSimulationRegion> SandSimulationRegion::Create(const SandSimulationRegionCreateInfo& info) {
		// TODO 静态函数访问不了成员变量
		auto region = std::make_shared<SandSimulationRegion>();//从cpp的SandSimulationRegion::Create

		auto impl = region->_impl.get();
		/*	impl->SetHeight(info.height_data, info.height_resolution_x, info.height_resolution_y);
			for (auto&car_info : info.cars) {
				impl->AddCar(car_info);
			}*/
		impl->Init(info);
		return region;
	}

	SandSimulationRegion::~SandSimulationRegion() = default;
	SandSimulationRegion::SandSimulationRegion() {
		_impl = std::make_unique<Impl>();
	}


	//所以现在问题在这个类
	struct VPE::PhysIKACar::Impl2 {
	public:
		shared_ptr<PhysIKA::PBDCar> m_car;

		Vector3f wheelupDirection;//轮子在z轴上与底盘的相对位置
		Vector3f wheelRightDirection;  // wheel right direction in car frame.

		/*Vec3 a;
		Vec3 b;
		Vec3 ab;
		Vec3 ab_010;
		double cos;
		double sin;
		double cos0_5;
		double sin0_5;*/


		void GetChassisPositionRotation(VPE::Vec3& pos, VPE::Quat& rot)
		{
			pos.x = m_car->carPosition[0] + m_car->chassisMeshTranslate[0];//要的是底盘，这里要加上便平移translation*单位法向量（法向量由轮子位置向量叉乘确定）
			pos.y = m_car->carPosition[1] + m_car->chassisMeshTranslate[1];
			pos.z = m_car->carPosition[2] + m_car->chassisMeshTranslate[2];

			// TODO chassis//底盘角度在源代码里面怎么体现？要搞清楚：不体现，恒为0000，换句话说就是不在乎这个数据，就像小车速度。
			//方案：输出其他用到的数据：车位置，轮子相对位置，轮子相对角度，底盘相对位置。

			//先把四轮位置做成两个向量，然后做叉积得法向量ab(再单位化)，和初始法向量(010)做叉积得到轴向量ab_010(再单位化)。
			//再通过两个法向量做叉积得到夹角余弦cos
			Vec3 a = { m_car->wheelRelPosition[1][0] - m_car->wheelRelPosition[0][0],m_car->wheelRelPosition[1][1] - m_car->wheelRelPosition[0][1], m_car->wheelRelPosition[1][2] - m_car->wheelRelPosition[0][2], };
			Vec3 b = { m_car->wheelRelPosition[2][0] - m_car->wheelRelPosition[0][0],m_car->wheelRelPosition[2][1] - m_car->wheelRelPosition[0][1], m_car->wheelRelPosition[2][2] - m_car->wheelRelPosition[0][2], };
			//a,b叉乘
			Vec3 ab = { a.y*b.z - b.y*a.z ,a.z*b.x - b.z*a.x ,a.x*b.y - a.y*b.x };
			//单位化
			ab = { static_cast <float>(ab.x / sqrt(pow(ab.x, 2) + pow(ab.y, 2) + pow(ab.z, 2)) ), static_cast <float>(ab.y / sqrt(pow(ab.x, 2) + pow(ab.y, 2) + pow(ab.z, 2)) ) , static_cast <float>(ab.z / sqrt(pow(ab.x, 2) + pow(ab.y, 2) + pow(ab.z, 2)) )};
			//ab叉乘010
			Vec3 ab_010 = { -ab.z ,0 ,ab.x };
			//单位化，得到轴法向量
			ab_010 = { static_cast <float>(ab_010.x / sqrt(pow(ab_010.x, 2) + pow(ab_010.y, 2) + pow(ab_010.z, 2)) ), static_cast <float>(ab_010.y / sqrt(pow(ab_010.x, 2) + pow(ab_010.y, 2) + pow(ab_010.z, 2)) ) , static_cast <float>(ab_010.z / sqrt(pow(ab_010.x, 2) + pow(ab_010.y, 2) + pow(ab_010.z, 2)) )};

			double cos = ab.y * 1 / sqrt(pow(ab.x, 2) + pow(ab.y, 2) + pow(ab.z, 2));
			double sin = sqrt(1 - pow(cos, 2));

			double cos0_5 = sqrt(0.5*(1 + cos));
			double sin0_5 = sqrt(0.5*(1 - cos));

			//按照四元数公式：四元数的基本数学方程为 : q = cos (a/2) + i(x * sin(a/2)) + j(y * sin(a/2)) + k(z * sin(a/2)) 其中a表示旋转角度，(x,y,z)表示旋转轴。
			rot.x = cos0_5;
			rot.y = ab_010.x * sin0_5;
			rot.z = ab_010.y * sin0_5;
			rot.w = ab_010.z * sin0_5;
		}

		void GetWheelPositionRotation(uint32_t wheel_index, Vec3& pos, Quat& rot)
		{

			pos.x = m_car->wheelRelPosition[wheel_index][0] + m_car->carPosition[0];
			pos.y = m_car->wheelRelPosition[wheel_index][1] + m_car->carPosition[1];
			pos.z = m_car->wheelRelPosition[wheel_index][2] + m_car->carPosition[2];

			//rot.x = m_car->wheelRelRotation[wheel_index][0];//这个也始终是0，要改//所以轮子绝对角度到底在哪，包括打轮速度
			//rot.y = m_car->wheelRelRotation[wheel_index][1];
			//rot.z = m_car->wheelRelRotation[wheel_index][2];
			//rot.w = m_car->wheelRelRotation[wheel_index][3];
			//float theta = m_car->currentSteering;//车轮转角（还是private），这个是相对角度，轴是010，参考系是底盘旋转。

			//前轮角度=底盘角度+转角
			//后轮角度=底盘角度
			//底盘角度在上面那个函数里面，复制一遍就好。
			
				Vec3 a = { m_car->wheelRelRotation[1][0] - m_car->wheelRelRotation[0][0],m_car->wheelRelRotation[1][1] - m_car->wheelRelRotation[0][1], m_car->wheelRelRotation[1][2] - m_car->wheelRelRotation[0][2], };
				Vec3 b = { m_car->wheelRelRotation[2][0] - m_car->wheelRelRotation[0][0],m_car->wheelRelRotation[2][1] - m_car->wheelRelRotation[0][1], m_car->wheelRelRotation[2][2] - m_car->wheelRelRotation[0][2], };
				//a,b叉乘
				Vec3 ab = { a.y*b.z - b.y*a.z ,a.z*b.x - b.z*a.x ,a.x*b.y - a.y*b.x };
				//单位化
				ab = { static_cast <float>(ab.x / sqrt(pow(ab.x, 2) + pow(ab.y, 2) + pow(ab.z, 2))) , static_cast <float>(ab.y / sqrt(pow(ab.x, 2) + pow(ab.y, 2) + pow(ab.z, 2)) ) , static_cast <float>(ab.z / sqrt(pow(ab.x, 2) + pow(ab.y, 2) + pow(ab.z, 2))) };
				//ab叉乘010
				Vec3 ab_010 = { -ab.z ,0 ,ab.x };
				//单位化，得到轴法向量
				ab_010 = { static_cast <float>(ab_010.x / sqrt(pow(ab_010.x, 2) + pow(ab_010.y, 2) + pow(ab_010.z, 2)) ), static_cast <float>(ab_010.y / sqrt(pow(ab_010.x, 2) + pow(ab_010.y, 2) + pow(ab_010.z, 2))  ), static_cast <float>(ab_010.z / sqrt(pow(ab_010.x, 2) + pow(ab_010.y, 2) + pow(ab_010.z, 2))) };

				double cos = ab.y * 1 / sqrt(pow(ab.x, 2) + pow(ab.y, 2) + pow(ab.z, 2));
				double sin = sqrt(1 - pow(cos, 2));

				double cos0_5 = sqrt(0.5*(1 + cos));
				double sin0_5 = sqrt(0.5*(1 - cos));
			
				//这里写关于车轮转角!
				double wheel_angle=m_car->currentSteering;
				

			if (wheel_index >=2) {
				{
					//按照四元数公式：四元数的基本数学方程为 : q = cos (a/2) + i(x * sin(a/2)) + j(y * sin(a/2)) + k(z * sin(a/2)) 其中a表示旋转角度，(x,y,z)表示旋转轴。
					rot.x = cos0_5;
					rot.y = ab_010.x * sin0_5;
					rot.z = ab_010.y * sin0_5;
					rot.w = ab_010.z * sin0_5;
				}
			}
			
			if (wheel_index < 2) {
				{
					double cos2 = cos * std::cos(wheel_angle) - sin * std::sin(wheel_angle);
					double sin2 = sin * std::cos(wheel_angle) + cos * std::sin(wheel_angle);
					cos0_5 = sqrt(0.5*(1 + cos2));
					sin0_5 = sqrt(0.5*(1 - cos2));
					rot.x = cos0_5;
					rot.y = ab_010.x * sin0_5;
					rot.z = ab_010.y * sin0_5;
					rot.w = ab_010.z * sin0_5;
				}

			}
		}

		void SetChassisPositionRotation(const Vec3& pos, const Quat& rot)
		{
			m_car->carPosition[0] = pos.x;//等号右边要减去平移
			m_car->carPosition[1] = pos.y;
			m_car->carPosition[2] = pos.z;

			//底盘角度没法赋值，无处赋值，可以赋值轮子位置。
			//不如就认为底盘没有浮动吧，他最多只会原地转圈！//以carPosition为中心点，就给四个轮子设置位置，等同于设置底盘角度。
			
			double cos0_5 = rot.x;
			double sin0_5 = 1-pow(cos0_5,2);
			//偏转角正弦余弦如下
			double sin = 2 * cos0_5*sin0_5;
			double cos = pow(cos0_5, 2) - pow(sin0_5, 2);
			//0号车轮的旋转
			Vec3 a;
			a.x = m_car->wheelRelPosition[0][0] - m_car->carPosition[0];
			a.y = 0;
			a.z = m_car->wheelRelPosition[0][2] - m_car->carPosition[2];
			//0号车轮的向量，长度和夹角
			double length = pow(pow(a.x, 2) + pow(a.z, 2), 0.5);
			double cos_ = a.x / length;
			double sin_ = pow(1-pow(cos_,2), 0.5);
			//这个是旋转后角度
			double final_cos = cos_ * cos - sin_ * sin;
			double final_sin = cos_ * sin + sin_ * cos;

			m_car->wheelRelPosition[0][0] = length * final_cos;
			m_car->wheelRelPosition[0][2] = length * final_sin;

			//1号车轮的旋转
			//Vec3 a;
			a.x = m_car->wheelRelPosition[1][0] - m_car->carPosition[0];
			a.y = 0;
			a.z = m_car->wheelRelPosition[1][2] - m_car->carPosition[2];
			//0号车轮的向量，长度和夹角
			double length1 = pow(pow(a.x, 2) + pow(a.z, 2), 0.5);
			double cos_1 = a.x / length1;
			double sin_1 = pow(1 - pow(cos_, 2), 0.5);
			//这个是旋转后角度
			double final_cos1 = cos_1 * cos - sin_1 * sin;
			double final_sin1 = cos_1 * sin + sin_1 * cos;

			m_car->wheelRelPosition[1][0] = length1 * final_cos1;
			m_car->wheelRelPosition[1][2] = length1 * final_sin1;


			//2号车轮的旋转
			//Vec3 a;
			a.x = m_car->wheelRelPosition[2][0] - m_car->carPosition[0];
			a.y = 0;
			a.z = m_car->wheelRelPosition[2][2] - m_car->carPosition[2];
			//0号车轮的向量，长度和夹角
			double length2 = pow(pow(a.x, 2) + pow(a.z, 2), 0.5);
			double cos_2 = a.x / length2;
			double sin_2 = pow(1 - pow(cos_2, 2), 0.5);
			//这个是旋转后角度
			double final_cos2 = cos_2 * cos - sin_2 * sin;
			double final_sin2 = cos_2 * sin + sin_2 * cos;

			m_car->wheelRelPosition[2][0] = length2 * final_cos;
			m_car->wheelRelPosition[2][2] = length2 * final_sin;


			//3号车轮的旋转
			//Vec3 a;
			a.x = m_car->wheelRelPosition[3][0] - m_car->carPosition[0];
			a.y = 0;
			a.z = m_car->wheelRelPosition[3][2] - m_car->carPosition[2];
			//0号车轮的向量，长度和夹角
			double length3 = pow(pow(a.x, 2) + pow(a.z, 2), 0.5);
			double cos_3 = a.x / length3;
			double sin_3 = pow(1 - pow(cos_3, 2), 0.5);
			//这个是旋转后角度
			double final_cos3 = cos_3 * cos - sin_3 * sin;
			double final_sin3 = cos_3 * sin + sin_3 * cos;

			m_car->wheelRelPosition[3][0] = length3 * final_cos3;
			m_car->wheelRelPosition[3][2] = length3 * final_sin3;
		}

		void SetWheelPositionRotation(uint32_t wheel_index, const Vec3& pos, const Quat& rot)
		{//这四个都是绝对的
		//set chassis height
		//TODO

			//轮子位置
			m_car->wheelRelPosition[wheel_index][0] = pos.x - m_car->carPosition[0];
			m_car->wheelRelPosition[wheel_index][1] = pos.y - m_car->carPosition[1];
			m_car->wheelRelPosition[wheel_index][2] = pos.z - m_car->carPosition[2];

			
			//轮子角度赋值部分，在上一个函数
		}

		void Go(PhysIKACarDirection dir)
		{
			switch (dir)
			{
			case VPE::PhysIKACarDirection::Forward:
				m_car->forward(0.016);
				break;
			case VPE::PhysIKACarDirection::Backward:
				m_car->backward(0.016);
				break;
			case VPE::PhysIKACarDirection::Left:
				m_car->goLeft(0.016);
				break;
			case VPE::PhysIKACarDirection::Right:
				m_car->goRight(0.016);
				break;
			default:
				break;
			}
		}

	};

	void VPE::PhysIKACar::GetChassisPositionRotation(VPE::Vec3& pos, VPE::Quat& rot) {
		return _impl2->GetChassisPositionRotation(pos, rot);//?with static will call error.
	}

	void VPE::PhysIKACar::GetWheelPositionRotation(uint32_t wheel_index, Vec3& pos, Quat& rot) {
		return _impl2->GetWheelPositionRotation(wheel_index, pos, rot);//?with static will call error.
	}

	void VPE::PhysIKACar::SetChassisPositionRotation(const Vec3& pos, const Quat& rot) {
		return _impl2->SetChassisPositionRotation(pos, rot);//?with static will call error.
	}

	void VPE::PhysIKACar::SetWheelPositionRotation(uint32_t wheel_index, const Vec3& pos, const Quat& rot) {
		return _impl2->SetWheelPositionRotation(wheel_index, pos, rot);//?with static will call error.
	}

	void VPE::PhysIKACar::Go(PhysIKACarDirection dir) {
		return _impl2->Go(dir);
	}

	VPE::PhysIKACar::~PhysIKACar() = default;
	VPE::PhysIKACar::PhysIKACar() {
		_impl2 = std::make_unique<Impl2>();
	}


	inline std::shared_ptr<SandSimulationRegion> SandSimulationRegion::Impl::Init(const SandSimulationRegionCreateInfo & info)//info这一系列参数都传进去了吗？没有，甚至那俩函数功能都没有合并进去！
	{
		//setHeight

		//这里前面的两行就设置成员变量缓存吧，后面剪切到build里
		sandinfo.nx = info.height_resolution_x;
		sandinfo.ny = info.height_resolution_y;
		sandinfo.griddl = info.total_width_in_meter / info.height_resolution_x;//格子间距，也是沙粒大小！
		sandinfo.mu = 0.7;//改改看看,应该是摩擦，或者阻尼之类//这几个，应该是都没用到。
		sandinfo.drag = 0.95;//改改看看拖拽力，没有感觉也一样
		sandinfo.slide = 10 * sandinfo.griddl;//10改成100看不出变化
		sandinfo.sandRho = 1000.0;////改改看看，没明白
		double sandParticleHeight = 0.05;//0.05是沙粒子距离地面高度！

										 //1这边要导入data

		landHeight.resize(sandinfo.nx, sandinfo.ny);//设置硬地面大小
		landHeight.setSpace(sandinfo.griddl, sandinfo.griddl);//设置硬地面高度场网格长宽
		//printf("%d\n%d\n", sandinfo.nx, sandinfo.ny);//256256没错
		//printf("%d\n%d\n", landHeight.Nx(), landHeight.Ny());//256256

															 //下面从data导入硬地面高度场数据
		for (int i = 0; i < sandinfo.nx; ++i) {//地面高度设置，两层循环
			for (int j = 0; j < sandinfo.ny; ++j) {
				double aa = info.height_data[i * sandinfo.nx + j];
				landHeight(i, j) = aa;
				//(info.height_data)++;//这里报错
			}
		}

		//--------------------------------------------------------------------------------------------

		car_cache = info.cars;

		//--------------------------------------------------------------------------------------------
		//double sandParticleHeight = 0.05;

		SceneGraph& scene = SceneGraph::getInstance();
		scene.setUpperBound(Vector3f(10, 10, 10));
		scene.setLowerBound(Vector3f(-10, -5, -10));
		// 1 Root node. Also the simulator.
		root = scene.createNewScene<ParticleSandRigidInteraction>();//这行出问题，引发了未经处理的异常：读取访问权限冲突
		root->setActive(true);
		root->setDt(info.time_delta);

		root->varBouyancyFactor()->setValue(50);
		root->varDragFactor()->setValue(1.0);
		root->varCHorizontal()->setValue(1.);
		root->varCVertical()->setValue(2.);
		root->varCprobability()->setValue(100);

		// 2 Sand simulator.
		std::shared_ptr<PhysIKA::SandSimulator> sandSim = std::make_shared<PhysIKA::SandSimulator>();
		std::shared_ptr<PhysIKA::PBDSandSolver> psandSolver = std::make_shared<PhysIKA::PBDSandSolver>();
		psandSolver->setSandGridInfo(sandinfo);
		sandSim->needForward(false);
		sandSim->setSandSolver(psandSolver);
		root->setSandSolver(psandSolver);
		root->addChild(sandSim);

		auto sandinitfun = [](PhysIKA::PBDSandSolver* solver) { solver->freeFlow(1); };
		psandSolver->setPostInitializeFun(std::bind(sandinitfun, std::placeholders::_1));

		//printf("%f\n%f", landHeight.Nx(), landHeight.Ny());//是0！知道了！landHeight在set函数里面初始化，实际是初始化成功之后没能传进来！
														   //合并函数结构体整体传参之后，这里咋还是0，0呢？！因为合并传参之后忘记合并函数功能了。。尴尬

		psandSolver->setLand(landHeight);//cy:是这里出的bug  wkm:因为没有初始化landHeight，需要调用前文函数！但现在调了还是报错，，可能没调成功？

										 // TODO 渲染全部删掉
										 // 4 Land mesh.
		{
			auto landrigid = std::make_shared<PhysIKA::RigidBody2<PhysIKA::DataType3f>>("Land");
			root->addChild(landrigid);

			// Mesh triangles.
			auto triset = std::make_shared<PhysIKA::TriangleSet<PhysIKA::DataType3f>>();
			landrigid->setTopologyModule(triset);

			// Generate mesh.
			auto&           hfland = psandSolver->getLand();
			PhysIKA::HeightFieldMesh hfmesh;
			hfmesh.generate(triset, hfland);

			// Mesh renderer.网格渲染，涂颜色
			auto renderModule = std::make_shared<PhysIKA::RigidMeshRender>(landrigid->getTransformationFrame());
			renderModule->setColor(PhysIKA::Vector3f(210.0 / 255.0, 180.0 / 255.0, 140.0 / 255.0));
			landrigid->addVisualModule(renderModule);
		}

		// Sand height 5沙子高度场（并未实质运用）
		//std::vector<int>  humpBlock = { 31 - 10, 31 + 9, 31 - 8, 31 + 8 };
		//设置高度场网格
		HostHeightField1d sandHeight;//用的是CPU//md这声明也能出错？
		sandHeight.resize(sandinfo.nx, sandinfo.ny);
		psandSolver->setHeight(sandHeight);

		// 6 Sand particles.

		std::vector<PhysIKA::ParticleType>    particleType;
		std::vector<double>                   particleMass;//粒子质量
		std::default_random_engine            e(31);//随机数生成器,31是随机数种子
		std::uniform_real_distribution<float> u(-0.3, 0.3);//均匀分布类模板，u是个均匀分布

														   // 7 Sand plane.
		double spacing = sandinfo.griddl / 2.0;//半格长度
		double m0 = sandParticleHeight * sandinfo.sandRho * spacing * spacing;//50*0.015^2//密度用在这里，来算单个沙粒质量
		double mb = m0 * 5;
		double spacing2 = spacing;

		//8沙粒子信息初始化，位置类型质量
		for (int i = sandinfo.nx/2/*0*/; i < sandinfo.nx; ++i) {
			for (int j = 0; j < sandinfo.ny; ++j) {
				if (landHeight(i, j) < info.sand_layer_thickness)
				{
					PhysIKA::Vector3d centerij = landHeight.gridCenterPosition(i, j);//这里先把硬地面中心作为沙地中心
					for (int ii = 0; ii < 2; ++ii)
						for (int jj = 0; jj < 2; ++jj)
						{
							PhysIKA::Vector3d curp = centerij;//沙地中心
							curp[0] -= sandinfo.griddl / 2.0;//这四行调了一下沙地中心
							curp[2] -= sandinfo.griddl / 2.0;
							curp[0] += ii * spacing2 + spacing2 / 2.0 * (1.0 + u(e));//这里用到均匀分布？用途待研究
							curp[2] += jj * spacing2 + spacing2 / 2.0 * (1.0 + u(e));
							particlePos.push_back(curp);//push_back代表在vector末尾再加一个元素，所以这双层循环相当于给vector赋值
							particleType.push_back(PhysIKA::ParticleType::SAND);
							particleMass.push_back(m0);//这仨vector容量大小都是高度场网格数量
						}
				}
				//printf("%d\n",particlePos.size());
			}
		}

		std::vector<PhysIKA::Vector3d> particleVel(particlePos.size());
		psandSolver->setParticles(&particlePos[0], &particleVel[0], &particleMass[0], &particleType[0], particlePos.size(), sandinfo.sandRho, m0, sandinfo.griddl * 1.0, 0.85, sandParticleHeight * 0.5);
		//这是个空的vector，难道sandinfo.nx是0？
		std::cout << "Particle number:   " << particlePos.size() << std::endl;

		// 10 Rendering module of simulator.渲染模块不要
		auto pRenderModule = std::make_shared<PhysIKA::PointRenderModule>();
		pRenderModule->setSphereInstaceSize(sandinfo.griddl * 0.5);
		pRenderModule->setColor(PhysIKA::Vector3f(1.0f, 1.0f, 122.0f / 255.0f));
		sandSim->addVisualModule(pRenderModule);

		// 11 topology
		auto topology = std::make_shared<PhysIKA::PointSet<PhysIKA::DataType3f>>();
		sandSim->setTopologyModule(topology);
		topology->getPoints().resize(1);

		// 12 Render point sampler (module). 
		auto psampler = std::make_shared<PhysIKA::ParticleSandRenderSampler>();
		psampler->Initialize(psandSolver);
		sandSim->addCustomModule(psampler);

		/// ------  Rigid ------------
		//13 PBD simulation
		rigidSim = std::make_shared<PhysIKA::PBDSolverNode>();
		rigidSim->getSolver()->setUseGPU(true);
		rigidSim->needForward(false);
		rigidSolver = rigidSim->getSolver();

		root->setRigidSolver(rigidSolver);
		root->addChild(rigidSim);

		//--------------------------------------------------------------------
		// Car.14小车内部数据类型
		double   scale1d = 1.;
		PhysIKA::Vector3d scale3d(scale1d, scale1d, scale1d);//没用到
		PhysIKA::Vector3f scale3f(scale1d, scale1d, scale1d);//(1,1,1)//用在网格里

		PhysIKA::Vector3f chassisCenter;//三维数组，初始都是000
		PhysIKA::Vector3f wheelCenter[4];
		PhysIKA::Vector3f chassisSize;
		PhysIKA::Vector3f wheelSize[4];

		std::shared_ptr<PhysIKA::TriangleSet<PhysIKA::DataType3f>> chassisTri;//底盘和轮子都设置为三角网格
		std::shared_ptr<PhysIKA::TriangleSet<PhysIKA::DataType3f>> wheelTri[4];

		PhysIKA::DistanceField3D<PhysIKA::DataType3f> chassisSDF;
		PhysIKA::DistanceField3D<PhysIKA::DataType3f> wheelSDF[4];

		interactionSolver = root->getInteractionSolver();//ParticleSandRigidInteraction类里的

														 // Load car mesh.15载入底盘和轮子的网格和SDF
		{
			// Chassis mesh.底盘网格和包围盒
			PhysIKA::ObjFileLoader chassisLoader("../../Media/car2/chassis_cube.obj");//载入底盘文件

			chassisTri = std::make_shared<PhysIKA::TriangleSet<PhysIKA::DataType3f>>();//底盘上设置三角网格
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
												  // TODO 这些路径从info输入model_path
			chassisSDF.loadSDF("../../Media/car2/chassis_cube.sdf");//载入底盘SDF文件carinfo.chassis.sdf_path//但是这样不是很方便，carinfo不是create的形参，建议写死里面
			chassisSDF.scale(scale1d);
			chassisSDF.translate(-chassisCenter);
			//interactionSolver->addSDF(sdf);

			for (int i = 0; i < 4; ++i)//四个轮子
			{
				string objfile("../../Media/car2/wheel.obj");
				string sdffile("../../Media/car2/wheel.sdf");

				// Wheel mesh.轮子设置上网格
				PhysIKA::ObjFileLoader wheelLoader(objfile);
				wheelTri[i] = std::make_shared<PhysIKA::TriangleSet<PhysIKA::DataType3f>>();
				wheelTri[i]->setPoints(wheelLoader.getVertexList());//这两行是啥？
				wheelTri[i]->setTriangles(wheelLoader.getFaceList());
				computeBoundingBox(wheelCenter[i], wheelSize[i], wheelLoader.getVertexList());//计算包围盒？？？
				wheelTri[i]->scale(scale3f);
				wheelTri[i]->translate(-wheelCenter[i]);

				// Wheel sdf.
				PhysIKA::DistanceField3D<PhysIKA::DataType3f>& sdf = wheelSDF[i];
				sdf.loadSDF(sdffile);
				sdf.scale(scale1d);
				sdf.translate(-wheelCenter[i]);
				//interactionSolver->addSDF(sdf);
			}
		}

		//16 m_car
		newcarnumber = car_cache.size();
		for (int u = 0; u < newcarnumber; u++) //this is addcar cycle.
		{//每次循环都应该针对m_car[u]
		 //加一辆车的话，newcarnumber是1，走一次循环。
			m_car.push_back(std::make_shared<PhysIKA::PBDCar>());
			m_PhysIKACar.push_back(std::make_shared<VPE::PhysIKACar>());
			m_PhysIKACar.back()->_impl2->m_car = m_car.back();

			// TODO over
			//下面这些有newcarnumber的应该也在循环里！
			rigidSim->addChild(m_car[u]);
			m_car[u]->m_rigidSolver = rigidSolver;


			m_car[u]->carPosition = PhysIKA::Vector3f(car_cache[u].carPosition.x, car_cache[u].carPosition.y, car_cache[u].carPosition.z) + chassisCenter;//设置车子初始位置！！！！！

																																						  //四个轮子的相对位置和相对旋转，要粘在接口函数GetWheelPositionRotation里
																																						  //下面这些使用是要通过另一个类赋值的，而不是现在的写死,从vector carcache[newcarnumber]分门别类传进去！！！
			m_car[u]->wheelRelPosition[0] = PhysIKA::Vector3f(car_cache[u].wheels[0].translation.x, car_cache[u].wheels[0].translation.y, car_cache[u].wheels[0].translation.z) * scale1d + wheelCenter[0] - chassisCenter;//(-0.3f, -0.2, -0.4f)
			m_car[u]->wheelRelPosition[1] = PhysIKA::Vector3f(car_cache[u].wheels[1].translation.x, car_cache[u].wheels[1].translation.y, car_cache[u].wheels[1].translation.z) * scale1d + wheelCenter[1] - chassisCenter;//(+0.3f, -0.2, -0.4f)
			m_car[u]->wheelRelPosition[2] = PhysIKA::Vector3f(car_cache[u].wheels[2].translation.x, car_cache[u].wheels[2].translation.y, car_cache[u].wheels[2].translation.z) * scale1d + wheelCenter[2] - chassisCenter;//(-0.3f, -0.2,  0.4f)
			m_car[u]->wheelRelPosition[3] = PhysIKA::Vector3f(car_cache[u].wheels[3].translation.x, car_cache[u].wheels[3].translation.y, car_cache[u].wheels[3].translation.z) * scale1d + wheelCenter[3] - chassisCenter;//(+0.3f, -0.2,  0.4f)
			m_car[u]->wheelRelRotation[0] = PhysIKA::Quaternion<float>(car_cache[u].wheels[0].RelRotation.x, car_cache[u].wheels[0].RelRotation.y, car_cache[u].wheels[0].RelRotation.z, car_cache[u].wheels[0].RelRotation.w);  //(0, 0, 0, 1);
			m_car[u]->wheelRelRotation[1] = PhysIKA::Quaternion<float>(car_cache[u].wheels[1].RelRotation.x, car_cache[u].wheels[1].RelRotation.y, car_cache[u].wheels[1].RelRotation.z, car_cache[u].wheels[1].RelRotation.w);
			m_car[u]->wheelRelRotation[2] = PhysIKA::Quaternion<float>(car_cache[u].wheels[2].RelRotation.x, car_cache[u].wheels[2].RelRotation.y, car_cache[u].wheels[2].RelRotation.z, car_cache[u].wheels[2].RelRotation.w);
			m_car[u]->wheelRelRotation[3] = PhysIKA::Quaternion<float>(car_cache[u].wheels[3].RelRotation.x, car_cache[u].wheels[3].RelRotation.y, car_cache[u].wheels[3].RelRotation.z, car_cache[u].wheels[3].RelRotation.w);  //car_cache[newcarnumber].wheels[3].RelRotation
																																																								 //没有底盘的位姿，但是下面这两行，体现了底盘和轮子的相对位置！就是平移
			m_car[u]->wheelupDirection = PhysIKA::Vector3f(0, 0.5, 0);/*car_cache[u].wheels[0].translation.x, car_cache[u].wheels[0].translation.y, car_cache[u].wheels[0].translation.z*///轮子在z轴方向上和底盘的相对位置，原来是(0, 1, 0)(0, 0.5, 0)(0, 0.25, 0)//用注释里面的数据就会导致车轮的轴设置错误，深刻原因未知
			m_car[u]->wheelRightDirection = PhysIKA::Vector3f(1, 0, 0);//啥意思，反正去掉之后车就动不了了

			m_car[u]->chassisMass = car_cache[u].car_mass;  //设置底盘质量
			m_car[u]->chassisInertia = PhysIKA::RigidUtil::calculateCubeLocalInertia(m_car[u]->chassisMass, chassisSize);//计算底盘惯性并设置

			float wheelm = car_cache[u].wheel_mass;//单个轮子质量，原本是50

			PhysIKA::Vector3f wheelI = PhysIKA::RigidUtil::calculateCylinderLocalInertia(wheelm,//计算圆柱体局部惯性参数
				(wheelSize[0][1] + wheelSize[0][2]) / 2.0,
				wheelSize[0][0],
				0);
			m_car[u]->wheelMass[0] = wheelm;
			m_car[u]->wheelInertia[0] = wheelI;
			m_car[u]->wheelMass[1] = wheelm;
			m_car[u]->wheelInertia[1] = wheelI;
			m_car[u]->wheelMass[2] = wheelm;
			m_car[u]->wheelInertia[2] = wheelI;
			m_car[u]->wheelMass[3] = wheelm;
			m_car[u]->wheelInertia[3] = wheelI;

			m_car[u]->steeringLowerBound = -0.5;
			m_car[u]->steeringUpperBound = 0.5;

			m_car[u]->forwardForceAcc = car_cache[u].forward_force;
			m_car[u]->maxVel = car_cache[u].max_speed;

			m_car[u]->steeringSpeed = -1.0;//Done，这里加一行，使打舵速度不是无穷大

			// Build
			m_car[u]->build();

			// Add visualization module and topology module.
			m_car[u]->m_chassis->setTopologyModule(chassisTri);
			auto chassisRender = std::make_shared<PhysIKA::RigidMeshRender>(m_car[0]->m_chassis->getTransformationFrame());
			chassisRender->setColor(PhysIKA::Vector3f(0.8, std::rand() % 1000 / (double)1000, 0.8));
			m_car[u]->m_chassis->addVisualModule(chassisRender);
			interactionSolver->addSDF(chassisSDF, m_car[0]->m_chassis->getId());

			// Bounding radius of chassis.
			float chassisRadius = chassisTri->computeBoundingRadius();
			m_car[u]->m_chassis->setRadius(chassisRadius);
			
			m_rigids.push_back(m_car[u]->m_chassis);
			m_rigidRenders.push_back(chassisRender);

			for (int i = 0; i < 4; ++i)//这循环，给轮子，先添加可视化模块和拓扑模块，再设置底盘的边界半径
			{
				m_car[u]->m_wheels[i]->setTopologyModule(wheelTri[i]);
				auto renderModule = std::make_shared<PhysIKA::RigidMeshRender>(m_car[u]->m_wheels[i]->getTransformationFrame());
				renderModule->setColor(PhysIKA::Vector3f(0.8, std::rand() % 1000 / (double)1000, 0.8));
				m_car[u]->m_wheels[i]->addVisualModule(renderModule);
				interactionSolver->addSDF(wheelSDF[i], m_car[u]->m_wheels[i]->getId());

				// Bounding radius of chassis.
				float wheelRadius = wheelTri[i]->computeBoundingRadius();
				m_car[u]->m_wheels[i]->setRadius(wheelRadius);

				m_rigids.push_back(m_car[u]->m_wheels[i]);
				m_rigidRenders.push_back(renderModule);
			}
		}
		interactionSolver->m_prigids = &(rigidSolver->getRigidBodys());//TODO m_prigids只有一个，这可能是个问题



		return m_instance;//这个输出和本函数没关系呀？
	}

}  // namespace VPE