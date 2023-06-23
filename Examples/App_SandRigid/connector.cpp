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
//Ŀǰbug��vectorԽ��
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


	struct SandSimulationRegion::Impl {//ԭ���Ǹ��඼��Ҫ�ˣ��ĳ���һ�����impl
	public:
		std::shared_ptr<PhysIKA::Node> node;
		float _sand_layer_thickness;
		float _delta;
		double _total_width_in_meter;//�ⶫ���������룿setHeight
		double _total_height_in_meter;
		float    chassisMass = 1.0;//need corrected
		int newcarnumber = 0;
		std::vector<VPE::PhysIKACarCreateInfo> car_cache;

		//bianliang of create function
		std::shared_ptr<ParticleSandRigidInteraction> root;
		std::shared_ptr<PhysIKA::PBDSolverNode> rigidSim;
		std::vector<shared_ptr<PhysIKA::PBDCar>> m_car;
		std::vector<shared_ptr<VPE::PhysIKACar>> m_PhysIKACar;//��������С����λ�˺�����λ��
		std::shared_ptr<PhysIKA::SandInteractionForceSolver> interactionSolver;
		std::shared_ptr<PhysIKA::RigidBody2<PhysIKA::DataType3f>> m_chassis;
		std::string chassisFile = "";
		PhysIKA::Vector3f chassisInertia;
		PhysIKA::Vector3f          carPosition;
		PhysIKA::Quaternion<float> carRotation;
		std::vector<PhysIKA::RigidBody2_ptr>                   m_rigids;
		std::vector<std::shared_ptr<PhysIKA::RigidMeshRender>> m_rigidRenders;

		PhysIKA::SandGridInfo sandinfo;//����
		PhysIKA::HostHeightField1d landHeight;//�߶�
		std::vector<PhysIKA::Vector3d>  particlePos;//����λ��
		shared_ptr<SandSimulationRegion>m_instance;
		std::shared_ptr<PhysIKA::PBDSolver> rigidSolver;


		std::shared_ptr<PhysIKA::Node> GetRoot()//���root�������ĸ������ࣿParticleSandRigidInteraction
		{
			return root;
		}

		//void SetHeight(double *data, int resolution_x, int resolution_y)
		//{

		//	//����ǰ������о����ó�Ա��������ɣ�������е�build��
		//	sandinfo.nx = resolution_x;
		//	sandinfo.ny = resolution_y;
		//	sandinfo.griddl = _total_width_in_meter / resolution_x;//���Ӽ�࣬Ҳ��ɳ����С��
		//	sandinfo.mu = 0.7;//�ĸĿ���,Ӧ����Ħ������������֮��//�⼸����Ӧ���Ƕ�û�õ���
		//	sandinfo.drag = 0.95;//�ĸĿ�����ק����û�ио�Ҳһ��
		//	sandinfo.slide = 10 * sandinfo.griddl;//10�ĳ�100�������仯
		//	sandinfo.sandRho = 1000.0;////�ĸĿ�����û����
		//	double sandParticleHeight = 0.05;//0.05��ɳ���Ӿ������߶ȣ�

		//	//1���Ҫ����data

		//	landHeight.resize(sandinfo.nx, sandinfo.ny);//����Ӳ�����С
		//	landHeight.setSpace(sandinfo.griddl, sandinfo.griddl);//����Ӳ����߶ȳ����񳤿�
		//	printf("%d\n%d\n", sandinfo.nx, sandinfo.ny);//256256û��
		//	printf("%d\n%d\n", landHeight.Nx(), landHeight.Ny());//256256

		//	//�����data����Ӳ����߶ȳ�����
		//	for (int i = 0; i < sandinfo.nx; ++i) {//����߶����ã�����ѭ��
		//		for (int j = 0; j < sandinfo.ny; ++j) {
		//			double aa = *data;
		//			landHeight(i, j) = aa;
		//			data++;
		//		}
		//	}
		//}

		std::shared_ptr<VPE::PhysIKACar> GetCar(uint64_t car_handle)//car_handle��ɶ��С������
		{
			if (car_handle < newcarnumber) {//�Ҷ�����һ�������������һ�������Ķ��󣬾Ͱ������������Ϊ��̬��Ȼ�����ʱ����ԭ����������ͺ���
				//car_handle = Builder->m_car.size();//��֪���ˣ���Щ�����ߵģ�����Ϊû���г����󣬶���+->�Ŷԡ�
				return m_PhysIKACar[car_handle];//һ�ʣ�����õ������������ָ���أ�����build�м���һ�����������������ָ�롣����ѽ����һ������vector���ǻ��ǹ�����.h�ǹ�ѽ
			}//m_car����PhysIKACar�࣬����Ҫ��
			else return nullptr;
		}

		std::vector< VPE::Vec3> GetSandParticles()//��������Ż��������Ϣ//VPE::Vec3*ԭ��Ҫ����������
		{
			int particle_num = particlePos.size();//������Ҳ����build������һ�����������������������Ǹ��ط�
			vector<VPE::Vec3> a;
			VPE::Vec3 a_cache;
			a_cache.x = 0;
			a_cache.y = 0;
			a_cache.z = 0;
			// TODO over
			for (int i = 0; i < particle_num; i++) {
				a.push_back(a_cache);
				a[i].x = particlePos[i][0];//�����֣���build����������һ�������ά���飬��build��������Ѷ�ά���鸳ֵ��Ȼ���������������ά���������
				a[i].y = particlePos[i][1];//also put in impl-chengyuanbianliang,, 
				a[i].z = particlePos[i][2];
			}

			return a;
		}

		//uint64_t AddCar(const VPE::PhysIKACarCreateInfo& carinfo)//���س�����ţ�Ҳ��car_vector.size
		//{
		//	newcarnumber++;
		//	car_cache.push_back(carinfo);
		//	return newcarnumber;
		//}



		std::shared_ptr<SandSimulationRegion> Init(const SandSimulationRegionCreateInfo& info);

	};//create

	std::shared_ptr<PhysIKA::Node> SandSimulationRegion::GetRoot() {//��Ϊ����impl����Ҫ����Щ
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
		// TODO ��̬�������ʲ��˳�Ա����
		auto region = std::make_shared<SandSimulationRegion>();//��cpp��SandSimulationRegion::Create

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


	//�������������������
	struct VPE::PhysIKACar::Impl2 {
	public:
		shared_ptr<PhysIKA::PBDCar> m_car;

		Vector3f wheelupDirection;//������z��������̵����λ��
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
			pos.x = m_car->carPosition[0] + m_car->chassisMeshTranslate[0];//Ҫ���ǵ��̣�����Ҫ���ϱ�ƽ��translation*��λ��������������������λ���������ȷ����
			pos.y = m_car->carPosition[1] + m_car->chassisMeshTranslate[1];
			pos.z = m_car->carPosition[2] + m_car->chassisMeshTranslate[2];

			// TODO chassis//���̽Ƕ���Դ����������ô���֣�Ҫ������������֣���Ϊ0000�����仰˵���ǲ��ں�������ݣ�����С���ٶȡ�
			//��������������õ������ݣ���λ�ã��������λ�ã�������ԽǶȣ��������λ�á�

			//�Ȱ�����λ����������������Ȼ��������÷�����ab(�ٵ�λ��)���ͳ�ʼ������(010)������õ�������ab_010(�ٵ�λ��)��
			//��ͨ������������������õ��н�����cos
			Vec3 a = { m_car->wheelRelPosition[1][0] - m_car->wheelRelPosition[0][0],m_car->wheelRelPosition[1][1] - m_car->wheelRelPosition[0][1], m_car->wheelRelPosition[1][2] - m_car->wheelRelPosition[0][2], };
			Vec3 b = { m_car->wheelRelPosition[2][0] - m_car->wheelRelPosition[0][0],m_car->wheelRelPosition[2][1] - m_car->wheelRelPosition[0][1], m_car->wheelRelPosition[2][2] - m_car->wheelRelPosition[0][2], };
			//a,b���
			Vec3 ab = { a.y*b.z - b.y*a.z ,a.z*b.x - b.z*a.x ,a.x*b.y - a.y*b.x };
			//��λ��
			ab = { static_cast <float>(ab.x / sqrt(pow(ab.x, 2) + pow(ab.y, 2) + pow(ab.z, 2)) ), static_cast <float>(ab.y / sqrt(pow(ab.x, 2) + pow(ab.y, 2) + pow(ab.z, 2)) ) , static_cast <float>(ab.z / sqrt(pow(ab.x, 2) + pow(ab.y, 2) + pow(ab.z, 2)) )};
			//ab���010
			Vec3 ab_010 = { -ab.z ,0 ,ab.x };
			//��λ�����õ��ᷨ����
			ab_010 = { static_cast <float>(ab_010.x / sqrt(pow(ab_010.x, 2) + pow(ab_010.y, 2) + pow(ab_010.z, 2)) ), static_cast <float>(ab_010.y / sqrt(pow(ab_010.x, 2) + pow(ab_010.y, 2) + pow(ab_010.z, 2)) ) , static_cast <float>(ab_010.z / sqrt(pow(ab_010.x, 2) + pow(ab_010.y, 2) + pow(ab_010.z, 2)) )};

			double cos = ab.y * 1 / sqrt(pow(ab.x, 2) + pow(ab.y, 2) + pow(ab.z, 2));
			double sin = sqrt(1 - pow(cos, 2));

			double cos0_5 = sqrt(0.5*(1 + cos));
			double sin0_5 = sqrt(0.5*(1 - cos));

			//������Ԫ����ʽ����Ԫ���Ļ�����ѧ����Ϊ : q = cos (a/2) + i(x * sin(a/2)) + j(y * sin(a/2)) + k(z * sin(a/2)) ����a��ʾ��ת�Ƕȣ�(x,y,z)��ʾ��ת�ᡣ
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

			//rot.x = m_car->wheelRelRotation[wheel_index][0];//���Ҳʼ����0��Ҫ��//�������Ӿ��ԽǶȵ������ģ����������ٶ�
			//rot.y = m_car->wheelRelRotation[wheel_index][1];
			//rot.z = m_car->wheelRelRotation[wheel_index][2];
			//rot.w = m_car->wheelRelRotation[wheel_index][3];
			//float theta = m_car->currentSteering;//����ת�ǣ�����private�����������ԽǶȣ�����010���ο�ϵ�ǵ�����ת��

			//ǰ�ֽǶ�=���̽Ƕ�+ת��
			//���ֽǶ�=���̽Ƕ�
			//���̽Ƕ��������Ǹ��������棬����һ��ͺá�
			
				Vec3 a = { m_car->wheelRelRotation[1][0] - m_car->wheelRelRotation[0][0],m_car->wheelRelRotation[1][1] - m_car->wheelRelRotation[0][1], m_car->wheelRelRotation[1][2] - m_car->wheelRelRotation[0][2], };
				Vec3 b = { m_car->wheelRelRotation[2][0] - m_car->wheelRelRotation[0][0],m_car->wheelRelRotation[2][1] - m_car->wheelRelRotation[0][1], m_car->wheelRelRotation[2][2] - m_car->wheelRelRotation[0][2], };
				//a,b���
				Vec3 ab = { a.y*b.z - b.y*a.z ,a.z*b.x - b.z*a.x ,a.x*b.y - a.y*b.x };
				//��λ��
				ab = { static_cast <float>(ab.x / sqrt(pow(ab.x, 2) + pow(ab.y, 2) + pow(ab.z, 2))) , static_cast <float>(ab.y / sqrt(pow(ab.x, 2) + pow(ab.y, 2) + pow(ab.z, 2)) ) , static_cast <float>(ab.z / sqrt(pow(ab.x, 2) + pow(ab.y, 2) + pow(ab.z, 2))) };
				//ab���010
				Vec3 ab_010 = { -ab.z ,0 ,ab.x };
				//��λ�����õ��ᷨ����
				ab_010 = { static_cast <float>(ab_010.x / sqrt(pow(ab_010.x, 2) + pow(ab_010.y, 2) + pow(ab_010.z, 2)) ), static_cast <float>(ab_010.y / sqrt(pow(ab_010.x, 2) + pow(ab_010.y, 2) + pow(ab_010.z, 2))  ), static_cast <float>(ab_010.z / sqrt(pow(ab_010.x, 2) + pow(ab_010.y, 2) + pow(ab_010.z, 2))) };

				double cos = ab.y * 1 / sqrt(pow(ab.x, 2) + pow(ab.y, 2) + pow(ab.z, 2));
				double sin = sqrt(1 - pow(cos, 2));

				double cos0_5 = sqrt(0.5*(1 + cos));
				double sin0_5 = sqrt(0.5*(1 - cos));
			
				//����д���ڳ���ת��!
				double wheel_angle=m_car->currentSteering;
				

			if (wheel_index >=2) {
				{
					//������Ԫ����ʽ����Ԫ���Ļ�����ѧ����Ϊ : q = cos (a/2) + i(x * sin(a/2)) + j(y * sin(a/2)) + k(z * sin(a/2)) ����a��ʾ��ת�Ƕȣ�(x,y,z)��ʾ��ת�ᡣ
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
			m_car->carPosition[0] = pos.x;//�Ⱥ��ұ�Ҫ��ȥƽ��
			m_car->carPosition[1] = pos.y;
			m_car->carPosition[2] = pos.z;

			//���̽Ƕ�û����ֵ���޴���ֵ�����Ը�ֵ����λ�á�
			//�������Ϊ����û�и����ɣ������ֻ��ԭ��תȦ��//��carPositionΪ���ĵ㣬�͸��ĸ���������λ�ã���ͬ�����õ��̽Ƕȡ�
			
			double cos0_5 = rot.x;
			double sin0_5 = 1-pow(cos0_5,2);
			//ƫת��������������
			double sin = 2 * cos0_5*sin0_5;
			double cos = pow(cos0_5, 2) - pow(sin0_5, 2);
			//0�ų��ֵ���ת
			Vec3 a;
			a.x = m_car->wheelRelPosition[0][0] - m_car->carPosition[0];
			a.y = 0;
			a.z = m_car->wheelRelPosition[0][2] - m_car->carPosition[2];
			//0�ų��ֵ����������Ⱥͼн�
			double length = pow(pow(a.x, 2) + pow(a.z, 2), 0.5);
			double cos_ = a.x / length;
			double sin_ = pow(1-pow(cos_,2), 0.5);
			//�������ת��Ƕ�
			double final_cos = cos_ * cos - sin_ * sin;
			double final_sin = cos_ * sin + sin_ * cos;

			m_car->wheelRelPosition[0][0] = length * final_cos;
			m_car->wheelRelPosition[0][2] = length * final_sin;

			//1�ų��ֵ���ת
			//Vec3 a;
			a.x = m_car->wheelRelPosition[1][0] - m_car->carPosition[0];
			a.y = 0;
			a.z = m_car->wheelRelPosition[1][2] - m_car->carPosition[2];
			//0�ų��ֵ����������Ⱥͼн�
			double length1 = pow(pow(a.x, 2) + pow(a.z, 2), 0.5);
			double cos_1 = a.x / length1;
			double sin_1 = pow(1 - pow(cos_, 2), 0.5);
			//�������ת��Ƕ�
			double final_cos1 = cos_1 * cos - sin_1 * sin;
			double final_sin1 = cos_1 * sin + sin_1 * cos;

			m_car->wheelRelPosition[1][0] = length1 * final_cos1;
			m_car->wheelRelPosition[1][2] = length1 * final_sin1;


			//2�ų��ֵ���ת
			//Vec3 a;
			a.x = m_car->wheelRelPosition[2][0] - m_car->carPosition[0];
			a.y = 0;
			a.z = m_car->wheelRelPosition[2][2] - m_car->carPosition[2];
			//0�ų��ֵ����������Ⱥͼн�
			double length2 = pow(pow(a.x, 2) + pow(a.z, 2), 0.5);
			double cos_2 = a.x / length2;
			double sin_2 = pow(1 - pow(cos_2, 2), 0.5);
			//�������ת��Ƕ�
			double final_cos2 = cos_2 * cos - sin_2 * sin;
			double final_sin2 = cos_2 * sin + sin_2 * cos;

			m_car->wheelRelPosition[2][0] = length2 * final_cos;
			m_car->wheelRelPosition[2][2] = length2 * final_sin;


			//3�ų��ֵ���ת
			//Vec3 a;
			a.x = m_car->wheelRelPosition[3][0] - m_car->carPosition[0];
			a.y = 0;
			a.z = m_car->wheelRelPosition[3][2] - m_car->carPosition[2];
			//0�ų��ֵ����������Ⱥͼн�
			double length3 = pow(pow(a.x, 2) + pow(a.z, 2), 0.5);
			double cos_3 = a.x / length3;
			double sin_3 = pow(1 - pow(cos_3, 2), 0.5);
			//�������ת��Ƕ�
			double final_cos3 = cos_3 * cos - sin_3 * sin;
			double final_sin3 = cos_3 * sin + sin_3 * cos;

			m_car->wheelRelPosition[3][0] = length3 * final_cos3;
			m_car->wheelRelPosition[3][2] = length3 * final_sin3;
		}

		void SetWheelPositionRotation(uint32_t wheel_index, const Vec3& pos, const Quat& rot)
		{//���ĸ����Ǿ��Ե�
		//set chassis height
		//TODO

			//����λ��
			m_car->wheelRelPosition[wheel_index][0] = pos.x - m_car->carPosition[0];
			m_car->wheelRelPosition[wheel_index][1] = pos.y - m_car->carPosition[1];
			m_car->wheelRelPosition[wheel_index][2] = pos.z - m_car->carPosition[2];

			
			//���ӽǶȸ�ֵ���֣�����һ������
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


	inline std::shared_ptr<SandSimulationRegion> SandSimulationRegion::Impl::Init(const SandSimulationRegionCreateInfo & info)//info��һϵ�в���������ȥ����û�У����������������ܶ�û�кϲ���ȥ��
	{
		//setHeight

		//����ǰ������о����ó�Ա��������ɣ�������е�build��
		sandinfo.nx = info.height_resolution_x;
		sandinfo.ny = info.height_resolution_y;
		sandinfo.griddl = info.total_width_in_meter / info.height_resolution_x;//���Ӽ�࣬Ҳ��ɳ����С��
		sandinfo.mu = 0.7;//�ĸĿ���,Ӧ����Ħ������������֮��//�⼸����Ӧ���Ƕ�û�õ���
		sandinfo.drag = 0.95;//�ĸĿ�����ק����û�ио�Ҳһ��
		sandinfo.slide = 10 * sandinfo.griddl;//10�ĳ�100�������仯
		sandinfo.sandRho = 1000.0;////�ĸĿ�����û����
		double sandParticleHeight = 0.05;//0.05��ɳ���Ӿ������߶ȣ�

										 //1���Ҫ����data

		landHeight.resize(sandinfo.nx, sandinfo.ny);//����Ӳ�����С
		landHeight.setSpace(sandinfo.griddl, sandinfo.griddl);//����Ӳ����߶ȳ����񳤿�
		//printf("%d\n%d\n", sandinfo.nx, sandinfo.ny);//256256û��
		//printf("%d\n%d\n", landHeight.Nx(), landHeight.Ny());//256256

															 //�����data����Ӳ����߶ȳ�����
		for (int i = 0; i < sandinfo.nx; ++i) {//����߶����ã�����ѭ��
			for (int j = 0; j < sandinfo.ny; ++j) {
				double aa = info.height_data[i * sandinfo.nx + j];
				landHeight(i, j) = aa;
				//(info.height_data)++;//���ﱨ��
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
		root = scene.createNewScene<ParticleSandRigidInteraction>();//���г����⣬������δ��������쳣����ȡ����Ȩ�޳�ͻ
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

		//printf("%f\n%f", landHeight.Nx(), landHeight.Ny());//��0��֪���ˣ�landHeight��set���������ʼ����ʵ���ǳ�ʼ���ɹ�֮��û�ܴ�������
														   //�ϲ������ṹ�����崫��֮������զ����0��0�أ�����Ϊ�ϲ�����֮�����Ǻϲ����������ˡ�������

		psandSolver->setLand(landHeight);//cy:���������bug  wkm:��Ϊû�г�ʼ��landHeight����Ҫ����ǰ�ĺ����������ڵ��˻��Ǳ���������û���ɹ���

										 // TODO ��Ⱦȫ��ɾ��
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

			// Mesh renderer.������Ⱦ��Ϳ��ɫ
			auto renderModule = std::make_shared<PhysIKA::RigidMeshRender>(landrigid->getTransformationFrame());
			renderModule->setColor(PhysIKA::Vector3f(210.0 / 255.0, 180.0 / 255.0, 140.0 / 255.0));
			landrigid->addVisualModule(renderModule);
		}

		// Sand height 5ɳ�Ӹ߶ȳ�����δʵ�����ã�
		//std::vector<int>  humpBlock = { 31 - 10, 31 + 9, 31 - 8, 31 + 8 };
		//���ø߶ȳ�����
		HostHeightField1d sandHeight;//�õ���CPU//md������Ҳ�ܳ���
		sandHeight.resize(sandinfo.nx, sandinfo.ny);
		psandSolver->setHeight(sandHeight);

		// 6 Sand particles.

		std::vector<PhysIKA::ParticleType>    particleType;
		std::vector<double>                   particleMass;//��������
		std::default_random_engine            e(31);//�����������,31�����������
		std::uniform_real_distribution<float> u(-0.3, 0.3);//���ȷֲ���ģ�壬u�Ǹ����ȷֲ�

														   // 7 Sand plane.
		double spacing = sandinfo.griddl / 2.0;//��񳤶�
		double m0 = sandParticleHeight * sandinfo.sandRho * spacing * spacing;//50*0.015^2//�ܶ�����������㵥��ɳ������
		double mb = m0 * 5;
		double spacing2 = spacing;

		//8ɳ������Ϣ��ʼ����λ����������
		for (int i = sandinfo.nx/2/*0*/; i < sandinfo.nx; ++i) {
			for (int j = 0; j < sandinfo.ny; ++j) {
				if (landHeight(i, j) < info.sand_layer_thickness)
				{
					PhysIKA::Vector3d centerij = landHeight.gridCenterPosition(i, j);//�����Ȱ�Ӳ����������Ϊɳ������
					for (int ii = 0; ii < 2; ++ii)
						for (int jj = 0; jj < 2; ++jj)
						{
							PhysIKA::Vector3d curp = centerij;//ɳ������
							curp[0] -= sandinfo.griddl / 2.0;//�����е���һ��ɳ������
							curp[2] -= sandinfo.griddl / 2.0;
							curp[0] += ii * spacing2 + spacing2 / 2.0 * (1.0 + u(e));//�����õ����ȷֲ�����;���о�
							curp[2] += jj * spacing2 + spacing2 / 2.0 * (1.0 + u(e));
							particlePos.push_back(curp);//push_back������vectorĩβ�ټ�һ��Ԫ�أ�������˫��ѭ���൱�ڸ�vector��ֵ
							particleType.push_back(PhysIKA::ParticleType::SAND);
							particleMass.push_back(m0);//����vector������С���Ǹ߶ȳ���������
						}
				}
				//printf("%d\n",particlePos.size());
			}
		}

		std::vector<PhysIKA::Vector3d> particleVel(particlePos.size());
		psandSolver->setParticles(&particlePos[0], &particleVel[0], &particleMass[0], &particleType[0], particlePos.size(), sandinfo.sandRho, m0, sandinfo.griddl * 1.0, 0.85, sandParticleHeight * 0.5);
		//���Ǹ��յ�vector���ѵ�sandinfo.nx��0��
		std::cout << "Particle number:   " << particlePos.size() << std::endl;

		// 10 Rendering module of simulator.��Ⱦģ�鲻Ҫ
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
		// Car.14С���ڲ���������
		double   scale1d = 1.;
		PhysIKA::Vector3d scale3d(scale1d, scale1d, scale1d);//û�õ�
		PhysIKA::Vector3f scale3f(scale1d, scale1d, scale1d);//(1,1,1)//����������

		PhysIKA::Vector3f chassisCenter;//��ά���飬��ʼ����000
		PhysIKA::Vector3f wheelCenter[4];
		PhysIKA::Vector3f chassisSize;
		PhysIKA::Vector3f wheelSize[4];

		std::shared_ptr<PhysIKA::TriangleSet<PhysIKA::DataType3f>> chassisTri;//���̺����Ӷ�����Ϊ��������
		std::shared_ptr<PhysIKA::TriangleSet<PhysIKA::DataType3f>> wheelTri[4];

		PhysIKA::DistanceField3D<PhysIKA::DataType3f> chassisSDF;
		PhysIKA::DistanceField3D<PhysIKA::DataType3f> wheelSDF[4];

		interactionSolver = root->getInteractionSolver();//ParticleSandRigidInteraction�����

														 // Load car mesh.15������̺����ӵ������SDF
		{
			// Chassis mesh.��������Ͱ�Χ��
			PhysIKA::ObjFileLoader chassisLoader("../../Media/car2/chassis_cube.obj");//��������ļ�

			chassisTri = std::make_shared<PhysIKA::TriangleSet<PhysIKA::DataType3f>>();//������������������
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
												  // TODO ��Щ·����info����model_path
			chassisSDF.loadSDF("../../Media/car2/chassis_cube.sdf");//�������SDF�ļ�carinfo.chassis.sdf_path//�����������Ǻܷ��㣬carinfo����create���βΣ�����д������
			chassisSDF.scale(scale1d);
			chassisSDF.translate(-chassisCenter);
			//interactionSolver->addSDF(sdf);

			for (int i = 0; i < 4; ++i)//�ĸ�����
			{
				string objfile("../../Media/car2/wheel.obj");
				string sdffile("../../Media/car2/wheel.sdf");

				// Wheel mesh.��������������
				PhysIKA::ObjFileLoader wheelLoader(objfile);
				wheelTri[i] = std::make_shared<PhysIKA::TriangleSet<PhysIKA::DataType3f>>();
				wheelTri[i]->setPoints(wheelLoader.getVertexList());//��������ɶ��
				wheelTri[i]->setTriangles(wheelLoader.getFaceList());
				computeBoundingBox(wheelCenter[i], wheelSize[i], wheelLoader.getVertexList());//�����Χ�У�����
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
		{//ÿ��ѭ����Ӧ�����m_car[u]
		 //��һ�����Ļ���newcarnumber��1����һ��ѭ����
			m_car.push_back(std::make_shared<PhysIKA::PBDCar>());
			m_PhysIKACar.push_back(std::make_shared<VPE::PhysIKACar>());
			m_PhysIKACar.back()->_impl2->m_car = m_car.back();

			// TODO over
			//������Щ��newcarnumber��Ӧ��Ҳ��ѭ���
			rigidSim->addChild(m_car[u]);
			m_car[u]->m_rigidSolver = rigidSolver;


			m_car[u]->carPosition = PhysIKA::Vector3f(car_cache[u].carPosition.x, car_cache[u].carPosition.y, car_cache[u].carPosition.z) + chassisCenter;//���ó��ӳ�ʼλ�ã���������

																																						  //�ĸ����ӵ����λ�ú������ת��Ҫճ�ڽӿں���GetWheelPositionRotation��
																																						  //������Щʹ����Ҫͨ����һ���ำֵ�ģ����������ڵ�д��,��vector carcache[newcarnumber]���ű��ഫ��ȥ������
			m_car[u]->wheelRelPosition[0] = PhysIKA::Vector3f(car_cache[u].wheels[0].translation.x, car_cache[u].wheels[0].translation.y, car_cache[u].wheels[0].translation.z) * scale1d + wheelCenter[0] - chassisCenter;//(-0.3f, -0.2, -0.4f)
			m_car[u]->wheelRelPosition[1] = PhysIKA::Vector3f(car_cache[u].wheels[1].translation.x, car_cache[u].wheels[1].translation.y, car_cache[u].wheels[1].translation.z) * scale1d + wheelCenter[1] - chassisCenter;//(+0.3f, -0.2, -0.4f)
			m_car[u]->wheelRelPosition[2] = PhysIKA::Vector3f(car_cache[u].wheels[2].translation.x, car_cache[u].wheels[2].translation.y, car_cache[u].wheels[2].translation.z) * scale1d + wheelCenter[2] - chassisCenter;//(-0.3f, -0.2,  0.4f)
			m_car[u]->wheelRelPosition[3] = PhysIKA::Vector3f(car_cache[u].wheels[3].translation.x, car_cache[u].wheels[3].translation.y, car_cache[u].wheels[3].translation.z) * scale1d + wheelCenter[3] - chassisCenter;//(+0.3f, -0.2,  0.4f)
			m_car[u]->wheelRelRotation[0] = PhysIKA::Quaternion<float>(car_cache[u].wheels[0].RelRotation.x, car_cache[u].wheels[0].RelRotation.y, car_cache[u].wheels[0].RelRotation.z, car_cache[u].wheels[0].RelRotation.w);  //(0, 0, 0, 1);
			m_car[u]->wheelRelRotation[1] = PhysIKA::Quaternion<float>(car_cache[u].wheels[1].RelRotation.x, car_cache[u].wheels[1].RelRotation.y, car_cache[u].wheels[1].RelRotation.z, car_cache[u].wheels[1].RelRotation.w);
			m_car[u]->wheelRelRotation[2] = PhysIKA::Quaternion<float>(car_cache[u].wheels[2].RelRotation.x, car_cache[u].wheels[2].RelRotation.y, car_cache[u].wheels[2].RelRotation.z, car_cache[u].wheels[2].RelRotation.w);
			m_car[u]->wheelRelRotation[3] = PhysIKA::Quaternion<float>(car_cache[u].wheels[3].RelRotation.x, car_cache[u].wheels[3].RelRotation.y, car_cache[u].wheels[3].RelRotation.z, car_cache[u].wheels[3].RelRotation.w);  //car_cache[newcarnumber].wheels[3].RelRotation
																																																								 //û�е��̵�λ�ˣ��������������У������˵��̺����ӵ����λ�ã�����ƽ��
			m_car[u]->wheelupDirection = PhysIKA::Vector3f(0, 0.5, 0);/*car_cache[u].wheels[0].translation.x, car_cache[u].wheels[0].translation.y, car_cache[u].wheels[0].translation.z*///������z�᷽���Ϻ͵��̵����λ�ã�ԭ����(0, 1, 0)(0, 0.5, 0)(0, 0.25, 0)//��ע����������ݾͻᵼ�³��ֵ������ô������ԭ��δ֪
			m_car[u]->wheelRightDirection = PhysIKA::Vector3f(1, 0, 0);//ɶ��˼������ȥ��֮�󳵾Ͷ�������

			m_car[u]->chassisMass = car_cache[u].car_mass;  //���õ�������
			m_car[u]->chassisInertia = PhysIKA::RigidUtil::calculateCubeLocalInertia(m_car[u]->chassisMass, chassisSize);//������̹��Բ�����

			float wheelm = car_cache[u].wheel_mass;//��������������ԭ����50

			PhysIKA::Vector3f wheelI = PhysIKA::RigidUtil::calculateCylinderLocalInertia(wheelm,//����Բ����ֲ����Բ���
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

			m_car[u]->steeringSpeed = -1.0;//Done�������һ�У�ʹ����ٶȲ��������

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

			for (int i = 0; i < 4; ++i)//��ѭ���������ӣ�����ӿ��ӻ�ģ�������ģ�飬�����õ��̵ı߽�뾶
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
		interactionSolver->m_prigids = &(rigidSolver->getRigidBodys());//TODO m_prigidsֻ��һ����������Ǹ�����



		return m_instance;//�������ͱ�����û��ϵѽ��
	}

}  // namespace VPE