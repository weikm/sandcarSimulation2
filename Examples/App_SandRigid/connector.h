#pragma once

#include <memory>
#include <string>
#include <vector>
#include <random>
#include <iostream>




namespace PhysIKA {
	class Node;
}

// DO NOT include ANY physIKA or ViWo headers in this file.
// The encoding of CUDA headers and ViWo files is different, which messes up everything.
namespace VPE {
	// Bridge for primitive values
	struct Vec3 {
		float x, y, z;
	};

	struct Quat {
		float x, y, z, w;
	};

	enum class PhysIKACarType {
		FourWheel,
		EightWheel
	};

	struct PhysIKACarComponent {//��Ӧ
		Vec3 translation{};//�������ڳ����ĵ�ƽ��
		Vec3 scale{ 1.0f, 1.0f, 1.0f };
		Quat RelRotation{ 0.0f, 0.0f, 0.0f, 1.0f };
		std::string model_path{};//ֻ�ܷ�����������ṹ������
		std::string sdf_path{};
	};

	struct PhysIKACarCreateInfo {
		PhysIKACarType type;

		Vec3 carPosition;
		PhysIKACarComponent chassis;
		//Vec3 chassis_box_min{};
		//Vec3 chassis_box_max{};
		// Wheel translation and rotation is relative to chassis, scale and rotation is absolute.
		PhysIKACarComponent wheels[4]{};
		float wheel_cylinder_radius = 1;
		//float wheel_cylinder_height{};

		float car_mass = 1000.0;
		float max_speed = 20.0;
		float forward_force = 2000.0;//ǣ�����㶨�ͺ�
		float steering_speed = 1.0;//�����ٶ�
		float steering_lower = -0.5;
		float steering_upper = 0.5;
		float wheel_mass = 50.0;
		Vec3 up_dir = { 1, 0, 0 };
		Vec3 right_dir = { 0, 1, 0 };
		float suspension_length = 0;
		float suspension_strength = 0;
		float linear_damping = 0.2f;
		float angular_damping = 0.2f;
		uint32_t chassis_collision_group = 0;
		uint32_t chassis_collision_mask = {};
		uint32_t wheel_collision_group = 0;
		uint32_t wheel_collision_mask = {};
	};

	enum class PhysIKACarDirection {
		Forward,
		Backward,
		Left,
		Right
	};


	class PhysIKACar {//All function need writing.
	public:
		PhysIKACar();
		~PhysIKACar();
		//�����ĸ����Ǿ��Ե�
		//wheel_index�������룬��&�����ձ������������const//����ڲ�����
		void GetChassisPositionRotation(Vec3& pos, Quat& rot);
		void GetWheelPositionRotation(uint32_t wheel_index, Vec3& pos, Quat& rot);

		void SetChassisPositionRotation(const Vec3& pos, const Quat& rot);//�Ҿ��ã�set���־�set carPosition�����ˡ�
		void SetWheelPositionRotation(uint32_t wheel_index, const Vec3& pos, const Quat& rot);

		// The center of Sand simulation region should also be updated by this method.
		// The solution and physical size of the simulation region is defined upfront.

		// Car controls
		void Go(PhysIKACarDirection dir);

		//private://wkm copy
		struct Impl2;

		std::unique_ptr<Impl2> _impl2{};
	};



	struct SandSimulationRegionCreateInfo {//��һ�ѣ�����û�д���create����ȥ��������
		double total_width_in_meter;
		double total_height_in_meter;
		float sand_layer_thickness;
		float time_delta;
		// ground height
		double* height_data;
		int height_resolution_x;
		int height_resolution_y;
		std::vector<VPE::PhysIKACarCreateInfo> cars{};
		//int newcarnumber;
	};

	class SandSimulationRegion
	{
	public:
		SandSimulationRegion();
		std::shared_ptr<PhysIKA::Node> GetRoot();
		//���ýڵ��root��shearedptr�����
		//void SetHeight(double *data, int resolution_x, int resolution_y);
		//input height_data
		std::shared_ptr<VPE::PhysIKACar> GetCar(uint64_t car_handle);//car_handle��ɶ�����
		//����С����ţ����С��ptr
		std::vector<VPE::Vec3> GetSandParticles();//��������Ż��������Ϣ
			//���ɳ��λ�����飬particle_numҲ�����
		~SandSimulationRegion();
		//uint64_t AddCar(const VPE::PhysIKACarCreateInfo& carinfo);
		static std::shared_ptr<SandSimulationRegion> Create(const SandSimulationRegionCreateInfo& info);
		//���ƹ��캯������������
		//private://ldj
		struct Impl;
		std::unique_ptr<Impl> _impl;


		// std::shared_ptr<SandSimulationRegion>region;




	};


}// namespace VPE