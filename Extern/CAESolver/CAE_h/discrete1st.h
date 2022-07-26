#pragma once
#include"cuda_runtime.h"
#include"node.h"
#include"elementtypeenum.h"
#include "materialold.h"
#include"sectionold.h"
#include"element.h"
#include"hourglasscontrol.h"

struct HourglassControl;
struct Section;
struct Material;
struct MaterialNew;
struct NodeCuda;
struct Element;

typedef struct DiscreteCuda:Element
{
	int node_id_array[3];
	NodeCuda* node_array[3];
	
	int gpu_id;

	/**
	��Ԫ����
	*/
	double fint[2][6];

	/**
	����ʸ��
	*/
	double axi_vec[3];

	/**
	��ǰ����ֵ
	*/
	double length;

	/**
	���������
	*/
	double axi_deform_rate;

	/**
	�쳤��
	*/
	double length_change;

	/**
	������
	*/
	double axi_force;

	/**
	��Ԫʱ������
	*/
	double dt_ele;

	/**
	������������
	*/
	double mass_scale_rate;

public:

	__host__ double computeDiscreteMass(Material *material,Section *section);

	/*
	*������������
	*/
	__host__ __device__ double computeDiscreteCharactLength();

	/*
	*����ʱ�䲽��
	*/
	__host__ __device__ double computeDiscreteTimeStep(Material *material, double safe_factor);

	/*
	*ָ����ǰ��Ԫ��ʱ������
	*/
	__host__ __device__ void assignTimeStep(const double dt);

	/*
	* ��Ԫ��������
	*/
	__device__ double computeDiscreteMassScaleGpu(const double min_dt);

	__host__ double computeDiscreteMassScaleCpu(const double min_dt);

	/*
	* ��Ԫ�������
	*/
	__device__ void computeDiscreteInterForceGpu(MaterialNew *material, Section *section, HourglassControl *hourglass, double dt);
	__host__ void computeDiscreteInterForceCpu(MaterialNew *material, Section *section, HourglassControl *hourglass, double dt);

	/*
	* ��Ԫ�ڵ�ƥ��
	*/
	__host__ __device__ void bulidNodeMatch(NodeCuda *nodeArraySys);

	/*
	* ��Ԫ���ɶ�Ԥ����
	*/
	__device__ void builNodeIndex();

	/*
	* ��Ԫ�󷣲���
	*/
	__host__ __device__ void computeDiscretePenaltyFactor(double factor, Material *material, Section *section);

	/**
	��Ԫ������ʼ��
	*/
	__host__ __device__ void DiscreteNodeMassInitia();

	__host__ __device__ void setElementSectionType();

	__device__ void discreteDiscrete1stInterForceToNodeGpu();

private:
	/**
	���㵥Ԫ����
	*/
	__device__ void interForce_discrete_Gpu(MaterialNew *material, Section *section, double dt);
	__host__ void interForce_discrete_Cpu(MaterialNew *material, Section *section, double dt);

	/**
	���㵥Ԫ��������
	*/
	__host__ __device__ void calAxiVec(double &du);

	/**
	��Ԫ������װ
	*/
	__host__ __device__ void assemInterForce();

	/**
	����λ�ƻ����ٶȼ�������
	*/
	__host__ __device__ void calInterForce(const MaterialNew *material, const double dt, const double du);

	/**
	��������α���
	*/
	__host__ __device__ void calAxiDeformRate(const double dt);

}DiscreteCuda;