#pragma once
#include"cuda_runtime.h"
#include"node.h"
#include"elementtypeenum.h"
#include"materialold.h"
#include"sectionold.h"
#include"hourglasscontrol.h"
#include"element.h"

struct NodeCuda;
struct Element;

typedef struct ShellElementCuda:Element
{

	int node_id_array[4];
	NodeCuda* node_array[4];
	
	int gpu_id;

	/**
	���ɶ�Ԥ����������ڴ棬��ά�����һ��ָ���ʾ�ڵ㣬�ڶ���ָ���ʾ���ɶ�
	*/
	double fint[4][6];

	double ini_area;
	double area;
	double ini_thick;
	double thick;

	double strain_energy;
	double total_energy;

	/**
	��Ԫʱ������
	*/
	double dt_ele;

	/**
	��Ԫ��������
	*/
	double charact_length;

	/**
	��������ϵ��
	*/
	double mass_scale_rate;

	/**
	��������ת����
	*/
	double element_mass;
	double element_rot_mass[3];

	/**
	ɳ©����
	*/
	double qs[5];

public:

	/*
	 * �ǵ�Ԫ�������
	 */
	__host__ double computeShellMass(Material *matPtr, Section *sectionPtr);

	/**
	ʱ�䲽�������ֵ���
	*/
	__host__ __device__ double computeShellCharactLength();

	__host__ __device__ double computeShellTimeStep(MaterialNew *material, double safe_factor);

	/**
	ָ����ǰ��Ԫ��ʱ������
	*/
	__host__ __device__ void assignTimeStep(const double dt);

	/**
	�ǵ�Ԫ��������
	*/
	__device__ double computeShellMassScaleGpu(const double min_dt);
	__host__ double computeShellMassScaleCpu(const double min_dt);

	/*
	* �ǵ�Ԫ�������
	*/
	__device__ void computeShellInterForceGpu(const MaterialNew *material, Section *section, HourglassControl *hourglass, double dt);
	__host__ void computeShellInterForceCpu(const MaterialNew *material, Section *section, HourglassControl *hourglass, double dt);

	/**
	�ڵ�ƥ��
	*/
	__host__ __device__ void bulidNodeMatch(NodeCuda *node_array_gpu);

	/**
	����������Ϣ
	*/
	__device__ void builNodeIndex();

	/**
	���������
	*/
	__host__ __device__ void computeShellPenaltyFactor(const double penalty_scale_factor, Material *material, Section *section);

	/**
	��Ԫ������ʼ��
	*/
	__host__ __device__ void shellNodeMassInitia();

	/**
	��ⵥԪ�����
	*/
	__host__ __device__ double calElementArea();

	__host__ __device__ void setElementSectionType();

	/*
	* ��ɢ��Ԫ�������ڵ���
	*/
	__device__ void discreteShell1stInterForceToNodeGpu();

	/*
	ls:2020-07-30 added
	*/
	__host__ __device__ void applyUniformNormLoadToNode(const double loadValue);

	/*
	* �������
	*/
	__host__ __device__ void computeArea(double &ar);

private:
	/**
	�ǵ�Ԫ�������
	*/
	template<typename anyType>
	__device__ void interForce_BT_Gpu(const MaterialNew *material, Section *section, const double dt);
	__host__ void interForce_BT_Cpu(const MaterialNew *material, Section *section, const double dt);


	__device__ void interForce_HL_Gpu(const MaterialNew *material, const Section *section, const double dt);

	__device__ void interForce_S3_Gpu(const MaterialNew *material, const Section *section, const double dt);
	__host__ void interForce_S3_Cpu(const MaterialNew *material, Section *section, const double dt);


	/**
	Ĥ��Ԫ�������
	*/
	__device__ void interForce_M4_gpu(const MaterialNew *material, const Section *section, const double dt);
	__device__ void interForce_M3_gpu(const MaterialNew *material, const Section *section, const double dt);

	/**
	���㱾������ϵ
	*/
	__host__ __device__ void calTriLocateAxi(double xn[3], double yn[3], double zn[3]);
	__host__ __device__ void calQuadLocateAxi(double xn[3], double yn[3], double zn[3]);

	/**
	�����οǵ�ԪB�������
	*/
	__host__ __device__ void computeBMatrixCPDSG(const double xcoord[3], const double ycoord[3],
		double bm[3][18], double bb[3][18], double bs[2][18]);
	__host__ __device__ void computeBMatrixDSG(double bm[3][18], double bb[3][18], double bs[2][18],
		double convert_coord[3][3]);

	/*
	 * ���ڵ����Ľڵ㵥Ԫ�����������
	 */
	__host__ __device__ double computeQuad1stCharactLength();
	__host__ __device__ double computeTria1stCharactLength();

	/**
	bt������Ӧ�����
	*/
	/*__host__ __device__ void bmain(const int ipt, const double d1[6], const Material *material);*/
	/**
	vectorized routine to find lambda which satisfies the plane stress
	plastic consistency condition using an exact analytical solution
	(only valid for cases not involving isotropic hardening)
	the procedures used in this routine are quite sensitive to machine
	precision, esp.in intrinisic functions.performance should be
	verified before use on non - cray hardware.
	*/
	/*__host__ __device__ void lametx(double eta[], double r, double scle, double alfaht1, double betaht1, double ym, double &xlamkp1);*/

	/**
	�����ε�Ԫת���������
	*/
	__host__ __device__ void AreaTransMatrixCal_S3();
	__host__ __device__ void transform_S3(double xn_1[3][3],double cos_matrix[3][3],double convert_coord[3][3]);

	/**
	����Ӧ����
	*/
	__host__ __device__ void calStrainEnergy();

	/**
	���ó�ʼ���
	*/
	__host__ __device__ void setThick(Section *section);

} ShellElementCuda;