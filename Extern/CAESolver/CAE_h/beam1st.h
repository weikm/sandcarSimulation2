#pragma once
#include"cuda_runtime.h"
#include"elementtypeenum.h"
#include"node.h"
#include "materialold.h"
#include"hourglasscontrol.h"
#include"element.h"

struct NodeCuda;
struct MaterialNew;
struct Section;
struct Material;
struct HourglassControl;
struct Element;

typedef struct BeamElementCuda:Element
{
	NodeCuda* node_array[3];
	int node_id_array[3];

	int gpu_id;

	double ini_length;
	double ini_area;
	double strs[28];

	double strain_energy;
	double total_energy;

	double area;
	double element_mass;
	double element_rot_mass[3];

	/**
	��ǰ����ֵ
	*/
	double length;

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
	���ɶ�Ԥ����������ڴ�,��ά�����һ��ָ���ʾ�ڵ㣬�ڶ���ָ���ʾ���ɶ�
	*/
	double fint[3][6];

	/**
	����������Ļ��ֵ�
	*/
	double strain_xx_i[9];
	double strain_xx_j[9];
	double strain_xy_i[9];
	double strain_xy_j[9];
	double stress_xx_i[9];
	double stress_xx_j[9];
	double stress_xy_i[9];
	double stress_xy_j[9];

	/**
	��������ϵĻ��ֵ��Ӧ�����ЧӦ��
	*/
	double ys_k_i[9];
	double ys_k_j[9];
	double pstn_i[9];
	double pstn_j[9];

	/**
	�������Ե�������
	*/
	int kount_i[9];
	int kount_j[9];

public:
	__host__ double computeBeamMass(Material *matPtr,Section *sectionPtr);

	/*
	 *������������
	 */
	__host__ __device__ double computeBeamCharactLength();

	/*
	 *����ʱ�䲽��
	 */
	__host__ __device__ double computeBeamTimeStep(MaterialNew *material, double safe_factor);

	/*
	 *ָ����ǰ��Ԫ��ʱ������
	 */
	__host__ __device__ void assignTimeStep(const double dt);

	/*
	 * ����Ԫ��������
	 */
	__device__ double computeBeamMassScaleGpu(const double min_dt);

	__host__ double computeBeamMassScaleCpu(const double min_dt);

	/*
	 * ����Ԫ�������
	 */
	__device__ void computeBeamInterForceGpu(MaterialNew *material, Section *section, HourglassControl *hourglass, double dt);
	__host__ void computeBeamInterForceCpu(MaterialNew *material, Section *section, HourglassControl *hourglass, double dt);

	/*
	 * ����Ԫ�ڵ�ƥ��
	 */
	__host__ __device__ void bulidNodeMatch(NodeCuda *nodeArraySys);

	/*
	 * ����Ԫ���ɶ�Ԥ����
	 */
	__device__ void builNodeIndex();

	/*
	 * ����Ԫ�󷣲���
	 */
	__host__ __device__ void computeBeamPenaltyFactor(double factor,Material *material,Section *section);

	/**
	��������Ԫ��Ӧ����
	*/
	__host__ __device__ void calStrainEnergy(double ddln, double rot1[], double rot2[], double f21, double oldf21, double fm[][3], double oldfm[][3]);

	/**
	��Ԫ������ʼ��
	*/
	__host__ __device__ void beamNodeMassInitia();

	/**
	���ó�ʼ����Ӧ��
	*/
	__host__ __device__ void setInitialYield(const double iniYield);

	__host__ __device__ void setElementSectionType();

	/*
	* ����Ԫ������ɢ���ڵ���
	*/
	__device__ void discreteBeam1stInterForceToNodeGpu();
	
private:

	/**
	����Ԫ����˹Ӧ��Ӧ������
	*/
	__host__ __device__ void beamMises(const MaterialNew *elmPrp, double dexx, double dexo, double &sxx, double &sxo,
		double beam_sxx[], double beam_sxo[], double beam_ys[], int beam_kount[], int ipt);

	/**
	��Ԫ�����������
	*/
	__host__  double beamMassCalCpu(const double rho, const double area, const double sm1, const double sm2, const double smt);

	/**
	����Ԫ�����������
	*/
	__host__  double springMassCalCpu(double rho, double volumeSpring_, double iner);


	/**
	���㱾������ϵ
	*/
	__host__ __device__ void calBeamLocateAxi(double e32[], double e31[], double e1[], double e2[], double e3[], double rot1[], double rot2[],
		double &aln, double dt, double e21[], double e22[]);

	/**
	find rigid body rotation and local deformed nodal values
	*/
	__host__ __device__ void brigid(double dt, double &ddln, double dcx[], double dcy[], double dcz[], double rot1[], double rot2[],
		double aln, double e21[], double e22[], double e32[], double e31[]);

	/**
	����������
	*/
	__host__ __device__ void bsBeamElasticInterForce(const MaterialNew *material, const Section *section, double &dln, double &ddln, double f[][3], double fm[][3],
		double rot1[], double rot2[], double dt, double length);

	/**
	������������
	*/
	__host__ __device__ void bsBeamPlasticInterForce(const MaterialNew *material, const Section *section, double &dln, double &ddln, double f[][3], double fm[][3],
		double rot1[], double rot2[], double dt, double length);
	__host__ __device__ void open_cross(double ddln, double rot1[3], double rot2[3], double aln, double fm[3][3], double f[3][3], double dt,
		const MaterialNew *elmPrp, const Section *section);
	__host__ __device__ void close_cross(double ddln, double rot1[3], double rot2[3], double aln, double fm[3][3], double f[3][3], double dt,
		const MaterialNew *elmPrp, const Section *section);

	/**
	����Ԫ�������
	*/
	__device__ void interForce_bsbeam_Gpu(const MaterialNew *material, const Section *section, const double dt);
	__host__ void interForce_bsbeam_Cpu(MaterialNew *material, Section *section, const double dt);

	/**
	�˵�Ԫ�������
	*/
	__device__ void interforce_truss_gpu(const MaterialNew *material, const Section *section, const double dt);

	/**
	����Ԫ�������
	*/
	__device__ void interForce_spring_Gpu(const MaterialNew *material, const Section *section, const double dt);
	__host__ void interForce_spring_Cpu(MaterialNew *material, Section *section, const double dt);
	__host__ __device__ void cal_spring_disp(double disp[3], double rot_disp[3], double e1[], double e2[], double e3[]);
	__host__ __device__ void cal_spring_for(double dt, double dcx[], double dcy[], double dcz[], double f[3][3],
		double fm[3][3], double rot1[3], double rot2[3], const MaterialNew *elmPrp);


} BeamElementCuda;
