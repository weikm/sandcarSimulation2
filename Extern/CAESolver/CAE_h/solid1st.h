#pragma once
#include"cuda_runtime.h"
#include"node.h"
#include"elementtypeenum.h"
#include"element.h"
#include"materialnew.h"
#include"sectionold.h"
#include"hourglasscontrol.h"

struct HourglassControl;
struct Section;
struct NodeCuda;
struct Element;

typedef struct SolidElementCuda:Element
{
	NodeCuda* node_array[8];
	int node_id_array[8];

	int gpu_id;

	/**
	���ɶ�Ԥ����������ڴ棬��ά�����һ��ָ���ʾ�ڵ㣬�ڶ���ָ���ʾ���ɶ�
	*/
	double fint[8][3];

	/**
	����Ӧ������ЧӦ��
	*/
	/*double ys_k;
	double pstn;*/

	double strain_energy;
	double total_energy;

	/**
	ƽ����������ת����
	*/
	double element_mass;
	double element_rot_mass[3];

	/**
	��Ԫ���
	*/
	double volume;

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
	ɳ©���Ʋ���
	*/
	double hg_val[3][4];

	int additionalVariableNum;

	double *additionalVariablePtr;

public:

	/*
	 * �ͷ��ڱ����ڴ�
	 */
	__host__ __device__ void freeInterDofMem();

	/*
	 * �����ڱ�����ռ�õ��ڴ�
	 */
	__host__ __device__ void allocateInterDofMem();

	/*
	 * ���㵥Ԫ���
	 */
	__host__ __device__ double calSolid1stVolume();

	/*
	 * ��ʱ�����֮ǰ����TL���㵥Ԫ��B����
	 */
	__host__ __device__ void compteBMatrixTLInElement(const int gaussNum);

	/*
	 * ������ֵ���ſ˱Ⱦ���
	 */
	__host__ __device__ void computeHexa1stGaussJacobMat(double3 lcoords, double jacobMat[3][3])const;
	__host__ __device__ void computePent1stGaussJacobMat(double3 lcoords, double jacobMat[3][3])const;

	/*
	 * ������ֵ���κ���ƫ��
	 */
	__host__ __device__ void calHexa1stGaussShapeDerivative(double3 lcoords, double shpkde[3][8],double &vol)const;
	__host__ __device__ void calPent1stGaussShapeDerivative(double3 lcoords, double shpkde[3][6],double &vol)const;

	/*
	 * ������ֵ����Ȼ�����κ���ƫ��
	 */
	__host__ __device__ void calHexa1stGaussLocalDerivative(double3 lcoords, double dN[8][3])const;
	__host__ __device__ void calPent1stGaussLocalDerivative(double3 lcoords, double dN[6][3])const;

	/*
	 * ������ֵ��������������
	 */
	__host__ __device__ void computeHexa1stGaussVolume(double3 lcoords, double &vol)const;
	__host__ __device__ void computePent1stGaussVolume(double3 lcoords, double &vol)const;

	/*
	 * ��������
	 */
	__device__ void computeSolidInterForceGpu(MaterialNew *material, Section *section, HourglassControl *hourglass, double dt);
	__host__ void computeSolidInterForceCpu(MaterialNew *material, Section *section, HourglassControl *hourglass, double dt);

	/*
	 * ʱ�䲽������
	 */
	//�ṩ����ʵ�嵥Ԫ������������
	//һΪȡ��Ԫ���������ڵ����̾���
	//��Ϊȡ��Ԫ����̵ĸ�
	__host__ __device__ double computeSolidCharactLength();

	__host__ __device__ double computeHexa1stCharactLength();

	__host__ __device__ double computeTetr1stCharactLength();

	__host__ __device__ double computePent1stRCharactLength();

	__host__ __device__ double computePent1stCharactLength();
	
	__host__ __device__ double computePyrd1stCharactLength();

	__host__ __device__ double computeSolidTimeStep(MaterialNew *material, double safe_factor);

	__host__ __device__ void assignTimeStep(const double dt);

	/*
	 * ��������
	 */
	__host__ __device__ void solidNodeMassInitia();

	__device__ double computeSolidMassScaleGpu(const double min_dt);

	__host__ double computeSolidMassScaleCpu(const double min_dt);

	/*
	 * ��������
	 */
	__host__ __device__ void bulidNodeMatch(NodeCuda *nodeArraySys);

	__device__ void builNodeIndex();

	/*
	 * ��Ԫ��������
	 */
	__host__ double computeSolidMass(Material *matPtr, Section *sectionPtr);
	
	/*
	* ʵ�嵥Ԫ�󷣲���
	*/
	__host__ __device__ void computeSolidPenaltyFactor(double factor, Material *material, Section *section);

	/*
	 * ����ڵ�Ӧ��Ӧ��
	 */
	__device__ void computeSolid1stNodeStrGpu(const int gaussNum);
	__device__ void computeHexa1stNodeStressStrainGpu(const int gaussNum);
	__device__ void computeTetr1stNodeStressStrainGpu(const int gaussNum);
	__device__ void computePena1stNodeStressStrainGpu(const int gaussNum);


	/**
	��������嵥Ԫ���
	*/
	__host__ __device__ double calTetraVol()const;

	/**
	��������嵥Ԫ���
	*/
	__host__ __device__ double calHexVol()const;

	/**
	��������嵥Ԫ���
	*/
	__host__ __device__ double calPenaRVol()const;

	__host__ __device__ double calPenaVol()const;

	/*
	 * ����������Ԫ�����
	 */
	__host__ __device__ double calPyrdVol()const;

	__host__ __device__ void setElementSectionType();

	__host__ void setSFemFlag(SmoothFEMType st=SNull);

	__device__ void discreteSolid1stInterForceToNodeGpu();

	__host__ void discreteSolid1stInterForceToNodeCpu();

private:

	__host__ __device__ void computeGaussStressToInterForce8Node(GaussPoint *gp,double bMat[][8],int ig);

	__host__ __device__ void computeGaussStressToInterForce6Node(GaussPoint *gp, double bMat[][6],int ig);
	
	//ʵ�嵥Ԫ���࣬������
	/*__host__ __device__ void classificElmType(const Section *section);*/

	/*
	 * �����ڲ����ɶȱ���
	 */
	__host__ __device__ void computeImplicitAlpha(int numInterDof,double *kaa,double *kau,double *alpha,double *fe,double dt);

	/**
	ʵ�嵥Ԫ�������
	*/
	//TL
	__device__ void interForceTetr1stGpuTL(const MaterialNew *material, const double dt);

	__host__ void interForceTetr1stCpuTL(const MaterialNew *material, const double dt);

	__host__ __device__ void calBMatrixTetra1st(GaussPoint *gp, double* bMat[3]);

	__host__ __device__ void calInterForTetr1stTL(GaussPoint *gp, double* bMat[3], const MaterialNew *mat, double dt);

	__device__ void interForcePyramid1stGpuTL(const MaterialNew *material, const Section* section, double dt, HourglassControl *hourControl);

	__host__ __device__ void calBMatrixPyramid1st(GaussPoint* gp, double *bMat[3],int gpFlag);

	/*__host__ __device__ void calBMatrixPyramid1st(GaussPoint* gp, double* bMat[3], const int gpFlag,
		const double3 nodeSet[], const int faceIdSet[][5], const int subDomainIdSet[][5], const double phi0[][7], const int numb[]);*/

	__host__ __device__ void calInterForPyramid1stTL(GaussPoint *gp, double* bMat[3], const MaterialNew *mat, double dt);
	///__device__ void interForceTetra1stGpu(const MaterialNew *material, const double dt);
	///__host__ void interForceTetra1stCpu(MaterialNew *material, const double dt);

	//UL
	__host__ __device__ void interForceHexa1stIGpu(const MaterialNew* material, const Section* section, double dt, HourglassControl* hourControl);
	__host__ __device__ void calBMatrixHexa1stI(GaussPoint *gp, double bMat[3][8]);
	__host__ __device__ void calGMatrixHexa1stI(GaussPoint *gp, double gMat[9][6], int numInterNum);
	__host__ __device__ void calInterForHexa1stI(GaussPoint *gp, const MaterialNew *material,
		double bhMat[3][8], double G[9][6], double* alphaIncre, double fE[], const double dt, int numInterDof, int loopNum);

	//��������������
	__host__ __device__ void interForceHexa1stRGpu(const MaterialNew *material, double dt, HourglassControl *hourControl);
	__host__ __device__ void calBMatrixHexa1stR(GaussPoint *gp, double bMat[3][8]);
	__host__ __device__ void calInterForHexa1stR(GaussPoint *gp,const MaterialNew *material, double bMat[3][8], const double dt,
		const double hourg_para, double hg[4], int hourg_type);
	__host__ __device__ void hourglassForceHexa(double bMat[3][8],double hg[4], int hourg_type);

	//��������ȫ����
	__host__ __device__ void interForceHexa1stGpu(const MaterialNew *material, const Section* section,double dt);
	__host__ __device__ void calBMatrixHexa1st(GaussPoint *gp, double bMat[3][8]);
	__host__ __device__ void calInterForceHexa1st(GaussPoint *gp, const MaterialNew *material, double bMat[3][8], const double dt);

	//��������ȫ����
	__host__ __device__ void interForcePent1stGpu(const MaterialNew *material, const Section* section, double dt);
	__host__ __device__ void calBMatrixPent1st(GaussPoint *gp, double bMat[3][6]);
	__host__ __device__ void calInterForcePent1st(GaussPoint *gp, const MaterialNew *material, double bMat[3][6], const double dt);

	//��������������
	__host__ __device__ void interForcePent1stRGpu(const MaterialNew *material, double dt, HourglassControl *hourControl);
	__host__ __device__ void calBMatrixFlanBely(GaussPoint *gp, double bMat[3][8]);
	__host__ __device__ void calInterForPent1stR(GaussPoint *gp, const MaterialNew *material, double bMat[3][8], const double dt,
		const double hourg_para, double hg[4], int hourg_type);
	__host__ __device__ void hourglassForcePena(double bMat[3][8],double hg[4], int hourg_type);

	__device__ void interForceSolidShell8NodeGpu(const MaterialNew *material, const Section* section, double dt, HourglassControl *hourControl);

	__host__ __device__ void calBHMatrixSolidShell8Node(GaussPoint *gp,double bhMat[9][8]);

	__host__ __device__ void calBEMatrixSolidShell8Node(GaussPoint *gp,double beMat[6][8]);

	__host__ __device__ void calEASBMatrixSolidShell8Node(GaussPoint *gp,double bEAS[][6],int numInterDof);

	__host__ __device__ void calInterForSolidShell8Node(GaussPoint *gp, const MaterialNew *material,double bhMat[9][8],
		double bEAS[][6], double* alphaIncre,double fE[],double dt, int numInterDof, int loopNum = 1);

	__device__ void interForceSolidShell6NodeGpu(const MaterialNew *material, const Section* section, double dt, HourglassControl *hourControl);
	__host__ __device__ void calBEMatrixSolidShell6Node(GaussPoint *gp, double beMat[6][6]);
	__host__ __device__ void calBHMatrixSolidShell6Node(GaussPoint *gp, double bhMat[9][6]);
	__host__ __device__ void calEASBMatrixSolidShell6Node(GaussPoint *gp, double bEAS[][6], int numInterDof);
	__host__ __device__ void calInterForSolidShell6Node(GaussPoint *gp, const MaterialNew *material,
		double bhMat[9][6], double bEAS[1][6], double* alphaIncre,double fE[], double dt, int numInterDof, int loopNum = 1);

	/**
	���ó�ʼ����Ӧ��
	*/
	/*__host__ __device__ void setInitialYield(const double iniYield);*/

	/**
	����Ӧ����
	*/
	__host__ __device__ void calStrainEnergy();

	/**
	��������嵥Ԫ�ĸ�������
	*/
	__host__ __device__ double4 calTetraAllArea()const;

	__host__ __device__ void computeBondTransformationMatrix(double an[6][6],double src[3][3]);

} SolidElementCuda;