#pragma once
#include"cuda_runtime.h"
#include"node.h"
#include"elementtypeenum.h"
#include"element.h"
#include"materialnew.h"
#include"sectionold.h"

struct Section;
struct NodeCuda;
struct Element;

typedef struct SolidElement2nd:Element
{

	int nodeIdArray_[20];
	NodeCuda* nodeArray_[20];

	int gpuId_;

	double dtEle_;

	double lengthChart_;

	double massScaleRate_;

	double volume_;

	double translateMass_;

	double rotateMass_[3];

	double fint[20][3];

	/*
	 * ���õ�Ԫ������������
	 */
	__host__ __device__ void setElementSectionType();

	/*
	 * ���㵥Ԫ���
	 */
	__host__ __device__ double calSolid2ndVolume(const int gaussNum);

	/*
	 * ������ֵ��κ�����˵��ۼ�
	 */
	__host__ __device__ void computeTetr2ndNNMatrix(double *nn,const int gaussNum);

	/*
	 * ������ֵ���κ���
	 */
	__host__ __device__ void calTetr2ndGaussShape(GaussPoint *gp,double *shp);
	__host__ __device__ void calHexa2ndGaussShape(GaussPoint *gp,double *shp);
	__host__ __device__ void calPent2ndGaussShape(GaussPoint *gp,double *shp);

	/*
	* ������ֵ���ſ˱Ⱦ���
	*/
	__host__ __device__ void computeHexa2ndGaussJacobMat(GaussPoint *gp, double jacobMat[20][3])const;
	__host__ __device__ void computePent2ndGaussJacobMat(GaussPoint *gp, double jacobMat[15][3])const;
	__host__ __device__ void computeTetr2ndGaussJacobMat(GaussPoint *gp, double jacobMat[10][3])const;

	/*
	* ������ֵ㱾���κ���ƫ��
	*/
	__host__ __device__ void calHexa2ndGaussLocalDerivative(GaussPoint *gp, double dN[20][3])const;
	__host__ __device__ void calPent2ndGaussLocalDerivative(GaussPoint *gp, double dN[15][3])const;
	__host__ __device__ void calTetr2ndGaussLocalDerivative(GaussPoint *gp, double dN[10][3])const;

	/*
	* ������ֵ��������������
	*/
	__host__ __device__ void computeHexa2ndGaussVolume(GaussPoint *gp, double &vol)const;
	__host__ __device__ void computePent2ndGaussVolume(GaussPoint *gp, double &vol)const;
	__host__ __device__ void computeTetr2ndGaussVolume(GaussPoint *gp, double &vol)const;

	/*
	 * ��������
	 */
	__host__ void computeSolid2ndInterForceCpu(MaterialNew* material, Section* section, HourglassControl* hourglass, double dt);
	__device__ void computeSolid2ndInterForceGpu(MaterialNew* material, Section* section, HourglassControl* hourglass, double dt);

	__device__ void interForceTet2ndMGpu(const MaterialNew* material, const Section* section, const double dt);
	__host__ __device__ void calBMatrixTet2ndM(GaussPoint* gp, double bMat[3][10], double ncd[10][3]);
	__host__ __device__ void calInterForTet2ndM(GaussPoint *gp, double bMat[3][10], const MaterialNew *mat, double dt, double ft[10][3], double ncv[10][3]);

	__device__ void interForceTet2ndGpu(const MaterialNew* material, const Section *section, const double dt);
	__host__ __device__ void calBMatrixTet2nd(GaussPoint *gp,double bMat[3][10]);
	__host__ __device__ void calInterForTet2nd(GaussPoint *gp, double bMat[3][10],const MaterialNew *mat, double dt,int ig);

	__device__ void interForceHex2ndGpu(const MaterialNew* material, const Section *section, const double dt);
	__host__ __device__ void calBmaterixHex2nd(GaussPoint *gp,double bMat[3][20]);

	
	__device__ void interForcePenta2ndGpu(const MaterialNew* material, const Section *section, const double dt);
	__host__ __device__ void calBMaterixPenta2nd(GaussPoint *gp,double bMat[3][15]);

	/*
	 * ʱ�䲽������
	 */

	__host__ __device__ double computeSolidTimeStep(const double wave_num, const double safe_factor);

	/*
	 * �߽�ʵ�嵥Ԫ�����������ṩ�����㷨
	 * һ���Ե�Ԫ���������ڵ�֮�����̾���Ϊ��������
	 * ���������Ӧ�ĵͽ׵�Ԫ���������ȣ����Դ˳���2
	 */

	__host__ __device__ double computeSolid2ndCharactLength();

	__host__ __device__ double computeHexa2ndCharactLength();

	__host__ __device__ double computePent2ndCharactLength();

	__host__ __device__ double computeTetr2ndCharactLength();

	__host__ __device__ void assignTimeStep(const double dt);


	/*
	 * ��������
	 */
	__host__ __device__ void solid2ndNodeMassIntia();

	__device__ double computeSolid2ndMassScaleGpu(const double minDt);

	__host__ double computeSolid2ndMassScaleCpu(const double minDt);

	/*
	 * ��������
	 */

	__device__ void builNodeIndex();

	__host__ __device__ void bulidNodeMatch(NodeCuda *node_array_gpu);

	/*
	 * ��Ԫ��������
	 */
	__host__ double computeSolid2ndMass(Material *matPtr, Section *sectionPtr);

	/*
	 * ����ڵ��Ӧ��Ӧ��
	 */
	__device__ void computeSolid2ndNodeStrGpu(const int gaussNum);
	 __device__ void computeTetr2ndNodeStressStrainGpu(const int gaussNum);

	 /*
	 * ��ɢ��Ԫ�������ڵ���
	 */
	 __device__ void discreteSolid2ndInterForceToNodeGpu();

	 __host__ void discreteSolid2ndInterForceToNodeCpu();

private:

	/*
	 * ���㵥Ԫ�����Ŀǰֻ�ǽ��Ƶ�������Ӧ�ͽ׵�Ԫ�����
	 */
	__host__ __device__ void calPent2ndVol(double &vol);
	__host__ __device__ void calHexa2ndVol(double &vol);
	__host__ __device__ void calTetr2ndVol(double &vol);

}SolidElement2nd;
