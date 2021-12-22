#pragma once
#include"node.h"
#include"surface.h"
#include"boundarytype.h"

/**
������չ��Ĳ���
*/
#define EXTFACTOR 0.6
#define THICKFACTOR 0.5
#define MINSEGMENTTHICK 0.001

struct NodeCuda;
struct Surface;

typedef struct SegmentCuda
{
	//ls:2020-04-06
	double A[4];
	//

	/**
	���Ӵ��Ľڵ�
	*/
	int nodeID_array[4];
	NodeCuda* node_array[4];

	int parentSurfaceId;

	//��¼��segment�����е������Լ���ǰ���е���ţ���0��ʼ
	int global_id;
	int local_id;

	int node_amount;
	double thick;

	/**
	��������λ������
	*/
	double *dispIncre;

	/**
	���ڽӴ��鼰����
	*/
	SegmentCuda* adjacent_segment_array[4];
	int nb_sg_id[4];
	int nb_sg_normal_array[4];

	/**
	���Ӵ��鷨����
	*/
	double nor_vec[3];
	double ini_nor_vec[3];

	/**
	���Ӵ���ķ�����
	*/
	double interfaceStiff;
	double segment_mass;

	/**
	���ڽڵ���
	*/
	int node_of_field[20]; //ls:focus

	/**
	��չ��
	*/
	double hi_te_min[3];
	double hi_te_max[3];
	double expand_territory;

	/**
	Ԥ����ײѭ������
	*/
	int predictCyc;
	int previousPredictCyc;

	/**
	�Ӵ���߽���
	*/
	int lineOfEdgeSign[4];

	/**
	ͳ�Ʊ��Ӵ����ж��ٽӴ���
	*/
	void nodeCount();

	/**
	cpu�˽Ӵ������ӽڵ�
	*/
	void cpuNodeLink(vector<NodeCuda> &sys_node_array);

	/*
	* ���������������֮��������
	*/
	__host__ __device__ void computeMaxShellDiagonal(double &msd);

	/*
	* ���������������֮����С����
	*/
	__host__ __device__ void computeMinShellDiagonal(double& msd);
	
	/**
	���������
	*/
	__host__ __device__ void maxThickSet();

	/*
	* ʩ�������غ�
	*/
	__host__ __device__ void applyUniformNormLoadToNode(const double loadValue);

	/*
	 * ����ʩ�ӱ߽�����
	 */
	__host__ __device__ void imposeNormBoundaryToNode(const double boundValue, BoundaryType bType, double dt, double previous_dt);
	__host__ __device__ void imposeDofBoundaryToNode(const double boundValue, BoundaryType bType, double dt, double previous_dt,int dof);

	/**
	���㱾�Ӵ������չ��
	*/
	__host__ __device__ void calExtTerritory();

	/**
	gpu�Ӵ��鸴��
	*/
	__device__ void gpuSegmentCudaCpy(NodeCuda *system_node_array, SegmentCuda *segment_array);

	/**
	�Ӵ���Ԥ����ײ�����ļ���
	*/
	__host__ __device__ int calPreictCyc(const double dispIncre);

	/**
	ȷ�����Ӵ���ı���id
	*/
	__host__ __device__ void defineLocalId(const int temp_local_id, const int temp_global_id);

	/**
	�����ʼ������
	*/
	__host__ __device__ void solveIniNorVec();

	/*
	 * ���㷨����
	 */
	__host__ __device__ void solveNormVec();

	/*
	 * ������չ��Ĵ�С
	 */
	__host__ __device__ void computeExtTerritorySize();

	/*
	 * �������
	 */
	__host__ __device__ void computeArea(double &ar);
} SegmentCuda;

typedef struct SegmentData
{
	int predict_cycle;
	int position;
	int relative_position;
	double cos_value;
	SegmentCuda *test_segment;

}SegmentData;