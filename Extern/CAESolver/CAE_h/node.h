#pragma once
#include"cuda_runtime.h"
#include"shell1st.h"
#include"discrete1st.h"
#include"beam1st.h"
#include"solid1st.h"
#include"contactnode.h"
#include"solid2nd.h"
#include"beam2nd.h"
#include"shell2nd.h"

struct ShellElementCuda;
struct BeamElementCuda;
struct SolidElementCuda;
struct DiscreteCuda;
struct ContactNodeCuda;
struct SegmentCuda;
struct Element;

typedef struct NodeCuda
{
	int file_id;
	int store_id;
	double coord[6];
	double ini_coord[3];
	double thick;

//#define maxShellList 10
//#define maxBeamList 4
//#define maxSolidList 70
//#define maxDiscreteList 3

	/**
	���ɶ�Ԥ������Ԫ
	*/
	/*ShellElementCuda* shellList[maxShellList];
	BeamElementCuda* beamList[maxBeamList];
	SolidElementCuda* solidList[maxSolidList];
	DiscreteCuda* discreteList[maxDiscreteList];*/

	/**
	���ڵ���Ϣ��Ĭ�����нڵ�Ϊ�ӽڵ�
	*/
	NodeCuda *master_node;
	int master_node_id;

	/**
	������λ����Ϣ
	*/
	/*int shellNdex[maxShellList];
	int solidNdex[maxSolidList];
	int beamNdex[maxBeamList];
	int discreteNdex[maxDiscreteList];*/

	/**
	��������Ŀ
	*/
	/*int shellNum;
	int solidNum;
	int beamNum;
	int discreteNum;*/

#define MAXELEMENTNUM 72

	int elementNum_;
	int elementIndex_[MAXELEMENTNUM];
	Element* elementList_[MAXELEMENTNUM];
	double elmContriAccumulate_;
	
	/**
	�ɽڵ����ɵĲ���Ӵ�����ײ��
	*/
	ContactNodeCuda *contact_node;

	/**
	ƽ������
	*/
	double translate_mass;

	/**
	��ת����
	*/
	double rotate_mass[3];

	/**
	��ʼ��������
	*/
	double ini_tran_mass;
	double ini_rota_mass[3];

	/**
	�Ӵ�����������
	*/
	double effective_mass;

	/**
	����Ӵ�ʱ,�ýڵ�ķ�����
	*/
	double interfaceStiff;

	/**
	����
	*/
	double int_force[6];

	/**
	����
	*/
	double ext_force[6];

	/**
	�ڵ�Ӧ��
	*/
	double stress[6];
	double strain[6];

	double joint_force[6];
	double accel[6];
	double vel[6];
	double dfn[3];
	double disp[6];
	double disp_incre[6];

	/**
	�ڵ��Ե��־
	*/
	int edge_sign;
	SegmentCuda *parent_segment;

	/**
	�����־
	*/
	int weld_sign;
	int spotweld_OneNode;

	/**
	�����־
	*/
	int rigid_sign;

	/*
	* ��ת���ɶȱ�־
	*/
	int rotate_sign;

	__device__ __host__ NodeCuda();

	/**
	���ýڵ��<����
	*/
	bool operator <(const NodeCuda &nodeCuda)const;

	/**
	�ڵ�ʱ������
	*/
	double dt_node;

	/**
	�ڵ���ٶȸ��£�����ڵ���������Ӵ���
	*/
	__host__ __device__ void accelVelUpdate(const double dt, const double previous_dt);

	/**
	�ڵ��ٶȼ�λ�Ƶȱ�������
	*/
	__host__ __device__ void dispUpdate(const double dt, const int ndf);

	/*
	 * ����ڵ�Ӧ��Ӧ���뵥Ԫ�ۻ�ͳ��
	 */
	__host__ __device__ void clearStressStrain();

	/**
	����������Ϣ
	*/
	__device__ void buildShellIndexGpu(ShellElementCuda *shell, int num);
	__device__ void buildBeamIndexGpu(BeamElementCuda *beam, int num);
	__device__ void buildSolidIndexGpu(SolidElementCuda *solid, int num);
	__device__ void buildDiscreteIndexGpu(DiscreteCuda *discrete, int num);

	__device__ void buildElementIndexGpu(Element *el, int num);

	/**
	����������Ϣ����ڵ�����
	*/
	__host__ __device__ void calIndexInterFor();

	/**
	����������Ϣ����ڵ�Ӧ��Ӧ��
	*/
	__host__ __device__ void calNodeStressStrain();

	/**
	�����ʼ����
	*/
	__host__ __device__ void storeIniCoord();

	/**
	������ԭ
	*/
	__host__ __device__ void massReduction();

	/**
	�ӽڵ��������ӵ����ڵ���
	*/
	__device__ void massAttachedToMainNode(int type);

	/**
	�����Ч����
	*/
	__host__ __device__ void storeEquivalentMass();

	/**
	�����ʼ����
	*/
	__host__ __device__ void storeInitialMass();
} NodeCuda;