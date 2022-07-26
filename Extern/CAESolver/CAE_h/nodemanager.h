#pragma once
#include"cuda_runtime.h"
#include"node.h"
#include<map>
#include "helper_cuda.h"

using std::map;
struct MultiGpuManager;

typedef struct NodeManager
{
	/**
	cpu������
	*/
	vector<NodeCuda> node_array_cpu;

	/**
	gpu������
	*/
	NodeCuda* node_array_gpu;

	int dimesionPerNode;
	int dofPerNode;
	int totNodeNum;

	/**
	ϵͳ�ܶ���
	*/
	double kinetic_energy;

	map<int, NodeCuda*> node_id_link;

	/*
	 * ������нڵ�Ӧ��Ӧ��
	 */
	void clearAllNodeStressStrainGpu();

	/*
	 * ƽ�����нڵ��Ӧ��Ӧ��
	 */
	void averageNodeStressStrainGpu();

	/**
	��Ϣ��ʼ��
	*/
	void factorInitial();

	/**
	�������id�Ľڵ�
	*/
	NodeCuda* returnNode(int node_id);

	/**
	�γɽڵ���id֮���ӳ��
	*/
	void buildMap();

	/**
	gpu�����ݴ���
	*/
	void gpuNodeCreate();

	void multiGpuNodeCreate(MultiGpuManager *mulGpuMag);
	
	void managedNodeCreate();

	/**
	����ڵ�ĵ�Ч����
	*/
	void effectMassCal();

	/*
	* �������
	*/
	void nodeMassCheckCpu();
	void nodeMassCheckGpu();

	/**
	���ýڵ�����
	*/
	void resetNodeMassGpu();
	void resetNodeMassCpu();

	/**
	���нڵ���ٶ��ٶȸ���
	*/
	void allNodeAccelVelUpdateGpu(const double dt, double previous_dt);
	void allNodeAccelVelUpdateCpu(double dt, double previous_dt);

	/**
	���нڵ�λ�Ƹ���
	*/
	void allNodeDispUpdateGpu(double dt);
	void allNodeDispUpdateCpu(double dt);

	/*
	* ���ݽڵ�������ڵ�
	*/
	void sortNodeOnFileId();

	/**
	gpu�ڵ����ݸ��Ƶ�cpu
	*/
	void gpuDataToCpu();

	/**
	����Ԥ�������Լ��Ͻڵ�����
	*/
	void addNodeInterForcePreIndex();

	/**
	����ڵ�ĳ�ʼ����
	*/
	void storeAllNodeIniCoord();

	/**
	����ڵ�ĳ�ʼ����
	*/
	void storeNodeIniMassCpu();
	void storeNodeIniMassGpu();

	/**
	���ݷ��ʦ��ĳ����������ǰ���ϽӴ����ĳ���
	*/
	void improveContactEffectToDisp(double dt, double previous_dt);

	/**
	����ϵͳ�Ķ���
	*/
	void calKineticEnenrgy();

	/**
	�������ź���½ڵ�ĵ�Ч����
	*/
	void updateEquivalentMassAfterScaleGpu();
	void updateEquivalentMassAfterScaleCpu();

	~NodeManager();

	NodeManager();
} NodeManager;