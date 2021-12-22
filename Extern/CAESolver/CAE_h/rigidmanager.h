#pragma once
#include"rigid.h"
#include"rigidconstrained.h"
//
#include"femdynamic.h"
#include"historynode.h"
//
#include<map>
#include<vector>

using std::map;
struct RigidCuda;
struct RigidConstraCuda;
struct Historynode;
//
struct FEMDynamic;
struct RigidExtraNodeManager;
//
typedef struct RigidManager
{
	//ls:2020-03-18
	/*
	������ڵ㣨�ڵ㼯���ۺϹ�ϵ
	*/
	int num_part;
	vector<int>partId_array;
	vector<int>nodeId_array;
	vector<int>nodeSetId_array;
	vector<bool>isSet_array;

	/**
	�����ռ��ۺϽڵ㼯�Ը���ķ�������
	*/
	double *rigid_force_add_gpu;
	double *rigid_force_add_cpu;

	/*
	cpu������
	*/
	vector<RigidCuda*> RigidPartArray_cpu;
	vector<NodeCuda*> NodeArray_cpu;
	/*
	gpu������
	*/
	RigidCuda** RigidPartArray_gpu;
	NodeCuda** NodeArray_gpu;
	//

	/**
	���弯��
	*/
	vector<RigidCuda> rigid_array;

	/**
	����Լ��
	*/
	vector<RigidConstraCuda> rigidConstra_array;

	/**
	����ۺϹ�ϵ
	*/
	vector<int> masterRigidId;
	vector<int> slaveRigidId;
	vector<RigidCuda*> masterRigidArray;
	vector<RigidCuda*> slaveRigidArray;

	map<int, int> rigidMapToRefNode;

	__host__ __device__ RigidManager();

	/**
	�洢�����Ӹ���(merge rigid)
	*/
	void masterRigidToSlaveRigid_pre(RigidExtraNodeManager *rigid_extra_node_manager);

	/**
	���ݴӸ����޸����������������ĺ͹�����
	*/
	void masterRigidToSlaveRigid_mass();

	/**
	�����������޸ĴӸ�����˶�
	*/
	void masterRigidToSlaveRigid_vel();

	/**
	���ݴӸ����޸����������
	*/
	void masterRigidToSlaveRigid_force();

	//ls:2020-06-24 added
	/**
	�����������˶������޸Ĺ����ڵ��˶�����(׼��)
	*/
	void RigidPartToSlaveNodes_pre(FEMDynamic *domain, RigidExtraNodeManager *rigid_extra_node_manager);
	
	/**
	�����������˶������޸Ĺ����ڵ��˶�����(�ѱ��޸�Ϊ��������ڵ��ٶȸ��·�ʽ����)
	*/
	void RigidPartToSlaveNodesGpu(FEMDynamic *domain);
	
	/*	
	����copy
	*/
	void RigidManager::RigidToSlaveNodesCraetGpu(FEMDynamic *domain);
	//
	
	/**
	���ݴӽڵ������¸����˶�
	*/
	void SlaveNodesForceToRigidGpu(RigidCuda& rigid_array);

	/*
	 * ���������нڵ��ñ�־λ
	 */
	void setRigidFlagForAllNode();

	/*
	 * �����������нڵ�ĺ��������Ա���Ӱ��Ӵ�
	 */
	void clearRigidAllNodeThick();

	//ls:2020-06-24 added
	~RigidManager();
	//

} RigidManager;
