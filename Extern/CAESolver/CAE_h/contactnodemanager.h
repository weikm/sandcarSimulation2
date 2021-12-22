#pragma once
#include"surface.h"
#include <map>
#include"contactnode.h"
#include"contactmanager.h"
#include"helper_cuda.h"
#include"nodemanager.h"

struct NodeManager;
struct Surface;
struct Contact;
struct ContactNodeCuda;

typedef struct ContactNodeManager
{
	vector<ContactNodeCuda> contact_node_array_cpu;
	std::map<int, int> nodeLinkContactNodeCuda;

	/**
	�Ӵ�������
	*/
	int contact_node_num;

	/**
	gpu�Ӵ���
	*/
	ContactNodeCuda *contact_node_array_gpu;

	/*
	* ��gpu�Ӵ���
	*/
	vector<ContactNodeCuda*> contactNodeArrayMultiGpu;

	ContactNodeManager();

	/**
	cpu�Ӵ��ڵ�����
	*/
	void cpuContactNodeCudaCreate(vector<Surface> &surface_array,vector<Contact> &contactArray,NodeManager *ndMag);

	/**
	gpu�Ӵ�������
	*/
	void gpuContactNodeCudaCreate(NodeManager *node_manager);
	void multiGpuContactNodeCreate(vector<int> gpuIdArray, NodeManager *ndMag = nullptr);
	void multiGpuNodeLinkContactNode(NodeManager* ndMag,vector<int>& gpuIdArray);

	/*
	* managed�Ӵ�������
	*/
	void managedContactNodeCreate(NodeManager* node_manager);

	/*
	* ���ɽӴ����gpu��ţ�Ŀǰֻ�����������ʽ�����ﵽ���ܵĸ��ؾ���
	*/
	void calGpuIdForContactNodeMultiGpu(vector<int> gpuIdArray);

	/**
	���ϽӴ���
	*/
	void integrateContactForceMultiGpu(const int cnPairNum, int gpu_id, cudaStream_t stream);
	void integrateContactForceGpu(const int contact_pair_num);
	void integrateContactForceCpu();

	/**
	����������ײ��ķ�����
	*/
	//void calAllNodePenaFactorCpu(double penaScale, double dt);
	//void CalAllNodePenaFactorGpu(double penaScale, const double dt);

	/**
	������б��
	*/
	void clearAllFlag();

	~ContactNodeManager();

} ContactNodeManager;
