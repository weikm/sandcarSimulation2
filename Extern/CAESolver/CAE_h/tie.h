#pragma once
#include"node.h"
#include<set>
#include"nodemanager.h"

using std::set;

typedef struct SubTie
{
	int seg_store_id;
	int node_store_id;

	SegmentCuda *master_segment;
	NodeCuda *slave_node;
	double interp_funtion[4];

	/**
	����gpu�˵Ľڵ��Լ���
	*/
	__host__ __device__ void linkNodePtr(NodeCuda *node_array);
	__host__ __device__ void linkSegmentPtr(SegmentCuda *segment_array);

	/**
	����ֵ����
	*/
	__host__ __device__ void calInterpFun();

	/**
	�޸��������
	*/
	__device__ void addForToMasterGpu();
	__host__ void addForToMasterCpu();

	/**
	�޸Ĵӽڵ���ٶ�
	*/
	__host__ __device__ void modifySlaveVel();

	/**
	�޸����������
	*/
	__device__ void transferMassGpu();
	__host__ void transferMassCpu();
}SubTie;

typedef struct Tie
{
	int masterSurface_id;
	int slaveSurface_id;

	std::string masterName;
	std::string slaveName;

	Surface *masterSurface;
	Surface *slaveSurface;

	enum Type
	{
		node_to_surface, surface_to_surface
	};
	/**
	����Ĭ��Ϊ node_to_surface
	*/
	Type type;

	/**
	�γɰ󶨵ľ���
	*/
	double position_tolerance;

	/**
	adjustΪ true �򽫴ӽڵ��ƶ������ϣ�Ĭ��Ϊfalse
	*/
	bool adjust;

	/**
	Ϊtrue ���޸���ת���ɶȣ�Ĭ��Ϊfalse
	*/
	bool is_no_rotation;

	/**
	Ϊtrue �����ӿǵĺ�ȣ�Ĭ��Ϊfalse
	*/
	bool is_no_thickness;

	/**
	�γɵ��Ӱ󶨶�
	*/
	vector<SubTie> sub_tie_array_cpu;
	SubTie *sub_tie_array_gpu;

	/**
	���ɰ󶨵Ĵӽڵ����Ӧ������
	*/
	void produceSubTie(set<int> *slave_nodeid_array, set<int> *master_nodeid_array);

	/**
	����gpu�˵��Ӱ���
	*/
	void produceTieArrayGpu(NodeManager *node_manager);

	/**
	����ӽڵ��������ϵ�ͶӰ�Ĳ�ֵ����
	*/
	void calInterpolateFunForSlave();

	/**
	���ӽڵ������ӵ�������
	*/
	void addSlaveForceToMasterGpu();
	void addSlaveForceToMasterCpu();

	/**
	����������ٶ��޸Ĵӽڵ���ٶ�
	*/
	void modifySlaveVelByMasterGpu();
	void modifySlaveVelByMasterCpu();

	/**
	ת�ƴӽڵ��������������
	*/
	void transferAllSubTieMassGpu();
	void transferAllSubTieMassCpu();

	virtual ~Tie();
}Tie;
