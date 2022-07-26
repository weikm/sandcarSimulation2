#pragma once
#include"node.h"
#include"set.h"
#include"setmanager.h"
#include"nodemanager.h"

typedef struct Constrained
{
	enum Type
	{
		spotweld, spline, extra_node,
		//ls:2020-03-18
		rigid, joint_spherical, joint_revolute, joint_cylindrical, joint_universal
		//
	};
	Type type;

	//ls:2020-03-18
	//joint
	int jointId;
	string HEADING;
	int node1;
	int node2;
	int node3;
	int node4;
	int node5;
	int node6;
	double RPS;
	double DAMP;

	int coordinateId;
	double timeFailure;
	double COUPL;
	double NXX;
	double NYY;
	double NZZ;
	double MXX;
	double MYY;
	double MZZ;

	bool hangFlag;
	
	//void mass_exam();
	//-----------------------
	//

	//ls:2020-04-06
	//CONSTRAINED_NODAL_RIGID_BODY
	int PID;
	int CID;
	int NSID;
	int PNODE;
	int IPRT;
	int DRFLAG;
	int RRFLAG;

	//spotweld---------------
	int N1;
	int N2;
	double EP;
	//

	/**
	�ڵ㼯��Id
	*/
	int setId;

	/**
	Լ��id
	*/
	int id;

	/**
	spotweld
	*/
	double failureNormalForce;
	double failureShearForce;
	double exponentNormalForce;
	double exponentShearForce;
	double failureTime;

	/**
	�������ڵ�ӽڵ����ڵ����нڵ���Ŀ
	*/
	int node_num;

	/**
	�γɺ���ļ���
	*/
	Set *set;

	/**
	���ڵ�
	*/
	NodeCuda *master_node_cpu;
	vector<NodeCuda *> joint_mnode_array_cpu;

	/**
	gpu���ڵ�
	*/
	NodeCuda **master_node_gpu;

	/**
	�ӽڵ㼯��
	*/
	vector<NodeCuda *> slave_node_array_cpu;
	vector<NodeCuda *> joint_snode_array_cpu;

	/**
	gpu�ӽڵ�
	*/
	NodeCuda** slave_node_array_gpu;

	/**
	cpu�������ڵ�����
	*/
	void calConstraMassCpu();

	/**
	gpu�������ڵ�����
	*/
	void calConstraMassGpu();

	/**
	�޸Ĵӽڵ��ٶ�
	*/
	__host__ __device__ void modifySlaveSpeed(int slave_node_num);

	/**
	�޸����ڵ���
	*/
	__host__ __device__ void modifyMasterForce(int slave_node_num);

	/**
	����cpu�˽ڵ�����
	*/
	void cpuNodeCreate(SetManager *setManager);

	/**
	Ϊ�ӽڵ�ȷ�����ڵ���
	*/
	void verMasterIdForSlave();

	/**
	Ϊ�½Ӵӽڵ�ȷ�����ڵ���
	*/
	void verJointMasterIdforSlave();

	/**
	����gpu������
	*/
	void gpuNodeCreate(NodeManager *nodeManager);

	/**
	gpu��ȷ���ӽڵ��е����ڵ�ָ��
	*/
	void gpuVerMasterPtr(NodeCuda* node_array_gpu);

	~Constrained();
}Constrained;