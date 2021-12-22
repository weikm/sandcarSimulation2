#pragma once
#include "node.h"
#include "part.h"
#include "set.h"
#include "nodemanager.h"
#include <string>
#include <vector>
#include "rigidmanager.h"

using std::string;
struct RigidManager;

typedef struct RigidCuda
{
	/**
	cpu�˽ڵ�
	*/
	vector<NodeCuda *> node_array_cpu;

	/**
	gpu�˽ڵ�
	*/
	NodeCuda** node_array_gpu;

	/**
	cpu�˽ڵ�
	*/
	vector<NodeCuda*> extra_node_array_cpu;
	vector<NodeCuda*> rigids_node_array_cpu;

	/**
	gpu�˽ڵ�
	*/
	NodeCuda** extra_node_array_gpu;
	NodeCuda** rigids_node_array_gpu;

	/**
	�����ĸpart
	*/
	Part *parentPart;

	/**
	�����ĸset
	*/
	Set *parentSet;

	/**
	�ڵ���Ŀ
	*/
	int node_num;

	int refNodeId;

	int FinishRigidInitial;
	int ismergerigid;

	NodeCuda* refNode;

	int dofRigidBoundSign[6];
	double mass[6];
	double moment_of_inertia[3][3];
	double disp[6];
	double dispIncre[6];
	double accel[6];
	double position_of_rotCenter[3];
	double rotAxis[3];
	double rot_vel_center[3];
	double joint_force[6];
	double fext[6];
	double fint[6];

	double3 position;
	double3 tra_vel;
	double3 rot_vel;

	/**
	gpu�˸�������
	*/
	double *rot_gpu;

	/**
	�����ռ��������ܵ���
	*/
	double *force_gpu;
	double *force_cpu;

	/**
	cpu����ڵ�������
	*/
	void rigidCpuNodeCreate(NodeManager *node_manager);

	/**
	gpu����ڵ�������
	*/
	void rigidGpuNodeCreate(NodeManager *node_manager);

	/**
	�����������
	*/
	/*void rigidMass();*/
	void rigidMass(RigidManager *rigidManager);
	/**
	���ٶȸ���
	*/
	void rigidAccelUpdate();

	/**
	���������ٶȸ���
	*/
	void rigidVelUpdate(const double dt, const double previous_dt);

	/**
	�����ڽڵ����
	*/
	void rigidNodeVelUpdateGpu(double dt);
	void rigidNodeVelUpdateCpu(const double dt);

	/**
	�����������
	*/
	void rigidBodyForceCalGpu();
	void rigidBodyForceCalCpu();

	/**
	�����㿽���ڴ�
	*/
	void allZeroMemRigid();

	/**
	�����������нڵ���Ϊ����
	*/
	void allNodeMark();

	/**
	����������нڵ�ĺ��
	*/
	void clearNodeThick();

	virtual ~RigidCuda();
} RigidCuda;
