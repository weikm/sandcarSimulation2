#pragma once
#include"node.h"
#include"nodemanager.h"
#include"rigid.h"
#include"part.h"
#include"cuda_runtime.h"
#include<string>
#include"rigidmanager.h"
#include"curve.h"
#include"set.h"
#include"setmanager.h"
#include"rigidmanager.h"
#include"surfacemanager.h"
#include"boundarytype.h"

using std::string;

struct SetManager;
struct Curve;
struct NodeCuda;
struct Set;
struct Part;
struct RigidCuda;
struct RigidManager;
struct SurfaceManager;

typedef struct Boundary
{
	/**
	�غ�����
	*/
	enum Type { disp, vel, accel, spc };
	Type type;

	int id;
	int set_id;
	int node_id;
	int part_id;

	int dof[6];
	int curveId;
	double curve_scale_factor;

	/**
	�߽�����ʩ���ڵ����ڵ���
	*/
	NodeCuda *node;

	/**
	�߽�����ʩ����set�е����нڵ���
	*/
	Set *set;

	/**
	�߽�����ʩ����part�����нڵ���
	*/
	Part *part;

	/**
	�߽�����ʩ���ڸ�����
	*/
	RigidCuda *rigid;

	/**
	����������ܱ߽����������нڵ�
	*/
	vector<NodeCuda *> node_array_cpu;

	/**
	gpu�˳��ܱ߽����������ýڵ�
	*/
	NodeCuda **node_array_gpu;
	int node_num;

	/**
	gpu�����ؽڵ㴴��
	*/
	void gpuNodeArrayCreate(NodeManager *node_manager);


	/**
	cpu�����ؽڵ㴴��
	*/
	void cpuNodeArrayCreate(NodeManager *node_manager, vector<Part> *partArray_, SetManager *set_manager);

	/**
	�ڵ�ʩ�ӱ߽�����
	*/
	void imposebBounGpu(double dt, double previous_dt, CurveManager *curve_manager);
	void imposebBounCpu(double dt, double previous_dt, CurveManager *curve_manager);

	~Boundary()
	{
		checkCudaErrors(cudaFree(node_array_gpu));
		node_array_gpu = nullptr;
		node_array_cpu.clear();
	}
} Boundary;