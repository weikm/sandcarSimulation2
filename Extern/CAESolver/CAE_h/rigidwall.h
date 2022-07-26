#pragma once
#include"cuda_runtime.h"
#include"node.h"
#include"femdynamic.h"
#include"curvemanager.h"

typedef struct RigidWall
{
	enum Gtype
	{
		flat, unlimited_flat, sphere, cylinder
	};

	enum Mtype
	{
		vel, disp
	};

	//ls:2020-03-18
	int id;
	string title;
	int NSID; //Nodal set ID containing slave nodes 
	int NSIDEX; //Nodal set ID containing nodes that are exempted as slave nodes
	int BOXID; //All nodes in box are included as slave nodes to rigid wall
	/*All nodes within a normal offset distance, OFFSET,
	to the rigid wall are included as slave nodes for the rigid wall*/
	double OFFSET; 
	double birth_time;
	double death_time;
	double RWKSF; // Stiffness scaling factor

	double XT;
	double YT;
	double ZT;
	double XH;
	double YH;
	double ZH;
	double FRIC;  //
	double WVEL;
	//Additional card for FORCES keyword option.
	int SOFT;
	int SSID;
	int N[4];
	// Additional card for FINITE keyword option
	double XHEV;
	double YHEV;
	double ZHEV;
	double LENL;
	double LENM;
	//
	int OPT;
	//

	/**
	����ǽ��״
	*/
	Gtype gtype;

	/**
	����ǽ����
	*/
	Mtype mtype;

	/**
	�������ڸ���ǽ�еĽڵ�(ls:�޸�ΪslaveNode)
	*/
	int node_set_id;
	Set *node_set;

	/**
	���������Ķ���
	*/
	double3 nor_vec;
	double3 tan_vec_1;
	double3 tan_vec_2;

	/**
	�������
	*/
	double3 xt;
	/**
	�����յ�
	*/
	double3 xh;

	/**
	��������ƽ������Ĳ���
	*/
	double3 xhev;
	double len_l;
	double len_m;

	/**
	����Բ���������
	*/
	double rad_sph;

	/**
	����Բ��������Ĳ���
	*/
	double rad_cyl;
	double len_cyl;

	/**
	�����˶�����Ĳ���
	*/
	int curve_id;
	Curve *curve;
	double3 vector_motion;
	double disp_value;
	double velocity;

	/**
	Ħ������
	*/
	double friction_factor;

	//ls:2020-07-01 added
	/**
	�ӽڵ㼯��
	*/
	//vector<NodeCuda *> slave_node_array_cpu;
	/*
	����ǽ������
	*/
	double totMass;

	int TotslaveNodes;

	/**
	gpu�ӽڵ�
	*/
	NodeCuda** slave_node_array_gpu;
	
	/*
	�ռ��ӽڵ�
	*/
	void slaveNodeCollect(SetManager *setManager, FEMDynamic *domain);

	/*
	����copy
	*/
	void gpuSlaveNodeCopy(FEMDynamic *domain);

	/*
	���÷�������/�����ڵ㷨����Ӵ���
	*/
	void calContactForce(FEMDynamic *domain, double current_time, double dt);

	/*
	����ǽ�ƶ�
	*/
	void movingRigidwall(CurveManager *curveManager);

	RigidWall();
	//

	/**
	��ⷨ��������������������
	*/
	void calAllVector();

	/**
	���Ƹ���ǽ���˶�
	*/
	__host__ __device__ void calWallMotion(const double dt, const double current_time);

	/**
	������������Ӵ���
	*/
	__host__ __device__ void calContactForcePenlty(ContactNodeCuda *contact_node);
} RigidWall;