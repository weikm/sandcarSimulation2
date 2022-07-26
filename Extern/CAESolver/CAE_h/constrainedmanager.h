#pragma once
#include"constrained.h"

struct Constrained;

typedef struct ConstrainedManager
{
	vector<Constrained> constrained_array;
	Constrained *constrained_array_gpu;
	int constra_num;

	/**
	���У����ӣ��ӽڵ㹹�ɵļ���
	*/
	NodeCuda** slave_node_array_gpu;
	int slave_node_num;

	/**
	�������ιؽڣ��½ӣ��ӽڵ㣨�������ӣ����ɵļ���
	*/
	NodeCuda** joint_snode_array_gpu;
	int joint_snode_num;

	ConstrainedManager();

	/**
	���Ӵ�cpu���Ƶ�gpu
	*/
	void gpuConstrainedCpy();

	/**
	����:������
	*/
	void allConstForCalGpu();
	void allConstForCalCpu();

	/**
	����:�ٶȼ���
	*/
	void allConstVelCalGpu();
	void allConstVelCalCpu();

	/**
	���ɴӽڵ㼯��
	*/
	void creatSlaveNodeArray();

	/**
	gpuִ������(����)�ӽڵ��������
	*/
	void allConstForImproveCalGpu();

	/**
	gpuִ������(����)�ӽڵ���ٶȼ���
	*/
	void allConstVelImproveCalGpu();

	/**
	���дӽڵ������ת�Ƶ����ڵ�
	*/
	void allSlaveMassToMasterGpu();
	void allSlaveMassToMasterCpu();

	/**
	gpuִ���������ιؽڣ��½ӣ��ڵ��������
	*/
	void allJointForImproveCalGpu();

	~ConstrainedManager();
}ConstrainedManager;