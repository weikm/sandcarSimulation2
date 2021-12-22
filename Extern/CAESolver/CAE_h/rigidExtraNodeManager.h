#pragma once
#include"rigid.h"
#include"node.h"
#include<vector>

struct RigidCuda;
struct NodeCuda;

typedef struct RigidExtraNodeManager
{
	int num_extra_part;

	vector<RigidCuda*>  extra_rigid_part_cpu;

	int num_rigids_part;

	vector<RigidCuda*> rigids_part_cpu;

	/*��ʼ���ṹ�����*/
	__host__ __device__ RigidExtraNodeManager();

	~RigidExtraNodeManager();

}RigidExtraNodeManager;