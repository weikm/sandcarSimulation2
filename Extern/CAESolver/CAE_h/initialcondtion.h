#pragma once
#include "node.h"
#include"set.h"
#include"part.h"
#include"rigid.h"
#include "rigidmanager.h"

struct RigidManager;
struct RigidCuda;
struct NodeCuda;
struct Part;

typedef struct InitialCondition
{
	enum Type
	{
		vel, disp
	};

	/**
	��ʼ����
	*/
	Type type;

	/**
	ʩ�ӳ�ʼֵ�ļ���
	*/
	int nodeId;
	int nodeSetId;
	int partId;

	int NSIDEX;
	int BOXID;
	int IRIGID;
	int ICID;

	/**
	�趨�ĳ�ʼֵ
	*/
	double value[6];

	/**
	single node initial
	*/
	NodeCuda *node;

	/**
	�ڵ㼯�ϳ�ʼֵ
	*/
	Set *node_set;

	/**
	������ʼֵ
	*/
	Part *part;

	/**
	��ʼ���������ڸ�����
	*/
	RigidCuda *rigid;

	/**
	�������ⱻ���ó�ʼ���������нڵ�
	*/
	vector<NodeCuda *> node_array_cpu;

	InitialCondition();

	/**
	���ɳ�ʼ�����ڵ㼯��
	*/
	void InitialNodeCreate();

	/**
	ʩ�ӳ�ʼ����
	*/
	void setInitialCondition(RigidManager *rigidManager);
} InitialCondition;
