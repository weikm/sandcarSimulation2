#pragma once
#include"node.h"
#include"contactpair.h"
#include "testpair.h"
#include "unified_contact.h"

struct NodeCuda;
struct ContactNodeManager;
struct ContactPairCuda;
struct TestPairCuda;
struct SubDomain;

typedef struct ContactNodeCuda
{
	NodeCuda *node;
	int nodeId;

	int gpu_id;

	/**
	itct��ʾ�õ���������ײ��
	*/
	int itct;

	/**
	istp��ʾ�ɽӴ����˻��ɲ��Զ�
	int istp;*/

	int id;

	/**
	���ڸýӴ���Ĳ��Զ�
	*/
	TestPairCuda test_pair;

	/**
	�����ýڵ�ĽӴ���
	*/
#define max_parent_num 12
	SegmentCuda* parent_segment[max_parent_num];
	int parent_segment_num;

	/**
	���ڸýӴ������ײ��
	*/
	ContactPairCuda contact_pair;

	/**
	�Ӵ��ڵ��������ĽӴ���
	*/
	int surfaceId;

	/**
	������
	*/
	double interfaceStiff;

	/**
	Ӧ������ʱ����ײ������������,ΪGpu�ϵ�ָ��
	*/
	SubDomain *parent_sub_domain;

	/**
	gpu�Ӵ��㸴��
	*/
	__host__  __device__ void gpuContactNodeCudaCpy(NodeCuda *node_array);

	/**
	���㱾�Ӵ�����ٶ�ʸ����ģ
	*/
	__host__ __device__ double calSpeedMode();

	/**
	���ϽӴ���
	*/
	__host__ __device__ void singleInterContactForce();

	/**
	�����һ����ĽӴ���
	*/
	__host__ __device__ void clearOldContactForce();

	/**
	�������б�ǩ
	*/
	__host__ __device__ void clearFlag();

	/**
	�Ӵ��㷣�������
	*/
	//����һ������
	/*__host__ __device__ void calPenaltyFactor(const double dt, const double scale_factor);*/

} ContactNodeCuda;