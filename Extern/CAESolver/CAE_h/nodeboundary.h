#pragma once
#include"boundary.h"
#include"rigidmanager.h"
#include"surfacemanager.h"
#include"part.h"
#include"nodemanager.h"
#include<string>

using namespace std;

struct NodeManager;
struct Part;
struct SurfaceManager;
struct RigidManager;
struct BasicBoundary;

typedef struct NodeBoundary:BasicBoundary
{
	//ls:2020-06-24 
	int id;
	string title;

	/*
	 * �ڽڵ㼯����ʩ��ǿ�Ʊ߽�����
	 */
	Set* nodeSet_;

	/*
	 * �ڵ����ڵ���ʩ�ӱ߽�����
	 */
	NodeCuda* nodeCpu_;
	NodeCuda** nodeGpu_;

	NodeBoundary();

	virtual void getBoundaryObject(SetManager* setMag, vector<Part>& partMag, NodeManager* nodeMag,
		RigidManager *rigidMag = nullptr, SurfaceManager* surMag = nullptr) override;

	virtual void imposebBounGpu(double dt, double previous_dt) override;

	virtual void imposebBounCpu(double dt, double previous_dt) override;

	virtual ~NodeBoundary();

}NodeBoundary;