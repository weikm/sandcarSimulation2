#pragma once
#include"load.h"

/*
 * �ڵ��غ�Ĭ��Ϊ�������غɻ���ת���غ�
 */
typedef struct NodeLoad:BasicLoad
{
	/*
	 * �غ�ʩ���ڽڵ㼯��
	 */
	Set* nodeSet_;

	/*
	 * �غ�ʩ���ڵ����ڵ���
	 */
	NodeCuda* nodeCpu_;
	NodeCuda** nodeGpu_;

	virtual void getLoadObject(SetManager* setMag, SurfaceManager* surfMag, vector<Part>& partMag, NodeManager* nodeMag) override;

	virtual void applyLoadGpu() override;

	virtual void applyLoadCpu() override;

	virtual ~NodeLoad();
}NodeLoad;