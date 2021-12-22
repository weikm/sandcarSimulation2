#pragma once
#include"load.h"

/*
 * ��Ҫ����ʩ��������ȫ�ֽڵ��ϵ��غɣ��������غɵ�
 */
typedef struct GlobalLoad:BasicLoad
{
	//ls:2020-04-06
	int LCIDDR;
	double XC;
	double YC;
	double ZC;
	int CID;
	//

	NodeManager* nodeManager;

	virtual void getLoadObject(SetManager* setMag, SurfaceManager* surfMag, vector<Part>& partMag, NodeManager* nodeMag) override;

	virtual void applyLoadGpu() override;

	virtual void applyLoadCpu() override;

	virtual ~GlobalLoad() { ; }
}GlobalLoad;