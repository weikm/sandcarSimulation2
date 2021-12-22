#pragma once
#include"load.h"

/*
 * ��Ԫ�غ�ĿǰĬ��Ϊ���غ�
 */
typedef struct ElementLoad:BasicLoad
{
	//ls:2020-06-01
	int id;
	string title;
	int EID;
	int LCID;
	double SF;
	double AT = 0;
	//

	/*
	 * �غ�ʩ���ڵ�Ԫ������
	 */
	Set* elementSet_;

	/*
	 * ��Ԫʩ���ڵ�����Ԫ��
	 */
	Element* elementCpu_;
	Element* elementGpu_;

	virtual void getLoadObject(SetManager* setMag, SurfaceManager* surfMag, vector<Part>& partMag, NodeManager* nodeMag) override;

	virtual void applyLoadGpu() override;

	virtual void applyLoadCpu() override;

	virtual ~ElementLoad();
}ElementLoad;