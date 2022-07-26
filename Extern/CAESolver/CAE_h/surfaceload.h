#pragma once
#include"load.h"

/*
 * ���غ�ĿǰĬ��Ϊ����ѹ���غ�
 * ��ĿǰĬ��Ϊʩ��������
 */
typedef struct SurfaceLoad:BasicLoad
{
	//ls:2020-06-01 added
	//LOAD_SEGMENT_SET
	int id;
	string title;
	int SSID;
	int LCID;
	double SF;
	double AT;

	//LOAD_SHELL_ELEMENT
	int EID;
	//

	Surface* surface_;

	SegmentCuda* segmentCpu_;
	SegmentCuda* segmentGpu_;

	virtual void getLoadObject(SetManager* setMag, SurfaceManager* surfMag, vector<Part>& partMag, NodeManager* nodeMag) override;

	virtual void applyLoadGpu() override;

	virtual void applyLoadCpu() override;

	virtual ~SurfaceLoad();
}SurfaceLoad;