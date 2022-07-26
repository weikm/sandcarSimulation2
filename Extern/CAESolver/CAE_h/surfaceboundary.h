#pragma once
#include"boundary.h"

typedef struct SurfaceBoundary:BasicBoundary
{
	/*
	 * ������ʩ�ӱ߽�����
	 */
	Surface* surface_;

	bool isNorm;

	virtual  void getBoundaryObject(SetManager* setMag, vector<Part>& partMag, NodeManager* nodeMag,
		RigidManager *rigidMag = nullptr, SurfaceManager* surMag = nullptr) override;

	virtual void imposebBounCpu(double dt, double previous_dt) override;

	virtual void imposebBounGpu(double dt, double previous_dt) override;

	virtual ~SurfaceBoundary() { ; }
};