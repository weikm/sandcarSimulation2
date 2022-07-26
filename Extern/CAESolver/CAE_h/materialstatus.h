#pragma once
#include"cuda_runtime.h"

struct GaussPoint;

struct  MaterialStatus
{
	GaussPoint* gaussPoint_;

	/*
	* Ӧ�������¹���洢: 0 1 2 3 4 5��Ӧ�� xx yy zz xy yz zx
	*/
	double stress_[6];

	double pressure_;

	/*
	* Ӧ�䰴���¹���洢: 0 1 2 3 4 5��Ӧ�� xx yy zz xy yz zx
	*/
	double strain_[6];

public:
	__host__ __device__ MaterialStatus();

	__host__ __device__ void linkGaussPoint(GaussPoint *parentGauss);
};
