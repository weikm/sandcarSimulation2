#pragma once
#include<map>
#include<set>
#include"node.h"


typedef struct IntegratPoint
{
	int id;
	int node_id_array[16];
	NodeCuda* node_array[16];
	double fint[16][6];
	double shpk[16];
	double shpkdx[16];
	double shpkdy[16];
	double shpkdz[16];

	/**
	计算光滑域的B矩阵
	*/
	__host__ __device__ void calGradientMatrix();

	/**
	将光滑域的内力离散到每个节点上
	*/
	__host__ __device__ void atomicDiscreInterForce();
	__host__ __device__ void indexDiscreInterForce();
}IntegratPoint;
