#pragma once
#include"cuda_runtime.h"
#include"abaquscurve.h"
#include<vector>
#include<string>

using std::string;

using std::vector;
#define curve_max_node_num 128

struct AbaqusCurve;

typedef struct Curve
{
	int node_num;
	double x_axis[curve_max_node_num];
	double y_axis[curve_max_node_num];
	int serialNum;

	//ls:2020-04-06
	string title;
	int LCID;
	int SIDR;
	int DATTYP;
	//

	/**
	scale factor for abscissa value
	*/
	double SFA;

	/**
	scale factor for ordinate value (function)
	*/
	double SFO;

	/**
	offset for abscissa values, see explanation below
	*/
	double OFFA;

	/**
	offset for ordinate values(function), see explanation below
	*/
	double OFFO;

	/**
	����x��ʱ��ֵ
	*/
	double y_value;

	AbaqusCurve* abaqusCurve_;

	Curve(double sfa = 1.0, double sfo = 1.0, double offa = 0.0, double offo = 0.0);

	virtual ~Curve();

	__host__ __device__ void getSerialId(int id) { serialNum = id; }

	/*
	 *��ȡ����
	 */
	__host__ void getCurve(vector<double> &x,vector<double> &y);

	__host__ void getCurve(int nd, double *x, double *y);

	/**
	�趨��ǰֵ
	*/
	__host__ __device__ void setValue(const double x_value);

	/*
	ȡ����ǰֵ
	*/
	__host__ __device__ double returnValue(const double x_value)const;

	/*
	 *�������к�
	 */
	__host__ __device__ double giveSerialNum() const { return serialNum; }

} Curve;

