#pragma once
#include"curve.h"
#include<vector>
#include<map>
using std::map;
using std::vector;

typedef struct  CurveManager
{
	vector<Curve> curve_array;
	vector<double> curveValue;
	Curve *curve_array_gpu;
	int totCurveNum;
	map<int, Curve*> curve_map;

	CurveManager();

	/**
	��ʱ���趨����ֵ
	*/
	void set_curve_value(const double time);

	/**
	���߸��Ƶ�gpu
	*/
	void curve_cpy_gpu();

	/**
	gpu�������
	*/
	void gpuCurveClear();

	/**
	��������id��ϵ����
	*/
	void curveLinkId();

	Curve* returnCurve(const int id);

	~CurveManager();
} CurveManager;

