#pragma once
#include"rigid.h"
#include"boundary.h"
#include"curvemanager.h"

struct RigidCuda;
struct BasicBoundary;
struct CurveManager;

enum RigidConstraType
{
	rSpc,
	rDisp,
	rVel,
	rAccel,
};

typedef struct RigidConstraCuda
{
	RigidConstraType rcType_;

	int rigidId;

	int curveId_;

	int boundaryId_;

	int dof[6];

	double value;

	double scaleFactor[6];

	double spc[6];

	double directCosines[9];

	Curve* curve_;

	RigidCuda *rigid;

	BasicBoundary* parentBoundary;

	/**
	ʩ��Լ��
	*/
	void rigidConstraint();

	/*
	 * ʩ�Ӹ����ϵ�ǿ�Ʊ߽�����
	 */
	void imposeBoundaryToRigid(const double dt, const double previous_dt);

	/*
	 * ���ӵ���Ӧ����
	 */
	void linkCurveToRigidConstra(CurveManager* cvMag);

	/**
	Լ������
	*/
	void initialRigidConstra(vector<RigidCuda> &rigid_array);

	/**
	��������
	*/
	/*void rigidSet(vector<RigidCuda> &rigid_array);*///���ϵ���һ��������

	/*
	 * ��������ֵ
	 */
	void computeRigidConstraValue(const double time);
} RigidConstraCuda;
