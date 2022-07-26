#pragma once
#include <vector>
#include"materialmanager.h"
#include"nodemanager.h"
#include"elmmanager.h"
#include"tiemanager.h"
#include"constrained.h"
#include"constrainedmanager.h"
#include"part.h"
#include"hourglasscontrol.h"
#include"elementcontrol.h"
#include"sectionmanager.h"
#include"elementcontrol.h"

struct ConstrainedManager;
struct ElementControl;
struct MultiGpuManager;
struct ElmManager;
struct TieManager;

typedef struct TimeControl
{
	/**
	���Ʒ�����ֹ�Ĳ���
	*/
	int endCyc;
	double endTime;
	double DTMIN;
	double ENDENG;
	double ENDMas;
	double outTimeStep;
	//ls:2020-03-18
	vector<double> outTimeStep_array;
	double initialTimeStep;
	double TSSFAC;
	int ISDO;
	double TSLIMT;
	double timeStep;
	int LCTM;
	int ERODE;
	int MS1ST;
	int NOSOL;
	//
	//ls:2020-04-06
	double DTINIT;            //initialTimeStep;
	double DT2MS;                   //timeStep;
	//
	

	/**
	ʱ�䲽����ȫ����
	*/
	double safeFactor;

	/**
	��������Ƶ��
	*/
	int massSacleFrequen;

	/**
	��ʼʱ�䲽
	*/
	double iniDt;

	/**
	��ָ����Сʱ�䲽
	*/
	double min_shell_time_step;

	/**
	�ǵ�Ԫ����������ⷽʽ
	*/
	int shell_char_leng_type;

	/*
	�������ű�ʶ��Ĭ��Ϊ0
	*/
	int isMassScaled;

	/**
	������������ʱ����Сʱ�䲽
	*/
	double minDtmassScale;

	/**
	���и������ӵ��������ż���
	*/
	void massRedistributeForMassScaledGpu(TieManager *tieManager, ConstrainedManager *constrainManager,
		ElementControl *elementControl, NodeManager *nodeManager, const int cycNum, double &new_dt);

	void massRedistributeForMassScaledMultiGpu(TieManager *tieManager, ConstrainedManager *constrainManager,
		MultiGpuManager* multiGpuMag, NodeManager *nodeManager, const int cycNum, double &new_dt);

	void massRedistributeForMassScaledCpu(TieManager *tieManager, ConstrainedManager *constrainManager,
		vector<ElmManager> &elmManagerArray, NodeManager *nodeManager, const int cycNum, double &new_dt);

	/**
	cpu����ʱ������
	*/
	double timeStepCalCpu(vector<Part> &partArray_,double& dt);

	/**
	������Сʱ�䲽��
	*/
	void calMinTimeStep();

	/**
	gpu����Ч�ʼ���ʱ������
	*/
	double timeStepCalGpuImprove(SectionManager*section_manager, MaterialManager *mater_manager, ElementControl *element_control);
	double timeStepCalMultiGpuImprove(SectionManager*section_manager, MaterialManager *mater_manager, MultiGpuManager *multiGpuMag);

	/**
	�����ʼʱ������
	*/
	void printIniTimeStep(const double dt);

	/**
	Ԥ���ܵļ���ʱ��
	*/
	void evaTotalCalTime(MaterialManager *material_manager, SectionManager *section_mananger,
		ElementControl *elm_control, const double dt, HourglassControl *hourg_control);

	TimeControl()
	{
		isMassScaled = 0;
		safeFactor = 1.0;
	}
} TimeControl;