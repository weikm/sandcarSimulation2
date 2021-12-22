#pragma once
#include"node.h"
#include"nodemanager.h"
#include"rigid.h"
#include<string>
#include"curve.h"
#include"surface.h"
#include"curve.h"
#include"curvemanager.h"
#include"part.h"
using std::string;

struct Part;
struct Curve;
struct RigidCuda;
struct NodeCuda;
struct Surface;
struct CurveManager;

typedef struct Load
{
	enum Type
	{
		Concentrate, uniform, body
	};
	enum LoadType
	{
		accel, vel, disp
	};
	enum ObjectType
	{
		setObject, partObject, surfaceObject
	};
	int id;
	int dof[6];
	int setId;
	int nodeId;
	int curveId;
	double loadValue;
	Type type;
	LoadType load_type;
	ObjectType object_type;

	/**
	�Ŵ�����
	*/
	double scaleFactor;

	/**
	���ܵ��غ�����
	*/
	Curve *curve;

	/**
	�ռ��غ��е����нڵ�
	*/
	NodeCuda** node_array_gpu;
	int tot_node_num;

	/**
	���������ڵ����ڵ���
	*/
	NodeCuda *node;

	/**
	����������set�е����нڵ���
	*/
	Set *set;

	/**
	����������part�����нڵ���
	*/
	Part *part;

	/**
	����ʩ���ڸ�����
	*/
	RigidCuda *rigid;

	/**
	����ʩ���ڱ�����
	*/
	Surface *surface;

	/**
	��������������������нڵ�
	*/
	vector<NodeCuda *> node_array;

	/**
	�ռ��غ��е����нڵ�
	*/
	void createLoadNodeArray(NodeManager *nodeManager);

	/**
	�������нڵ�ʩ���غ�
	*/
	void imposeLoadAccel(double current_time);

	/**
	ȷ����ǰʱ����غ�ֵ
	*/
	void verfLoadValue(double current_time);

	/**
	�����غ�����
	*/
	void linkLoadCurve(CurveManager *curveManager);

	/**
	����ʩ���غɵĲ���������
	*/
	void linkLoadObject(SetManager *setManager, vector<Part> &partArray, SurfaceManager *surfaceManager);
} Load;


enum NewLoadType
{
	LNull,
	ConcentratedForce,
	Pressure,
	BodyForce,
	Gravity,
	Moment,
	ShellEdgeForce,
	SegLoad_,
};

enum LoadObject
{
	nodeLoad,
	surfaceLoad,
	elementLoad,
	globalLoad
};

typedef struct BasicLoad
{
	int file_id;

	int curve_id;

	int object_id;

	int dof[6];

	double value_;

	double scaleFactor_;

	Curve* curve;

	NewLoadType lType_;

	LoadObject objectType_;

	bool isSet;

	string objectName_;

	string curveName_;

	BasicLoad();

	BasicLoad(int fid, int cv_id, bool setFlag = false,string name="noname",string ncvname="noname");

	virtual void getLoadObject(SetManager* setMag,SurfaceManager* surfMag, vector<Part>& partMag,NodeManager* nodeMag)=0;

	void linkLoadCurve(CurveManager* cvMag);

	virtual void applyLoadGpu()=0;

	virtual void applyLoadCpu()=0;

	virtual void computeLoadValueAt(double time);

 	virtual ~BasicLoad() { ; }
}BasicLoad;