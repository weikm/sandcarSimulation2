#pragma once
#include"surface.h"
#include"contactnode.h"
#include"pairmanager.h"
#include"contactmanager.h"

typedef struct Sub_Hita_Field
{
	double xmax[3];
	double xmin[3];

	/**
	主从接触面
	*/
	Surface *sgManager_1;
	Surface *sgManager_2;

	/**
	以点对面形式参与接触的碰撞点集合
	*/
	vector<ContactNodeCuda*> contactNodeArray;

	bool impactSign;
	int minPredictCyc;
	int cycCount;
} Sub_Hita_Field;

typedef struct HitaContactManager
{
	/**
	以点的形式参与接触的节点
	*/
	vector<ContactNodeCuda *> slaveNodeArray;
	/**
	参与接触的所有面
	*/
	vector<Surface *> surfaceArray;

	/**
	参与接触的元素的编号
	*/
	vector<int> surfaceIdArray;
	vector<int> partId;
	vector<int> setId;

	/**
	分割的子域
	*/
	vector<Sub_Hita_Field> subFieldArray;

	/**
	所有参与接触的碰撞点
	*/
	vector<ContactNodeCuda *> AllContactNodeCudaArray;

	/**
	接触控制距离
	*/
	double controlDist;

	/**
	自接触标识
	*/
	bool selfContactSign;

	/**
	接触转换
	*/
	void masterSlaveTansform(vector<Contact> &contactArray);

	/**
	清除重复接触点
	*/
	void clearDupliConNode();

	/**
	生成测试对
	*/
	void produceTestPair(const double dt,int cyc_num,PairManager *pair_manager);

}HitaContactManager;