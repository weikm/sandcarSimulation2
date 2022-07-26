//#pragma once
//#include "beam2nd.h"
//#include"shell2nd.h"
//#include"solid2nd.h"
//#include"materialold.h"
//#include"nodemanager.h"
//#include"sectionold.h"
//
//struct Section;
//struct NodeManager;
//struct BeamElement2nd;
//struct ShellElement2nd;
//struct SolidElement2nd;
//struct Material;
//
//typedef struct Elm2ndManager
//{
//	vector<BeamElement2nd> beam2ndArrayCpu_;
//	BeamElement2nd* beam2ndArrayGpu_;
//
//	vector<ShellElement2nd> shell2ndArrayCpu_;
//	ShellElement2nd* shell2ndArrayGpu_;
//
//	vector<SolidElement2nd> solid2ndArrayCpu_;
//	SolidElement2nd* solid2ndArrayGpu_;
//
//	int nodePerElement;
//	int node_amount;
//	int elementAmount;
//
//	/**
//	��Ԫ������Сֵ���ֵ
//	*/
//	double *eign_cpu;
//	double *eign_gpu;
//
//	/**
//	��Ԫ�������е�������
//	*/
//	double tot_mass;
//
//	/**
//	��Ԫ�����������
//	*/
//	void calElementMass(Material *material, Section *section);
//
//	/**
//	cpu��Ԫ�ڵ�����ƥ��
//	*/
//	void elementNodeMatch(NodeManager *node_manager);
//
//	/**
//	gpu�˵�Ԫ���ݴ���
//	*/
//	void gpuElementCreate(const Section *section);
//
//	/**
//	gpu�ڵ�����ƥ��
//	*/
//	void gpuNodeMatch(NodeManager *node_manager, const Section *section);
//
//	/**
//	gpu���������
//	*/
//	void gpuDataClear();
//
//	/**
//	�ڵ��ȷ���
//	*/
//	void shellNodeThickAndRadius(Section *section);
//
//	/**
//	������е�Ԫ����
//	*/
//	void allElementInterForceCalGpu(Material *material, Section *section, double dt);
//	void allElementInterForceCalCpu(Material *material, Section *section, double dt);
//
//	/**
//	����Ԥ�������������е�Ԫ��ڵ�֮���������Ϣ
//	*/
//	void bulidElementIndexNodeGpu();
//
//	/**
//	ȷ�������������е�Ԫ�Ĳ������Ա��
//	*/
//	void verElementMatSecId(const int mater_id, const int section_id);
//
//	/**
//	�����㿽���ڴ�
//	*/
//	void allocZeroMemEle();
//}Elm2ndManager;