#pragma once
#include "ElementMass.h"

struct ElementMassCuda;


typedef struct ElementMassManager
{
	vector<MassElementCuda> element_mass_array;
	int elm_num;

	//ElementMassManager();
	///**
	//���������ӵ��ڵ���
	//*/
	//void addMassToNode();

	///**
	//���ӽڵ�
	//*/
	//void linkNode(NodeManager *nodeManager);
}ElementMassManager;