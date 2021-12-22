#pragma once
#include"cuda_runtime.h"
#include"device_launch_parameters.h"
#include"formulatemode.h"
#include "elasticplasticmatrial.h"
#include"gausspoint.h"
#include"intergrationdomain.h"

struct MaterialNew;
struct GaussPoint;
struct Section;

typedef struct Element
{
	int part_id;

	int file_id;

	int store_id;

	int gaussPointIndex_;

	int node_num;

	int section_id;

	int material_id;

	/*
	ls:2020-07-22 added
	�ڲ����ɶȣ�����wilson����
	*/
	int numInterDof_;

	int addMemSizeForElement;
	int nodeNumForElement;

	int integratePointZ_;
	int integratePointXY_;
	int integratePointNum_;
	IntegrationDomain intType_;
	//

	GaussPoint *gaussPoint_;

	ElementNewType eType_;

	FormulateMode fMode;

	//Ŀǰ�⻬����Ԫֻ֧��һ��ʵ�嵥Ԫ
	SmoothFEMType sFemType_;

	ElementSectionType esType_;

	int additionalVariableNum;

	double *additionalVariablePtr = NULL;

	__host__ void setFormulateMode(FormulateMode fm = UL);

	/*
	ls:2020-07-22 added
	*/
	/*
	* ���ݻ��������״���û��ֵ�
	*/
	__host__ void computeGaussCoordWeight(GaussPoint *gpPtr);

	/*
	* �������Ի�������ֵ�
	*/
	__host__ void computeGaussOnLine(GaussPoint *gpPtr);

	/*
	* ���������λ�������ֵ�
	*/
	__host__ void computeGaussOnTriangle(GaussPoint *gpPtr);

	/*
	* �����ı��λ�������ֵ�
	*/
	__host__ void computeGaussOnSquare(GaussPoint *gpPtr);

	/*
	* �����������������ֵ�
	*/
	__host__ void computeGaussOnCube(GaussPoint *gpPtr);

	/*
	* �����������������ֵ�
	*/
	__host__ void computeGaussOnTetra(GaussPoint *gpPtr);

	/*
	* ���Ի��ֵ���Ȼ�����Լ�Ȩ������
	*/
	__host__ void setLineGaussCoordWeight(int nPoint, double coord[], double weight[]);

	/*
	* �����λ��ֵ���Ȼ�����Լ�Ȩ������
	*/
	__host__ void setTriGaussCoordWeight(int nPoint, double coordXi1[], double coordXi2[], double weight[]);

	/*
	* ��������ֵ���Ȼ�����Լ�Ȩ������
	*/
	__host__ void setTetrCoordWeight(int nPoint, double coordXi1[], double coordXi2[], double coordX3[], double weight[]);

	/*
	* ������������������ֵ�
	*/
	__host__ void computeGaussOnWedge(GaussPoint *gpPtr);

	__host__ __device__ void linkGaussPoint(GaussPoint* gaussPointArray,const ElementNewType myType,int gaussNum);

	__host__ void setGaussPointMatStatusId(const int gaussNum,const int matStatusIdStart);

	__host__ __device__ void linkMatStatusToGauss(const int gaussNum,MaterialStatus *matStatuArray,MaterialNew *mat);

	__host__ __device__ void convertMatStatusOfGaussPoint(int gaussNum, MaterialNew *material);

	__host__ __device__ void setGaussPointCoordWeight(Section *sec);

	__host__ void setETypeNodeNum(Section *sec);

	__host__ __device__ void computeAddMemSize(Section *sec);

	__host__ __device__ void allocateAddMemory();

	__host__ __device__ void freeAddMemory();

	__host__ __device__ void operatorOnAddMem(Section *sec);

	__host__ __device__ void setElementSectionType()
	{
		printf("not overloaded\n");
	}

}Element;

struct ElementSortTypeCmp
{
	__host__ __device__ bool operator()(const Element x, const Element y) const
	{
		if (x.eType_ < y.eType_)
		{
			return true;
		}
		else if (x.eType_ == y.eType_)
		{
			return x.material_id < y.material_id;
		}
		else
		{
			return false;
		}
		/*return x.eType_ < y.eType_;*/
	}
};

struct ElementPtrSortTypeCmp
{
	__host__ __device__ bool operator()(const Element* x, const Element* y) const
	{
		if (x->eType_ < y->eType_)
		{
			return true;
		}
		else if (x->eType_ == y->eType_)
		{
			return x->material_id < y->material_id;
		}
		else
		{
			return false;
		}
		/*return x->eType_ < y->eType_;*/
	}
};