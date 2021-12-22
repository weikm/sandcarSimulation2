#pragma once
#include<string>
#include"materialold.h"
#include"sectionold.h"
#include<vector>
#include"node.h"
#include "Elm2ndManager.h"
#include"elmmanager.h"
#include"materialmanager.h"
#include"sectionmanager.h"

using std::vector;

struct ElmManager;
struct MaterialNew;
struct Material;
struct Section;
struct NodeManager;
struct MaterialManager;
struct SectionManager;
struct FEMDynamic;
struct Surface;
struct SurfaceManager;

typedef struct Part
{
	int id;
	int SECID;
	int MID;
	int EOSID;
	int HGID;
	int GRAV;
	int ADPOPT;
	int TMID;
	std::string name;

	//ls:2020-04-06
	int storeNum;

	//PART_CONTACT
	double FS;
	double FD;
	double DC;
	double VC;
	double OPTT;
	double SFT;
	double SSF;
	//

	//ls:2020-03-17
	/**
	PART_INERTIA 
	*/
	double translationalMass;
	int IRCS;// Flag for inertia tensor reference coordinate system:
	int NODEID;
	int coordinateID;

	//ls:2020-04-06
	double XC;
	double YC;
	double ZC;
	double TM;
	double IXX;//component of inertia tensor
	double IXY;
	double IXZ;
	double IYY;
	double IYZ;
	double IZZ;
	double VTX;
	double VTY;
	double VTZ;
	double VRX;
	double VRY;
	double VRZ;
	double XL;
	double YL;
	double ZL;
	double XLIP;
	double YLIP;
	double ZLIP;
	//

	//

	/**
	�Ľڵ�������ڵ�ǵ���Ŀ
	*/
	/*int quad_shell_num;
	int tri_shell_num;*/

	/**
	��������
	*/
	double part_mass;

	/**
	part�еõ�Ԫ������
	*/
	ElmManager *elmManager;

	/**
	part����
	*/
	Material *material;
	Material *materialGpu;
	
	MaterialNew *matNew;
	MaterialNew *matNewGpu;

	/**
	part����
	*/
	Section *section;
	Section *sectionGpu;

	/**
	cpu��part�еĽڵ�
	*/
	vector<NodeCuda *> node_array_cpu;

	/**
	gpu��part�еĽڵ�
	*/
	NodeCuda **node_array_gpu;

	/*
	 * ��part�����ɵ�surface��
	 */
	int surfaceId;
	Surface *surface;
	

	Part();

	/**
	ͳ��part�еĽڵ�
	*/
	void setPartNode(NodeManager *node_manager);

	//ls:2020-04-18 added (Ϊ�˲�ȫ����)
	void partNodelist(NodeManager *node_manager, std::vector<int> node_list);
	//

	/**
	����gpu�˵Ľڵ�
	*/
	void gpuNodeCreate(NodeManager *node_manager);

	/**
	part��Ա����
	*/
	void memberSet(vector<ElmManager> &elm1stManager_array,	MaterialManager* materialManager, SectionManager* sectionManager);

	/*
	 * part������ʽ����
	 */
	void setMaterialNew(MaterialManager* materialManager);

	/**
	���ϼ����Ը���
	*/
	void matPrpCpyGpu(MaterialManager *mat_manager, SectionManager *section_manager);

	/*
	 * ����part�����е�Ԫ�Ļ��ֵ�Ȩ������Ȼ����
	 */
	void calElementGaussCoordWeight(SectionManager* secManger);

	/**
	���ò����нڵ�ĺ��
	*/
	void setNodeThick();

	/**
	��gpu�ָ��
	*/
	void splitPartMultiGpu(int totNdNum, int num_gpu, vector<int> &elm_num_gpu);

	/*
	 * ����part�е���С��Ԫ��������
	 */
	void computeElementCharactLengthCpu(double& min_lg);
	void computeElementCharactLengthGpu(double& min_lg);

	/*
	 * ��part����������
	 */
	void produceSurface(FEMDynamic *domain,Surface *surf,SurfaceManager *surfMag);

	void setSubSurface(Surface *sf,int sfId);

	virtual ~Part();
} Part;