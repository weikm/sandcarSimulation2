#pragma once
#include"rigidmanager.h"
#include"boundarymanager.h"
#include "elmmanager.h"
#include"setmanager.h"
#include"tiemanager.h"
#include"loadmanager.h"
#include"elementcontrol.h"
#include"constrainedmanager.h"
#include"timecontrol.h"
#include"multigpumanager.h"
#include"contactmanager.h"
#include"curvemanager.h"
#include"initialmanager.h"
#include"materialmanager.h"
#include"sectionmanager.h"
#include"hita.h"
#include"unified_contact.h"
#include"nodemanager.h"
#include"part.h"
#include"hourglasscontrolmanager.h"
#include"outputmanager.h"
#include<vector>
#include "computedevice.h"
//ls:2020-03-18
#include "accuracycontrol.h"
#include "contactcontrol.h" 
#include "historynode.h" 
#include "cpucontrol.h"
#include "energycontrol.h"
#include "extentbinary.h"
#include "outputcontrol.h"
#include "shellcontrol.h"
#include "solidcontrol.h"
#include "Airbagfile.h"
#include "AirbagManager.h"
#include "ElementMass.h"
#include "Seatbelt_Accelerometer.h"
#include "Tablemanager.h"
#include "coor_nodesmanager.h"
#include "inc_transformanager.h"
#include "def_transformanager.h"
#include "elementmassmanager.h"
#include "accelermetermanager.h"
//
//ls:2020-04-06
#include "boxmanager.h"
#include "controlmanager.h"
#include "databasemanager.h"
#include "rigidwallmanager.h"
#include "historynodemanager.h"
//#include "rigidwall.h"
//
#include "rigidExtraNodeManager.h"

using std::vector;

struct OutputManager;
struct ContactManager;
struct NodeManager;
struct ContactNodeManager;
struct PairManager;
struct SurfaceManager;
struct MaterialManager;
struct SectionManager;
struct CurveManager;
struct ElementMassManager;
struct RigidManager;
struct BoundaryManager;
struct RigidwallManager;
struct LoadManager;

struct RigidExtraNodeManager;

typedef struct FEMDynamic
{
	double *globalSpeed;
	double *globalDisp;
	double *globalAcceleration;
	double *globalMass;
	double *globalJointForce;
	double *globalDispIncre;
	double *cc;

	/**
	ʱ�䰲ȫ����
	*/
	double scl;
	int *dofBounSign;
	int node_amount;
	int elementAmout;

	int cyc_num;
	double dt;
	double previous_dt;
	double current_time;

	string intFilePath_;
	string outputFilePath_;

	ComputeDevice calDevice_;

	vector<Part> partArray_;
	vector<ElmManager> elmentManager_array;
	NodeManager *nodeManager;
	ContactNodeManager *contactNodeManager;
	PairManager *pair_manager;
	SurfaceManager* surfaceManager;
	MaterialManager* materialManager;
	SectionManager *sectionManager;
	CurveManager *curveManager;
	//ls:2020-03-17
	TableManager *tableManager;
	Coor_nodesManager *coor_nodesManager;
	Def_transforManager *def_transforManager;
	Inc_transforManager *inc_transforManager;
	//
	//ls:2020-03-18
	RigidwallManager *rigidwallManager;
	HistorynodeManager *historynodeManager;
	AirbagManager *airbagManager;
	ElementMassManager *elementMassManager;
	AccelermeterManager *accelermeterManager;
	//
	//ls:2020-04-06
	BoxManager *boxManager;
	ControlManager *controlManager;
	DatabaseManager *databaseManager;
	//
	SetManager *setManager;
	BoundaryManager *boundManager;
	LoadManager *loadManager;
	ConstrainedManager *constraManager;
	InitialManager *initialManager;
	TimeControl *timeControl;
	RigidManager *rigidManager;
	ElementControl *elmControl;
	HourglassControlManager *hgControlManager;  //ls:2020-04-10
	TieManager *tieManager;
	IntegratePointManager *integraPointManager;
	SmoothFemManager *smoothFemManager;
	OutputManager* outputManager;

	RigidExtraNodeManager *rigidExtraNodeManager;

	//ls:2020-04-06
	map<int, int>partID_ls; //part
	map<int, int>node_id_ls; //node
	map<int, int>element_id_ls; // element
	map<int, int>surface_id_ls; //surface
	//

	FEMDynamic();

	/**
	��gpu������
	*/
	MultiGpuManager *multiGpuManager;

	/**
	�Ӵ������ṩͳһ�Ķ�ȡ����ʵ���������������Ӵ��㷨
	*/
	ContactManager *contactManager;

	/**
	���нӴ��㷨��һ�廯���Ӵ��㷨
	*/
	UnifiedContactManager *unifiedContactManager;

	/**
	���нӴ��㷨�����򷨽Ӵ��㷨
	*/
	HitaContactManager *hitaManager;

	/**
	�Ӵ���ײ��gpu���м���
	*/
	void DynamicAnalysisParallelMultiGpu(std::ofstream &fout, double &cnTime, double& fileTime);

	/**
	�Ӵ���ײ��ʽ����ԪGpu���м���
	*/
	void DynamicAnalysisParallel(std::ofstream &fout, double& cnTime, double& fileTime);

	/**
	�Ӵ���ײ��ʽʽ����Ԫ����
	*/
	void DynamicAnalysis(std::ofstream &fout, double &cnTime, double& fileTime);

	/**
	cpu�����Ӵ���ķ�����
	*/
	void cpuCalSegmentPenaFactor(const double pena_scale);

	/**
	��gpu���м����������
	*/
	/**
	��gpu��ʼ��
	*/
	bool initialComputeMultiGpu();

	/**
	��gpu�Դ����
	*/
	void allocateAndCopyMultiGpu();

	/*
	 * ��GPU��ʼ��
	 * ������gpu�ĽӴ��㷨�ĳ�ʼ�����㿽���ڴ�ķ���
	 */
	bool initialComputeSingleGpu();

	/*
	 * ��GPU�Դ�����븴��
	 * ������Ҫ����ĵ����ݣ��絥Ԫ�ڵ���Ͻ������Եȵĸ���
	 * ����Լ����Ϣ�纸�Ӱ󶨵����ݵĸ���
	 */
	void allocateAndCopySingleGpu();

	/*
	 * cpu��ʼ��
	 * �����Ӵ���Ѱǰ������׼�����������㣬���Ӱ󶨵ȹ�ϵ�ĳ�ʼ����
	 * ����TL��B����ļ���
	 */
	bool initialComputeCpu();

	/**
	�ڴ����
	*/
	void femAllocateCpu();

	/**
	�ڴ��ͷ�
	*/
	void femFreeCpu();

	~FEMDynamic();

} FEMDynamic;
