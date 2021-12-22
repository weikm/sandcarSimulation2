#pragma once
#include"testpair.h"
#include"contactpair.h"
#include"unified_contact.h"
#include"contactnodemanager.h"
#include"surface.h"
#include "contactmanager.h"

#define DefenNodeDecople

struct UnifiedContactManager;
struct ContactPairCuda;
struct TestPairCuda;

typedef struct PairManager
{
	typedef enum SearchType
	{
		OneDirec, TwoDirec
	}SearchType;

	typedef enum ContactForceMethod
	{
		DefenceNode, Penalty
	}ContactForceMethod;

	/**
	��ǽӴ�����Ѱ���Ӵ��������Ƿ���˫���
	*/
	SearchType search_type;

	/**
	��۽Ӵ����ļ��㷽��
	*/
	ContactForceMethod cal_con_method;

	/**
	gpu���Զ�
	*/
	TestPairCuda** test_pair_array_gpu;

	/*
	* ��gpu���Զ�
	*/
	vector<TestPairCuda**> testPairArrayMultiGpu;
	vector<int> testPairNumMultiGpu;

	/**
	cpu���Զ�
	*/
	vector<TestPairCuda*> test_pair_array_cpu;

	/**
	�ܵĽӴ�����Ŀ
	*/
	int contact_node_num;

	/**
	��һ���ĽӴ�����Ŀ��Ϊ��һ���ĽӴ�������׼��
	*/
	int previous_contact_pair_num;

	/**
	gpu�˲��Զ���Ŀ����
	*/
	int* test_pair_num_gpu;
	int* test_pair_num_cpu;

	/*
	* ��gpu������һ����Ĳ��Զ���Ŀ
	*/
	vector<int> testPairForNextCalNumCpu;
	vector<int*> testPairForNextCalNumMultiGpu;

	/**
	gpu����ײ��
	*/
	ContactPairCuda** contact_pair_array_gpu;

	/*
	* ��Gpu�Ӵ���
	*/
	vector<ContactPairCuda**> contactPairArrayMultiGpu;
	vector<int> contactPairNumMultiGpu;

	/**
	cpu����ײ��
	*/
	vector<ContactPairCuda*> contact_pair_array_cpu;

	/**
	gpu����ײ����Ŀ����
	*/
	int* contact_pair_num_gpu;
	int* contact_pair_num_cpu;

	/*
	* ��gpu������һ������ĽӴ�����Ŀ
	*/
	vector<int> contactPairNumForNextCalCpu;
	vector<int*> contactPairNumForNextCalMultiGpu;

	int minPredictCycles;

	/**
	���Զ�����
	*/
	void testPairSortGpu();
	void testPairSortCpu();

	/*
	* ��gpu���Զ�����
	*/
	void testPairSortMultiGpu(int gpu_id);

	/**
	��ײ������
	*/
	void contactPairSortGpu();
	void contactPairSortCpu();

	/*
	* ��gpu��ײ����
	*/
	void contactPairSortMultiGpu(int gpu_id);

	/**
	�Ӵ��ԽӴ�������
	*/
	void defenCalContactForGpu(double dt, int cyc_num, ContactNodeManager *contact_node_manager);
	void contactForCalCpu(double dt, int cyc_num);

	/*
	* ��gpu����Ӵ���
	*/
	void defenCalContactMultiGpu(vector<int> gpuIdArray, vector<cudaStream_t> streamArray,
		double dt, int cyc_num, ContactNodeManager *cnNdMag);

	/**
	���ɽӴ���
	*/
	void produceContactPairGpu(int cyc_num, int mstp);
	void produceContactPAirCpu(int cyc_num, int mstp);

	/*
	* ��gpu���ɽӴ���
	*/
	void produceContactPairMultiGpu(vector<int> gpuIdArray, vector<cudaStream_t> streamArray, int cyc_num, int mstp);

	/**
	�Ӵ�������
	*/
	void searchAfterContactGpu();
	void searchAfterContactCpu();
	void searchAfterContactMultiGpu(vector<int> gpuIdArray, vector<cudaStream_t> streamArray);

	/*
	 * ˳��ʽ�Ӵ�����Ѱ
	 */
	void searchAfterContactGpuBySequential();

	/**
	�Ӵ�ǰ����
	*/
	void searchBeforeContactGpu(UnifiedContactManager *unifiedContactManager);

	void searchBeforeContactGpuImprove(UnifiedContactManager *unifiedContactManager);

	/*
	* ��gpuһ�廯���Ӵ��㷨
	*/
	void searchBeforeContactMultiGpu(UnifiedContactManager* unifiedContactManager, vector<int> gpuIdArray, vector<cudaStream_t> steamArray);

	/**
	˫��Ӵ�ǰ����
	*/
	void searchBeforeContactBilateralGpu(UnifiedContactManager *unifiedContactManager);

	/**
	�����еĽӴ������ɷ���ĽӴ���
	*/
	void produceReverseContactPair();

	/**
	�Ӵ��Լ�������
	*/
	void gpuContactPairSetCreate(ContactNodeManager *contact_node_manager);
	void multiGpuContactPairSetCreate(ContactNodeManager *cnNdMag, vector<int> gpuIdArray);

	/**
	gpu�˲��ԶԼ�������
	*/
	void gpuTestPairSetCreate(ContactNodeManager *contact_node_manager);
	void multiGpuTestPairSetCreate(ContactNodeManager *cnNdMag, vector<int> gpuIdArray);

	/**
	�����еĲ��ԶԽ��м�⣬�޳���������Ĳ��Զ�
	*/
	void gpuTestAllPair();

	/**
	�����������
	*/
	void clearCount();

	/**
	gpu��ͳ����Ҫ������һ������Ľڵ�
	*/
	void countNextTestPairCal();

	/*
	* ��gpuͳ����Ҫ������һ������Ľڵ�
	*/
	void countNextTestPairCalMultiGpu(vector<int> gpuIdArray, vector<cudaStream_t> streamArray);

	/**
	�����������Ӵ���
	*/
	void penaltyCalContForGpu(double dt, int cyc_num, int penalty_method);

	/**
	�����㸴���ڴ�
	*/
	void allocZeroMemPair();
	void allocMemForMultiGpu(vector<int> gpuIdArray);

	~PairManager();
} PairManager;