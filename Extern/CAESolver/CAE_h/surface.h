#pragma once
#include"node.h"
#include"nodemanager.h"
#include"segment.h"
#include"contactnode.h"
#include"contactnodemanager.h"
#include"shell1st.h"
#include<string>

using std::string;

struct ShellElementCuda;
struct ContactNodeManager;
struct SegmentCuda;
struct ContactNodeCuda;
struct SubDomain;
struct Part;
struct FEMDynamic;

typedef struct Surface
{
	string name;

	string partName;

	vector<Part*> partArray;

	vector<int> partStoreIdArray;

	//ls:2020-04-06
	/**
	First\Second\Third\Fourth segment attribute default value
	*/
	double DA[4];
	string solver;
	//

	/*
	 *�������еĿ�
	 */
	vector<SegmentCuda> segment_array;
	SegmentCuda *segment_array_gpu;

	/*
	* ��Gpu�Ӵ���
	*/
	vector<SegmentCuda*> segmentArrayMulyiGpu;

	/*
	 *�������еĵ�
	 */
	vector<NodeCuda *> node_array;

	/*
	 *���������ײʱ�����ɵ���ײ�㣬��������ײ������
	 */
	vector<ContactNodeCuda *> contact_node_array_cpu;
	ContactNodeCuda **contact_node_array_gpu;

	/*
	* ��gpu�ϵĽӴ���
	*/
	vector<ContactNodeCuda**> contactNodeArrayMultiGpu;
	vector<int> contactNodeNumMultiGpu;

	/*
	 * ����������ڵ�����
	 */
	int subDomainNum_;
	SubDomain* subDomainArrayCpu_;
	SubDomain* subDomainArrayGpu_;
	vector<SubDomain*> subDomainArrayMultiGpu_;

	/*
	 * ���������ϵ�������������
	 */
	int3 axi_num;//ֻ�ڽӴ�׼���׶μ���һ��
	double3 max_val;
	double3 min_val;
	double3 scal;

	double3 another_max_cpu;
	double3 another_min_cpu;

	double3 *max_gpu;
	double3 *min_gpu;

	double3 *max_cpu;
	double3 *min_cpu;

	double velMaxModulus;

	/*
	 * ��Ǹ����Ƿ��������ٶ�
	 */
	bool isCalMaxVel;

	/*
	 * ��Ǹ����Ƿ�������ڵ�ĳߴ�
	 */
	bool isSearchNodeSize_;

	/*
	 * ��Ǹ����Ƿ���������
	 */
	bool isFullSubDomain_;

	/*
	 * ��Ǹ����Ƿ��������С
	 */
	bool isSearchSize_;

	/*
	 * ����Ƿ���¹���չ��
	 */
	bool isUdateExtDomain_;

	/*
	 * ����Ƿ������һ�廯�����е�����
	 */
	bool isClearSubDomainData_;

	double maxThick1;
	double *max_disp_incre;

	int id;
	int isym;
	int segment_num;

	/**
	�Ӵ��������С����
	*/
	double xmax[3];
	double xmin[3];

	/**
	�Ӵ�������ٶ�
	*/
	double vmax;

	/*
	 * �Ӵ�����С�߳�
	 */
	double min_length;

	/**
	�Ӵ���ƽ���߳�
	*/
	double ave_length;

	/**
	�Ӵ���ƽ����չ���С
	*/
	double ave_expand;

	/**
	�Ӵ������λ������
	*/
	double maxDispIncre;

	Surface();

	void clearDataInSubDomainGpu();

	void clearDataInSubDomainMultiGpu(int gpuId, cudaStream_t& stream);
	/**
	�������λ��������gpu�ڴ�
	*/
	void allocDispMem();

	void allocNeddMem();

	/**
	�������λ������
	*/
	void cpyDispIncre(double* disp_incre);

	/**
	�Ӵ���Ӵ���ͳ��
	*/
	void segmentCount();

	/**
	ͳ�Ʊ��Ӵ����ж��ٽڵ��Լ������Ӵ����ж��ٸ��ڵ�
	*/
	void nodeCount(NodeManager &node_manager);

	/**
	��Ȿ�����п�ķ�����
	*/
	void solveAllSegmentNorVec();

	/**
	ͳ�Ʊ��Ӵ����ж��ٽӴ���
	*/
	void contactNodeLink(ContactNodeManager *contactNodeManager);

	/**
	gpu����ײ������
	*/
	void gpuContactNodeGet(ContactNodeManager* contactNodeManager);

	/*
	* ��gpu��ײ������
	*/
	void multiGpuContactNodeGet(vector<int> gpuIdArray, ContactNodeManager* cnNdMag);

	/**
	Ѱ�����ڵĽӴ���,����ƽ���߳�
	*/
	void neighborSegmentCudaSearch();

	/*
	 * �������п����չ��ߴ�
	 */
	void calAllSegmentExtTerritorySize();

	/**
	cpu�˽Ӵ�������
	*/
	void cpuSegmentCudaCreate(NodeManager *node_manager);

	/**
	gpu�Ӵ�������
	*/
	void gpuSegmentCudaCreate(NodeManager *node_manager);

	/*
	* ��gpu�Ӵ�������
	*/
	void multiGpuSegmentCreate(vector<int> gpuIdArray, NodeManager* ndMag = nullptr);

	/**
	�ǵ�Ԫ���ɽӴ���
	*/
	void shellElementToSegmentCuda(const vector<ShellElementCuda> &elementArray);

	/**
	��Ԫ�������ɽӴ��棨�ǵ�Ԫ��ls:2020-07-10 added
	*/
	void shellElementSetTosegmentCuda(vector<ShellElementCuda*> element_array_cpu);

	/**
	ʵ�嵥Ԫ�������ɽӴ���
	*/
	void solidElementToSegmentCuda(Part* part_temp, NodeManager *nodeManager);

	/**
	���ɽӴ������չ��
	*/
	void produceExtendedDomain();

	/**
	gpu���ɽӴ������չ��
	*/
	void gpuUpdateExtendedDomainImprove();
	void gpuUpdateExtendedDomain();

	/*
	* ��gpu���½Ӵ������չ��
	*/
	void multiGpuUpdateExtendedDomainImprove(vector<int> gpuIdAraray, vector<cudaStream_t> streamArray);
	void multiGpuUpdateExtendedDomain(vector<int> gpuIdAraray, vector<cudaStream_t> streamArray);
	/**
	���ýӴ��������еĽӴ���ĺ��
	*/
	void setAllSegmentCudaThick();

	/**
	ָ�������еĽӴ����id
	*/
	void assignSegmentId(const int global_id);

	/**
	���Ӵ�����չ���ƽ����С
	*/
	void calAveExpandSize();

	void produceSegmentSetFromPart(FEMDynamic *domain);

	virtual ~Surface();
} Surface;