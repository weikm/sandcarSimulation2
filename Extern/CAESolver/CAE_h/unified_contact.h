#pragma once
#include"contactnode.h"
#include"contactnodemanager.h"
#include"contactmanager.h"
#include"surface.h"
#include"pairmanager.h"

struct ContactNodeCuda;
struct PairManager;
struct Surface;
struct FEMDynamic;

using std::vector;
typedef struct SubDomain
{
#define subDomainMaxContactNodeCuda 150
#define subDomainMaxSegmentCuda 150
	ContactNodeCuda* contact_node_array_gpu[subDomainMaxContactNodeCuda];
	SegmentCuda* segment_array_gpu[subDomainMaxSegmentCuda];

	/*
	子域中接触点个数
	*/
	int contact_node_num;

	/**
	子域中接触块个数
	*/
	int segment_num;

	/**
	子域坐标
	*/
	int co_I;
	int co_J;
	int co_K;

	/**
	子域编号
	*/
	int id;

	/**
	进一步计算标志
	*/
	int nextCalFlag;

	SubDomain();

	/**
	数据清除
	*/
	__host__ __device__ void clearData();

	/**
	数据验证
	*/
	__device__ void dataVer(int *next_cal_num,int *next_node_cal_num);
}SubDomain;

struct SubDomainSortCmp
{
	__host__ __device__ bool operator()(const SubDomain* x, SubDomain* y)
	{
		if ((x->nextCalFlag==1) && (y->nextCalFlag==0))
		{
			return true;
		}
		else
		{
			return false;
		}
	}
};

typedef struct SubDomainManager
{
	/**
	cpu端子域集合
	*/
	SubDomain* sub_domain_array_cpu;

	/**
	gpu端子域集合
	*/
	SubDomain *sub_domain_array_gpu;

	/*
	* 多gpu端子域集合
	*/
	vector<SubDomain*> subDomainArrayMultiGpu;

	/**
	gpu端子域指针集合
	*/
	SubDomain** sub_domain_ptr_array_gpu;

	/**
	标记是否是以从面为基准建立子域
	*/
	int isSlaveBased;

	/**
	子域总数
	*/
	int sub_domian_num;

	/**
	需要进一步计算的子域数目
	*/
	int *next_cal_num_gpu;
	int *next_cal_num_cpu;

	vector<int> domainNumForNextCalCpu;
	vector<int*> domainNumForNextCalMultiGpu;

	/*
	 * 根据两个面的扩展域是否存在相交判断下一计算与否
	 */
	bool isNeedNextCal_;

	/**
	需要进一步计算的子域中所有的节点数目
	*/
	int *next_cal_node_num_gpu;
	int *next_cal_node_num_cpu;

	vector<int> nodeNumForNextCalCpu;
	vector<int*> nodeNumForNextCalMultiGpu;

	/**
	三个坐标轴最大编号，只计算一次
	*/
	int3 axi_num;

	/**
	三个坐标轴最大值
	*/
	double3 max_val;

	/**
	三个坐标轴最小值
	*/
	double3 min_val;

	/**
	子域三坐标轴上的单位长
	*/
	double3 scal;

	/**
	各个block块中的坐标最大最小值
	*/
	double3 *max_gpu;
	double3 *min_gpu;

	double3 *max_cpu;
	double3 *min_cpu;

	/**
	作为检查是否需要生成测试对的不被子域包含的另一部分的最大最小值
	*/
	double3 *another_max_gpu;
	double3 *another_min_gpu;

	double3 *another_max_cpu;
	double3 *another_min_cpu;

	/**
	主从面重叠区域的最大最小坐标
	*/
	double3 overlap_max;
	double3 overlap_min;

	/**
	以主面所属的碰撞管理器为基准
	*/
	Contact *parentContactManager;

	/**
	主面
	*/
	Surface *masterSurface;
	
	/**
	以面的形式参与接触的从面
	*/
	Surface *slaveSurface;

	/**
	以点的形式参与接触的碰撞点集合
	*/
	vector<ContactNodeCuda *> slave_contact_node_array_cpu;
	ContactNodeCuda** slave_contact_node_array_gpu;

	/*
	* 多gpu接触点合集
	*/
	vector<int> slaveContactNodeNumMultiGpu;
	vector<ContactNodeCuda**> slaveContactNodeArrayMultiGpu;

	/**
	cpu及gpu端子域生成
	*/
	void cpuSubDomainCreate();

	void gpuSubDomainCreate();
	void multiGpuSubDomainCreate(vector<int> gpuIdArray);

	void loopSubDomainCreate();
	void loopSubDomainCreateMultiGpu(int gpu_id, cudaStream_t stream);

	void getSubDomainDataFromMasterSurf();

	void updateSubDomain();

	/**
	在子域数目不变或者减少的情况下，重新规划子域
	*/
	void replanSubdomains(Surface *segment_manager);

	/**
	子域填充
	*/
	void segmentFullSubDomain(Surface *surface_temp);
	void segmentFullSubDomainMultiGpu(Surface* sur, int gpuId, cudaStream_t& stream);

	void contactNodeFullSubDomain(ContactNodeCuda **contact_node_array_gpu, int contact_node_num);
	void contactNodeFullSubDomainMultiGpu(ContactNodeCuda** contactNodeArrayGpu, int gpu_id, int nodeNum, cudaStream_t& stream);

	void nodeAndSegmentFullSegment(Surface *surface_temp, ContactNodeCuda **contact_node_array_gpu, int contact_node_num);

	/**
	子域清除
	*/
	void clearSubDmain();

	/**
	清楚子域中的数据
	*/
	void clearDataInSubDomain();

	/*
	* 多Gpu清除子域中的数据
	*/
	void clearDataInSubDomainMultiGpu(int gpuId, cudaStream_t& stream);

	/**
	子域数据验证
	*/
	void allDataVer();
	void allDataVerMultiGpu(int gpu_id, cudaStream_t& stream);

	void allDataVerBaseOnContactNode();
	void allDataVerBaseOnContactNodeMultiGpu(int gpu_id, cudaStream_t& stream);

	/**
	分配零拷贝内存
	*/
	void allZeroMemSub();
	void allNeedMemMultiGpu(vector<int>& gpuIdArray);

	/**
	对子域指针进行排序
	*/
	void sortSubDomainPtr();

	/**
	对碰撞点进行排序
	*/
	void sortContactNode();

	/*
	* 多gpu对接触点进行排序
	*/
	void sortContactNodeMultiGpu(int gpu_id);

	/**
	生成子域Gpu指针集合
	*/
	void createPtrArray();

	/**
	根据主面与从节点是否有相交区域判断是否需要进行接触前搜寻，并返回其交叉域
	*/
	bool judgeNeedForPreContactSearch();
	bool judgeNeedForPreContactSearchMultigpu();

	bool judgeNeedForPreContactSearchImprove();

	virtual ~SubDomainManager();
} SubDomainManager;

typedef struct UnifiedContactManager
{
	/**
	各自成体系的子域部分
	*/
	vector<SubDomainManager> subDomainManagerArray;

	vector<bool> isNeedJudge;

	/**
	以part形式参与接触
	*/
	vector<int> part_id;

	double control_dist;
	bool self_contact_sign;

	/**
	子域法接触算法的准备阶段,统计接触面与碰撞点
	*/
	void dataCollect(vector<Contact> &contactArray,ContactNodeManager* contact_node_manager,FEMDynamic *domain);

	/*
	* 多gpu子域法准备阶段，统计每个gpu上的接触点
	*/
	void dataCollectMultiGpu(vector<Contact> &contactArray, ContactNodeManager* contact_node_manager, vector<int> gpuIdArray);

	/**
	接触前搜索
	*/
	void searchBeforeContactGpu(PairManager *pair_manager);

	/**
	双向接触搜索
	*/
	void searchBeforeContactBilateralGpu(PairManager *pair_manager);

	~UnifiedContactManager()
	{
		subDomainManagerArray.clear();
	}
} UnifiedContactManager;
