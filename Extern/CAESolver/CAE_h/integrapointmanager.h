#pragma once
#include<vector>
#include"part.h"
#include"gausspoint.h"
#include"elementcontrol.h"
#include"thread_allocate.h"
#include"smoothFemManager.h"
#include"materialmanager.h"
#include"sectionmanager.h"
using std::vector;

struct ElementControl;
struct GaussPoint;
struct SmoothFemManager;
struct Part;
struct MaterialManager;

typedef struct IntegratePointManager
{
	vector<GaussPoint> gaussPointArrayCpu_;
	GaussPoint* gaussPointArrayGpu_;

	int gaussPointNum;

	IntegratePointManager(): gaussPointArrayGpu_(nullptr), gaussPointNum(0)
	{
		;
	}


	/**
	生成cpu积分点集合
	*/
	void createCpuInterPointForElemet(vector<Part> &partArray_, SmoothFemManager *smoothFemManager, MaterialManager *matManager);
	void createCpuInterPointForSFEM(SmoothFemManager *smoothFemManager);

	/**
	生成gpu积分点集合
	*/
	void createGpuInterPoint();

	/*
	 * 积分点从Cpu复制到Gpu
	 */
	void copyGaussPointFromCpuToGpu();

	/*
	 * 积分点与单元或者光滑域连接
	 */
	void gaussPointLinkToElementCpu(vector<Part> &partArray_, SmoothFemManager *smoothFemManager, MaterialManager *matManager);
	void gaussPointLinkToElementGpu(ElementControl *elm1stControl, MaterialNew **matNewArrayGpu,Section *secArrayGpu);

	void gaussPointLinkToSFEMCpu(SmoothFemManager *smoothFemManager,MaterialManager *materialManager);
	void gaussPointLinkToSFEMGpu(SmoothFemManager *smoothFemManager, MaterialManager *materialManager);

	/**
	计算积分点内力并离散到所有节点,积分点并行
	*/
	void calAllInterPointInterForceGpu();
	void calAllinterPointInterForceCpu();

	/*
	 * 根据TL格式分配积分点的形函数偏导矩阵内存
	 * UL格式下积分点形函数偏单矩阵不分配内存
	 */
	void allocateBMatrixForGaussCpu();
	void allocateBMatrixForGaussGpu();

	/*
	 * TL格式中积分点B矩阵从CPU复制到GPU
	 */
	void allocatCopyGaussBMatrixFromCpuToGpu();
	void allocatCopyGaussBMatrixFromCpuToMultiGpu(GaussPoint* gPArray);

	/*
	 * 验证TL格式中积分点B矩阵的正确性
	 */
	void verTLBMatrixGpu();

	/**
	析构删除所有内存
	*/
	~IntegratePointManager();
}IntegratePointManager;