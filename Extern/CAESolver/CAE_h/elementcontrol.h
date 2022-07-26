#pragma once
#include <cuda_runtime.h>
#include <cublas.h>
#include "beam1st.h"
#include"shell1st.h"
#include"solid1st.h"
#include"discrete1st.h"
#include"node.h"
#include"timecontrol.h"
#include"materialold.h"
#include"sectionold.h"
#include"smoothFEM.h"
#include"gausspoint.h"
#include"timecontrol.h"
#include"smoothFemManager.h"
#include"integrapointmanager.h"
#include "hourglasscontrolmanager.h"

struct TimeControl;
struct IntegratePointManager;
struct GaussPoint;
struct SmoothFEM;
struct NodeCuda;
struct BeamElementCuda;
struct SolidElementCuda;
struct ShellElementCuda;
struct DiscreteCuda;
struct BeamElement2nd;
struct ShellElement2nd;
struct SolidElement2nd;
struct Section;
struct Material;
struct MaterialNew;
struct NodeManager;
struct Part;
struct SmoothFemManager;
struct HourglassControl;

/*
 * ������ԪGPU����������Ҫ������
 * ������Ԫ�����ϣ��������ԣ����ֵ㣬�ڵ��
 */
typedef struct ElementControl
{
	GaussPoint* gauss_array_gpu;
	NodeCuda* node_array_gpu;

	/*
	 * ��gpu��
	 */
	BeamElementCuda** beamArrayGpu;
	SolidElementCuda** solidArrayGpu;
	ShellElementCuda** shellArrayGpu;
	DiscreteCuda** discreteArrayGpu;
	BeamElement2nd** beam2ndArrayGpu_;
	ShellElement2nd** shell2ndArrayGpu_;
	SolidElement2nd** solid2ndArrayGpu_;
	SmoothFEM* smoothArrayGpu;

	/*
	 * ��gpu��
	 */
	BeamElementCuda* beamMultiGpuArray_;
	SolidElementCuda* solidMultiGpuArray_;
	ShellElementCuda* shellMultiGpuArray_;
	DiscreteCuda* discreteMultiGpuArray_;
	BeamElement2nd* beam2ndArrayMultiGpu_;
	ShellElement2nd* shell2ndArrayMultiGpu_;
	SolidElement2nd* solid2ndArrayMultiGpu_;

	int beam_num;

	int solid_num;

	int shell_num;

	int discrete_num;

	int beam2ndNum_;
	int solid2ndNum_;
	int shell2ndNum_;

	int smooth_num;

	int gauss_num;

	int node_num;

	double *dt_cpu;
	double *dt_gpu;

	double *dt_array_gpu;

	Section *section_array_gpu;
	Material *material_array_gpu;
	MaterialNew** matnew_array_gpu;
	HourglassControl *hourglass_array_gpu;

	ElementControl();

	/*
	 * �������нڵ��Ӧ��Ӧ��,ע������ֻ�ǽ��ڵ��Ӧ�����ۼ�
	 */
	void accumulateElementStreeStrainToNode(Section *secArrayGpu);

	void allocateAllElementAddMemMultiGpu();

	void freeAllElementAddMemMultiGpu();

	void operatorOnAllElementAddMem(Section *sectionArrayGpu);

	/*
	 * ������gpu�ڵ㼯��
	 */
	void createNodeArraySingleGpu(NodeManager *nodeManager);

	/*
	 * ������gpu�ڵ㼯��
	 */
	void createNodeArrayMultiGpu(NodeManager *nodeManager, const int in,
		const vector<cudaStream_t> &streamArray, const vector<int> &gpu_id_array);

	/*
	 * ������Gpu�Ĳ��ϡ������Լ�ɳ©���ƵȲ��뵥Ԫ�����ָ��
	 */
	void createMatSecHourgSingGpu(MaterialManager *matManager, SectionManager *secManager, HourglassControlManager *hgManager);

	/*
	 * ������Gpu�Ĳ��ϡ������Լ�ɳ©���ƵȲ��뵥Ԫ�����ָ��
	 */
	void createMatSecHourgMultiGpu(MaterialManager *matManager, SectionManager *secManager, HourglassControlManager *hgManager);

	/*
	 *������Ԫ����
	 */
	void createElementArraySingleGpu(vector<Part> &partArray_);

	/**
	������GPU��Ԫ����
	*/
	void createElementArrayMultiGpu(vector<Part> &partArray_, const int in,
		const vector<int> &elm_num_gpu, const vector<cudaStream_t> &streamArray, const vector<int> &gpu_id_array);

	/*
	 * ������gpu��ָ�븱��
	 */
	void createElementPtrCopy(const int in, const vector<cudaStream_t> &streamArray, const vector<int> &gpu_id_array);

	/*
	 * ������GPU�Ĺ⻬����Ԫ����
	 */
	void createSFEMArraySingleGpu(SmoothFemManager *smoothManager);

	/*
	 * ������gpu�汾�Ĺ⻬����Ԫ����
	 */
	void createSFEMArrayMulitlGpu(SmoothFemManager *smoothManager, NodeManager *ndMag, const int in,
		const vector<int> &elm_num_gpu, const vector<cudaStream_t> &streamArray, const vector<int> &gpu_id_array);

	/*
	 * ������gpu�Ļ��ֵ㼯��
	 */
	void createGaussPointArraySingleGpu(IntegratePointManager *integPointManager);

	/*
	 * ������gpu�Ļ��ֵ㼯��
	 */
	void createGaussPointArrayMultiGpu(IntegratePointManager *integPointManager, const int in,
		const vector<cudaStream_t> &streamArray, const vector<int> &gpu_id_array);

	/**
	��Ԫ�����������е�Ԫ��ϵ�ڵ�
	*/
	void allElementLinkToNode(NodeCuda *sys_node_array_gpu, cudaStream_t stream_gpu);

	/*
	* ��gpu��ʵ��Ԥ��������
	*/
	void bulidElementIndexNodeMultiGpu(const int in, const vector<cudaStream_t> &streamArray, const vector<int> &gpu_id_array);

	//����Ϊ�����еĵ�Ԫ����

	/**
	��Ԫ�������������ʱ����������������
	*/
	void allElemenArrayCalInterForGpu(MaterialNew **sys_material_array_gpu, Section *sys_section_array_gpu, HourglassControl *hourglass,
		double dt, cublasHandle_t cu_handle, double &dt_new, double safe_factor);

	void allElementArrayCalInterForMultiGpu(MaterialNew **sys_material_array_gpu, Section *sys_section_array_gpu, HourglassControl *hourglass,
		double dt, cublasHandle_t cu_handle, double &dt_new, double safe_factor, cudaStream_t stream = nullptr);

	/*
	 * �����е�Ԫ�����������ż���
	 */
	void massScaleForElementsGpu(const double min_dt);
	
	/**
	ͳ�������������ӵ�����
	*/
	void countAddedMassForMassScaled(const int isMassScaled);

	/**
	���ʱ������
	*/
	double calAllDt(Material *material_array, Section *section_array, const double time_step_scale);

	void callDtMultiGpu(cublasHandle_t cu_handle, double &dt_new, double safe_factor, cudaStream_t stream = nullptr);

	//��Ԫ���м����е�һЩ���⴦��

	/*
	 * ��Ԫָ�밴�㷨��������
	 */
	void sortElementPtrBaseType();

	/*
	 * ��Ԫ���㷨��������
	 */
	void sortElementBaseType();

	//����Ϊ��Ԫ�����������ڴ�ķ������ͷ�
	
	/**
	�����㿽���ڴ�
	*/
	void allocZeroMemElm();

	/**
	��֤���е�Ԫ�ڼ����в����ƻ�
	*/
	void verAllElementCompete();

	/**
	����dt_array��gpu�ڴ�
	*/
	void allocateDtArrayGpu();

	~ElementControl();

} ElementControl;
