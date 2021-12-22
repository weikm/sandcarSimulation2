#pragma once
#include"cublas.h"
#include"cublas_api.h"
#include"elementcontrol.h"
#include"hourglasscontrolmanager.h"
#include"materialmanager.h"
#include"sectionmanager.h"
#include"part.h"

struct MaterialManager;
struct ElementControl;
struct TimeControl;
struct HourglassControlManager;
struct SectionManager;
struct Part;

/*
* ��ǰ��gpu��˼·һ����������
* 1��ÿ��gpu�Ͼ������нڵ㣬��ͬgpu֮��ÿ�ε���������gpu�������ݣ��漰��ͨѶ���Ƚϴ�Ŀǰ������
* 2�����нڵ����������gpu�ϣ�����gpuͨ��p2p���з��ʣ��漰��ͨѶ�����٣�����ʵ�֣�Ŀǰ��������
* 3������gpu�������Σ�ÿ��gpu�Ͼ����沿�ֽڵ㣬���ڱ߽��Ͻ���ͨѶ��ͨѶ�����٣�������ʵ�֣��д���������
*/
typedef struct MultiGpuManager
{
	int num_gpu;

	/**
	�������gpu�ϵĵ�Ԫ���ƵĹ�����
	*/
	vector<ElementControl*> elementControlArray;

	/**
	�洢����gpu�Ͻڵ�����
	*/
	/*vector<double *> multi_intforce_array_gpu;*/

	/**
	�洢����gpu�Ͻڵ������
	*/
	/*vector<double *> multi_coord_array_gpu;*/

	/**
	�洢����gpu�Ͻڵ���ٶ�
	*/
	/*vector<double *> multi_vel_array_gpu;*/

	/**
	��gpu�ϵĴ����н���������ֶ�Ϊ�������֣���һ����Ϊ���꣬�ڶ�����Ϊ�ٶȣ���������Ϊ�ڵ�����
	*/
	/*vector<double *> mediun_variable_array;*/

	/**
	�洢����gpu�ϵĽڵ�
	*/
	/*vector<NodeCuda *> multi_node_array_gpu;*/

	/**
	�洢���������Ͻڵ�ı��
	*/
	vector<int*> multiNodeIdArrayAidGpu;
	vector<int*> multiNodeIdArrayMainGpu;
	vector<vector<int>> multiNodeIdArrayCpu;

	/**
	����gpu�ϵ�Ԫ��Ŀ
	*/
	vector<int> elm_num_gpu;

	/**
	����gpu��ʹ�õĽڵ���Ŀ
	*/
	vector<int> multi_node_num;

	/**
	�����ڸ���gpu�ϵ���
	*/
	vector<cudaStream_t> streamArray;
	vector<cublasHandle_t> cudaHandleArray;

	/**
	����ʹ�õļ�����������6.0��gpu_id����һ��gpuΪ��gpu������gpuΪ��GPU
	*/
	vector<int> gpu_id_array;

	bool isP2P;

	/**
	�洢�ڸ���gpu�ϵĲ���
	*/
	/*vector<MaterialNew **> material_array_gpu;*/

	/**
	�洢�ڸ���gpu�ϵ�����
	*/
	/*vector<Section *> section_array_gpu;*/

	MultiGpuManager();

	/**
	����gpu��Ŀ
	*/
	void judgeGpuNum();

	/**
	������gpu��ƥ�����
	*/
	void createStream();

	/*
	* �豸ͬ��
	*/
	void synchronousDevice();

	/*
	 * ��ͬ��
	 */
	void synchronousStream();

	/*
	 * ������
	 */
	void destroyStream();

	/*
	* �������
	*/
	void createCublasHandle();

	/*
	* ���پ��
	*/
	void destroyCublasHandle();

	/**
	����gpu�Ͻڵ�ķ����븴��
	*/
	/*void multiGpuNodeCpy(NodeCuda *sys_node_array_gpu,int tot_node_num);*/

	/**
	����gpu�ϵĲ��������Եĸ���
	*/
	void multiGpuMatSectionCpy(MaterialManager *material_manager, SectionManager *section_manager, HourglassControlManager* hgManager);

	/**
	����gpu�Ͻڵ��������ķ���
	*/
	/*void allocaIntForArrayMultiGpu();*/

	/**
	����gpu�Ͻڵ������ٶ��������Դ����
	*/
	/*void allocateCoordVelMultiGpu();*/

	/**
	��gpu���н�����������Դ����
	*/
	/*void allocateMediaVariable();*/

	/**
	��gpu�����ݵ���gpu��
	*/
	/*void assemblyIntForToMainGpu();*/

	/**
	��gpu�ٶ��Լ����괫�ݵ���gpu��
	*/
	/*void distributDispVelToAidGpu();*/

	/**
	�����������ɵĻ����ϣ��γ�gpu��cpu�Ͻڵ�����к�
	*/
	void createNodeIdCpu(const vector<Part>& partArray_, int node_num);
	void createNodeIdGpu();

	/**
	����������Ļ����ϣ��������������ĵ�Ԫ������
	*/
	void createElementControlForAllGpu(vector<Part> &partArray_);

	/*
	* p2p��֤
	*/
	void verP2PForMultiGpu();

	/**
	����gpu�Ϸ����ĵ�Ԫ���ӽڵ�
	*/
	/*void elementControlLinkNode();*/

	/**
	�������gpu�����еĵ�Ԫ�������еĵ�ԪӦ��
	*/
	void allGpuCalElementIntForAndDt(double &dt_new, double dt, HourglassControl *hourg_control, TimeControl *time_control);

	/*
	 * �����豸
	 */
	void resetDevice();

}MultiGpuManager;

//typedef struct MultiGpuManager
//{
//	int num_gpu;
//
//	/**
//	�������gpu�ϵĵ�Ԫ���ƵĹ�����
//	*/
//	vector<ElementControl*> elementControlArray;
//	
//	/**
//	�洢����gpu�Ͻڵ�����
//	*/
//	vector<double *> multi_intforce_array_gpu;
//
//	/**
//	�洢����gpu�Ͻڵ������
//	*/
//	vector<double *> multi_coord_array_gpu;
//
//	/**
//	�洢����gpu�Ͻڵ���ٶ�
//	*/
//	vector<double *> multi_vel_array_gpu;
//
//	/**
//	��gpu�ϵĴ����н���������ֶ�Ϊ�������֣���һ����Ϊ���꣬�ڶ�����Ϊ�ٶȣ���������Ϊ�ڵ�����
//	*/
//	vector<double *> mediun_variable_array;
//	
//	/**
//	�洢����gpu�ϵĽڵ�
//	*/
//	vector<NodeCuda *> multi_node_array_gpu;
//
//	/**
//	�洢���������Ͻڵ�ı��
//	*/
//	vector<int*> multi_nodeid_array_aid_gpu;
//	vector<int*> multi_nodeid_array_main_gpu;
//	vector<vector<int>> multi_nodeid_array_cpu;
//
//	/**
//	����gpu�ϵ�Ԫ��Ŀ
//	*/
//	vector<int> elm_num_gpu;
//
//	/**
//	����gpu��ʹ�õĽڵ���Ŀ
//	*/
//	vector<int> multi_node_num;
//
//	/**
//	�����ڸ���gpu�ϵ���
//	*/
//	vector<cudaStream_t> streamArray;
//
//	/**
//	����ʹ�õļ�����������6.0��gpu_id����һ��gpuΪ��gpu������gpuΪ��GPU
//	*/
//	vector<int> gpu_id_array;
//
//	/**
//	�洢�ڸ���gpu�ϵĲ���
//	*/
//	vector<MaterialNew **> material_array_gpu;
//
//	/**
//	�洢�ڸ���gpu�ϵ�����
//	*/
//	vector<Section *> section_array_gpu;
//
//	/**
//	����gpu��Ŀ
//	*/
//	bool judgeGpuNum();
//
//	/**
//	������gpu��ƥ�����
//	*/
//	void createStream();
//
//	/**
//	����gpu�Ͻڵ�ķ����븴��
//	*/
//	void multiGpuNodeCpy(NodeCuda *sys_node_array_gpu,int tot_node_num);
//
//	/**
//	����gpu�ϵĲ��������Եĸ���
//	*/
//	void multiGpuMatSectionCpy(MaterialManager *material_manager, SectionManager *section_manager);
//
//	/**
//	����gpu�Ͻڵ��������ķ���
//	*/
//	void allocaIntForArrayMultiGpu();
//
//	/**
//	����gpu�Ͻڵ������ٶ��������Դ����
//	*/
//	void allocateCoordVelMultiGpu();
//
//	/**
//	��gpu���н�����������Դ����
//	*/
//	void allocateMediaVariable();
//
//	/**
//	��gpu�����ݵ���gpu��
//	*/
//	void assemblyIntForToMainGpu();
//
//	/**
//	��gpu�ٶ��Լ����괫�ݵ���gpu��
//	*/
//	void distributDispVelToAidGpu();
//
//	/**
//	�����������ɵĻ����ϣ��γ�gpu��cpu�Ͻڵ�����к�
//	*/
//	void createNodeIdCpu(const vector<Part> partArray_, int node_num);
//	void createNodeIdGpu();
//
//	/**
//	����������Ļ����ϣ��������������ĵ�Ԫ������
//	*/
//	void createElementControlForAllGpu(vector<Part> &partArray_);
//
//	/**
//	����gpu�Ϸ����ĵ�Ԫ���ӽڵ�
//	*/
//	void elementControlLinkNode();
//
//	/**
//	�������gpu�����еĵ�Ԫ�������еĵ�ԪӦ��
//	*/
//	void allGpuCalElementIntForAndDt(double &dt_new,double dt, HourglassControl *hourg_control, TimeControl *time_control, cublasHandle_t cu_handle);
//
//}MultiGpuManager;