#pragma once
#include"shell1st.h"
#include"solid1st.h"
#include"beam1st.h"
#include"discrete1st.h"
#include"nodemanager.h"
#include"materialold.h"
#include"sectionold.h"
#include "beam2nd.h"
#include"shell2nd.h"
#include"solid2nd.h"
#include<map>

//ls:2020-03-18
//#include"Airbagfile.h"
//#include"ElementMass.h"
//#include"Seatbelt_Accelerometer.h"
//

using std::map;

struct ShellElementCuda;
struct SolidElementCuda;
struct BeamElementCuda;
struct DiscreteCuda;
struct Material;
struct Section;
struct NodeManager;
struct BeamElement2nd;
struct ShellElement2nd;
struct SolidElement2nd;

typedef struct ElmManager
{
	/**
	gpu��cpu�Ŀǵ�Ԫ
	*/
	vector<ShellElementCuda> shell_element_array_cpu;
	ShellElementCuda *shell_element_array_gpu;

	/*
	gpu��cpu��ʵ�嵥Ԫ
	*/
	vector<SolidElementCuda> solid_element_array_cpu;
	SolidElementCuda *solid_element_array_gpu;

	/**
	gpu��cpu������Ԫ
	*/
	vector<BeamElementCuda> beam_element_array_cpu;
	BeamElementCuda *beam_element_array_gpu;

	/**
	gpu��cpu����ɢԪ
	*/
	vector<DiscreteCuda> discrete_element_array_cpu;
	DiscreteCuda *discrete_element_array_gpu;

	vector<BeamElement2nd> beam2ndArrayCpu_;
	BeamElement2nd* beam2ndArrayGpu_;

	vector<ShellElement2nd> shell2ndArrayCpu_;
	ShellElement2nd* shell2ndArrayGpu_;

	vector<SolidElement2nd> solid2ndArrayCpu_;
	SolidElement2nd* solid2ndArrayGpu_;

	int nodePerElement;
	int node_amount;
	int elementAmount;

	/**
	��Ԫ������Сֵ���ֵ
	*/
	double *eign_cpu;
	double *eign_gpu;

	/**
	��Ԫ�������е�������
	*/
	double tot_mass;

	/*
	 * ��Ԫָ���뵥Ԫ��ŵ�ӳ���ϵ
	 */
	map<int, Element*> elementCpuMap;
	map<int, Element*> elementGpuMap;

	ElmManager();

	/*
	 * ͳ�Ƶ�Ԫ��
	 */
	void judgeElementAmount();

	/*
	 * ���ó�ʼ������������
	 */
	void setInitialMassScaleFactor(const double msf = 1.0);

	/*
	 * Ϊʵ�嵥Ԫ���ù⻬����Ԫ���
	 */
	void setSfemForSolid(Section *sec);

	/**
	��Ԫ�����������
	*/
	void calElementMass(Material *material, Section *section);

	/**
	cpu��Ԫ�ڵ�����ƥ��
	*/
	void elementNodeMatch(NodeManager *node_manager);

	/**
	gpu�˵�Ԫ���ݴ���
	*/
	void gpuElementCreate(const Section *section);

	/**
	gpu�ڵ�����ƥ��
	*/
	void gpuNodeMatch(NodeManager *node_manager, const Section *section);

	/**
	gpu���������
	*/
	void gpuDataClear();

	/**
	gpu��ⵥԪ�ķ����Ӳ����䵽�ڵ���
	*/
	void gpuCalPenaFactor(double penalty_scale_factor, Material *material, Section *section);

	/**
	�ڵ��ȷ���
	*/
	void shellNodeThick(Section *section);

	/**
	gpu�������С����ֵ
	*/
	void computeCharactLengthGpu(double &min_eigen);
	void computeCharactLengthCpu(double &min_eigen);

	/**
	������е�Ԫ����
	*/
	void allElementInterForceCalGpu(MaterialNew **material, Section *section, HourglassControl*hourglass, double dt);
	void allElementInterForceCalCpu(MaterialNew **material, Section *section, HourglassControl*hourglass, double dt, double &new_dt, double safe_factor);

	/**
	TL����ʱ�����֮ǰ��ⵥԪ��B���󣬲���ʱ������б��ֲ���
	*/
	void computeBMatrixTL(Section *section);

	/**
	����Ԥ�������������е�Ԫ��ڵ�֮���������Ϣ
	*/
	void bulidElementIndexNodeGpu();

	/**
	ȷ�����е�Ԫ�Ĳ������Ա��
	*/
	void verElementMatSecId(const int mater_id, const int section_id);

	/**
	�����㿽���ڴ�
	*/
	void allocZeroMemEle();

	/*
	 * ��Ԫ����
	 */
	void elementSortBaseType();

	/*
	 * ���ô�ĵ�Ԫ��������
	 */
	void setAllElementSectionType();

	void setElementNodeNumFormulateMode(Section *sec);

	/*
	 * ȷ����Ԫ��������Ķ����ڴ��С
	 */
	void setAllElementAddMemSize(Section *sec);

	/*
	 * cpu�˷��䵥Ԫ��������Ķ����ڴ�
	 */
	void allocateElementAddMem();

	/*
	 * cpu���ͷŵ�Ԫ��������Ķ����ڴ�
	 */
	void freeElementAddMem();

	/*
	 * �ڵ�Ԫ�Ķ����ڴ��Ͻ��в���
	 */
	void operatorOnElementAddMemCpu(Section *sec);

	/*
	 * cpu�������������
	 */
	void massScaleForElementCpu(const double min_dt);

	/*
	 * ������Ԫ��ID֮���ӳ���ϵ
	 */
	void bulidElementMapCpu();
	void bulidElementMapGpu();

	/*
	ls:2020-07-21 added
	������Ԫ���������еĵ�Ԫ����
	*/
	void buildElementSectionEtype(Section *section);

	/*
	ls:2020-07-22 added
	���յ�Ԫ���͸�������ʽUL/TL
	*/
	void buildFormulateModeBaseEType();

	/*
	ls:2020-07-22 added
	���������ɶȰ���Ԫ���͸���
	*/
	void computeInterDofNum();

	/*
	ls:2020-07-22 added
	addMemSizeForElement
	*/
	void computeAddMemorySize();

	/*
	ls:2020-07-22 added
	�жϵ�Ԫ�ڵ���Ŀ
	*/
	void judgeElementNodeNumBaseType();

	/*
	ls:2020-07-22 added
	���õ�Ԫ���ֵ���Ŀ
	*/
	void computeGaussNum(Section *section);

	/*
	ls:2020-07-22 added
	���õ�Ԫ������������
	*/
	void judgeElementIntegrateDomainType();

	Element* returnElementPtrCpu(int file_id);
	Element* returnElementPtrGpu(int file_id);

	virtual ~ElmManager();
} ElmManager;