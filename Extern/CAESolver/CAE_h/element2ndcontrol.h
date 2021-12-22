//#pragma once
//#include"gausspoint.h"
//#include"part.h"
//#include"materialold.h"
//#include"sectionold.h"
//#include"hourglasscontrol.h"
//#include"sectionmanager.h"
//#include"materialstatus.h"
//#include "cublas_v2.h"
//#include"cublas_api.h"
//#include"beam2nd.h"
//#include"shell2nd.h"
//#include"solid2nd.h"
//
//typedef 
//{
//	BeamElement2nd** beam2ndArrayGpu_;
//	ShellElement2nd** shell2ndArrayGpu_;
//	SolidElement2nd** solid2ndArrayGpu_;
//
//	int beam2ndNum_;
//	int solid2ndNum_;
//	int shell2ndNum_;
//
//	double *dtArrayGpu_;
//
//	GaussPoint* gaussPointArrayGpu_;
//
//	int gaussPointNum_;
//
//	Section *section_array_gpu;
//	MaterialNew **material_array_gpu;
//
//	/*
//	 *������Ԫ�ܼ���
//	 */
//	void createElementArraySingleGpu(vector<Part> &partArray_);
//
//	/*
//	 *��GPU������Ԫ����
//	 */
//	void createElementArrayMultiGpu(vector<Part> &partArray_, const int in,
//		const vector<int> &elm_num_gpu, const vector<cudaStream_t> &streamArray, const vector<int> &gpu_id_array);
//
//	/*
//	 *�������е�Ԫ������
//	 */
//	void allElement2ndArrayCalInterForce(MaterialNew** sys_material_array_gpu, Section *sys_section_array_gpu, HourglassControl *hourg_control,
//		double dt, cublasHandle_t cu_handle, double &dt_new, double safe_factor);
//
//	/*
//	 *���ӵ�Ԫ�е����нڵ�
//	 */
//	void allElementLinkNode(NodeCuda *sys_node_array_gpu, cudaStream_t stream_gpu);
//
//	/*
//	 *���׵�Ԫ��������
//	 */
//	void massScaleForElement2nd(const double min_dt);
//
//	/*
//	 *���е�Ԫ�������ֵ�
//	 */
//	void computeGaussPointArrayGpu(SectionManager *sectionManager,MaterialManager *matManager,vector<Part> &partArray_);
//
//	/*
//	 *�������е�Ԫ��ʱ������
//	 */
//	void allocateDtArrayGpu();
//
//	/*
//	 *ͳ�����е�Ԫ�����������ӵ�����
//	 */
//	void countAddedMassForMassScaled(const int isMassScaled);
//}Element2ndControl;