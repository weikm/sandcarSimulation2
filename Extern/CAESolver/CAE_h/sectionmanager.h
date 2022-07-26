#pragma once
#include"sectionold.h"
#include<vector>
#include<map>
using std::vector;
using std::map;

typedef struct SectionManager
{
	vector<Section> section_array;
	map<int, Section*> section_map;
	Section* section_array_gpu;

	/**
	����section��id������
	*/
	void sectionLinkId();

	/**
	����id����section
	*/
	Section* returnSection(const int id);

	/*
	 * ȷ����������������ĵ�Ԫ�Ļ��ֵ����
	 */
	void comuteIntegraPointNumDomainType();

	/*
	 * ȷ�����н������������еĶ����ڴ��С
	 */
	void computeAllSectionAddMemSize();

	/**
	����gpu������
	*/
	void createOldSectionArrayGpu();

	virtual ~SectionManager();
}SectionManager;