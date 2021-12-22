#pragma once
#include"materialold.h"
#include<vector>
#include<map>
#include"materialnew.h"
using std::vector;
using std::map;

typedef struct MaterialManager
{
	vector<Material> material_array;
	map<int, Material*> materialMap;
	Material* material_array_gpu;

	vector<MaterialNew*> materialNewArrayCpu_;
	map<int, MaterialNew*>materialNewMap_;
	MaterialNew** materialNewArrayGpu_;

	int material_num;

	MaterialManager();

	/**
	����������id��ӳ��
	*/
	void materialLinkId();

	/**
	����id���ز���
	*/
	Material* returnMaterial(const int id);
	MaterialNew* returnMaterialNew(const int id);

	/**
	����gpu������
	*/
	void createOldMaterialArrayGpu();

	/*
	 * ������ʽ������id��ӳ��
	 */
	void bulidIdMapMaterialNew();

	/*
	 * ��ȡ����ָ��
	 */
	MaterialNew* giveMaterialNewPtr(const int id);

	/*
	 * ������ʽ���ϵ�gpu������
	 */
	void createNewMaterialArrayGpu();

	/*
	 * ���ݾ�ʽ���ϴ�����ʽ����
	 */
	void createMaterialCpuData();

	/*
	 * ��֤���в������ʵ���������ɹ�
	 */
	void verNewMaterialGpuCreate();

	/*
	 * ����cpu����״̬
	 */
	void createAllMatStatusCpu();

	/*
	 * ����gpu����״̬
	 */
	void createAllMatStatusGpu(const int gpu_id = 0);

	/*
	 * ������״̬���ϴ�cpu���Ƶ�Gpu
	 */
	void copyMatStatusFromCpuToGpu(MaterialNew **matNewArrayGpu,const int gpu_id=0);

	virtual ~MaterialManager();
}MaterialManager;
