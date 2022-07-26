#pragma once
#include"tie.h"
#include "surfacemanager.h"
#include"nodemanager.h"
#include<set>

using std::set;

struct NodeManager;
struct SurfaceManager;
struct Tie;

typedef struct TieManager
{
	int tie_num;

	vector<Tie> tie_array;
	
	set<int> slave_nodeId_array;
	
	set<int> master_nodeId_array;

	TieManager();

	/**
	����������Ĵ���
	*/
	void allTieAddSlaveNodeForceToMasterGpu();
	void allTieAddSlaveNodeForceToMasterCpu();

	/**
	��������ٶȵĴ���
	*/
	void allTieModifySlaveVelByMasterGpu();
	void allTieModifySlaveVelByMasterCpu();

	/**
	ת�����а󶨶Ե�����
	*/
	void allTieTransferMassGpu();
	void allTieTransferMassCpu();

	/**
	����һ�԰󶨶�
	*/
	void createNewTie(std::string masterSurface, std::string slaveSurface, const double position_tolerance,
		const int masterSurfaceId, const int slaveSurfaceId);

	/**
	���󶨵������洮������
	*/
	void linkTieSurface(SurfaceManager *surfaceManager);

	/**
	�ж��Ƿ����һ�����������
	*/
	void judgeSurfaceInNumTie();

	/**
	�������а󶨴ӽڵ����Ӧ������
	*/
	void produceAllSubTie();

	/**
	�������а���Ĳ�ֵ����
	*/
	void calAllTieInterpolateFun();

	/**
	����Gpu�ϵ����а󶨶�
	*/
	void produceAllTieArrayGpu(NodeManager *nodeManger);

	/**
	������а󶨣�ȷ��û���ظ��󶨵����
	*/
	void verfAllTie();
}TieManager;