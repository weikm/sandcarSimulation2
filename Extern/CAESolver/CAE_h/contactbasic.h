#pragma once
#include"surface.h"
#include"contactnode.h"
#include"surfacemanager.h"
#include"setmanager.h"
#include "part.h"
#include"contactnodemanager.h"
#include"SurfaceInteract.h"
#include <cublas.h>

#define controlDistance 0.0
#define cds2 controlDistance*controlDistance

struct ContactNodeManager;
struct Part;
struct Surface;
struct SetManager;
struct SurfaceManager;
struct SurfaceInteract;

/**
tol:Ԥ��ϵ��, ������0, ��ʾ������Ԥ��, ÿ���������γ��µĽӴ���, ����ʱ���, �������
������1ʱ,�������ֹ۵Ĺ���,����ʱ�����,�������
����ȡֵΪ0.5��0.7֮��
*/
#define SearchPredictCoef 0.2

struct ContactNodeCuda;

typedef struct Contact
{
	enum Type
	{
		nodeToSurface, surfaceToSurface, singleSurface, general, shellEdgeToSurface
	};

	Type contactType;

	int id;

	int slaveId;
	int masterId;
	int slaveType;
	int masterType;
	double fnu;

	//ls:2020-03-17
	/**
	maximum prescribed velocity
	*/
	double vmax;

	/**
	control distance
	*/
	double controlDist;

	/**
	Static coefficient of friction
	*/
	double FS;

	/**
	Dynamic coefficient of friction
	*/
	double FD;

	/**
	Birth time (contact surface becomes active at this time).
	*/
	double BT;

	/**

	*/
	double DT;//Death time (contact surface is deactivated at this time).

			  /**
			  Scale factor on default slave penalty stiffness
			  */
	double SFS;

	/**
	Scale factor on default master penalty stiffness
	*/
	double SFM;

	/**
	*/
	double SST;//Optional thickness for slave surface (overrides true thickness

			   /**
			   Optional thickness for master surface (overrides true thickness)
			   */
	double MST;

	/**
	*/
	double SFST;//Scale factor for slave surface thickness (scales true thickness).

				/**
				Scale factor for master surface thickness (scales true thickness).
				*/
	double SFMT;

	/**
	Coulomb friction scale factor.
	*/
	double FSF;

	/**
	Viscous friction scale factor
	*/
	double VSF;

	//KDH
	int SOFT;
	double SOFSCL;
	int LCIDAB;
	double MAXPAR;
	double SBOPT;
	int DEPTH;
	int BSORT;
	int FRCFRQ;

	double PENMAX;
	int THKOPT;
	int SHLTHK;
	int SNLOG;
	int ISYM;
	int I2D3D;
	double SLDTHK;
	double SLDSTF;

	//ls:2020-04-06
	int IGAP;
	int IGNORE0;
	double DPRFAC;
	double DTSTIF;
	double UNUSED;
	double FLANGL;
	int CID_RCF;
	string title;
	int SBOXID, MBOXID, SPR, MPR;
	//
	//MPP
	int BCKT;
	int LCBCKT;
	int NS2TRK;
	int INITITR;
	double PARMAX;
	int CPARM8;
	//

	/**
	������γɵ���ײ��
	*/
	int contact_node_num;
	vector<ContactNodeCuda*> slave_contact_node_array_cpu;
	ContactNodeCuda** slave_contact_node_array_gpu;

	/*
	* ��gpu�Ӵ���
	*/
	vector<ContactNodeCuda**> slaveContactNodeArrayMultiGpu;
	vector<int> slaveContactNodeNumMultiGpu;

	/**
	surfaceToSurface
	*/
	Surface* slaveSurface;
	Surface* masterSurface;

	bool isNeedJudge;

	/**
	nodeToSurface
	*/
	vector<NodeCuda *> slave_node_array;

	/**
	���е���ײ��
	*/
	vector<ContactNodeCuda *> AllContactNodeArray;

	/**
	maximum prescribed velocity
	*/
	double velMaxModulus;

	/**
	�ӽӴ�������λ������
	*/
	double dispIncre;

	/**
	Static coefficient of friction
	*/
	double static_coeffic_friction;

	/**
	Dynamic coefficient of friction
	*/
	double dynamic_coeffic_friction;

	/**
	Exponential decay coefficien
	*/
	double DC;

	/**
	Coefficient for viscous friction.
	*/
	double VC;

	/**
	Viscous damping coefficient in percent of critical
	*/
	double VDC;

	/**
	Small penetration in contact search option
	*/
	double PENCHK;

	/**
	�ԽӴ�����ߴӽӴ�������ٶ�ֵ
	*/
	double vmax_slave;
	double* velModulusGpu;
	double* velModulusCpu;

	/**
	�����Ӵ���λ����С��Ԥ�����緢���Ӵ��ĵ�������
	*/
	int minPredicIterNum;
	double* predicIterNumGpu;
	double* predicIterNumCpu;

	Contact();

	/**
	��������
	*/
	int isch;

	/*
	 * ��������Ӵ�������֮����໥����
	 */
	int surfInterNum_;
	vector<SurfaceInteract*> surfaceInteractCpu_;
	vector<SurfaceInteract**> surfaceInteractGpu_;

	/**
	�������ӽӴ���
	*/
	void surfaceProudce(SurfaceManager *surfaceManager, vector<Part> &partArray_, SetManager *setManager, NodeManager *nodeManager);

	/**
	������ײ��
	*/
	void contactNodeCollect(ContactNodeManager *contactNodeManager);

	/**
	����gpu�ϵĽӴ���
	*/
	void produceGpuContactNodeCuda(ContactNodeManager* contact_node_manager);
	void produceMultiGpuContactNode(ContactNodeManager* cnNdMag, vector<int> gpuIdArray);

	/**
	����ӽӴ�Ԫ�ص��������ֵ
	*/
	void calVamxSlaveGpu(cublasHandle_t& cu_handle);
	void CalVmaxSlaveCpu();

	/**
	���㱾�Ӵ��������м��������Ӵ��ĵ���С��������
	*/
	void calMinCyclesGpu(cublasHandle_t& cu_handle, const double dt);
	void CalMinCyclesCpu(const double dt);

	void imposeSpecificInteractCpu(double dt);

	void imposeSpecificInteractGpu(double dt);

	void imposeSpecificInteractMultiGpu(FEMDynamic *domain, double dt);

	/**
	�����㸴���ڴ�
	*/
	void allocZeroMemContact();

	virtual ~Contact();

} Contact;

