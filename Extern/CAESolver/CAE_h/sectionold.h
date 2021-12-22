#pragma once
#include"elementtypeenum.h"
#include"elmmanager.h"
#include"intergrationdomain.h"
#include"gausspoint.h"
#include"formulatemode.h"
#include <string>
using std::string;

struct ElmManager;
struct GaussPoint;

enum IntegrationRule
{
	Gauss,
	Lobatto,
};

typedef struct Section
{
	GeometricNewType gType_;
	ElementNewType eType_;
	SmoothFEMType sType_;
	IntegrationDomain intType_;
	FormulateMode fMode;

	//ls:2020-03-18
	int typeId;
	int llkt;
	double NIP;
	double thickIntegrPoints;
	double rotool;
	double tktool;
	double qhg;
	double QR;
	string title;
	//

	//ls:2020-04-06
	//shell
	int ICOMP;
	int SETYP;
	double NLOC;
	double MAREA;
	double IDOF;
	int EDGSET;
	double PROPT;
	//

	//ls:2020-04-06
	//beam
	double CST; //Cross section type EQ.0.0: rectangular,EQ.1.0: tubular(circular only),EQ.2.0 : arbitrary(user defined integration rule).
	double SCOOR;
	double NSM;

	//type=2,3
	double A; //Cross-sectional area
	//type=2,12
	double SA; //Shear area.

	double IST;

	//type=6 
	//double volumeSpring_;
	double massInertiaSpring;
	double CID;
	double CA;
	double OFFSET;
	double RRCON;
	double SRCON;
	double TRCON;
	//

	//ls:2020-04-06
	//SECTION_DISCRETE
	int DRO;
	double KD;
	double V0;
	double CL;
	double FD;
	double CDL;
	double TDL;
	//

	//ls:2020-04-06
	//solid
	int aet;                                 //P2805 ��Ԫ������
	//

	int id;
	int elmFormulateOptionDYNA_;

	int addMemSizeForElement;

	int nodeNumForElement;

	int integratePointZ_;
	int integratePointXY_;
	int integratePointNum_;

	IntegrationRule integrationRule_;
	
	double thick[4];
	double shearCorrectFactor_;
	
	int thick_update;

	/*
	 * �ڲ����ɶȣ�����wilson����
	 */
	int numInterDof_;

	/**
	����Ԫ�����ϵĻ��ֵ�
	*/
	double beam_inter_point_y[16];
	double beam_inter_point_z[16];

	/**
	����Ԫ
	*/
	double volumeSpring_;
	double inertiaSpring;

	/**
	Cross section type EQ.0.0: rectangular,EQ.1.0: tubular(circular only),
	EQ.2.0 : arbitrary(user defined integration rule).
	*/
	double crossSectionType_;

	/**
	Ĥ��Ԫ�������ѹ����ֹ
	*/
	double compre_cutoff;

	/**
	outer diameter node 1
	*/
	double TS1;

	/**
	outer diameter node 2
	*/
	double TS2;

	/**
	inner diameter node 1
	*/
	double TT1;

	/**inner diameter node 2
	*/
	double TT2;

	/**
	location of reference surface
	*/
	double NSLOC;

	/**
	location of reference surface
	*/
	double NTLOC;

	/**
	cross sectional area
	*/
	double crossSectionArea_;
	double shearArea_;

	/**
	ת������
	*/
	double yInertiaMoment_;
	double zInertiaMoment_;
	double polarInertiaMoment_;

	/**
	������Ӧ����ϵ��
	*/
	double pcad;

	/**
	������Ӧ����ϵ��
	*/
	double pcbd;

	/**
	moment of inertia
	*/
	double ISS;

	/**
	moment of inertia about local t-axis
	*/
	double ITT;

	/**
	torsional constant
	*/
	double IRR;

	/**
	bt�ǵĺ�Ȼ��ֵ㼰Ȩ��
	*/
	//double zeta[6][6];
	//double gw[6][6];

	/**
	�����οǵ�Ԫ��Ȼ��ּ�Ȩ��
	*/
	//double tk_weight[5];
	//double tk_point[5];

	/**
	��ɢԪ��תƽ�Ʊ�־
	*/
	int disc_disp_rot_flag;

	/**
	Ĥ��Ԫɳ©���Ʋ���
	*/
	double hourg_para_mem;

	/**
	���ÿǵ�Ԫ��Ȼ��ֵ�
	*/
	/*void setShellIntegrPoints();*/

	/*
	 * ��Ҫʱ�Ե�Ԫ�Ķ����ڴ����һ������
	 */
	__host__ __device__ void operatorOnElementAddMem(double *addMemptr);

	/*
	 * ���㵥Ԫ��������Ķ����ڴ��С
	 */
	void computeAddMemorySize();
	/**
	�������Ԫ����Ҫ�ı���
	*/
	void beamFactorCal();

	/**
	������������ϵĻ��ֵ�
	*/
	void setBeamInterPoint();

	/**
	���ɳ©����
	*/
	void testHorgParam();

	/*
	 * ���㵱ǰ��Ԫ���͵Ļ��ֵ����
	 */
	void computeGaussNum();

	/*
	 * ���㵱ǰ��Ԫ���ڱ�������
	 */
	void computeInterDofNum();

	/*
	 * ������ֵ���Ȼ������Ȩ��
	 */
	void computeGaussPointNaturalCoord();
	void computeGaussPointWeight();

	/*
	 * ����dyna��Ԫ���ͱ��ȷ����Ԫ����
	 */
	void judgeElementTypeForDyna(ElmManager* elm1stManager);

	/*
	 * ���ݵ�Ԫ�����жϵ�Ԫ�ڵ����
	 */
	void judgeElementNodeNumBaseType();

	/*
	 * ���ݵ�Ԫ�����жϻ��ָ�ʽ
	 */
	void judgeFormulateModeBaseEType();

	/*
	 * �жϵ�Ԫ�Ļ�������״
	 */
	void judgeElementIntegrateDomainType();

	/*
	 * ���ݻ��������״���û��ֵ�
	 */
	void computeGaussCoordWeight(GaussPoint *gpPtr);

	/*
	 * �������Ի�������ֵ�
	 */
	void computeGaussOnLine(GaussPoint *gpPtr);

	/*
	 * ���������λ�������ֵ�
	 */
	void computeGaussOnTriangle(GaussPoint *gpPtr);

	/*
	 * �����ı��λ�������ֵ�
	 */
	void computeGaussOnSquare(GaussPoint *gpPtr);

	/*
	 * �����������������ֵ�
	 */
	void computeGaussOnCube(GaussPoint *gpPtr);

	/*
	 * �����������������ֵ�
	 */
	void computeGaussOnTetra(GaussPoint *gpPtr);

	/*
	 * ������������������ֵ�
	 */
	void computeGaussOnWedge(GaussPoint *gpPtr);

private:
	/*
	 * ���Ի��ֵ���Ȼ�����Լ�Ȩ������
	 */
	void setLineGaussCoordWeight(int nPoint,double coord[],double weight[]);

	/*
	 * �����λ��ֵ���Ȼ�����Լ�Ȩ������
	 */
	void setTriGaussCoordWeight(int nPoint,double coordXi1[],double coordXi2[],double weight[]);

	/*
	 * ��������ֵ���Ȼ�����Լ�Ȩ������
	 */
	void setTetrCoordWeight(int nPoint,double coordXi1[],double coordXi2[],double coordX3[],double weight[]);

} Section;