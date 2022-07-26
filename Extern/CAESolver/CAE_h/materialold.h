#pragma once
#include"cuda_runtime.h"
#include"curve.h"
#include"curvemanager.h"
#include <string>
using std::string;

enum MaterialNewType
{
	mNull, 
	Rigid,
	LinearElastic, 
	HyperElastic,
	ElasticPlastic,
	ElasticViscoPlastic,
	Mat66Dyna,
	Mat119Dyna,
	MooneyRivlin,
	LowDensityFoamAbaqus,
	//ls:2020-03-17
	Spotweld,   
	Damper0,
	//
	//ls:2020-04-06
	Mat_non_plas_beam
};

typedef struct Material
{
	int id;
	
	string title;

	MaterialNewType type;
	
	int curveId;

	int dyna_type_id;

	/**
	����Ԫ������������ɶȻ��߸ն�����
	*/
	double r_stiffness;
	double s_stiffness;
	double t_stiffness;
	double r_rotate_stiffness;
	double s_rotate_stiffness;
	double t_rotate_stiffness;
	
	//ls:2020-07-08 added
	double r_trans_visdamper;
	double s_trans_visdamper;
	double t_trans_visdamper;
	double r_rotate_visdamper;
	double s_rotate_visdamper;
	double t_rotate_visdamper;

	double r_trans_preLoad;
	double s_trans_preLoad;
	double t_trans_preLoad;
	double r_rotate_preLoad;
	double s_rotate_preLoad;
	double t_rotate_preLoad;
	//

	int curve_mat119_id[24];
	Curve curve_mat119[24];

	double trans_stiffness; //KT
	double rotate_stiffness; //KR
	int Unloading;   //IUNLD
	double offset;
	double dampingfactor;
	int IFLAG;

	//for failure calculation
	double r_trans_failure_tension, s_trans_failure_tension, t_trans_failure_tension;
	double r_rotate_failure_tension, s_rotate_failure_tension, t_rotate_failure_tension;
	double failure_criterion;
	//double UTFAILR, UTFAILS, UTFAILT, WTFAILR, WTFAILS, WTFAILT, FCRIT;
	double r_trans_failure_compression, s_trans_failure_compression, t_trans_failure_compression;
	double r_rotate_failure_compression, s_rotate_failure_compression, t_rotate_failure_compression;
	//double UCFAILR, UCFAILS, UCFAILT, WCFAILR, WCFAILS, WCFAILT;

	//initial translational displacement
	double r_initial_trans, s_initial_trans, t_initial_trans;
	double r_initial_rotate, s_initial_rotate, t_initial_rotate;

	//for IFLAG =2
	int LM1R1S, LM1R2S, LM1R1T, LM1R2T, LM2R1S, LM2R1T;
	int LUM1R1S, LUM1R2S, LUM1R1T, LUM1R2T, LUM2R1S, LUM2R1T;
	int KUM1R1S, KUM1R2S, KUM1R1T, KUM1R2T, KUM2R1S, KUM2R1T, KUM2R2S, KUM2R2T;

	/**
	��ЧӦ��-Ӧ�����ߵĵ�
	*/
	int point_num;

	double y_stress_gpu[curve_max_node_num];
	double x_strain_gpu[curve_max_node_num];

	/**
	ͳһ�����Կ��Ʋ���(������)
	*/
	double young_modulus;
	double density;
	double poisson_ratio;
	double initial_yield_stress;
	double hardening_modulus;
	double shear_modulus;
	double bulk_modulus;
	double tangent_modulus;
	double hardening_parameter;
	double failure_strain;

	//ls:2020-04-06
	//ELASTIC
	double DA;//Axial damping factor
	double DB;//Bending damping factor
	double K; //Bulk modulus(define for fluid option only)

	//Null
	double PC;
	double MU;
	double TEROD; //Relative volume. V/V_0, for erosion in tension
	double CEROD; //Relative volume. V/V_0, for erosion in compression
	//

	//PLASTIC_KINEMATIC
	double SRC;//Strain rate parameter
	double SRP;//Strain rate parameter,
	double FS;//Failure strain
	double VP;//Formulation for rate effects

	//PIECEWISE_LINEAR_PLASTICITY
	int LCSS;

	//MODIFIED_HONEYCOMB
	double VF;
	double BULK;

	//MODIFIED_PIECEWISE_LINEAR_PLASTICITY
	double EPSTHIN;
	double  EPSMAJ;

	//SPOTWELD
	double NRT;
	double MRR;
	double MSS;
	double MTT;
	double NF;
	double EH;
	double DT;
	double TFAIL;
	//

	//ls:2020-03-17
	double failure_stress;
	//

	//ls:2020-03-17
	/**
	Rigid ���� (���޸�)
	*/
	double N;
	double COUPLE;
	double M;
	double ALIAS;
	double CMO;
	double CON1;
	double CON2;
	double A[3];
	double V[3];
	double LCOorA1;
	//

	//ls:2020-03-17
	/**
	MAT_PIECEWISE_LINEAR_PLASTICITY �ֶ����Բ��� (���޸�)
	*/
	double TDEL;
	double c_temp;
	double p_temp;
	int LCSR;//Load curve ID defining strain rate scaling effect on yield stress.
	double EPS[8];//Effective plastic strain values 
	double ES[8];//Corresponding yield stress values to EPS1 - EPS8
	//

	//ls:2020-03-17
	/**
	MODIFIED_HONEYCOMB ���� (���޸�)
	*/
	double Mu; //����ճ��ϵ��
	int LCA;
	int LCB;
	int LCC;
	int LCS;
	int LCAB;
	int LCBC;
	int LCCA;
	//Elastic modulus 
	double EAAU;
	double EBBU;
	double ECCU;
	//����ģ��
	double GABU;
	double GBCU;
	double GCAU;
	double AOPT;//Material axes option
	int MACF;  //Material axes change flag
	double XP, YP, ZP;//Coordinates of point P for AOPT = 1 and 4 
	double A1, A2, A3;//Components of vector A for AOPT = 2 
	double D1, D2, D3;//Components of vector D for AOPT = 2 
	double TSEF;
	double SSEF;
	double VREF;
	double TREF;
	double SHDFLG;
	//

	//ls:2020-03-17
	/**
	MAT_MODIFIED_PIECEWISE_LINEAR_PLASTICITY ���� (���޸�)
	*/
	double NUMINT;
	//

	//ls:2020-03-17
	/**
	MAT_SPOTWELD ���� (���޸�)
	*/
	double EFAIL;
	double NRR;
	double NRS;
	//

	//ls:2020-03-17
	/**
	MAT_SPRING_NONLINEAR_ELASTIC ���� (���޸�)
	*/
	int LCR;
	//

	//ls:2020-04-06
	//NONLINEAR_PLASTIC_DISCRETE_BEAM
	//������ע���޸�
	double lcpdr, lcpds, lcpdt, lcpmr, lcpms, lcpmt;
	double ffailr, ffails, ffailt, mfailr, mfails, mfailt;
	double ufailr, ufails, ufailt, tfailr, tfails, tfailt;
	//

	//ls:2020-03-17
	/**
	DAMPER_VISCOUS ���� (���޸�)
	*/
	//���᳣��
	double dampingConst;


	double axialdampingFactor;
	double bendingdampingFactor;

	/**
	����Ӳ������
	*/
	double amt1;
	double amt2;
	double amt4;

	/**
	ָ��ģ�͵����Բ���
	*/
	double strength_coeffi;
	double harden_exponent;
	double elastic_strain_to_yield;

	/**
	�����������ϵ��
	*/
	double Rval;
	double R0;
	double R45;
	double R90;

	/**
	�Ϳ������������
	*/
	double pem;
	double bc[6];

	/**
	mechanical_variables
	*/
	double matA;
	double matB;
	double matC;
	double matP;
	double matM;


	double pcad;//������Ӧ����ϵ��
	double pcbd;//������Ӧ����ϵ��
	int ipl_model;

	double Dmat66[6][6];
	double Dmat33[3][3];
	double Dmats[2][2];

	/**
	��ɢԪ���Ըն�
	*/
	double elastic_stiff_discre;

	/**
	��ɢԪ���᳣��
	*/
	double damper_constant_discre;

	/**
	��ɢԪ���߸ն�������Ӧ��
	*/
	double tangent_stiff_discrete;
	double yield_discre;

	/**
	��ɢԪӲ��ģ��
	*/
	double plastic_modul_discrete;

	/**
	��ɢԪ����Ӧ����
	*/
	int load_curve_id_discre;
	Curve load_curve_discre;

	/**
	�����Բ��ϲ���
	*/
	double constant1_hyelas;
	double constant2_hyelas;
	double pena_coeff_lambda;

	/**
	��ǰ���ϵĲ���
	*/
	double solid_wave_number;
	double shell_wave_number;
	double beam_wave_number;
	double discrete_wave_number;

	/**
	������������ز���
	*/
	double a0;
	double a1;
	double a2;

	/**
	ճ������ز���
	*/
	double shortTimeShearModulus;
	double longTimeShearModulus;
	double DecayConstant;

	/**
	abaqus���ܶ����޲���
	*/
	double mu1;
	double mu2;
	double alphaLowDensityFoam;
	int tensionLowDensityFoamCurveId;
	int compresLowDensityFoamCurveId;

	Curve tensionLowDensityFoamCurve;
	Curve compresLowDensityFoamCurve;

	void parameter_complet();

	void curveValueCopy(Curve *curve);

	void mat119TieCurve(CurveManager *curveManager);

	/**
	�����Բ���
	*/
	__host__ __device__ void radialReturnAlgorith(double sts[3][3], double &yieldStress, double &effectiveStrain,
		double stsTrial[3][3], double devsts_trial[3][3])const;

	__host__ __device__ void radialEvl_k_prime(const double pstn, double &ys_k_prime)const;

	__host__ __device__ void radialEvl_k(const double pstn, double &ys_k)const;

	/**
	����ѹ���ĳ����Բ���
	*/
	__host__ __device__ void incompMooney(double store_variable[3][3], double sts[3][3],
		double strain_rate_diag[6], double strain_rate_nondiag[6], const double dt)const;

	/**
	��ѹ������޲���
	*/
	__host__ __device__ void crushableFoam(double sts[3][3], const double strain_rate[6], double ini_vol,
		const double w[3], double stress[6], double dt, double volume, double &press, double &vol_strain)const;

	/**
	Abaqus���ܶ�����
	*/
	__host__ __device__ void abaqusLowDensityFoam(double sts[3][3], const double deform_gradient[3][3],
		const double strain_rate[3][3], const double dt)const;

	/**
	����ճ���Բ���
	*/
	__host__ __device__ void visoelastic(double sts[3][3])const;

	/**
	������ϵĲ���
	*/
	void calWaveNumber();
} Material;
