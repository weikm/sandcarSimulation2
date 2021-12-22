#pragma once
#include"basicoutput.h"
#include<fstream>

using std::ofstream;

/*
 * tecplot������ص�Ϊֻ�������ڵ���ƥ�����Ϣ
 * �޷������Ԫ���������
 */
typedef struct TecplotOutput:BasicOutput
{
	int fileCycNum_;

	bool isNeedDisp_;

	bool isNeedVel_;

	bool isNeedAccel_;

	bool isNeedStress_;

	bool isNeedStrain_;

	string outputPath;

	double intDt_;

	ofstream foutStream;

	TecplotOutput();

	virtual void GetDataFromDomain(FEMDynamic* domain, const double time, const double dt) override;

	virtual void bulidDataPathToDomain(FEMDynamic* domain) override;

	virtual void outputDataForNeed(FEMDynamic* domain, const double time, const double dt) override;

	/*
	ls:2020-07-22 added / from gwp
	*/
	virtual void outputDataForNeed_bin(FEMDynamic* domain, const double time, const double dt);

	virtual void outputDataForNeed_bin_read(FEMDynamic* domain, const double time, const double dt);

}TecplotOutput;