//this file is used to compute shuang xiang ou he!
#include "Dynamics/Sand/SandInteractionForceSolver.h"
#include <thrust/execution_policy.h>
#include <thrust/reduce.h>
#include <cmath>

#include "Core/Utility/CTimer.h"
#include "Core/Utility/CudaRand.h"

namespace PhysIKA {

void SandInteractionForceSolver::addSDF(DistanceField3D<DataType3f>& sdf, int rigidid)
{

    float* tmpa = new float[10];
    memset(tmpa, 0, sizeof(float) * 10);
    DeviceArray<double> devTmpa;
    devTmpa.resize(10);
    cudaMemcpy(devTmpa.begin(), tmpa, sizeof(float) * 10, cudaMemcpyHostToDevice);
    cudaDeviceSynchronize();
    cudaError_t err = cudaGetLastError();
    delete[] tmpa;
    devTmpa.release();

    if (rigidid < 0)
        rigidid = m_sdfMap.size();
    m_sdfMap[rigidid] = sdf;
}

__global__ void SandIFS_updateSinkInfo(//更新下沉信息，这应该是耦合第一步
    DeviceDArray<double>             topH,
    DeviceDArray<double>             botH,
    DeviceDArray<Vector3d>           topNormal,
    DeviceDArray<Vector3d>           botNormal,//最后算这四个
    DeviceDArray<Vector3d>           positions,//沙子高度场
    DeviceHeightField1d              land,
    DistanceField3D<DataType3f>      sdf,
    DeviceArray<PBDBodyInfo<double>> body,
    int                              sdfid,
    double                           dh)
{
    int tid = threadIdx.x + blockDim.x * blockIdx.x;
    if (tid >= positions.size())
        return;

    PBDBodyInfo<double>& bodyi = body[sdfid];

    Vector3d pointd = positions[tid];
    Vector3d normaltop;
    Vector3d normalbot;

    double hland = land.get(pointd[0], pointd[2]);//硬地面高
    double curh  = pointd[1] /*+ hland*/;//沙面高？
    double htop  = hland;//这俩是不是写反了？
    double hbot  = curh;
    while (curh > hland)
    {
        pointd    = positions[tid];
        pointd[1] = curh;
        bodyi.pose.invTransform(pointd);
        Vector3f pointf(pointd[0], pointd[1], pointd[2]);

        Vector3f normalf;
        float    dis;
        sdf.getDistance(pointf, dis, normalf);//有向距离场！给到dis
        normalf *= -1;

        if (dis <= 0)//接触了——
        {

            //Vector3d tmpp = positions[tid];
            //tmpp[1] = curh;
            //tmpp -= bodyi.pose.position;
            //double tmpnorm = tmpp.norm();
            //printf("detected: %lf,  %lf; %lf %lf %lf;  %lf %lf %lf\n", dis, tmpnorm,
            //	bodyi.pose.position[0], bodyi.pose.position[1], bodyi.pose.position[2],
            //	positions[tid][0], curh, positions[tid][2]
            //	);

            if (htop < curh)
            {
                htop      = curh;
                normaltop = Vector3d(normalf[0], normalf[1], normalf[2]);
                bodyi.pose.rotate(normaltop);
            }
            if (hbot > curh)
            {
                hbot      = curh;
                normalbot = Vector3d(normalf[0], normalf[1], normalf[2]);
                bodyi.pose.rotate(normalbot);
            }
        }

        curh -= dh;
    }

    topH[tid]      = htop;
    botH[tid]      = hbot;
    topNormal[tid] = normaltop;
    botNormal[tid] = normalbot;
}

void SandInteractionForceSolver::updateSinkInfo(int i)
{

	m_topH.resize(m_particlePos->size());
	m_botH.resize(m_particlePos->size());
	m_topNormal.resize(m_particlePos->size());
	m_botNormal.resize(m_particlePos->size());

	m_topH.reset();
	m_botH.reset();
	m_topNormal.reset();
	m_botNormal.reset();

	int rid = i;
	if (m_prigids)
		rid = (*m_prigids)[i]->getId();

	cuExecute(m_particlePos->size(), SandIFS_updateSinkInfo, m_topH, m_botH, m_topNormal, m_botNormal, *m_particlePos, *m_land,
		//(*m_sdfs)[i],
		m_sdfMap[rid],
		*m_body,
		i,
		m_sampleSize);

	////  debug
	//HostDArray<double> hostTop;
	//hostTop.resize(m_topH.size());
	//Function1Pt::copy(hostTop, m_topH);

	//HostDArray<double> hostBot;
	//hostBot.resize(m_botH.size());
	//Function1Pt::copy(hostBot, m_botH);

	//HostDArray<Vector3d> hostpos;
	//hostpos.resize(m_particlePos->size());
	//Function1Pt::copy(hostpos, *m_particlePos);

	//double* hostland = new double[m_land->Nx() * m_land->Ny()];
	//cudaMemcpy2D(hostland, m_land->Nx(), m_land->GetDataPtr(), m_land->Pitch(),
	//	m_land->Nx(), m_land->Ny(), cudaMemcpyDeviceToHost);

	//hostTop.release();
	//hostBot.release();
	//hostpos.release();
	//delete[] hostland;
}

__global__ void SandIFS_computeBuoyancy(
    DeviceDArray<Vector3d>            buoF,
    DeviceDArray<Vector3d>            buoT,
    DeviceDArray<double>              relvDf,//最后就要求上面这仨量，两个浮力，一个夹角相关。
    DeviceDArray<double>              topH,
    DeviceDArray<double>              botH,
    DeviceDArray<Vector3d>            botNor,
    DeviceDArray<Vector3d>            posArr,
    DeviceDArray<Vector3d>            parVel,
    DeviceDArray<double>              massArr,
    DeviceHeightField1d               land,
    DeviceDArray<PBDBodyInfo<double>> body,
    int                               bodyid,
    double                            gravity)
{
    int tid = threadIdx.x + blockDim.x * blockIdx.x;
    if (tid >= massArr.size())
        return;

    double   htop   = topH[tid];//两个heightfield
    double   hbot   = botH[tid];
    Vector3d pointd = posArr[tid];
    if (htop > hbot)
    {
        int                  botid = tid * 2;
        PBDBodyInfo<double>& bodyi = body[bodyid];

        double hland = land.get(pointd[0], pointd[2]);
        double force = gravity * massArr[tid] * (htop - hbot) / (pointd[1] - hland);//浮力计算在这！！butwhy 如此 naive？

        buoF[botid][0] = 0;  // force * botNor[tid][0] / (botNor[tid][1] - 1e-2);
        buoF[botid][1] = force;
        buoF[botid][2] = 0;  // force * botNor[tid][2] / (botNor[tid][1] - 1e-2);

        //Vector3d force(0, buoF[botid], 0);
        pointd[1]   = hbot;
        buoT[botid] = (pointd - bodyi.pose.position).cross(buoF[botid])*0;//支持力矩，去掉看看

        //Vector3d tmpv = buoF[botid];// (pointd - body[bodyid].pose.position);
        //if (buoF[botid][0] != 0 || buoF[botid][1] != 0 || buoF[botid][2] != 0)
        //{
        //	printf("%d: F, %lf;   val, %lf %lf %lf\n", tid, buoF[tid*2][1], htop, hbot, hland);
        //}
        relvDf[botid] = buoF[botid].dot(bodyi.linVelocity - parVel[tid])
                        /*+ buoT[botid].dot(bodyi.angVelocity)*/;//dot求夹角cos值//这里不该再有buoT了吧？这是那个深刻bug？
    }
    else
    {
        buoF[tid * 2]   = Vector3d();
        //buoT[tid * 2]   = Vector3d();//支持力矩，去掉看看
        relvDf[tid * 2] = 0.0;
    }
    buoF[tid * 2 + 1]   = Vector3d();
    //buoT[tid * 2 + 1]   = Vector3d();//支持力矩，去掉看看
    relvDf[tid * 2 + 1] = 0.0;
}



//void SandInteractionForceSolver::computeBuoyancy()
//{
//	if (!m_body || m_body->size() <= 0)
//		return;
//	for (int i = 0; i < m_body->size(); ++i)
//	{
//		this->computeSingleBuoyance(i);
//	}
//}

void SandInteractionForceSolver::computeSingleBuoyance(int i, Real dt)
{
    if (!m_particlePos || m_particlePos->size() <= 0)
        return;

    m_buoyancyF.resize(m_particlePos->size() * 2);
    m_buoyancyT.resize(m_particlePos->size() * 2);
    m_relvDf.resize(m_particlePos->size() * 2);
    m_buoyancyF.reset();
    m_buoyancyT.reset();
    m_relvDf.reset();

    cuExecute(m_particlePos->size(), SandIFS_computeBuoyancy, m_buoyancyF, m_buoyancyT, m_relvDf, m_topH, m_botH, m_botNormal, *m_particlePos, *m_particleVel, *m_particleMass, *m_land,
              //*m_body,
              m_averageBodyInfo,
              i,
              m_gravity);

    //m_devArr1d.resize(m_buoyancyF.size());
    //Function1Pt::copy(m_devArr1d, m_buoyancyF);
    //m_devArr3d.resize(m_buoyancyT.size());
    //Function1Pt::copy(m_devArr3d, m_buoyancyT);

    //double buoF = 1;
    Vector3d buoF = thrust::reduce(thrust::device, m_buoyancyF.begin(), m_buoyancyF.begin() + m_buoyancyF.size(), Vector3d(), thrust::plus<Vector3d>());

    Vector3d buoT /*= thrust::reduce(thrust::device, m_buoyancyT.begin(), m_buoyancyT.begin() + m_buoyancyT.size(),
			Vector3d(), thrust::plus<Vector3d>())
        (0.0, 0.0, 0.0)*/;

    double relvdf = thrust::reduce(thrust::device, m_relvDf.begin(), m_relvDf.begin() + m_relvDf.size(), ( double )0.0, thrust::plus<double>());

    //// debug
    //HostDArray<Vector3d> hostF;
    //hostF.resize(m_buoyancyF.size());
    //Function1Pt::copy(hostF, m_buoyancyF);

    //HostDArray<Vector3d> hostT;
    //hostT.resize(m_buoyancyT.size());
    //Function1Pt::copy(hostT, m_buoyancyT);

    //HostDArray<double> hostMass;
    //hostMass.resize(m_particleMass->size());
    //Function1Pt::copy(hostMass, *m_particleMass);

    //

    //double tmpv = buoF[1];
    //hostF.release();
    //hostT.release();
    //hostMass.release();

    if ((m_hostBody)[i].invMass > 0)
    {
        m_Abuo = this->_enlargerBuoyancy(buoF[1], buoT, 1.0 / (m_hostBody)[i].invMass);
        m_Abuo = m_Abuo > 1e9 ? 1e9 : m_Abuo;

        if (abs(buoF[1]) > EPSILON)
        {
            m_Abuo /= abs(buoF[1]);
        }
        m_Abuo *= m_gravity;
    }
    else
    {
        m_Abuo = 0.0;
    }
    buoF *= m_Abuo;
    buoT *= m_Abuo;
    relvdf *= m_Abuo;

    double alpha = this->_minEng(buoF, buoT, relvdf, i, dt);
    //double alpha = 1.0;
    buoF *= alpha;
    buoT *= alpha;

    // debug
    Vector3d& debLinv = (m_hostBody + i)->linVelocity;
    Vector3d& debAngv = (m_hostBody + i)->angVelocity;

    if (true)
    {
        //printf("**** %d \n", i);

        //printf("  Body vel(before BUO):  %lf %lf %lf,  %lf %lf %lf\n",
        //       debLinv[0],
        //       debLinv[1],
        //       debLinv[2],
        //       debAngv[0],
        //       debAngv[1],
        //       debAngv[2]);

        printf("Buoy F: %lf %lf %lf, Buoy T: %lf %lf %lf, Abuo: %lf , Alpha: %lf\n",
               buoF[0],
               buoF[1],
               buoF[2],
               buoT[0],
               buoT[1],
               buoT[2],
               m_Abuo,
               alpha);
    }

    (m_hostBody + i)->linVelocity = m_prevBody[i].linVelocity;
    (m_hostBody + i)->angVelocity = m_prevBody[i].angVelocity;

    this->_applyForceTorque(buoF, buoT, i, dt);//把力用于计算位姿pose 7维向量！！！！！！！！！！！！！！

    if (false)
    {
        printf("  Body vel(after BUO):  %lf %lf %lf,  %lf %lf %lf\n",
               debLinv[0],
               debLinv[1],
               debLinv[2],
               debAngv[0],
               debAngv[1],
               debAngv[2]);

        auto& prevPose  = m_prevBody[i].prevPose;
        auto  pbodyPose = (m_hostBody + i)->pose;
        printf("  Body Pos(prev):  %lf %lf %lf,  %lf %lf %lf %lf\n",
               prevPose.position[0],
               prevPose.position[1],
               prevPose.position[2],
               prevPose.rotation[0],
               prevPose.rotation[1],
               prevPose.rotation[2],
               prevPose.rotation[3]);
        printf("  Body Pos(cur) :  %lf %lf %lf,  %lf %lf %lf %lf\n",
               pbodyPose.position[0],
               pbodyPose.position[1],
               pbodyPose.position[2],
               pbodyPose.rotation[0],
               pbodyPose.rotation[1],
               pbodyPose.rotation[2],
               pbodyPose.rotation[3]);
    }
}

__global__ void SandIFS_DragForceVel(//算拖拽阻力
    DeviceDArray<Vector3d>           dragF,
    DeviceDArray<Vector3d>           dragT,
    DeviceDArray<double>             relvDf,//算上面这仨//这个应该是队速度的削减值
    DeviceDArray<double>             topH,
    DeviceDArray<double>             botH,//单值的两个限定值
    DeviceDArray<Vector3d>           topN,
    DeviceDArray<Vector3d>           botN,
    DeviceDArray<double>             massArr,//arr意为数组
    DeviceDArray<Vector3d>           posArr,
    DeviceDArray<Vector3d>           velArr,
    DeviceHeightField1d              land,
    DeviceArray<PBDBodyInfo<double>> body,
    int                              bodyid,
    double                           rho,
    double                           mu,
    double                           gravity,
    double                           alpha,
    double                           beta,
    double                           Cdrag)
{
    int tid = threadIdx.x + blockDim.x * blockIdx.x;
    if (tid >= massArr.size())
        return;

    if (topH[tid] <= botH[tid])
        return;

    Vector3d posi  = posArr[tid];//每个方格最高处粒子的位置。
    double   hland = land.get(posi[0], posi[2]);//硬地高

    // Effective area.
    double effA = massArr[tid] / ((posArr[tid][1] - hland) * rho);

    if (posi[1] > topH[tid])
    {
        double depth = posi[1] - topH[tid];//topH[tid]是刚体最低处？
        int    topid = tid * 2 + 1;
        // Relative velocity.
        Vector3d relp = posi;
        relp[1]       = topH[tid];
        relp -= body[bodyid].pose.position;
        Vector3d relv = body[bodyid].linVelocity + body[bodyid].angVelocity.cross(relp);//每个方格对应的刚体耦合速度。
        relv -= velArr[tid];//减去沙子自身速度，就是耦合双方的相对速度

        double effAi = topN[tid].dot(relv) * effA / (abs(topN[tid][1]) + 0.05);

        // Force
        double f = beta * rho * effAi;
        f        = f > 0 ? -f : 0;//令f恒小于零

        // Frictional drag.
        double relvNorm = relv.norm();
        if (relvNorm > EPSILON)
            effAi /= (relvNorm * relvNorm);
        else
            effAi = 0;
        double f_fric = alpha * rho * gravity * mu * depth * effAi;//摩擦公式来自论文，密度在这
        f_fric        = f_fric > 0 ? -f_fric : 0;
        f += f_fric;
        f *= Cdrag;

		//这里就没运行

        dragF[topid] = relv * f;
        dragT[topid] = relp.cross(relv * f);

        relvDf[topid] = dragF[topid].dot(body[bodyid].linVelocity - velArr[tid])
                        + dragT[topid].dot(body[bodyid].angVelocity);
    }
    else
    {
        dragF[tid * 2 + 1]  = Vector3d();
        dragT[tid * 2 + 1]  = Vector3d();
        relvDf[tid * 2 + 1] = 0.0;
    }
	//说到底是这一段没看明白……
    // bottom contact.
    {
        double depth = posi[1] - botH[tid];//刚体最低处？
        int    botid = 2 * tid;

        // Relative velocity.
        Vector3d relp = posi;
        relp[1]       = botH[tid];
        relp -= body[bodyid].pose.position;
        Vector3d relv = body[bodyid].linVelocity + body[bodyid].angVelocity.cross(relp);//表征了刚体速度和角速度
        //relv -= velArr[tid];//wkm注释掉了，不然drag方向有误，导致小球拐弯。

        double effAi = botN[tid].dot(relv) * effA / (abs(botN[tid][1]) + 0.05);

        // Force
        double f = beta * rho * effAi;
        f        = f > 0 ? -f : 0;

        // Frictional drag.
        double relvNorm = relv.norm();
        if (relvNorm > EPSILON)
            effAi /= (relvNorm * relvNorm);
        else
            effAi = 0;

        double f_fric = alpha * rho * gravity * mu * depth * effAi;

        //if (body[bodyid].linVelocity[0] != 0 || body[bodyid].linVelocity[1] != 0 || body[bodyid].linVelocity[2] != 0)
        //	printf("Vel>0 : %lf  %lf %lf, Norm: %lf %lf %lf\n", f_fric, botN[tid].dot(relv), effAi,
        //		relv[0], relv[1], relv[2]);

        f_fric = f_fric > 0 ? -f_fric : 0;
        f += f_fric;
        f *= Cdrag;

		//printf("\n\nf=%f\n\n", f);//f确实是负的。

        dragF[botid] = relv * f*20;//relv * f
		//printf("dragF[%d]=%f\n", botid, dragF[botid]);
        dragT[botid] = relp.cross(relv * f/100.0)*0;//cross是计算向量积。去掉拖拽力矩试试

		//printf("dragT[%d]=%f\n", botid, dragT[botid]);

		//找到问题了：小球轨迹不正常拐弯，是因拖拽力方向导致，去掉拖拽力之后就不拐了。所以问题在于这个拖拽力是在哪调用的！这里只是个值，不是向量。

        relvDf[botid] = dragF[botid].dot(body[bodyid].linVelocity - velArr[tid])  + dragT[botid].dot(body[bodyid].angVelocity);
    //有了这行，就自传了。
	}
}

void SandInteractionForceSolver::computeSingleDragForce(int i, Real dt)
{
    m_dragF.resize(m_particlePos->size() * 2);
    m_dragT.resize(m_particlePos->size() * 2);
    m_relvDf.resize(m_particlePos->size() * 2);

    m_dragF.reset();
    m_dragT.reset();
    m_relvDf.reset();

    // debug
    CTimer timer;
    timer.start();

    cuExecute(m_particlePos->size(), SandIFS_DragForceVel, m_dragF, m_dragT, m_relvDf, m_topH, m_botH, m_topNormal, m_botNormal, *m_particleMass, *m_particlePos, *m_particleVel, *m_land, *m_body,
              //m_averageBodyInfo,
              i,// m_topH, m_botH这俩都是全0.
              m_rho,
              (m_sandMu + m_hostBody[i].mu) / 2.0,
              m_gravity,
              m_alpha,
              m_beta,
              m_Cdrag);

    timer.stop();
    //printf("      DragForce kernel time:  %lf \n", timer.getElapsedTime());

    //// debug
    //HostDArray<Vector3d> hostF;
    //hostF.resize(m_dragF.size());
    //Function1Pt::copy(hostF, m_dragF);

    //HostDArray<Vector3d> hostT;
    //hostT.resize(m_dragT.size());
    //Function1Pt::copy(hostT, m_dragT);

    //hostF.release();
    //hostT.release();

    timer.start();

    Vector3d dragF = thrust::reduce(thrust::device, m_dragF.begin(), m_dragF.begin() + m_dragF.size(), Vector3d(), thrust::plus<Vector3d>());

    Vector3d dragT/* = thrust::reduce(thrust::device, m_dragT.begin(), m_dragT.begin() + m_dragT.size(), Vector3d(), thrust::plus<Vector3d>())*/;

    double relvdf = thrust::reduce(thrust::device, m_relvDf.begin(), m_relvDf.begin() + m_relvDf.size(), ( double )0.0, thrust::plus<double>());

    timer.stop();
    //printf("      DragForce summation time:  %lf \n", timer.getElapsedTime());

    // debug

    Vector3d& debLinv = (m_hostBody + i)->linVelocity;
    Vector3d& debAngv = (m_hostBody + i)->angVelocity;
    if (false)
    {
        printf("  Body vel (before DRAG):  %lf %lf %lf,  %lf %lf %lf\n",//拖拽力能有两千多？而且各方向的力都很大。
               debLinv[0],
               debLinv[1],
               debLinv[2],
               debAngv[0],
               debAngv[1],
               debAngv[2]);
        printf("BEF:   Drag F: %lf %lf %lf, Drag T: %lf %lf %lf, Reldf: %lf \n",
               dragF[0],
               dragF[1],
               dragF[2],
               dragT[0],
               dragT[1],
               dragT[2],
               relvdf);
    }

    //if (dragF.norm() > 0 || dragT.norm() > 0)
    this->_stableDamping(i, dragF, dragT, dt);

    double alpha = this->_minEng(dragF, dragT, relvdf, i, dt);
    //double alpha = 1.0;
    dragF *= alpha;
    dragT *= alpha;
	dragT[0] = dragT[0] > 2000 ? 2000.0 : dragT[0];//截断2000
	dragT[1] = dragT[1] > 2000 ? 2000.0 : dragT[1];
	dragT[2] = dragT[2] > 2000 ? 2000.0 : dragT[2];

    if (true)
    {
        printf("AFT:   Drag F: %lf %lf %lf, Drag T: %lf %lf %lf, Alpha: %lf \n",
               dragF[0],
               dragF[1],
               dragF[2],
               dragT[0],
               dragT[1],
               dragT[2],
               alpha);
    }

    this->_applyForceTorque(dragF, dragT, i, dt);//把力和力矩用于计算位姿pose 7维向量！！！！！！！！

    if (false)
    {
        printf("  Body vel(after DRAG):  %lf %lf %lf,  %lf %lf %lf\n",
               debLinv[0],
               debLinv[1],
               debLinv[2],
               debAngv[0],
               debAngv[1],
               debAngv[2]);
    }
}





__global__ void SandIFS_updateParticleVel(
    DeviceDArray<Vector3d>           dVel,
    DeviceDArray<Vector3d>           parVel,
    DeviceDArray<double>             massArr,
    DeviceDArray<Vector3d>           posArr,
    DeviceDArray<double>             topH,
    DeviceDArray<double>             botH,
    DeviceDArray<Vector3d>           topN,
    DeviceDArray<Vector3d>           botN,
    DeviceHeightField1d              land,
    DeviceArray<PBDBodyInfo<double>> body,
    int                              bodyid,
    double                           sampleDl,
    double                           rho_s, /*double rho_r,*/
    double                           e,
    double                           Chorizon,
    double                           Cvertical,
    double                           Cprob = 10000)
{
    int tid = threadIdx.x + blockDim.x * blockIdx.x;
    if (tid >= massArr.size())
        return;

    if (topH[tid] <= botH[tid])
        return;

    Vector3d posi  = posArr[tid];
    double   hsand = posi[1] - land.get(posi[0], posi[2]);

    if (hsand < EPSILON)
        return;

    Vector3d dvel;
    //double curh = topH[tid];
    //while (curh > botH[tid])
    //{
    //	Vector3d curp = posi;
    //	curp[1] = curh;
    //	Vector3d velObj = body[bodyid].getVelocityAt(curp);
    //	dvel += (velObj - parVel[tid])*((1.0 + e) * sampleDl);

    //	curh -= sampleDl;
    //}

    //dvel /= (topH[tid] - botH[tid]);

    posi[1]         = botH[tid];
    Vector3d velObj = body[bodyid].getVelocityAt(posi);
    dvel            = (velObj - parVel[tid]) * (1.0 + e);
    double dvelN    = (dvel.dot(botN[tid]));
    if (dvelN > 0)
    {
        dvel = botN[tid] * dvelN;
        //dvel = topN[tid] * (dvel.dot(topN[tid]));
    }

    dvel[0] *= Chorizon;
    dvel[2] *= Chorizon;
    dvel[1] *= Cvertical;

    double     prob = (topH[tid] - botH[tid]) / hsand * Cprob;
    RandNumber gen(posi[0] * 7000 + posi[2] * 999999);
    double     probval = gen.Generate();

    if (probval >= prob)
    {
        dvel = Vector3d();
    }

    dvel[1]   = dvel[1] > 0.0 ? 0.0 : dvel[1];//是不是就这个错了？！
	/*for(int i=0;i<3;i++){
	dvel[i] = dvel[i] > 1.0 ?  dvel[i]:1.0;
	}*/
    dVel[tid] = dvel;


	//printf("沙子速度：\n dVel[tid]=%f\n", pow(dvel[0]* dvel[0]+ dvel[1] * dvel[1]+ dvel[2] * dvel[2],0.5));//慢慢的耦合速度变成千分位级的了，比较小了，因存在SSE截断，所以就直接保型了看起来不耦合。

    ////dVel[tid] = Vector3d();
    //Vector3d finalvel = parVel[tid];// +dVel[tid];
    //Vector3d objLinv = body[bodyid].linVelocity;
    //printf("Vel final: %lf %lf %lf;  %lf %lf %lf\n", finalvel[0], finalvel[1], finalvel[2],
    //	objLinv[0], objLinv[1], objLinv[2]);

    //parVel[tid][1] = 0.0;
}

__global__ void SandIFS_updateParticleVel_Stick(
    DeviceDArray<Vector3d>           dVel,
    DeviceDArray<Vector3d>           parVel,
    DeviceDArray<double>             massArr,
    DeviceDArray<Vector3d>           posArr,
    DeviceDArray<double>             topH,
    DeviceDArray<double>             botH,
    DeviceDArray<Vector3d>           topN,
    DeviceDArray<Vector3d>           botN,
    DeviceHeightField1d              land,
    DeviceArray<PBDBodyInfo<double>> body,
    int                              bodyid,
    double                           sampleDl,
    double                           rho_s,
    /*double rho_r,*/ double         e,
    double                           csHorizon,
    double                           csVertial)
{
    int tid = threadIdx.x + blockDim.x * blockIdx.x;
    if (tid >= massArr.size())
        return;

    if (topH[tid] <= botH[tid])
        return;

    Vector3d posi  = posArr[tid];
    double   hsand = posi[1] - land.get(posi[0], posi[2]);

    if (hsand < EPSILON)
        return;

    Vector3d dvel;
    double   curh = topH[tid];
    //while (curh > botH[tid])
    //{
    //	Vector3d curp = posi;
    //	curp[1] = curh;
    //	Vector3d velObj = body[bodyid].getVelocityAt(curp);
    //	dvel += (velObj - parVel[tid])*((1.0 + e) * sampleDl);

    //	curh -= sampleDl;
    //}
    //dvel *= (body[bodyid].rho /
    //	(massArr[tid] + massArr[tid] / rho_s * body[bodyid].rho * (topH[tid] - botH[tid])));

    while (curh > botH[tid])
    {
        Vector3d curp   = posi;
        curp[1]         = curh;
        Vector3d velObj = /*body[bodyid].linVelocity;*/ body[bodyid].getVelocityAt(curp);
        dvel += velObj * sampleDl;

        curh -= sampleDl;

        //// debug
        //if (abs(velObj[0]) >EPSILON || abs(velObj[1]) > EPSILON || abs(velObj[2]) > EPSILON)
        //{
        //	printf("velObj:  %lf %lf %lf\n", velObj[0], velObj[1], velObj[2]);
        //}
    }
    dvel /= hsand;  // (topH[tid] - botH[tid]);

    //// debug
    //if (dvel[0] != 0 || dvel[1] != 0 || dvel[2] != 0)
    //{
    //	printf("DVEL:  %lf %lf %lf\n", dvel[0], dvel[1], dvel[2]);
    //}

    dvel[0] *= csHorizon;
    dvel[2] *= csHorizon;
    dvel[1] *= csVertial;

    //dvel[0] = 0.0;
    //dvel[2] = 0.0;

    dvel[1]   = dvel[1] > 0.0 ? 0.0 : dvel[1];
    dVel[tid] = dvel;

    //parVel[tid][1] = 0.0;
}

void SandInteractionForceSolver::computeParticleInteractVelocity(int i, Real dt)
{
    if (!m_particleVel)
        return;

    m_dVel.resize(m_particlePos->size());
    m_dVel.reset();

    if (m_useStickParticleVelUpdate)//两种模式
    {
        cuExecute(m_particlePos->size(), SandIFS_updateParticleVel_Stick, m_dVel, *m_particleVel, *m_particleMass, *m_particlePos, m_topH, m_botH, m_topNormal, m_botNormal, *m_land, *m_body, i, m_sampleSize, m_rho, m_e, m_CsHorizon, m_CsVertical);
    }
    else
    {//用的这个
        cuExecute(m_particlePos->size(), SandIFS_updateParticleVel, m_dVel, *m_particleVel, *m_particleMass, *m_particlePos, m_topH, m_botH, m_topNormal, m_botNormal, *m_land, *m_body, i, m_sampleSize, m_rho, m_e, m_CsHorizon, m_CsVertical, m_Cprob);
    }
}

__global__ void SandIFS_accumulate(//没调用
    DeviceDArray<double>             relvDf,
    DeviceDArray<Vector3d>           dforce,
    DeviceDArray<Vector3d>           dtorque,
    DeviceDArray<double>             buoF,
    DeviceDArray<Vector3d>           buoT,
    DeviceDArray<Vector3d>           dragF,
    DeviceDArray<Vector3d>           dragT,
    DeviceDArray<Vector3d>           parVel,
    double                           buoA,
    DeviceArray<PBDBodyInfo<double>> body,
    int                              bodyid)
{
    int tid = threadIdx.x + blockDim.x * blockIdx.x;
    if (tid >= parVel.size())
        return;
    PBDBodyInfo<double>& bodyi = body[bodyid];

    int botid = 2 * tid;
    int topid = 2 * tid + 1;

    // bot.
    dforce[botid]  = dragF[botid] + Vector3d(0.0, buoF[botid], 0.0) * buoA;
    dtorque[botid] = dragT[botid] + buoT[botid] * buoA;
    relvDf[botid]  = dforce[botid].dot(bodyi.linVelocity - parVel[tid])
                    + dtorque[botid].dot(bodyi.angVelocity);

    // top.
    dforce[topid]  = dragF[topid] + Vector3d(0.0, buoF[topid], 0.0) * buoA;
    dtorque[topid] = dragT[topid] + buoT[topid] * buoA;
    relvDf[topid]  = dforce[topid].dot(bodyi.linVelocity - parVel[tid])
                    + dtorque[topid].dot(bodyi.angVelocity);
}

//void SandInteractionForceSolver::computeSingleInteractionForce(int i, Real dt, Vector3d& force, Vector3d& torque)
//{
//	m_relvDf.resize(m_particlePos->size() * 2);
//	m_dForce.resize(m_particlePos->size() * 2);
//	m_dTorque.resize(m_particlePos->size() * 2);

//	m_relvDf.reset();
//	m_dForce.reset();
//	m_dTorque.reset();

//	cuExecute(m_particlePos->size(), SandIFS_accumulate,
//		m_relvDf, m_dForce, m_dTorque,
//		m_buoyancyF, m_buoyancyT,
//		m_dragF, m_dragT,
//		*m_particleVel,
//		m_Abuo,
//		*m_body, i
//	);

//	//// debug
//	//HostDArray<double> hostrelvdf;
//	//hostrelvdf.resize(m_relvDf.size());
//	//Function1Pt::copy(hostrelvdf, m_relvDf);

//	//hostrelvdf.release();

//	double relvdf = thrust::reduce(thrust::device,
//		m_relvDf.begin(), m_relvDf.begin() + m_relvDf.size(), (double)0.0, thrust::plus<double>());

//	force = thrust::reduce(thrust::device,
//		m_dForce.begin(), m_dForce.begin() + m_dForce.size(), Vector3d(), thrust::plus<Vector3d>());

//	torque = thrust::reduce(thrust::device,
//		m_dTorque.begin(), m_dTorque.begin() + m_dTorque.size(), Vector3d(), thrust::plus<Vector3d>());

//	double alpha = this->_minEng(force, torque, relvdf, i, dt);

//	force *= alpha;
//	torque *= alpha;

//}

//void SandInteractionForceSolver::compute(Real dt)//这个是整体算，下面是分开算。
//{
//    if (!m_body || m_body->size() <= 0)
//        return;
//    if (!m_particlePos || m_particlePos->size() <= 0)
//        return;
//
//    for (int i = 0; i < m_body->size(); ++i)
//    {
//
//        // Check collision filter.
//        if (m_prigids && !collisionValid((*m_prigids)[i]))
//            continue;
//
//        this->updateSinkInfo(i);
//        this->computeSingleBuoyance(i, dt);
//        this->_copyHostBodyToGPU(i);
//        this->computeSingleDragForce(i, dt);
//        this->_copyHostBodyToGPU(i);
//
//        this->computeParticleInteractVelocity(i, dt);
//
//        this->_smoothVelocityChange();
//    }
//}

void SandInteractionForceSolver::computeSingleBody(int i, Real dt)//TODO
{

    if (!m_hostBody || !m_body || m_body->size() <= 0)
        return;
    if (!m_particlePos || m_particlePos->size() <= 0)
        return;

    // Check collision filter.
    if (m_prigids && !collisionValid((*m_prigids)[i]))
        return;

    //// debug
    //CTimer timer;

    //timer.start();
    this->updateSinkInfo(i);
    //timer.stop();
    //printf("   Interact, Update SinkInfo time:  %lf\n", timer.getElapsedTime());

    //timer.start();

	//沙对刚
    this->computeSingleBuoyance(i, dt);//注释掉看看
    //timer.stop();
    //printf("   Interact, Buoyance time:  %lf\n", timer.getElapsedTime());

    //timer.start();
    this->_copyHostBodyToGPU(i);//也注释掉试试
    //timer.stop();
    //printf("   Interact, Update body info time:  %lf\n", timer.getElapsedTime());

    //timer.start();
    this->computeSingleDragForce(i, dt);//注释掉看看
    //timer.stop();
    //printf("   Interact, Drag force time:  %lf\n", timer.getElapsedTime());

    //timer.start();
    this->_copyHostBodyToGPU(i);
    //timer.stop();
    //printf("   Interact, Update body info time:  %lf\n", timer.getElapsedTime());

    //timer.start();

	//刚对沙
    this->computeParticleInteractVelocity(i, dt);
    //timer.stop();
    //printf("   Interact, Particle vel change time:  %lf\n", timer.getElapsedTime());

    //timer.start();
    this->_smoothVelocityChange();//注释掉看看
    //timer.stop();
    //printf("   Interact, Smooth vel change time:  %lf\n", timer.getElapsedTime());
}

__global__ void SandIFS_smoothVelocityChange(
    DeviceDArray<Vector3d> parVel,
    DeviceDArray<Vector3d> dVel,
    DeviceDArray<double>   massArr,
    DeviceDArray<Vector3d> posArr,
    NeighborList<int>      neighbors,
    SpikyKernel2D<double>  kern,
    double                 smoothlength)
{
    int tid = threadIdx.x + blockDim.x * blockIdx.x;
    if (tid >= parVel.size())
        return;

    Vector3d pos_i  = posArr[tid];
    double   weight = 0.0;
    Vector3d dv;

    int nbSize = neighbors.getNeighborSize(tid);
    for (int ne = 0; ne < nbSize; ne++)
    {
        int    j = neighbors.getElement(tid, ne);
        double r = (pos_i - posArr[j]).norm();
        //if (r < EPSILON) continue;
        double w = kern.Weight(r, smoothlength);
        dv += dVel[j] * w;
        weight += w;
    }
    if (weight > EPSILON /*&& nbSize>5*/)
    {
        dv /= weight;
        parVel[tid] += dv;

        /*if (dv[0] < -0.05)
				printf("%d,  Dv:  %lf %lf %lf,  origin: %lf %lf %lf\n", tid,
					dv[0], dv[1], dv[2], dVel[tid][0], dVel[tid][1], dVel[tid][2]);*/
    }
}

__global__ void SandIFS_directUpdateVelocityChange(
    DeviceDArray<Vector3d> parVel,
    DeviceDArray<Vector3d> dVel)
{
    int tid = threadIdx.x + blockDim.x * blockIdx.x;
    if (tid >= parVel.size())
        return;

    parVel[tid] += dVel[tid];

    //// debug
    //if (dVel[tid][0] != 0 || dVel[tid][1] != 0 || dVel[tid][2] != 0)
    //{
    //	printf("ParticleVel: %lf %lf %lf\n", parVel[tid][0], parVel[tid][1], parVel[tid][2]);
    //}
}

void SandInteractionForceSolver::_smoothVelocityChange()
{
    if (m_dVel.size() <= 0)
        return;

    if (m_neighbor.isEmpty())
    {

        //static int callCount = 0;
        //++callCount;

        //HostDArray<Vector3d> hostvel;
        //hostvel.resize(m_dVel.size());
        //Function1Pt::copy(hostvel, m_dVel);

        //for (int i = 0; i < hostvel.size(); ++i)
        //{
        //	if (abs(hostvel[i][1]) > 0.001 || abs(hostvel[i][0]) > 0.00 || abs(hostvel[i][2]) > 0.00)
        //	{
        //		printf("  Grid vel: %d,   %lf %lf %lf \n", i, hostvel[i][0], hostvel[i][1], hostvel[i][2]);
        //	}
        //}

        //if (callCount > 170)
        //	return;
        cuExecute(m_particleVel->size(), SandIFS_directUpdateVelocityChange, *m_particleVel, m_dVel);
    }
    else
    {

        cuExecute(m_particlePos->size(), SandIFS_smoothVelocityChange, *m_particleVel, m_dVel, *m_particleMass, *m_particlePos, m_neighbor.getValue(), m_kernel, m_smoothLength

        );
    }
}

double SandInteractionForceSolver::_enlargerBuoyancy(double f, const Vector3d& t, double mass)
{
    double A = mass * m_gravity / m_buoyancyFactor;
    A        = A * (std::exp(f / A) - 1);
    //f *= A;
    //t *= A;
    return A;
}

double SandInteractionForceSolver::_minEng(const Vector3d& dF, const Vector3d& dT, double relvdf, int i, double dt)
{
    if (!m_hostBody)
        return 0.0;
    auto prigid = m_hostBody + i;
    if (!prigid)
        return 0.0;

    float    invMass    = prigid->invMass;
    Vector3d invInertia = prigid->invInertia;

    //Vector3f dTlocal(dT[0], dT[1], dT[2]);
    Vector3d dTlocal = prigid->pose.rotation.getConjugate().rotate(dT);

    double dfInvmDf = dF.dot(dF) * invMass + dTlocal.dot(dTlocal * invInertia);

    if (abs(dfInvmDf) < EPSILON)
        return 0.0;

    double alpha = -relvdf / (dfInvmDf * dt);

    alpha = alpha > 1.0 ? 1.0 : alpha;
    alpha = alpha < 0.0 ? 0.0 : alpha;

    return alpha;
}

void SandInteractionForceSolver::_applyForceTorque(const Vector3d& F, const Vector3d& T, int i, Real dt)
{
    if (!m_hostBody)
        return;

    auto pbody = m_hostBody + i;
    pbody->integrateForceToVelPos(F, T, dt);
    //pbody->integrateForce(F, T, dt);
}

void SandInteractionForceSolver::_stableDamping(int i, Vector3d& F, Vector3d& T, Real dt)
{
    if (!m_hostBody)
        return;

    auto   pbody    = m_hostBody + i;
    double linvnorm = pbody->linVelocity.norm();

    Vector3d tmpv    = F * (pbody->invMass * dt);
    double   maxlinv = tmpv.norm();
    if (tmpv.dot(pbody->linVelocity) < 0 && linvnorm < /*m_gravity*dt * 0.5*/ maxlinv)
    {
        pbody->linVelocity = Vector3d();
        F                  = Vector3d();
    }
    double angvnorm = pbody->angVelocity.norm();
    tmpv            = (T * pbody->invInertia * dt);
    double maxangv  = tmpv.norm();
    if (tmpv.dot(pbody->angVelocity) < 0 && angvnorm < maxangv)
    {
        pbody->angVelocity = Vector3d();
        T                  = Vector3d();
    }
}

void SandInteractionForceSolver::_copyHostBodyToGPU(int i)
{
    cudaMemcpy(m_body->begin() + i, m_hostBody + i, sizeof(PBDBodyInfo<double>), cudaMemcpyHostToDevice);
}

bool SandInteractionForceSolver::collisionValid(RigidBody2_ptr prigid)
{
    if (!prigid)
        return false;
    bool collide = (prigid->getCollisionFilterGroup() & m_sandCollisionMask);
    collide      = collide && (prigid->getCollisionFilterMask() & m_sandCollisionGroup);
    return collide;
}

void SandInteractionForceSolver::setPreBodyInfo()
{
    if (!m_hostBody || !m_body)
        return;

    m_prevBody.resize(m_body->size());

    cuSafeCall(cudaMemcpy(m_prevBody.begin(), m_body->begin(), sizeof(PBDBodyInfo<double>) * m_body->size(), cudaMemcpyDeviceToHost));
}

void SandInteractionForceSolver::updateBodyAverageVel(Real dt)
{
    if (m_prevBody.size() <= 0 || !m_hostBody)
        return;

    for (int i = 0; i < m_prevBody.size(); ++i)
    {
        auto pbody             = m_hostBody + i;
        m_prevBody[i].prevPose = m_prevBody[i].pose;
        m_prevBody[i].pose     = pbody->pose;//算过之后的pose了，，咋算的？
        m_prevBody[i].updateVelocity(dt);//更新刚体速度，在PBDBodyInfo里面了，是刚体计算了
    }

    //
    m_averageBodyInfo.resize(m_prevBody.size());
    Function1Pt::copy(m_averageBodyInfo, m_prevBody);

    //cuSafeCall(cudaMemcpy(m_body->begin(), m_hostBody, sizeof(PBDBodyInfo<double>)*m_body->size(),
    //	cudaMemcpyHostToDevice));
}

}  // namespace PhysIKA