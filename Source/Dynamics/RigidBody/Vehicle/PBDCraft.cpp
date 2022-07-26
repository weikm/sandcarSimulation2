#include "Dynamics/ParticleSystem/StaticBoundary.h"
#include "Dynamics/RigidBody/FreeJoint.h"
#include "Dynamics/RigidBody/RigidUtil.h"
#include "VehicleFrontJoint.h"
#include "VehicleRearJoint.h"
#include "Dynamics/RigidBody/FixedJoint.h"
#include "Framework/Action/Action.h"
#include <string>
#include "PBDCraft.h"
#include <functional>
#include <cmath>
namespace PhysIKA {
	bool PBDCraft::build()//这是很重要的函数
	{
		// *** Rigid chassis.
		m_chassis = std::make_shared<RigidBody2<DataType3f>>("Chassis");
		this->addChild(m_chassis);

		if (chassisFile != "")
		{
			m_chassis->loadShape(chassisFile);
			//Vector3f chassisMeshScale(0.3, 0.2, 0.5);
			((std::dynamic_pointer_cast<TriangleSet<DataType3f>>)(m_chassis->getTopologyModule()))->scale(chassisMeshScale);//挂上拓扑模块，以及下面的scale和Translate
			((std::dynamic_pointer_cast<TriangleSet<DataType3f>>)(m_chassis->getTopologyModule()))->translate(chassisMeshTranslate);
		}

		// Rigid inertia and position.设置底盘的参数
		m_chassis->setI(Inertia<float>(chassisMass, chassisInertia));
		m_chassis->setGlobalR(carPosition);// 位置
		m_chassis->setGlobalQ(carRotation);//小车旋转角
		m_chassis->setExternalForce(Vector3f(0.0 * chassisMass, -9.8 * chassisMass, 0));//设置外部力（重力）
		m_chassis->setLinearDamping(linearDamping);//设置阻尼
		m_chassis->setAngularDamping(angularDamping);

		// Collision filter.碰撞过滤//过滤？试试注释掉
		m_chassis->setCollisionFilterGroup(chassisCollisionGroup);
		m_chassis->setCollisionFilterMask(chassisCollisionMask);

		int idchassis = m_rigidSolver->addRigid(m_chassis);

		//return true;
		Quaternionf localToStd_ = _rotationToStandardLocal();
		Quaterniond localToStd(localToStd_.x(), localToStd_.y(), localToStd_.z(), localToStd_.w());

		

		forwardForcePoint = (wheelRelPosition[0] + wheelRelPosition[1]+ wheelRelPosition[2] + wheelRelPosition[3]) / 4;

		m_chassis->setLinearVelocity(m_chassis->getLinearVelocity() * 0.99);//最后这个0.99是加速之后的用来自动减速的比率吧，但是速度是个假参数
		m_rigidSolver->addCustomUpdateFunciton(std::bind(&PBDCraft::updateForce, this, std::placeholders::_1));
		//上面这句到底啥意思？
		return true;
	}

	void PBDCraft::advance(Real dt)//没有这个函数那么车一跑就没完了//但这个函数在沙地项目中未引用
	{
		if (!m_accPressed)
		{
			forwardForce = 0;//牵引力置零，，按下w或s松手之后应该调用这个，卸掉牵引力
			upForce = 0;
			rightForce = 0;
			forwardTorque = { 0,0,0 };
			rightTorque = { 0,0,0 };
			upTorque = { 0,0,0 };

			//非水平角度也要置零，保持水平。要操作四元数。//其实不用了，角速度再下面已经自杀车了

			m_chassis->setLinearVelocity(m_chassis->getLinearVelocity() * 0.8);//0.99改小的话就给上了自动刹车，实现了阻尼！0.88适用于无人机
			m_chassis->setAngularVelocity(m_chassis->getAngularVelocity() * 0.8);
			//角速度也要一个自杀车！
			//点乘判断前进/后退
			//然后给一个阻力，大小与速度大小成正比，方向与速度指针方向相反
			
		}
		m_accPressed = false;

		//this->_updateWheelRotation(dt);
		this->_doVelConstraint(dt);
		this->m_rigidSolver->setBodyDirty();

		//Vector3f carVel = m_chassis->getLinearVelocity();
		//printf("Car Vel:  %lf %lf %lf \n", carVel[0], carVel[1], carVel[2]);

		Vector3f a = m_chassis->getLinearVelocity();
		//std::printf("wkmznb%f%f%f", a[0], a[1], a[2]);//输出速度向量

		return;
	}

	//前进，按下w则调用此函数
	void PBDCraft::forward(Real dt)
	{
		//forwardDir = this->_getForwardDir();
		if (forwardForce<10000 && forwardForce>-10000) {
			forwardForce += forwardForceAcc * (dt >= 0 ? 1 : -1);//前向牵引力！//前向牵引力增加量forwardForceAcc在demoparticlesand里面设置的是1000
		}
		/*
		倾角=(wheelRelPosition[0] - wheelRelPosition[1])叉乘(wheelRelPosition[2] - wheelRelPosition[1])点乘{0,1,0}/向量的模的乘积
		*/
		//Vector3f faxiangliang = (wheelRelPosition[0] - wheelRelPosition[1]).cross(wheelRelPosition[3] - wheelRelPosition[1]);
		//float dianji = faxiangliang.dot({ 0.0,1.0,0.0 });
		//float jiao = acos(dianji / faxiangliang.norm());
		////相当于是要求一个二面角（线面角），不用四元数。由轮子位置得到垂直向上向量，再求此向量与010的夹角。小于15度即是if条件
		////给倾角力矩
		//printf("%f,%f,%f\n", faxiangliang[0], faxiangliang[1], faxiangliang[2]);//说明，这个法向量有问题。知道了，开始运动之后四轮位置没有写回来
		//printf("faxiangliang.dot({ 0.0,1.0,0.0 })=%f\n", faxiangliang.dot({ 0.0,1.0,0.0 }));
		//printf("faxiangliang.norm()=%f\n", faxiangliang.norm());
		//printf("倾角大小=%f\n",jiao);
		//if (jiao < 3.14 / 12) {
		//	if (dt > 0) {
		//		rightTorque = { 0, 0, -100 };
		//	}
		//	else if (dt < 0) {
		//		rightTorque = { 0, 0, 100 };
		//	}
		//}

		m_accPressed = true;
	}

	//后退，按下s则调用此函数
	void PBDCraft::backward(Real dt)
	{
		forward(-dt);
	}
	//左打轮，按下a则调用此函数
	void PBDCraft::goLeft(Real dt)//要改，直接向左移
	{
		if (rightForce<10000 && rightForce>-10000) {
			rightForce += rightForceAcc * (dt >= 0 ? 1 : -1);//右向牵引力！
		}
		m_accPressed = true;

		//Vector3f faxiangliang = (wheelRelPosition[0] - wheelRelPosition[1]).cross(wheelRelPosition[2] - wheelRelPosition[1]);
		//float jiao = acos(faxiangliang.dot({ 0.0,1.0,0.0 }) / faxiangliang.norm());
		////给个倾角力矩
		//if (jiao < 3.14 / 12) {
		//	if (dt > 0) {
		//		forwardTorque = { -100, 0, 0 };
		//	}
		//	else if (dt < 0) {
		//		forwardTorque = { 100, 0, 0 };
		//	}
		//}
	}

	//右打轮，按下d则调用此函数
	void PBDCraft::goRight(Real dt)
	{
		goLeft(-dt);
	}
	//向上，按下q调用此函数
	void PBDCraft::goUp(Real dt)
	{
		//forwardDir = this->_getForwardDir();
		if (upForce<10000 && upForce>-10000) {
			upForce += upForceAcc * (dt >= 0 ? 1 : -1);//上向牵引力！
		}
		m_accPressed = true;
	}

	void PBDCraft::goDown(Real dt)
	{
		goUp(-dt);
	}

	void PBDCraft::zizhuan(Real dt) {
		upTorque = { 0,1000,0 };

	}



	//brake刹车
	void PBDCraft::brake(Real dt)
	{
		m_chassis->setLinearVelocity(m_chassis->getLinearVelocity() * 0.5);
	}

	void PBDCraft::_setRigidForceAsGravity()
	{//设置外部力和力矩
		m_chassis->setExternalForce(m_gravity * chassisMass);
		m_chassis->setExternalTorque(Vector3f());

		for (int i = 0; i < 4; ++i)
		{
			m_wheels[i]->setExternalForce(m_gravity * wheelMass[i]);
			m_wheels[i]->setExternalTorque(Vector3f());

			m_steeringRigid[i]->setExternalForce(m_gravity * wheelMass[i]);
			m_steeringRigid[i]->setExternalTorque(Vector3f());
		}
	}

	void PBDCraft::computeAABB(std::shared_ptr<PointSet<DataType3f>> points, Vector3f& center, Vector3f& halfSize)
	{
		int nPoints = points->getPointSize();
		if (nPoints <= 0)
			return;

		auto&               pointArr = points->getPoints();
		HostArray<Vector3f> hpoints;
		hpoints.resize(nPoints);
		PhysIKA::Function1Pt::copy(hpoints, pointArr);

		Vector3f pmin = hpoints[0];
		Vector3f pmax = hpoints[0];
		for (int i = 1; i < nPoints; ++i)
		{
			Vector3f curp = hpoints[i];
			pmin[0] = min(pmin[0], curp[0]);
			pmin[1] = min(pmin[1], curp[1]);
			pmin[2] = min(pmin[2], curp[2]);
			pmax[0] = max(pmax[0], curp[0]);
			pmax[1] = max(pmax[1], curp[1]);
			pmax[2] = max(pmax[2], curp[2]);
		}

		center = (pmin + pmax) * 0.5;
		halfSize = (pmax - pmin) * 0.5;
	}

	void PBDCraft::_doVelConstraint(Real dt)
	{
		double linDamp = pow(1.0 - linearDamping, dt);//幂函数
		double angDamp = pow(1.0 - angularDamping, dt);

		m_chassis->setLinearVelocity(m_chassis->getLinearVelocity() * linDamp);
		m_chassis->setAngularVelocity(m_chassis->getAngularVelocity() * angDamp);

		if (m_chassis->getLinearVelocity().norm() > maxVel)
		{
			double fac = maxVel / m_chassis->getLinearVelocity().norm();
			m_chassis->setLinearVelocity(m_chassis->getLinearVelocity() * fac);
		}

		if (m_chassis->getAngularVelocity().norm() > maxAngVel)
		{
			double fac = maxAngVel / m_chassis->getAngularVelocity().norm();
			m_chassis->setAngularVelocity(m_chassis->getAngularVelocity() * fac);
		}
	}

	Quaternionf PBDCraft::_rotationToStandardLocal()
	{
		// updir <==> (0, 1, 0)
		// rightdir <==> (1, 0, 0)

		Vector3f stdup(0, 1, 0);
		Vector3f stdright(1, 0, 0);

		Quaternionf rot;

		Vector3f axisUp = wheelupDirection.cross(stdup);
		if (axisUp.norm() > 1e-3)
		{
			axisUp.normalize();

			float rad = wheelupDirection.dot(stdup);
			rad = acos(rad);
			rot = Quaternionf(axisUp, rad);
		}

		Vector3f axisRight = wheelRightDirection.cross(stdright);
		if (axisRight.norm() > 1e-3)
		{
			axisRight.normalize();
			float rad = wheelRightDirection.dot(stdright);
			rad = acos(rad);
			rot = Quaternionf(axisRight, rad) * rot;
		}

		return rot;
	}

	void PBDCraft::updateForce(Real dt)//在build里调用，更新受力
	{
		Vector3f chaForce;
		Vector3f chaTorque;

		forwardDir[0] = 1.0;//可能要根据车头朝向来定。
		forwardDir[1] = 0;
		forwardDir[2] = 0;
		rightDir[0] = 0;
		rightDir[1] = 0;
		rightDir[2] = 1.0;
		upDir[0] = 0;
		upDir[1] = 1.0;
		upDir[2] = 0;

		Vector3f forwardF = forwardDir * forwardForce;//牵引力矢量
		Vector3f rightF = rightDir * rightForce;
		Vector3f upF = upDir * upForce;

		Vector3f forwardT = m_chassis->getGlobalQ().rotate(forwardForcePoint).cross(forwardF);

		chaForce += forwardF;//合外力，那么有了这个力之后，是怎么调用的呢？没用到速度？直接时间积分算距离？
		chaForce += rightF;
		chaForce += upF;

		chaTorque += forwardT*0;//合力矩
		chaTorque += forwardTorque;
		chaTorque += rightTorque;
		chaTorque += upTorque;
		



		chaForce += m_gravity * m_chassis->getI().getMass();//重力加速度*底盘质量
		m_chassis->setExternalForce(chaForce);//底盘调用其力和力矩
		m_chassis->setExternalTorque(chaTorque);
	}

}  // namespace PhysIKA