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
#include <math.h>
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
		//std::cout << "advance跑了" << std::endl;
		if (!m_accPressed)
		{
			//std::cout << "advance里面跑了" << std::endl;
			forwardForce = 0;//牵引力置零，，按下w或s松手之后应该调用这个，卸掉牵引力
			upForce = 0;
			rightForce = 0;
			forwardTorque = { 0,0,0 };
			rightTorque = { 0,0,0 };
			upTorque = { 0,0,0 };

			EastWindForce = 0.0;
			SouthWindForce = 0.0;
			WestWindForce = 0.0;
			NorthWindForce = 0.0;

			WindCount = 0;
			restoreTorque = { 0,0,0 };

			m_chassis->setLinearVelocity(m_chassis->getLinearVelocity() * 0.8);//0.99改小的话就给上了自动刹车，实现了阻尼！0.88适用于无人机
			m_chassis->setAngularVelocity(m_chassis->getAngularVelocity() * 0.8);


			anglecount++;
			if(anglecount>2){//only really not level move ,do this.//20221013
				Vector3f faxiangliang = m_chassis->getGlobalQ().rotate({ 0, 1, 0 });
				if (faxiangliang.cross({ 0,1,0 })[0] != 0.0 || faxiangliang.cross({ 0,1,0 })[1] != 0.0 || faxiangliang.cross({ 0,1,0 })[2] != 0.0) {
					restoreTorque = faxiangliang.cross({ 0,1,0 }).normalize();
				}
			}
		}
		m_accPressed = false;

		//this->_updateWheelRotation(dt);
		this->_doVelConstraint(dt);
		this->m_rigidSolver->setBodyDirty();

		return;
	}

	void PBDCraft::forward(Real dt)
	{
		//forwardDir = this->_getForwardDir();
		if (forwardForce<10000 && forwardForce>-10000) {
			forwardForce += forwardForceAcc * (dt >= 0 ? 1 : -1);//前向牵引力！//前向牵引力增加量forwardForceAcc在demoparticlesand里面设置的是1000
		}

		m_accPressed = true;

		//make a move angle
		Vector3f faxiangliang = m_chassis->getGlobalQ().rotate({ 0, 1, 0 });//(wheelRelPosition[0] - wheelRelPosition[1]).cross(wheelRelPosition[2] - wheelRelPosition[1]);
		//std::cout << "法向量"<<faxiangliang[0] << faxiangliang[1] << faxiangliang[2]<<std::endl;
		float jiao = acos(faxiangliang.dot({ 0.0,1.0,0.0 }) / faxiangliang.norm());
		//set recover Torque
		if (jiao < 3.14 / 20 ) {
			if (dt > 0) {
				forwardTorque = { 0.0, 0.0, -800.0 };
			}
			else if (dt < 0) {
				forwardTorque = { 0, 0, 800 };
			}
		}
		anglecount = 0;
		//m_accPressed = true;
	}

	void PBDCraft::backward(Real dt)
	{
		forward(-dt);
	}
	
	void PBDCraft::goLeft(Real dt)
	{
		if (rightForce<10000 && rightForce>-10000) {
			rightForce += rightForceAcc * (dt >= 0 ? 1 : -1);//右向牵引力！
		}
		m_accPressed = true;
		
		//make a move angle
		Vector3f faxiangliang = m_chassis->getGlobalQ().rotate({ 0, 1, 0 });//(wheelRelPosition[0] - wheelRelPosition[1]).cross(wheelRelPosition[2] - wheelRelPosition[1]);
		//std::cout << "法向量"<<faxiangliang[0] << faxiangliang[1] << faxiangliang[2]<<std::endl;
		float jiao = acos(faxiangliang.dot({ 0.0,1.0,0.0 }) / faxiangliang.norm());
		//set recover Torque
		if (jiao < 3.14 / 20 ) {
			if (dt > 0) {
				forwardTorque = { 800.0 * 1.0, 0.0, 0.0  };
			}
			else if (dt < 0) {
				forwardTorque = { -800, 0, 0 };
			}
		}
		anglecount = 0;
		//m_accPressed = true;
	}

	
	void PBDCraft::goRight(Real dt)
	{
		goLeft(-dt);
	}
	
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

	void PBDCraft::zizhuan() {
		upTorque = { 0,1000,0 };

	}

	void PBDCraft::EastWind(){
		if (WindCount < 10) {
			EastWindForce = 1000.0f;//回头在advance里面要置零。
			++WindCount;
		}

		m_accPressed = true;
	}

	void PBDCraft::SouthWind(){
		if (WindCount < 10) {
			SouthWindForce = 1000.0f;//回头在advance里面要置零。
			++WindCount;
		}
		m_accPressed = true;

	}

	void PBDCraft::WestWind(){
		if (WindCount < 10) {
			WestWindForce = 1000.0f;//回头在advance里面要置零。
			++WindCount;
		}
		m_accPressed = true;

	}

	void PBDCraft::NorthWind(){
		if (true ) {
			NorthWindForce = 1000.0f;
			++WindCount;
		}

		m_accPressed = true;

	}

	void PBDCraft::brake()
	{
		m_chassis->setLinearVelocity(m_chassis->getLinearVelocity() * 0.5);
		m_accPressed = true;

	}

	void PBDCraft::_setRigidForceAsGravity()
	{
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

	void PBDCraft::updateForce(Real dt)
		//run 20 times
	{
		

		Vector3f chaForce;
		Vector3f chaTorque;

		forwardDir = { 1.0,0.0,0.0 };//可能要根据车头朝向来定.现在都是写死了！！！
		rightDir = { 0.0,0.0,1.0 };
		upDir = { 0.0,1.0,0.0 };


		Vector3f forwardF = forwardDir * forwardForce;//牵引力矢量
		Vector3f rightF = rightDir * rightForce;
		Vector3f upF = upDir * upForce;

		Vector3f forwardT = m_chassis->getGlobalQ().rotate(forwardForcePoint).cross(forwardF);

		chaForce += forwardF;//合外力，那么有了这个力之后，是怎么调用的呢？没用到速度？直接时间积分算距离？
		chaForce += rightF;
		chaForce += upF;

		chaTorque += forwardT*0;
		chaTorque += forwardTorque;
		chaTorque += rightTorque;
		chaTorque += upTorque;
		chaTorque += restoreTorque * 500;

		Vector3f EastWindDir = { -1.0,0.0,0.0 };
		Vector3f SouthWindDir = { 0.0,0.0,-1.0 };
		//Vector3f WestWindDir = { 1.0,0.0,0.0 };
		Vector3f NorthWindDir = { 0.0,0.0,1.0 };

		

		//add wind force
		chaForce += EastWindForce * EastWindDir;
		chaForce += SouthWindForce * SouthWindDir;
		//chaForce += WestWindForce * WestWindDir;
		chaForce += NorthWindForce * NorthWindDir;

		//std::cout << chaForce[0] << chaForce[1] << chaForce[2]<<std::endl;
		
		chaForce += m_gravity * m_chassis->getI().getMass();
		
		m_chassis->setExternalForce(chaForce);
		m_chassis->setExternalTorque(chaTorque);
	}

}  // namespace PhysIKA