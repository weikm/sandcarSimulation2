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
	bool PBDCraft::build()//���Ǻ���Ҫ�ĺ���
	{
		// *** Rigid chassis.
		m_chassis = std::make_shared<RigidBody2<DataType3f>>("Chassis");
		this->addChild(m_chassis);

		if (chassisFile != "")
		{
			m_chassis->loadShape(chassisFile);
			//Vector3f chassisMeshScale(0.3, 0.2, 0.5);
			((std::dynamic_pointer_cast<TriangleSet<DataType3f>>)(m_chassis->getTopologyModule()))->scale(chassisMeshScale);//��������ģ�飬�Լ������scale��Translate
			((std::dynamic_pointer_cast<TriangleSet<DataType3f>>)(m_chassis->getTopologyModule()))->translate(chassisMeshTranslate);
		}

		// Rigid inertia and position.���õ��̵Ĳ���
		m_chassis->setI(Inertia<float>(chassisMass, chassisInertia));
		m_chassis->setGlobalR(carPosition);// λ��
		m_chassis->setGlobalQ(carRotation);//С����ת��
		m_chassis->setExternalForce(Vector3f(0.0 * chassisMass, -9.8 * chassisMass, 0));//�����ⲿ����������
		m_chassis->setLinearDamping(linearDamping);//��������
		m_chassis->setAngularDamping(angularDamping);

		// Collision filter.��ײ����//���ˣ�����ע�͵�
		m_chassis->setCollisionFilterGroup(chassisCollisionGroup);
		m_chassis->setCollisionFilterMask(chassisCollisionMask);

		int idchassis = m_rigidSolver->addRigid(m_chassis);

		//return true;
		Quaternionf localToStd_ = _rotationToStandardLocal();
		Quaterniond localToStd(localToStd_.x(), localToStd_.y(), localToStd_.z(), localToStd_.w());
		
		

		forwardForcePoint = (wheelRelPosition[0] + wheelRelPosition[1]+ wheelRelPosition[2] + wheelRelPosition[3]) / 4;

		m_chassis->setLinearVelocity(m_chassis->getLinearVelocity() * 0.99);//������0.99�Ǽ���֮��������Զ����ٵı��ʰɣ������ٶ��Ǹ��ٲ���
		m_rigidSolver->addCustomUpdateFunciton(std::bind(&PBDCraft::updateForce, this, std::placeholders::_1));
		//������䵽��ɶ��˼��
		return true;
	}

	void PBDCraft::advance(Real dt)//û�����������ô��һ�ܾ�û����//�����������ɳ����Ŀ��δ����
	{
		//std::cout << "advance����" << std::endl;
		if (!m_accPressed)
		{
			//std::cout << "advance��������" << std::endl;
			forwardForce = 0;//ǣ�������㣬������w��s����֮��Ӧ�õ��������ж��ǣ����
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

			m_chassis->setLinearVelocity(m_chassis->getLinearVelocity() * 0.8);//0.99��С�Ļ��͸������Զ�ɲ����ʵ�������ᣡ0.88���������˻�
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
			forwardForce += forwardForceAcc * (dt >= 0 ? 1 : -1);//ǰ��ǣ������//ǰ��ǣ����������forwardForceAcc��demoparticlesand�������õ���1000
		}

		m_accPressed = true;

		//make a move angle
		Vector3f faxiangliang = m_chassis->getGlobalQ().rotate({ 0, 1, 0 });//(wheelRelPosition[0] - wheelRelPosition[1]).cross(wheelRelPosition[2] - wheelRelPosition[1]);
		//std::cout << "������"<<faxiangliang[0] << faxiangliang[1] << faxiangliang[2]<<std::endl;
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
			rightForce += rightForceAcc * (dt >= 0 ? 1 : -1);//����ǣ������
		}
		m_accPressed = true;
		
		//make a move angle
		Vector3f faxiangliang = m_chassis->getGlobalQ().rotate({ 0, 1, 0 });//(wheelRelPosition[0] - wheelRelPosition[1]).cross(wheelRelPosition[2] - wheelRelPosition[1]);
		//std::cout << "������"<<faxiangliang[0] << faxiangliang[1] << faxiangliang[2]<<std::endl;
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
			upForce += upForceAcc * (dt >= 0 ? 1 : -1);//����ǣ������
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
			EastWindForce = 1000.0f;//��ͷ��advance����Ҫ���㡣
			++WindCount;
		}

		m_accPressed = true;
	}

	void PBDCraft::SouthWind(){
		if (WindCount < 10) {
			SouthWindForce = 1000.0f;//��ͷ��advance����Ҫ���㡣
			++WindCount;
		}
		m_accPressed = true;

	}

	void PBDCraft::WestWind(){
		if (WindCount < 10) {
			WestWindForce = 1000.0f;//��ͷ��advance����Ҫ���㡣
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
		double linDamp = pow(1.0 - linearDamping, dt);//�ݺ���
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

		forwardDir = { 1.0,0.0,0.0 };//����Ҫ���ݳ�ͷ��������.���ڶ���д���ˣ�����
		rightDir = { 0.0,0.0,1.0 };
		upDir = { 0.0,1.0,0.0 };


		Vector3f forwardF = forwardDir * forwardForce;//ǣ����ʸ��
		Vector3f rightF = rightDir * rightForce;
		Vector3f upF = upDir * upForce;

		Vector3f forwardT = m_chassis->getGlobalQ().rotate(forwardForcePoint).cross(forwardF);

		chaForce += forwardF;//����������ô���������֮������ô���õ��أ�û�õ��ٶȣ�ֱ��ʱ���������룿
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