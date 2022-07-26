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
		if (!m_accPressed)
		{
			forwardForce = 0;//ǣ�������㣬������w��s����֮��Ӧ�õ��������ж��ǣ����
			upForce = 0;
			rightForce = 0;
			forwardTorque = { 0,0,0 };
			rightTorque = { 0,0,0 };
			upTorque = { 0,0,0 };

			//��ˮƽ�Ƕ�ҲҪ���㣬����ˮƽ��Ҫ������Ԫ����//��ʵ�����ˣ����ٶ��������Ѿ���ɱ����

			m_chassis->setLinearVelocity(m_chassis->getLinearVelocity() * 0.8);//0.99��С�Ļ��͸������Զ�ɲ����ʵ�������ᣡ0.88���������˻�
			m_chassis->setAngularVelocity(m_chassis->getAngularVelocity() * 0.8);
			//���ٶ�ҲҪһ����ɱ����
			//����ж�ǰ��/����
			//Ȼ���һ����������С���ٶȴ�С�����ȣ��������ٶ�ָ�뷽���෴
			
		}
		m_accPressed = false;

		//this->_updateWheelRotation(dt);
		this->_doVelConstraint(dt);
		this->m_rigidSolver->setBodyDirty();

		//Vector3f carVel = m_chassis->getLinearVelocity();
		//printf("Car Vel:  %lf %lf %lf \n", carVel[0], carVel[1], carVel[2]);

		Vector3f a = m_chassis->getLinearVelocity();
		//std::printf("wkmznb%f%f%f", a[0], a[1], a[2]);//����ٶ�����

		return;
	}

	//ǰ��������w����ô˺���
	void PBDCraft::forward(Real dt)
	{
		//forwardDir = this->_getForwardDir();
		if (forwardForce<10000 && forwardForce>-10000) {
			forwardForce += forwardForceAcc * (dt >= 0 ? 1 : -1);//ǰ��ǣ������//ǰ��ǣ����������forwardForceAcc��demoparticlesand�������õ���1000
		}
		/*
		���=(wheelRelPosition[0] - wheelRelPosition[1])���(wheelRelPosition[2] - wheelRelPosition[1])���{0,1,0}/������ģ�ĳ˻�
		*/
		//Vector3f faxiangliang = (wheelRelPosition[0] - wheelRelPosition[1]).cross(wheelRelPosition[3] - wheelRelPosition[1]);
		//float dianji = faxiangliang.dot({ 0.0,1.0,0.0 });
		//float jiao = acos(dianji / faxiangliang.norm());
		////�൱����Ҫ��һ������ǣ�����ǣ���������Ԫ����������λ�õõ���ֱ���������������������010�ļнǡ�С��15�ȼ���if����
		////���������
		//printf("%f,%f,%f\n", faxiangliang[0], faxiangliang[1], faxiangliang[2]);//˵������������������⡣֪���ˣ���ʼ�˶�֮������λ��û��д����
		//printf("faxiangliang.dot({ 0.0,1.0,0.0 })=%f\n", faxiangliang.dot({ 0.0,1.0,0.0 }));
		//printf("faxiangliang.norm()=%f\n", faxiangliang.norm());
		//printf("��Ǵ�С=%f\n",jiao);
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

	//���ˣ�����s����ô˺���
	void PBDCraft::backward(Real dt)
	{
		forward(-dt);
	}
	//����֣�����a����ô˺���
	void PBDCraft::goLeft(Real dt)//Ҫ�ģ�ֱ��������
	{
		if (rightForce<10000 && rightForce>-10000) {
			rightForce += rightForceAcc * (dt >= 0 ? 1 : -1);//����ǣ������
		}
		m_accPressed = true;

		//Vector3f faxiangliang = (wheelRelPosition[0] - wheelRelPosition[1]).cross(wheelRelPosition[2] - wheelRelPosition[1]);
		//float jiao = acos(faxiangliang.dot({ 0.0,1.0,0.0 }) / faxiangliang.norm());
		////�����������
		//if (jiao < 3.14 / 12) {
		//	if (dt > 0) {
		//		forwardTorque = { -100, 0, 0 };
		//	}
		//	else if (dt < 0) {
		//		forwardTorque = { 100, 0, 0 };
		//	}
		//}
	}

	//�Ҵ��֣�����d����ô˺���
	void PBDCraft::goRight(Real dt)
	{
		goLeft(-dt);
	}
	//���ϣ�����q���ô˺���
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

	void PBDCraft::zizhuan(Real dt) {
		upTorque = { 0,1000,0 };

	}



	//brakeɲ��
	void PBDCraft::brake(Real dt)
	{
		m_chassis->setLinearVelocity(m_chassis->getLinearVelocity() * 0.5);
	}

	void PBDCraft::_setRigidForceAsGravity()
	{//�����ⲿ��������
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

	void PBDCraft::updateForce(Real dt)//��build����ã���������
	{
		Vector3f chaForce;
		Vector3f chaTorque;

		forwardDir[0] = 1.0;//����Ҫ���ݳ�ͷ����������
		forwardDir[1] = 0;
		forwardDir[2] = 0;
		rightDir[0] = 0;
		rightDir[1] = 0;
		rightDir[2] = 1.0;
		upDir[0] = 0;
		upDir[1] = 1.0;
		upDir[2] = 0;

		Vector3f forwardF = forwardDir * forwardForce;//ǣ����ʸ��
		Vector3f rightF = rightDir * rightForce;
		Vector3f upF = upDir * upForce;

		Vector3f forwardT = m_chassis->getGlobalQ().rotate(forwardForcePoint).cross(forwardF);

		chaForce += forwardF;//����������ô���������֮������ô���õ��أ�û�õ��ٶȣ�ֱ��ʱ���������룿
		chaForce += rightF;
		chaForce += upF;

		chaTorque += forwardT*0;//������
		chaTorque += forwardTorque;
		chaTorque += rightTorque;
		chaTorque += upTorque;
		



		chaForce += m_gravity * m_chassis->getI().getMass();//�������ٶ�*��������
		m_chassis->setExternalForce(chaForce);//���̵�������������
		m_chassis->setExternalTorque(chaTorque);
	}

}  // namespace PhysIKA