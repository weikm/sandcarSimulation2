#pragma once

#include "GUI/GlutGUI/GLApp.h"
#include "Framework/Framework/Node.h"

#include "Dynamics/RigidBody/RigidBody2.h"
#include "Dynamics/RigidBody/PBDRigid/PBDSolver.h"



//using namespace PhysIKA;

namespace PhysIKA {
	class PBDCraft : public Node
	{
	public:
		PBDCraft() {}

		bool build();

		//virtual bool initialize() override;

		virtual void advance(Real dt);

		void forward(Real dt);

		void backward(Real dt);

		void goLeft(Real dt);

		void goRight(Real dt);

		void goUp(Real dt);

		void goDown(Real dt);

		void EastWind();
		void SouthWind();
		void WestWind();
		void NorthWind();

		void zizhuan();

		void brake();



		void PBDCraft::computeAABB(std::shared_ptr<PointSet<DataType3f>> points, Vector3f& center, Vector3f& halfSize);
		

		std::shared_ptr<RigidBody2<DataType3f>> getChassis()
		{
			return m_chassis;
		}
		std::shared_ptr<RigidBody2<DataType3f>> getWheels(int i)
		{
			return m_wheels[i];
		}

		void updateForce(Real dt);

	private:
		
		void _setRigidForceAsGravity();

		void _doVelConstraint(Real dt);

		Quaternionf _rotationToStandardLocal();

	public:
		Vector3f          carPosition;
		Quaternion<float> carRotation;

		Vector3f          wheelRelPosition[4];
		Quaternion<float> wheelRelRotation[4];

		Vector3f wheelupDirection;
		Vector3f wheelRightDirection;  // wheel right direction in car frame.

		// Visualization information.
		bool        needVisualization = true;
		std::string chassisFile = "";
		std::string wheelFile[4] = { "", "", "", "" };
		Vector3f    chassisMeshScale;//scale在这
		Vector3f    wheelMeshScale[4];
		Vector3f    chassisMeshTranslate;//translate在这
		Vector3f    wheelMeshTranslate[4];
		//质量和惯性系数
		float    chassisMass = 1.0;
		Vector3f chassisInertia;
		// adjust by HNU
		// C2397   
		float    wheelMass[4] = { 0.1f, 0.1f, 0.1f, 0.1f };
		Vector3f wheelInertia[4];
		//
		float wheelRadius[4] = { 1.0, 1.0, 1.0, 1.0 };
		
		float steeringLowerBound;
		float steeringUpperBound;

		Vector3f wheelLocalRight[2];  // wheel right direction in wheel local frame.

		Vector3f m_gravity = { 0, /*-9.8*/0, 0 };

		//std::shared_ptr<RigidBody2<DataType3f>> m_steeringRigid[2];

		std::shared_ptr<PBDSolver> m_rigidSolver;

		std::shared_ptr<RigidBody2<DataType3f>> m_chassis;

		std::shared_ptr<RigidBody2<DataType3f>> m_steeringRigid[4];
		std::shared_ptr<RigidBody2<DataType3f>> m_wheels[4];

		int chassisCollisionGroup = 1;
		int chassisCollisionMask = 1;
		int wheelCollisionGroup = 1;
		int wheelCollisionMask = 1;

		float forwardForceAcc=1000;
		float upForceAcc=1000;
		float rightForceAcc=1000;
		//float breakForceAcc;
		float steeringSpeed;

		float maxVel = 2.5;
		float maxAngVel = 25;
		
		float linearDamping = 0.9;
		float angularDamping = 0;
		
		float suspensionLength = 0.1;
		float suspensionStrength = 1000000;

		float currentSteering = 0;

	private:
		Vector3f forwardForcePoint;

		Vector3f forwardDir;
		Vector3f rightDir;
		Vector3f upDir;

		float    forwardForce = 0;//三个方向的力
		float    rightForce = 0;
		float    upForce = 0;

		Vector3f    forwardTorque = { 0,0,0 };//三个方向的力矩
		Vector3f    rightTorque = { 0,0,0 };
		Vector3f    upTorque = { 0,0,0 };

		Vector3f    restoreTorque = { 0,0,0 };

		Vector3f vertical= { 0,1,0 };

		float breakForce = 0;

		float EastWindForce = 0.0;
		float SouthWindForce = 0.0;
		float WestWindForce = 0.0;
		float NorthWindForce = 0.0;

		//do not set too many 

		int WindCount = 0;

		int anglecount = 0;

		bool m_accPressed = false;

		//float m_curSuspensionExt[0]
	};

}  // namespace PhysIKA