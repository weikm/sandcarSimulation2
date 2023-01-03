#pragma once

#include "GUI/GlutGUI/GLApp.h"
#include "Framework/Framework/Node.h"

#include "Dynamics/RigidBody/RigidBody2.h"
#include "Dynamics/RigidBody/PBDRigid/PBDSolver.h"



//using namespace PhysIKA;

namespace PhysIKA {
class PBDCar : public Node
{
public:
    PBDCar() {}

    bool build();

    //virtual bool initialize() override;

    virtual void advance(Real dt);

    void forward(Real dt);

    void backward(Real dt);

    void goLeft(Real dt);

    void goRight(Real dt);

	void brake(Real dt);

	void PBDCar::computeAABB(std::shared_ptr<PointSet<DataType3f>> points, Vector3f& center, Vector3f& halfSize);
    //void setDt(Real dt);

    std::shared_ptr<RigidBody2<DataType3f>> getChassis()
    {
        return m_chassis;
    }
    std::shared_ptr<RigidBody2<DataType3f>> getWheels(int i)
    {
        return m_wheels[i];
    }

	void set_SlipperyRate(float rate);
	void set_linearVelocityDamping(float rate);

    void updateForce(Real dt);

private:
    void _updateWheelRotation(Real dt);

    Vector3f _getForwardDir();

    Vector3f _getRightDir();

    Vector3f _getUpDir();

    void _setRigidForceAsGravity();

    void _doVelConstraint(Real dt);

    Quaternionf _rotationToStandardLocal();

public:
    Vector3f          carPosition;//车子（质点）位置
    Quaternion<float> carRotation;//车子角度

    Vector3f          wheelRelPosition[4];//轮子相对位置
    Quaternion<float> wheelRelRotation[4];//轮子相对角度//车轮旋转！

    Vector3f wheelupDirection;//轮子在z轴上与底盘的相对位置
    Vector3f wheelRightDirection;  // wheel right direction in car frame.

    // Visualization information.
    bool        needVisualization = true;
    std::string chassisFile       = "";
    std::string wheelFile[4]      = { "", "", "", "" };
    Vector3f    chassisMeshScale;//scale在这
    Vector3f    wheelMeshScale[4];
    Vector3f    chassisMeshTranslate;//translate在这
    Vector3f    wheelMeshTranslate[4];
	//质量和惯性系数
    float    chassisMass = 1.0;
    Vector3f chassisInertia;
    // adjust by HNU
    // C2397    从“double”转换到“float”需要收缩转换
    float    wheelMass[4] = { 0.1f, 0.1f, 0.1f, 0.1f };
    Vector3f wheelInertia[4];
	//
    float wheelRadius[4] = { 1.0, 1.0, 1.0, 1.0 };
	//打轮的左右边界
    float steeringLowerBound;
    float steeringUpperBound;

    Vector3f wheelLocalRight[2];  // wheel right direction in wheel local frame.

    Vector3f m_gravity = { 0, -9.8, 0 };//重力加速度

    //std::shared_ptr<RigidBody2<DataType3f>> m_steeringRigid[2];

    std::shared_ptr<PBDSolver> m_rigidSolver;

    std::shared_ptr<RigidBody2<DataType3f>> m_chassis;

    std::shared_ptr<RigidBody2<DataType3f>> m_steeringRigid[4];
    std::shared_ptr<RigidBody2<DataType3f>> m_wheels[4];

    int chassisCollisionGroup = 1;
    int chassisCollisionMask  = 1;
    int wheelCollisionGroup   = 1;
    int wheelCollisionMask    = 1;

    float forwardForceAcc;//前向加速度，//前向牵引力增加量forwardForceAcc在demoparticlesand里面设置的是1000
    //float breakForceAcc;
    float steeringSpeed;//打舵速度

	float SlipperyRate=1.0;
	float maxForce = 10000;

    float maxVel = 2.5;
	//长度和角度衰减
    float linearDamping  = 0.9;//原来是0。这个阻尼控制的是加速的速度。约接近1，加速越慢。
    float angularDamping = 0;
	//线速度的衰减系数
	float linearVelocityDamping  = 0.99;
	//悬架长度和强度
    float suspensionLength   = 0.1;//原来是0.05
    float suspensionStrength = 1000000;

	float currentSteering = 0;//这个被我拿到public里了 wkm

private:
    Vector3f forwardForcePoint;
    Vector3f forwardDir;
    float    forwardForce = 0;

    float breakForce = 0;

    

    bool m_accPressed = false;

    //float m_curSuspensionExt[0]
};

}  // namespace PhysIKA