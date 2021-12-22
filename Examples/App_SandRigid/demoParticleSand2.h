

#include "GUI/GlutGUI/GLApp.h"
#include "Dynamics/RigidBody/RigidBody2.h"
#include "Rendering/RigidMeshRender.h"

#include "Dynamics/RigidBody/Vehicle/PBDCar.h"

#include "demoCallbacks.h"
#include "Dynamics/RigidBody/PBDRigid/HeightFieldPBDInteractionNode.h"

using namespace PhysIKA;

class DemoParticleSandMultiRigid2 : public GLApp
{

private:
    DemoParticleSandMultiRigid2()//这个与类同名的函数，叫//构造函数//，是用系统自动调用的，调用了键盘操作函数
    {
        setKeyboardFunction(DemoParticleSandMultiRigid2::demoKeyboardFunction);
        createWindow(1024, 768);
    }
    static DemoParticleSandMultiRigid2* m_instance;//当前类（塔尖类）的静态指针对象

public:
    static DemoParticleSandMultiRigid2* getInstance()
    {
        if (m_instance == 0)
            m_instance = new DemoParticleSandMultiRigid2;
        return m_instance;
    }

    void createScene(/*double *data,int sandinfo_length,int sandinfo_wide*/);

    void run()
    {
        Log::setOutput("console_log.txt");//log日志
        Log::setLevel(Log::Info);
        Log::sendMessage(Log::Info, "Simulation begin");

        mainLoop();

        Log::sendMessage(Log::Info, "Simulation end!");
    }
	//下面是键盘操作，是个静态函数
    static void demoKeyboardFunction(unsigned char key, int x, int y)
    {
		//应该是时刻调用这个函数，所以几乎所有时候都default，调用了takeoneframe
        if (!m_instance)
            return;
        switch (key)
        {
            case 'a':
                m_instance->m_car->goLeft(0.016);//这个使小车左打轮
				m_instance->m_car2->goLeft(0.016);
                break;
            case 'd':
                m_instance->m_car->goRight(0.016);
				m_instance->m_car2->goRight(0.016);
                break;
            case 'w':
                m_instance->m_car->forward(0.016);//这个赋予小车前向牵引
				//验证得wheelRelRotation恒为0000.
				std::printf("wkm%f %f %f %f", m_instance->m_car->wheelRelRotation[0], m_instance->m_car->wheelRelRotation[1], m_instance->m_car->wheelRelRotation[2], m_instance->m_car->wheelRelRotation[3]);
				std::printf("wkm%f %f %f %f", m_instance->m_car->carRotation[0], m_instance->m_car->carRotation[1], m_instance->m_car->carRotation[2], m_instance->m_car->carRotation[3]);
				//m_instance->m_car2->forward(0.016);
                break;
            case 's':
                m_instance->m_car->backward(0.016);
				//m_instance->m_car2->backward(0.016);
                break;
            case 'v':
                m_instance->_changeVisibility();

                break;
            default:
                GLApp::keyboardFunction(key, x, y);//这里是除了方向键之外其他的按键操作//这里还有takeOneFrame,是时刻执行的吗？
                break;
        }
    }
	void computeAABB(std::shared_ptr<PointSet<DataType3f>> points, Vector3f& center, Vector3f& halfSize)
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
        pmin[0]       = min(pmin[0], curp[0]);
        pmin[1]       = min(pmin[1], curp[1]);
        pmin[2]       = min(pmin[2], curp[2]);
        pmax[0]       = max(pmax[0], curp[0]);
        pmax[1]       = max(pmax[1], curp[1]);
        pmax[2]       = max(pmax[2], curp[2]);
    }

    center   = (pmin + pmax) * 0.5;
    halfSize = (pmax - pmin) * 0.5;
	}


private:
    void _changeVisibility()
    {
        if (m_rigidVisible)
        {
            for (int i = 0; i < m_rigids.size(); ++i)
            {
                m_rigids[i]->deleteVisualModule(m_rigidRenders[i]);
            }
        }
        else
        {
            for (int i = 0; i < m_rigids.size(); ++i)
            {
                m_rigids[i]->addVisualModule(m_rigidRenders[i]);
            }
        }
        m_rigidVisible = !m_rigidVisible;
    }
//	std::shared_ptr<HeightFieldPBDInteractionNode> m_groundRigidInteractor;
	std::shared_ptr<HeightFieldPBDInteractionNode> m_groundRigidInteractor;//会不会是这里错了，应该是public？

public:
    std::shared_ptr<PBDCar>                       m_car;
	std::shared_ptr<PBDCar>                       m_car2;
    std::vector<RigidBody2_ptr>                   m_rigids;
    std::vector<std::shared_ptr<RigidMeshRender>> m_rigidRenders;

	

    bool m_rigidVisible = true;
};


