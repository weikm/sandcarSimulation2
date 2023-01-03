#pragma once

#include "GUI/GlutGUI/GLApp.h"
#include "Dynamics/RigidBody/RigidBody2.h"
#include "Rendering/RigidMeshRender.h"

#include "Dynamics/RigidBody/Vehicle/PBDCar.h"
#include "Dynamics/RigidBody/Vehicle/PBDCraft.h"
#include "Dynamics/RigidBody/Vehicle/PBDFixedWingUAVs.h"

#include "Dynamics/Sand/SSESandSolver.h"
#include "Dynamics/Sand/SandVisualPointSampleModule.h"


using namespace PhysIKA;

//
//
//void PkAddBoundaryRigid(std::shared_ptr<Node> root, Vector3f origin, float sizex, float sizez,
//    float boundarysize, float boundaryheight);

class DemoHeightFieldSand : public GLApp
{

private:
    DemoHeightFieldSand()
    {
        setKeyboardFunction(DemoHeightFieldSand::demoKeyboardFunction);
        createWindow(1024, 768);
    }
    static DemoHeightFieldSand* m_instance;

public:
    static DemoHeightFieldSand* getInstance()
    {
        if (m_instance == 0)
            m_instance = new DemoHeightFieldSand;
        return m_instance;
    }

    void createScene();

    void run()
    {
        Log::setOutput("console_log.txt");
        Log::setLevel(Log::Info);
        Log::sendMessage(Log::Info, "Simulation begin");

        mainLoop();

        Log::sendMessage(Log::Info, "Simulation end!");
    }

    static void demoKeyboardFunction(unsigned char key, int x, int y)
    {

        {
            if (!m_instance)
                return;
            switch (key)
            {
                case 'v':
                    m_instance->_changeVisibility();

                    break;
                default:
                    GLApp::keyboardFunction(key, x, y);
                    break;
            }
        }
    }

private:
    void _changeVisibility()
    {
        if (!m_rigidVisible)
        {
            m_sampler->m_sandHeight = &(m_sandsolver->getSandGrid().m_sandHeight);
        }
        else
        {
            m_sampler->m_sandHeight = &(m_sandsolver->m_sandStaticHeight);

            //m_sampler->m_sandHeight = &(m_sandsolver->getSandGrid().m_landHeight);// &(m_sandsolver->m_sandStaticHeight);
        }
        m_rigidVisible = !m_rigidVisible;
    }

public:
    std::shared_ptr<SSESandSolver>                   m_sandsolver;
    std::shared_ptr<SandHeightRenderParticleSampler> m_sampler;

    //std::vector<RigidBody2_ptr> m_rigids;
    //std::vector<std::shared_ptr<RigidMeshRender>> m_rigidRenders;

    bool m_rigidVisible = true;
};

class DemoHeightFieldSandRigid_Sphere : public GLApp
{

private:
    DemoHeightFieldSandRigid_Sphere()
    {
        setKeyboardFunction(DemoHeightFieldSandRigid_Sphere::demoKeyboardFunction);
        createWindow(1024, 768);
    }
    static DemoHeightFieldSandRigid_Sphere* m_instance;

public:
    static DemoHeightFieldSandRigid_Sphere* getInstance()
    {
        if (m_instance == 0)
            m_instance = new DemoHeightFieldSandRigid_Sphere;
        return m_instance;
    }

    void createScene();

    void run()
    {
        Log::setOutput("console_log.txt");
        Log::setLevel(Log::Info);
        Log::sendMessage(Log::Info, "Simulation begin");

        mainLoop();

        Log::sendMessage(Log::Info, "Simulation end!");
    }

    static void demoKeyboardFunction(unsigned char key, int x, int y)
    {

        {
            if (!m_instance)
                return;
            switch (key)
            {
                case 'v':
                    m_instance->_changeVisibility();

                    break;
                default:
                    GLApp::keyboardFunction(key, x, y);
                    break;
            }
        }
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

public:
    std::vector<RigidBody2_ptr>                   m_rigids;
    std::vector<std::shared_ptr<RigidMeshRender>> m_rigidRenders;

    bool m_rigidVisible = true;
};

class DemoHeightFieldSandLandRigid : public GLApp
{

private:
    DemoHeightFieldSandLandRigid()
    {
        setKeyboardFunction(DemoHeightFieldSandLandRigid::demoKeyboardFunction);
        createWindow(1024, 400);
    }
    static DemoHeightFieldSandLandRigid* m_instance;

public:
    static DemoHeightFieldSandLandRigid* getInstance()
    {
        if (m_instance == 0)
            m_instance = new DemoHeightFieldSandLandRigid;
        return m_instance;
    }

    void createScene();

    void run()
    {
        Log::setOutput("console_log.txt");
        Log::setLevel(Log::Info);
        Log::sendMessage(Log::Info, "Simulation begin");

        mainLoop();

        Log::sendMessage(Log::Info, "Simulation end!");
    }

    static void demoKeyboardFunction(unsigned char key, int x, int y)
    {

        {
            if (!m_instance)
                return;
            switch (key)
            {
                case 'v':
                    m_instance->_changeVisibility();

                    break;
                default:
                    GLApp::keyboardFunction(key, x, y);
                    break;
            }
        }
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

public:
    std::vector<RigidBody2_ptr>                   m_rigids;
    std::vector<std::shared_ptr<RigidMeshRender>> m_rigidRenders;

    bool m_rigidVisible = true;
};

class DemoHeightFieldSandSlide : public GLApp
{

private:
    DemoHeightFieldSandSlide()
    {
        setKeyboardFunction(DemoHeightFieldSandSlide::demoKeyboardFunction);
        createWindow(1024, 768);
    }
    static DemoHeightFieldSandSlide* m_instance;

public:
    static DemoHeightFieldSandSlide* getInstance()
    {
        if (m_instance == 0)
            m_instance = new DemoHeightFieldSandSlide;
        return m_instance;
    }

    void createScene();

    void run()
    {
        Log::setOutput("console_log.txt");
        Log::setLevel(Log::Info);
        Log::sendMessage(Log::Info, "Simulation begin");

        mainLoop();

        Log::sendMessage(Log::Info, "Simulation end!");
    }

    static void demoKeyboardFunction(unsigned char key, int x, int y)
    {

        {
            if (!m_instance)
                return;
            switch (key)
            {
                case 'v':
                    m_instance->_changeVisibility();

                    break;
                default:
                    GLApp::keyboardFunction(key, x, y);
                    break;
            }
        }
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

public:
    std::vector<RigidBody2_ptr>                   m_rigids;
    std::vector<std::shared_ptr<RigidMeshRender>> m_rigidRenders;

    bool m_rigidVisible = true;
};

class DemoHeightFieldSandLandMultiRigid : public GLApp
{

private:
    DemoHeightFieldSandLandMultiRigid()
    {
        setKeyboardFunction(DemoHeightFieldSandLandMultiRigid::demoKeyboardFunction);
        createWindow(1024, 768);
    }
    static DemoHeightFieldSandLandMultiRigid* m_instance;

public:
    static DemoHeightFieldSandLandMultiRigid* getInstance()
    {
        if (m_instance == 0)
            m_instance = new DemoHeightFieldSandLandMultiRigid;
        return m_instance;
    }

    void createScene();

    void run()
    {
        Log::setOutput("console_log.txt");
        Log::setLevel(Log::Info);
        Log::sendMessage(Log::Info, "Simulation begin");

        mainLoop();

        Log::sendMessage(Log::Info, "Simulation end!");
    }

    static void demoKeyboardFunction(unsigned char key, int x, int y)
    {

        if (!m_instance)
            return;
        switch (key)
        {
            case 'a':
                m_instance->m_car->goLeft(0.016);
                break;
            case 'd':
                m_instance->m_car->goRight(0.016);
                break;
            case 'w':
                m_instance->m_car->forward(0.016);
                break;
            case 's':
                m_instance->m_car->backward(0.016);
                break;
            case 'v':
                m_instance->_changeVisibility();

                break;
            default:
                GLApp::keyboardFunction(key, x, y);
                break;
        }
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

public:
    std::vector<RigidBody2_ptr>                   m_rigids;
    std::vector<std::shared_ptr<RigidMeshRender>> m_rigidRenders;

    std::shared_ptr<PBDCar> m_car;

    bool m_rigidVisible = true;
};

class DemoHeightFieldSandLandMultiRigid2 : public GLApp
{

private:
    DemoHeightFieldSandLandMultiRigid2()
    {
        setKeyboardFunction(DemoHeightFieldSandLandMultiRigid2::demoKeyboardFunction);
        createWindow(1024, 768);
    }
    static DemoHeightFieldSandLandMultiRigid2* m_instance;

public:
    static DemoHeightFieldSandLandMultiRigid2* getInstance()
    {
        if (m_instance == 0)
            m_instance = new DemoHeightFieldSandLandMultiRigid2;
        return m_instance;
    }

    void createScene();

    void run()
    {
        Log::setOutput("console_log.txt");
        Log::setLevel(Log::Info);
        Log::sendMessage(Log::Info, "Simulation begin");

        mainLoop();

        Log::sendMessage(Log::Info, "Simulation end!");
    }

    static void demoKeyboardFunction(unsigned char key, int x, int y)
    {

        if (!m_instance)
            return;
        switch (key)
        {
            case 'a':
                m_instance->m_car->goLeft(0.016);
                break;
            case 'd':
                m_instance->m_car->goRight(0.016);
                break;
            case 'w':
                m_instance->m_car->forward(0.016);
                break;
            case 's':
                m_instance->m_car->backward(0.016);
                break;
            case 'v':
                m_instance->_changeVisibility();

                break;

            case 'o':
                m_instance->_setSandHeightTo(0.1);
                break;
            default:
                GLApp::keyboardFunction(key, x, y);
                break;
        }
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

    void _setSandHeightTo(float h);

public:
    std::vector<RigidBody2_ptr>                   m_rigids;
    std::vector<std::shared_ptr<RigidMeshRender>> m_rigidRenders;

    std::shared_ptr<PBDCar> m_car;

    std::shared_ptr<SSESandSolver> m_psandsolver;

    bool m_rigidVisible = true;
};

class DemoHeightFieldSandLandMultiRigidTest : public GLApp
{

private:
    DemoHeightFieldSandLandMultiRigidTest()
    {
        setKeyboardFunction(DemoHeightFieldSandLandMultiRigidTest::demoKeyboardFunction);
        createWindow(1024, 768);
    }
    static DemoHeightFieldSandLandMultiRigidTest* m_instance;

public:
    static DemoHeightFieldSandLandMultiRigidTest* getInstance()
    {
        if (m_instance == 0)
            m_instance = new DemoHeightFieldSandLandMultiRigidTest;
        return m_instance;
    }

    void createScene();

    void run()
    {
        Log::setOutput("console_log.txt");
        Log::setLevel(Log::Info);
        Log::sendMessage(Log::Info, "Simulation begin");

        mainLoop();

        Log::sendMessage(Log::Info, "Simulation end!");
    }

    static void demoKeyboardFunction(unsigned char key, int x, int y)
    {

        {
            if (!m_instance)
                return;
            switch (key)
            {
                case 'v':
                    m_instance->_changeVisibility();

                    break;
                default:
                    GLApp::keyboardFunction(key, x, y);
                    break;
            }
        }
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

public:
    std::vector<RigidBody2_ptr>                   m_rigids;
    std::vector<std::shared_ptr<RigidMeshRender>> m_rigidRenders;

    //std::shared_ptr<PBDCar> m_car;

    bool m_rigidVisible = true;
};
//山体滑坡
class DemoHeightFieldSandValley : public GLApp
{

private:
    DemoHeightFieldSandValley()
    {
        setKeyboardFunction(DemoHeightFieldSandValley::demoKeyboardFunction);
        createWindow(1024, 768);
    }
    static DemoHeightFieldSandValley* m_instance;

public:
    static DemoHeightFieldSandValley* getInstance()
    {
        if (m_instance == 0)
            m_instance = new DemoHeightFieldSandValley;
        return m_instance;
    }

    void createScene();

    void run()
    {
        Log::setOutput("console_log.txt");
        Log::setLevel(Log::Info);
        Log::sendMessage(Log::Info, "Simulation begin");

        mainLoop();

        Log::sendMessage(Log::Info, "Simulation end!");
    }

    static void demoKeyboardFunction(unsigned char key, int x, int y)
    {

        if (!m_instance)
            return;
        switch (key)
        {

            case 'o':
                m_instance->_setSandHeightTo(0.1);
                break;
            default:
                GLApp::keyboardFunction(key, x, y);
                break;
        }
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

    void _setSandHeightTo(float h);

public:
    std::vector<RigidBody2_ptr>                   m_rigids;
    std::vector<std::shared_ptr<RigidMeshRender>> m_rigidRenders;

    std::shared_ptr<PBDCar> m_car;

    std::shared_ptr<SSESandSolver> m_psandsolver;

    bool m_rigidVisible = true;
};

class DemoHeightFieldMudLandRigid : public GLApp
{

private:
	DemoHeightFieldMudLandRigid()
	{
		setKeyboardFunction(DemoHeightFieldMudLandRigid::demoKeyboardFunction);
		createWindow(1024, 400);
	}
	static DemoHeightFieldMudLandRigid* m_instance;

public:
	static DemoHeightFieldMudLandRigid* getInstance()
	{
		if (m_instance == 0)
			m_instance = new DemoHeightFieldMudLandRigid;
		return m_instance;
	}

	void createScene();

	void run()
	{
		Log::setOutput("console_log.txt");
		Log::setLevel(Log::Info);
		Log::sendMessage(Log::Info, "Simulation begin");

		mainLoop();

		Log::sendMessage(Log::Info, "Simulation end!");
	}

	static void demoKeyboardFunction(unsigned char key, int x, int y)
	{

		{
			if (!m_instance)
				return;
			switch (key)
			{
			case 'v':
				m_instance->_changeVisibility();

				break;
			default:
				GLApp::keyboardFunction(key, x, y);
				break;
			}
		}
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

public:
	std::vector<RigidBody2_ptr>                   m_rigids;
	std::vector<std::shared_ptr<RigidMeshRender>> m_rigidRenders;

	bool m_rigidVisible = true;



	

};

class DemoHeightFieldMudLandCar : public GLApp
{

private:
	DemoHeightFieldMudLandCar()
	{
		setKeyboardFunction(DemoHeightFieldMudLandCar::demoKeyboardFunction);
		createWindow(1024, 400);
	}
	static DemoHeightFieldMudLandCar* m_instance;

public:
	static DemoHeightFieldMudLandCar* getInstance()
	{
		if (m_instance == 0)
			m_instance = new DemoHeightFieldMudLandCar;
		return m_instance;
	}

	void createScene();

	void run()
	{
		Log::setOutput("console_log.txt");
		Log::setLevel(Log::Info);
		Log::sendMessage(Log::Info, "Simulation begin");

		mainLoop();

		Log::sendMessage(Log::Info, "Simulation end!");
	}

	static void demoKeyboardFunction(unsigned char key, int x, int y)
	{

		{
			
			if (!m_instance)
				return;
			switch (key)
			{
			case 'a':
				m_instance->m_car->goLeft(0.016);//这个使小车左打轮
				break;
			case 'd':
				m_instance->m_car->goRight(0.016);
				break;
			case 'w':
				m_instance->m_car->forward(0.016);//这个赋予小车前向牵引
				break;
			case 's':
				m_instance->m_car->backward(0.016);
				break;
			case 'v':
				m_instance->_changeVisibility();

				break;
			default:
				GLApp::keyboardFunction(key, x, y);
				break;
			}
		}
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

public:
	std::vector<RigidBody2_ptr>                   m_rigids;
	std::vector<std::shared_ptr<RigidMeshRender>> m_rigidRenders;

	bool m_rigidVisible = true;
	

	std::shared_ptr<PBDCar>                       m_car;
	
	
};

class DemoHeightFieldcraft : public GLApp
{

private:
	DemoHeightFieldcraft()
	{
		setKeyboardFunction(DemoHeightFieldcraft::demoKeyboardFunction);
		createWindow(1024, 400);
	}
	static DemoHeightFieldcraft* m_instance;

public:
	static DemoHeightFieldcraft* getInstance()
	{
		if (m_instance == 0)
			m_instance = new DemoHeightFieldcraft;
		return m_instance;
	}

	void createScene();

	void run()
	{
		Log::setOutput("console_log.txt");
		Log::setLevel(Log::Info);
		Log::sendMessage(Log::Info, "Simulation begin");

		mainLoop();

		Log::sendMessage(Log::Info, "Simulation end!");
	}

	static void demoKeyboardFunction(unsigned char key, int x, int y)
	{

		{

			if (!m_instance)
				return;
			switch (key)
			{
			case 'a':
				m_instance->m_car->goLeft(0.016);//向左
				break;
			case 'd':
				m_instance->m_car->goRight(0.016);
				break;
			case 'w':
				m_instance->m_car->forward(0.016);//向前
				break;
			case 's':
				m_instance->m_car->backward(0.016);
				break;
			case 'q':
				m_instance->m_car->goUp(0.016);//向上
				break;
			case 'e':
				m_instance->m_car->goDown(0.016);
				break;
			case 'r':
				m_instance->m_car->zizhuan();
				break;
			case 'b':
				m_instance->m_car->brake();
				break;
			case 'v':
				m_instance->_changeVisibility();

			case 'i':
				m_instance->m_car->NorthWind();
			case 'h':
				m_instance->m_car->WestWind();
			case 'n':
				m_instance->m_car->SouthWind();
			case 'k':
				m_instance->m_car->EastWind();

				break;
			default:
				GLApp::keyboardFunction(key, x, y);
				break;
			}
		}
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

public:
	std::vector<RigidBody2_ptr>                   m_rigids;
	std::vector<std::shared_ptr<RigidMeshRender>> m_rigidRenders;

	bool m_rigidVisible = true;


	std::shared_ptr<PBDCraft>                       m_car;
};

class DemoHeightFieldSlippery: public GLApp
{

private:
	DemoHeightFieldSlippery()
	{
		setKeyboardFunction(DemoHeightFieldSlippery::demoKeyboardFunction);
		createWindow(1024, 400);
	}
	static DemoHeightFieldSlippery* m_instance;

public:
	static DemoHeightFieldSlippery* getInstance()
	{
		if (m_instance == 0)
			m_instance = new DemoHeightFieldSlippery;
		return m_instance;
	}

	void createScene();

	void run()
	{
		Log::setOutput("console_log.txt");
		Log::setLevel(Log::Info);
		Log::sendMessage(Log::Info, "Simulation begin");

		mainLoop();

		Log::sendMessage(Log::Info, "Simulation end!");
	}

	static void demoKeyboardFunction(unsigned char key, int x, int y)
	{

		{

			if (!m_instance)
				return;
			switch (key)
			{
			case 'a':
				m_instance->m_car->goLeft(0.016);//这个使小车左打轮
				break;
			case 'd':
				m_instance->m_car->goRight(0.016);
				break;
			case 'w':
				m_instance->m_car->forward(0.016);//这个赋予小车前向牵引
				break;
			case 's':
				m_instance->m_car->backward(0.016);
				break;
			case 'v':
				m_instance->_changeVisibility();

				break;
			default:
				GLApp::keyboardFunction(key, x, y);
				break;
			}
		}
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

public:
	std::vector<RigidBody2_ptr>                   m_rigids;
	std::vector<std::shared_ptr<RigidMeshRender>> m_rigidRenders;

	bool m_rigidVisible = true;


	std::shared_ptr<PBDCar>                       m_car;


};

class DemoHeightFieldFixedWingUAVs : public GLApp
{

private:
    DemoHeightFieldFixedWingUAVs()
    {
        setKeyboardFunction(DemoHeightFieldFixedWingUAVs::demoKeyboardFunction);
        createWindow(1024, 400);
    }
    static DemoHeightFieldFixedWingUAVs* m_instance;

public:
    static DemoHeightFieldFixedWingUAVs* getInstance()
    {
        if (m_instance == 0)
            m_instance = new DemoHeightFieldFixedWingUAVs;
        return m_instance;
    }

    void createScene();

    void run()
    {
        Log::setOutput("console_log.txt");
        Log::setLevel(Log::Info);
        Log::sendMessage(Log::Info, "Simulation begin");

        mainLoop();

        Log::sendMessage(Log::Info, "Simulation end!");
    }

    static void demoKeyboardFunction(unsigned char key, int x, int y)
    {
		
        {

            if (!m_instance)
                return;
            switch (key)
            {
                case 'a':
                    m_instance->m_car->goLeft(0.016); 
                    break;
                case 'd':
                    m_instance->m_car->goRight(0.016);
                    break;
                case 'w':
                    m_instance->m_car->Accelerate(0.016);  
                    break;
                case 's':
                    m_instance->m_car->Decelerate(0.016);
                    break;
                case 'q':
                    m_instance->m_car->goUp(0.016);  
                    break;
                case 'e':
                    m_instance->m_car->goDown(0.016);
                    break;
               /* case 'r':
                    m_instance->m_car->zizhuan(0.016);
                    break;*/
                case 'b':
                    m_instance->m_car->brake();
                    break;
                case 'v':
                    m_instance->_changeVisibility();

                    break;
                default:
                    GLApp::keyboardFunction(key, x, y);
                    break;
            }
        }
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

public:
    std::vector<RigidBody2_ptr>                   m_rigids;
    std::vector<std::shared_ptr<RigidMeshRender>> m_rigidRenders;

    bool m_rigidVisible = true;

    std::shared_ptr<PBDFixedWingUAVs> m_car;
};