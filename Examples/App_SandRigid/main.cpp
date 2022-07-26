#include <iostream>
#include <memory>
#include <cuda.h>
#include <cuda_runtime_api.h>
#include <GL/glew.h>
#include <GL/freeglut.h>
#include "GUI/GlutGUI/GLApp.h"

#include "Dynamics/ParticleSystem/StaticBoundary.h"

#include "demoParticleSand.h"
#include "demoParticleSand2.h"
#include "demoSandRigid.h"
//#include "connector.h"
//#include "connector_use_demo.h"
#include "new.h"
#include "connector_craft.h"
#include "connector_craft_use_demo.h"
#include <iostream>
using namespace std;
using namespace PhysIKA;

int main()
{
    //DemoSSESand demo;
    //demo.createScene();
    //demo.run();

    std::cout << "Input to choose a simulation scene: " << std::endl;
    std::cout << "   0:  Car running on sand." << std::endl;
    std::cout << "   1:  Sand-rigid coupling." << std::endl;
	std::cout << "   2:  HeightFieldtext-wei" << std::endl;
	std::cout << "   4:  两辆车" << std::endl;
	std::cout << "   5:  connector" << std::endl;
	std::cout << "   11: 四旋翼无人机" << std::endl;
    int caseid = 0;
    std::cin >> caseid;

	

	/*const VPE::PhysIKACarCreateInfo c;
	c = ;*/

	double data[256 * 256];
	
	for (int i = 0; i < 256*255; i++) {
		//for (int j = 0; j < 256; j++) {
			data[i] = 0.01;
		//}
	}

	VPE::SandSimulationRegionCreateInfo b;
	b.total_width_in_meter = 10;
	b.total_height_in_meter = 10;
	b.sand_layer_thickness = 0.02;//
	b.time_delta = 0.016;
	b.height_data = &data[0];//高度场数组
	b.height_resolution_x = 256;
	b.height_resolution_y = 256;
	//把这个b.cars参数传进去
	//b.cars[0].type = FourWheel;
	//可以做一个循环，赋值vector
	VPE::PhysIKACarCreateInfo carobject;
	b.cars.push_back(carobject);
	b.cars[0].carPosition.x = -4;
	b.cars[0].carPosition.y = 0.5;
	b.cars[0].carPosition.z = 0;

	b.cars[0].chassis.translation.x = 0;
	b.cars[0].chassis.translation.y = 0;
	b.cars[0].chassis.translation.z = 0;
	b.cars[0].chassis.model_path = "../../Media/car2/chassis_cube.obj";
	b.cars[0].chassis.sdf_path = "../../Media/car2/chassis_cube.sdf";
	//轮子
	for (int i=0;i<4;i++){
	b.cars[0].wheels[i].model_path = "../../Media/car2/wheel.obj";
	b.cars[0].wheels[i].sdf_path = "../../Media/car2/wheel.sdf";
	}

	b.cars[0].wheels[0].translation.x = -0.3f;
	b.cars[0].wheels[0].translation.y = -0.2;
	b.cars[0].wheels[0].translation.z = -0.4f;

	b.cars[0].wheels[1].translation.x = +0.3f;
	b.cars[0].wheels[1].translation.y = -0.2;
	b.cars[0].wheels[1].translation.z = -0.4f;

	b.cars[0].wheels[2].translation.x = -0.3f;
	b.cars[0].wheels[2].translation.y = -0.2;
	b.cars[0].wheels[2].translation.z = 0.4f;

	b.cars[0].wheels[3].translation.x = +0.3f;
	b.cars[0].wheels[3].translation.y = -0.2;
	b.cars[0].wheels[3].translation.z = 0.4f;




    switch (caseid)
    {
        case 0: {
            DemoParticleSandMultiRigid* demo = DemoParticleSandMultiRigid::getInstance();
            demo->createScene();
            demo->run();
            break;
        }
        case 1: {//这里面球是旋转的，可以研究这个来让车轮旋转。
            DemoParticleSandRigid_Sphere* demo = DemoParticleSandRigid_Sphere::getInstance();
            demo->createScene();
            demo->run();
            break;
		}
		case 2: {//小车不稳，自动解体，问一下肖。经过之前的代码调整，不知咋的，就不解体了。可能因为PBDCar。
			//现在是没有车辙印，没有车辙初步猜测是耦合不明显，是时间步长不合适太小了。
            DemoHeightFieldSandLandMultiRigid2* demo = DemoHeightFieldSandLandMultiRigid2::getInstance();
            demo->createScene();
            demo->run();
            break;
        }
		case 3: {//cuda出现异常
            DemoHeightFieldSandLandMultiRigidTest* demo = DemoHeightFieldSandLandMultiRigidTest::getInstance();
            demo->createScene();
            demo->run();
            break;
        }
		case 4: {//二车//现在car2：穿模（看一下能撞的car类，和本项目中的car类，又什么区别）
            DemoParticleSandMultiRigid2* demo = DemoParticleSandMultiRigid2::getInstance();
            demo->createScene();
			std::printf("wkm%f %f %f %f", demo->m_car->wheelRelRotation[0], demo->m_car->wheelRelRotation[1], demo->m_car->wheelRelRotation[2], demo->m_car->wheelRelRotation[3]);//始终是0000
			//底盘角度也输出看看
			std::printf("wkm%f %f %f %f", demo->m_car->carRotation[0], demo->m_car->carRotation[1], demo->m_car->carRotation[2], demo->m_car->carRotation[3]);//始终是0001
			demo->run();
            break;
        }

		
		//case 5: {//connector粒子法
		//	connector_use_demo* demo = connector_use_demo::getInstance();//mei delate//好好看看上面那四个是怎么调用的
		//	demo->createScene(b);
		//	demo->run();
		//	//接口上层写个样例，继承GLApp，这个样例来调用桥接接口。
		//	break;
		//}

		case 6: {//高度场方法，两辆车，有问题不稳定还慢得要死
			//慢的话估计调一下时间步会好。
			new1* demo = new1::getInstance();
			demo->createScene();
			demo->run();
			break;
		}
		//以这个为基础，做一个泥地的！
		case 7: {//高度场，小球在跑道上自由滚
			DemoHeightFieldSandLandRigid* demo = DemoHeightFieldSandLandRigid::getInstance();
			demo->createScene();
			demo->run();
			break;
		}
		case 8: {//小球&方块掉落
			DemoHeightFieldSandRigid_Sphere* demo = DemoHeightFieldSandRigid_Sphere::getInstance();
			demo->createScene();
			demo->run();
			break;
		}

		case 9: {//高度场，是小球在跑道上自由滚，现在沙子改泥地！用这个。现在没把mu导进去用上。
			DemoHeightFieldMudLandRigid* demo = DemoHeightFieldMudLandRigid::getInstance();
			demo->createScene();
			demo->run();
			break;
		}

		case 10: {//脱胎于case9，做个新的demo用车跑跑试试。车轮改成整车！
			DemoHeightFieldMudLandCar* demo = DemoHeightFieldMudLandCar::getInstance();
			demo->createScene();
			demo->run();
			break;
		}
		case 11: {//四旋翼无人机。车改成飞机！调用PBDCraft文件
			DemoHeightFieldcraft* demo = DemoHeightFieldcraft::getInstance();
			demo->createScene();
			demo->run();
			break;
		}
		//slippery
		case 12: {//湿滑地
			DemoHeightFieldSlippery* demo = DemoHeightFieldSlippery::getInstance();
			demo->createScene();
			demo->run();
			break;
		}
		//case 13: {//四旋翼无人机的接口
		//	connector_craft_use_demo* demo = connector_craft_use_demo::getInstance();
		//	demo->createScene(b);
		//	demo->run();
		//	break;
		//}


        default:
            break;
    }

    //DemoParticleSand* demo = DemoParticleSand::getInstance();
    //demo->createScene();
    //demo->run();

    //DemoParticleSandRigid_Sphere* demo = DemoParticleSandRigid_Sphere::getInstance();
    //demo->createScene();
    //demo->run();

    //DemoParticleSandSlop* demo = DemoParticleSandSlop::getInstance();
    //demo->createScene();
    //demo->run();

    //DemoParticleSandSlide* demo = DemoParticleSandSlide::getInstance();
    //demo->createScene();
    //demo->run();

    //DemoParticleSandSlide2* demo = DemoParticleSandSlide2::getInstance();
    //demo->createScene();
    //demo->run();

    //DemoParticleSandLand* demo = DemoParticleSandLand::getInstance();
    //demo->createScene();
    //demo->run();

    //DemoParticleSandLand2* demo = DemoParticleSandLand2::getInstance();
    //demo->createScene();
    //demo->run();

    //DemoParticleSandMultiRigid* demo = DemoParticleSandMultiRigid::getInstance();
    //demo->createScene();
    //demo->run();

    //DemoParticleSandPile* demo = DemoParticleSandPile::getInstance();
    //demo->createScene();
    //demo->run();

    //DemoParticleAvalanche* demo = DemoParticleAvalanche::getInstance();
    //demo->createScene();
    //demo->run();

    //DemoParticleRiver* demo = DemoParticleRiver::getInstance();
    //demo->createScene();
    //demo->run();

    // **********************

    //DemoHeightFieldSand* demo = DemoHeightFieldSand::getInstance();
    //demo->createScene();
    //demo->run();

    //DemoHeightFieldSandRigid_Sphere* demo = DemoHeightFieldSandRigid_Sphere::getInstance();
    //demo->createScene();
    //demo->run();

    //DemoHeightFieldSandLandRigid* demo = DemoHeightFieldSandLandRigid::getInstance();
    //demo->createScene();
    //demo->run();

    //DemoHeightFieldSandSlide* demo = DemoHeightFieldSandSlide::getInstance();
    //demo->createScene();
    //demo->run();

    //DemoHeightFieldSandLandMultiRigid* demo = DemoHeightFieldSandLandMultiRigid::getInstance();
    //demo->createScene();
    //demo->run();

    //DemoHeightFieldSandLandMultiRigid2* demo = DemoHeightFieldSandLandMultiRigid2::getInstance();
    //demo->createScene();
    //demo->run();

    //DemoHeightFieldSandLandMultiRigidTest* demo = DemoHeightFieldSandLandMultiRigidTest::getInstance();
    //demo->createScene();
    //demo->run();

    //DemoHeightFieldSandValley* demo = DemoHeightFieldSandValley::getInstance();
    //demo->createScene();
    //demo->run();

    return 0;
}
