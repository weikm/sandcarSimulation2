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
	std::cout << "   4:  ������" << std::endl;
	std::cout << "   5:  connector" << std::endl;
	std::cout << "   11: ���������˻�" << std::endl;
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
	b.height_data = &data[0];//�߶ȳ�����
	b.height_resolution_x = 256;
	b.height_resolution_y = 256;
	//�����b.cars��������ȥ
	//b.cars[0].type = FourWheel;
	//������һ��ѭ������ֵvector
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
	//����
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
        case 1: {//������������ת�ģ������о�������ó�����ת��
            DemoParticleSandRigid_Sphere* demo = DemoParticleSandRigid_Sphere::getInstance();
            demo->createScene();
            demo->run();
            break;
		}
		case 2: {//С�����ȣ��Զ����壬��һ��Ф������֮ǰ�Ĵ����������֪զ�ģ��Ͳ������ˡ�������ΪPBDCar��
			//������û�г���ӡ��û�г��޳����²�����ϲ����ԣ���ʱ�䲽��������̫С�ˡ�
            DemoHeightFieldSandLandMultiRigid2* demo = DemoHeightFieldSandLandMultiRigid2::getInstance();
            demo->createScene();
            demo->run();
            break;
        }
		case 3: {//cuda�����쳣
            DemoHeightFieldSandLandMultiRigidTest* demo = DemoHeightFieldSandLandMultiRigidTest::getInstance();
            demo->createScene();
            demo->run();
            break;
        }
		case 4: {//����//����car2����ģ����һ����ײ��car�࣬�ͱ���Ŀ�е�car�࣬��ʲô����
            DemoParticleSandMultiRigid2* demo = DemoParticleSandMultiRigid2::getInstance();
            demo->createScene();
			std::printf("wkm%f %f %f %f", demo->m_car->wheelRelRotation[0], demo->m_car->wheelRelRotation[1], demo->m_car->wheelRelRotation[2], demo->m_car->wheelRelRotation[3]);//ʼ����0000
			//���̽Ƕ�Ҳ�������
			std::printf("wkm%f %f %f %f", demo->m_car->carRotation[0], demo->m_car->carRotation[1], demo->m_car->carRotation[2], demo->m_car->carRotation[3]);//ʼ����0001
			demo->run();
            break;
        }

		
		//case 5: {//connector���ӷ�
		//	connector_use_demo* demo = connector_use_demo::getInstance();//mei delate//�úÿ����������ĸ�����ô���õ�
		//	demo->createScene(b);
		//	demo->run();
		//	//�ӿ��ϲ�д���������̳�GLApp����������������Žӽӿڡ�
		//	break;
		//}

		case 6: {//�߶ȳ��������������������ⲻ�ȶ�������Ҫ��
			//���Ļ����Ƶ�һ��ʱ�䲽��á�
			new1* demo = new1::getInstance();
			demo->createScene();
			demo->run();
			break;
		}
		//�����Ϊ��������һ����صģ�
		case 7: {//�߶ȳ���С�����ܵ������ɹ�
			DemoHeightFieldSandLandRigid* demo = DemoHeightFieldSandLandRigid::getInstance();
			demo->createScene();
			demo->run();
			break;
		}
		case 8: {//С��&�������
			DemoHeightFieldSandRigid_Sphere* demo = DemoHeightFieldSandRigid_Sphere::getInstance();
			demo->createScene();
			demo->run();
			break;
		}

		case 9: {//�߶ȳ�����С�����ܵ������ɹ�������ɳ�Ӹ���أ������������û��mu����ȥ���ϡ�
			DemoHeightFieldMudLandRigid* demo = DemoHeightFieldMudLandRigid::getInstance();
			demo->createScene();
			demo->run();
			break;
		}

		case 10: {//��̥��case9�������µ�demo�ó��������ԡ����ָĳ�������
			DemoHeightFieldMudLandCar* demo = DemoHeightFieldMudLandCar::getInstance();
			demo->createScene();
			demo->run();
			break;
		}
		case 11: {//���������˻������ĳɷɻ�������PBDCraft�ļ�
			DemoHeightFieldcraft* demo = DemoHeightFieldcraft::getInstance();
			demo->createScene();
			demo->run();
			break;
		}
		//slippery
		case 12: {//ʪ����
			DemoHeightFieldSlippery* demo = DemoHeightFieldSlippery::getInstance();
			demo->createScene();
			demo->run();
			break;
		}
		//case 13: {//���������˻��Ľӿ�
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
