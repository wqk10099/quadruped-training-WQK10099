#include <iostream>
#include "simulator.hpp"
#include <exception>
#include <filesystem>
#include "controller.hpp"
int main(int argc, char** argv)//argc是命令行参数的个数，argv是一个字符串数组，存储了命令行参数的值
{
    std::array<double, 12> q_stand={0.0,  0.6, -1.0,
                                   0.0, -0.6,  1.0,
                                   0.0, -0.6,  1.0,
                                   0.0,  0.6, -1.0};//站立姿态关节目标角
    std::array<double, 12> q_lie={0.0,  1.55, -2.45,
                                  0.0, -1.55,  2.45,
                                  0.0, -1.55,  2.45,
                                  0.0,  1.55, -2.45};//关节目标角
    RobotController controller(q_stand,q_lie);
    if(argc<2)//如果argc小于2，说明没有提供场景文件路径参数
    {
        std::cerr<<"Usage: "<<argv[0]<<" <scene_path>"<<std::endl;//输出错误信息，提示用户正确的使用方法
        return 1;
    }
    try
    {
        MuJoCoSimulator simulator(argv[1]);//创建MuJoCoSimulator对象，传入场景文件路径
        if(controller.mode()!=RobotMode::DAMPING)
        {return 1;}
        simulator.reset_to_keyframe("stand");
        simulator.data()->qvel[6]=1.0;
        controller.update(simulator);
        std::cout<<simulator.data()->ctrl[0]<<std::endl;
        controller.set_mode(RobotMode::STAND);
        if(controller.mode()==RobotMode::STAND)
        {
             std::cout<<"STAND"<<std::endl;
        }
        else{std::cout<<"error"<<std::endl;return 1;}
        controller.set_mode(RobotMode::LIE);
        if(controller.mode()==RobotMode::LIE)
        {
             std::cout<<"LIE"<<std::endl;
        }
        else{std::cout<<"error"<<std::endl;return 1;}
        controller.set_mode(RobotMode::DAMPING);
        if(controller.mode()==RobotMode::DAMPING)
        {
             std::cout<<"DAMPING"<<std::endl;
        }
        else{std::cout<<"error"<<std::endl;return 1;}
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
