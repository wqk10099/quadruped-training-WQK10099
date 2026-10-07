#pragma once
#include "simulator.hpp"
#include<array>
#include<vector>
class DampingController
{
public:
explicit DampingController(double kd=2.0,double clip=30.0);
void update(MuJoCoSimulator& simulator);
private:
double kd_;
double clip_;
};
class PDController
{
public:
explicit PDController(const std::array<double, 12>& q_des,double kp=60.0,double kd=3.0,double clip=30.0);
void update(MuJoCoSimulator& simulator);
private:
std::array<double, 12> q_des_;
double kp_;
double kd_;
double clip_;
};
enum class RobotMode
{
    DAMPING,
    STAND,
    LIE,
    MARCH,
    WALK
};
class RobotController
{
double KX[4] = {-0.420,  0.427,  0.427, -0.420};
double KZ[4] = {-0.085,  0.129,  0.129, -0.085};
double KC[4] = {-0.229,  0.2075, 0.2075, -0.229};
int GAIT_ORDER[4] = {0, 1, 3, 2};
public:
explicit RobotController(const std::array<double, 12>& q_stand,const std::array<double, 12>& q_lie,
                                double kp=60.0,double kd=3.0,double clip=30.0,double joint_speed=2.0);
void update(MuJoCoSimulator& simulator);
void set_mode(RobotMode mode);
RobotMode mode() const;

private:
std::array<double, 12> q_stand_;
std::array<double, 12> q_lie_;
double kp_;
double kd_;
double clip_;
double joint_speed_;
std::vector<std::array<double, 12>> waypoints_={};
RobotMode mode_=RobotMode::DAMPING;
std::array<double, 12> q_cmd_;
double phase_=0.0;
double freq_=1.2;
double lift_=0.06;
double stride_=0.0;
std::array<double, 12> gait_offset(double phase) const;
};