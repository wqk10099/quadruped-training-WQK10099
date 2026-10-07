#include "controller.hpp"
#include<algorithm>
#include<cmath>
DampingController::DampingController(double kd,double clip):kd_(kd),clip_(clip){}
void DampingController::update(MuJoCoSimulator& simulator)
{
    for (int i=0;i<simulator.nu();i++)
    {
        double dq_i=simulator.data()->qvel[6+i];//获取关节速度
        double tau_i=-kd_*dq_i;
        simulator.data()->ctrl[i]=std::clamp(tau_i,-clip_,clip_);
    }
}
PDController::PDController(const std::array<double, 12>& q_des,double kp,double kd,double clip):q_des_(q_des),kp_(kp),kd_(kd),clip_(clip){}
void PDController::update(MuJoCoSimulator& simulator)
{
    for (int i=0;i<simulator.nu();i++)
    {
        double q_i=simulator.data()->qpos[7+i];
        double dq_i=simulator.data()->qvel[6+i];//获取关节速度
        double tau_i=kp_*(q_des_[i]-q_i)-kd_*dq_i;
        simulator.data()->ctrl[i]=std::clamp(tau_i,-clip_,clip_);
    }
}
RobotController::RobotController(const std::array<double, 12>& q_stand,const std::array<double, 12>& q_lie,double kp,double kd,double clip,double joint_speed):
                                q_stand_(q_stand),q_lie_(q_lie),kp_(kp),kd_(kd),clip_(clip),joint_speed_(joint_speed),q_cmd_(q_stand){}
RobotMode RobotController::mode() const
{
    return mode_;
}
void RobotController::set_mode(RobotMode mode)
{
    if(mode==mode_)
    {
        return ;
    }
    mode_=mode;
    phase_=0.0;
    waypoints_.clear();
    if(mode==RobotMode::DAMPING)
    {
        waypoints_={};
    }
    else if(mode==RobotMode::STAND)
    {
        waypoints_.push_back(q_lie_);
        waypoints_.push_back(q_stand_);
    }
    else if(mode==RobotMode::LIE)
    {
        waypoints_.push_back(q_lie_);
    }
    else if(mode==RobotMode::MARCH||mode==RobotMode::WALK)
    {
        waypoints_.push_back(q_stand_);
        if (mode==RobotMode::MARCH)
        {
            stride_=0.0;
        }
        else
        {
            stride_=0.20;
        }
    }
}
void RobotController::update(MuJoCoSimulator& simulator)
{
    mjData* d=simulator.data();
    if (mode_==RobotMode::DAMPING)
    {
         for (int i=0;i<simulator.nu();i++)
        {
           double dq_i=d->qvel[6+i];//获取关节速度
           double tau_i=-kd_*dq_i;
           d->ctrl[i]=std::clamp(tau_i,-clip_,clip_);
           q_cmd_[i] = d->qpos[7 + i];
        }
        return;
    }
    if (mode_==RobotMode::LIE && waypoints_.empty())
    {
          for (int i=0;i<simulator.nu();i++)
        {
           d->ctrl[i]=0.0;
           q_cmd_[i] = d->qpos[7 + i];
        }
        return;
    }
    std::array<double, 12> target;
    std::array<double, 12> offset;
    if (!waypoints_.empty())
    {
         target=waypoints_[0];
    }
    else if(mode_==RobotMode::MARCH||mode_==RobotMode::WALK)
    {
        phase_+=freq_*simulator.timestep();
        phase_=std::fmod(phase_,1.0);
        offset=gait_offset(phase_);
        for (int i=0;i<simulator.nu();i++)
        {
            target[i]=q_stand_[i]+offset[i];
        }
    }
    else{ target=q_stand_;}

    if((mode_==RobotMode::MARCH||mode_==RobotMode::WALK)&&waypoints_.empty())
    {
        q_cmd_=target;
    }
    else
    {
         double max_step=joint_speed_*simulator.timestep();
         for(int i=0;i<simulator.nu();i++)
         {
            q_cmd_[i]+=std::clamp(target[i]-q_cmd_[i],-max_step,max_step);
         }
    }
    
    
   if (!waypoints_.empty())
   {
    double max_error = 0.0;

    for (int i = 0; i < simulator.nu(); i++)
    {
        double error = std::abs(q_cmd_[i] - waypoints_.front()[i]);
        max_error = std::max(max_error, error);
    }

    if (max_error < 1e-2)
    {
        waypoints_.erase(waypoints_.begin());
    }
   }
    for (int i = 0; i < simulator.nu(); i++)
    {
        double tau_i=kp_*(q_cmd_[i]-d->qpos[7+i])-kd_*(d->qvel[6+i]);
        d->ctrl[i]=std::clamp(tau_i,-clip_,clip_);
    }
    
}
std::array<double, 12> RobotController::gait_offset(double phase) const
{
    constexpr double pi = 3.14159265358979323846;
    std::array<double, 12> offset = {0.0, 0.0, 0.0, 0.0,
                                     0.0, 0.0, 0.0, 0.0,
                                     0.0, 0.0, 0.0, 0.0};
    double sw=0.25;
    for (int i = 0; i < 4; i++)
    {
       double u = std::fmod(phase - i / 4.0, 1.0);
      if (u < 0.0)
      {
          u += 1.0;
      }
       double s;
       double xoff;
       double lift;
       double dc;
       if(u<sw)
       {
           s=u/sw;
           xoff=-stride_/2+s*stride_;
           lift=lift_*std::sin(pi*s);
       }    
       else
       {
          s=(u-sw)/(1-sw);
          xoff=stride_/2-s*stride_;
          lift=0.0;
       }
       dc=lift/KZ[GAIT_ORDER[i]];
       offset[3*GAIT_ORDER[i]+2]=offset[3*GAIT_ORDER[i]+2]+dc;
       offset[3*GAIT_ORDER[i]+1]=offset[3*GAIT_ORDER[i]+1]+xoff/KX[GAIT_ORDER[i]]-KC[GAIT_ORDER[i]]*dc/KX[GAIT_ORDER[i]];
    }
    return offset;
}