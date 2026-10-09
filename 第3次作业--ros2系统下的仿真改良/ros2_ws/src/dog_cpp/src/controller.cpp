#include "controller.hpp"
#include <algorithm>
#include <cmath>

RobotController::RobotController(const std::array<double, 12>& q_stand,
                                 const std::array<double, 12>& q_lie,
                                 double kp, double kd, double clip, double joint_speed)
: q_stand_(q_stand), q_lie_(q_lie), kp_(kp), kd_(kd),
  clip_(clip), joint_speed_(joint_speed), q_cmd_(q_stand)
{
}

RobotMode RobotController::mode() const
{
    return mode_;
}

void RobotController::set_mode(RobotMode mode)
{
    if (mode == mode_) return;

    mode_ = mode;
    phase_ = 0.0;
    waypoints_.clear();

    if (mode == RobotMode::STAND)
    {
        waypoints_.push_back(q_lie_);
        waypoints_.push_back(q_stand_);
    }
    else if (mode == RobotMode::LIE)
    {
        waypoints_.push_back(q_lie_);
    }
    else if (mode == RobotMode::MARCH || mode == RobotMode::WALK)
    {
        waypoints_.push_back(q_stand_);
        stride_ = (mode == RobotMode::MARCH) ? 0.0 : 0.20;
    }
}

cmd_ans RobotController::update(const std::array<double, 12>& q_cur, double dt)
{
    cmd_ans cmd;   // 每次调用都是全新的；w / tau 由 {} 保证为 0

    // 阻尼：原来 tau = -kd*dq，等价于 kp=0, kd=kd_, w=0, tau=0
    if (mode_ == RobotMode::DAMPING)
    {
        for (int i = 0; i < 12; ++i)
        {
            q_cmd_[i] = q_cur[i];
            cmd.q[i]  = q_cur[i];
            cmd.kp[i] = 0.0;
            cmd.kd[i] = kd_;
        }
        return cmd;
    }

    // 已躺平：原来 ctrl=0（完全不出力）→ kp=kd=0, tau=0
    if (mode_ == RobotMode::LIE && waypoints_.empty())
    {
        for (int i = 0; i < 12; ++i)
        {
            q_cmd_[i] = q_cur[i];
            cmd.q[i]  = q_cur[i];
            cmd.kp[i] = 0.0;
            cmd.kd[i] = 0.0;
        }
        return cmd;
    }

    std::array<double, 12> target;
    std::array<double, 12> offset;

    if (!waypoints_.empty())
    {
        target = waypoints_[0];
    }
    else if (mode_ == RobotMode::MARCH || mode_ == RobotMode::WALK)
    {
        phase_ += freq_ * dt;
        phase_ = std::fmod(phase_, 1.0);
        offset = gait_offset(phase_);
        for (int i = 0; i < 12; ++i) target[i] = q_stand_[i] + offset[i];
    }
    else
    {
        target = q_stand_;
    }

    if ((mode_ == RobotMode::MARCH || mode_ == RobotMode::WALK) && waypoints_.empty())
    {
        q_cmd_ = target;
    }
    else
    {
        const double max_step = joint_speed_ * dt;
        for (int i = 0; i < 12; ++i)
            q_cmd_[i] += std::clamp(target[i] - q_cmd_[i], -max_step, max_step);
    }

    if (!waypoints_.empty())
    {
        double max_error = 0.0;
        for (int i = 0; i < 12; ++i)
            max_error = std::max(max_error, std::abs(q_cmd_[i] - waypoints_.front()[i]));
        if (max_error < 1e-2) waypoints_.erase(waypoints_.begin());
    }

    // 普通模式：kp=kp_, kd=kd_, q=q_cmd_, w=0, tau=0
    for (int i = 0; i < 12; ++i)
    {
        cmd.kp[i] = kp_;
        cmd.kd[i] = kd_;
        cmd.q[i]  = q_cmd_[i];
    }
    return cmd;
}

std::array<double, 12> RobotController::gait_offset(double phase) const
{
    constexpr double pi = 3.14159265358979323846;
    std::array<double, 12> offset = {0.0, 0.0, 0.0, 0.0,
                                     0.0, 0.0, 0.0, 0.0,
                                     0.0, 0.0, 0.0, 0.0};
    double sw = 0.25;
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
        if (u < sw)
        {
            s = u / sw;
            xoff = -stride_ / 2 + s * stride_;
            lift = lift_ * std::sin(pi * s);
        }
        else
        {
            s = (u - sw) / (1 - sw);
            xoff = stride_ / 2 - s * stride_;
            lift = 0.0;
        }
        dc = lift / KZ[GAIT_ORDER[i]];
        offset[3 * GAIT_ORDER[i] + 2] = offset[3 * GAIT_ORDER[i] + 2] + dc;
        offset[3 * GAIT_ORDER[i] + 1] = offset[3 * GAIT_ORDER[i] + 1]
            + xoff / KX[GAIT_ORDER[i]] - KC[GAIT_ORDER[i]] * dc / KX[GAIT_ORDER[i]];
    }
    return offset;
}
