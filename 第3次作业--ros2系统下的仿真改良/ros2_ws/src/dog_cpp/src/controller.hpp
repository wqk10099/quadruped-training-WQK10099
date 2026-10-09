#pragma once
#include <array>
#include <vector>

// controller 对外输出：12 个电机的 5 个 MIT 参数
struct cmd_ans
{
    std::array<double, 12> kp{};
    std::array<double, 12> kd{};
    std::array<double, 12> q{};
    std::array<double, 12> w{};
    std::array<double, 12> tau{};
};

enum class RobotMode { DAMPING, STAND, LIE, MARCH, WALK };

class RobotController
{
    double KX[4] = {-0.420,  0.427,  0.427, -0.420};
    double KZ[4] = {-0.085,  0.129,  0.129, -0.085};
    double KC[4] = {-0.229,  0.2075, 0.2075, -0.229};
    int GAIT_ORDER[4] = {0, 1, 3, 2};

public:
    RobotController(const std::array<double, 12>& q_stand,
                    const std::array<double, 12>& q_lie,
                    double kp = 60.0, double kd = 3.0,
                    double clip = 30.0, double joint_speed = 2.0);

    // 注意：不再接收 MuJoCoSimulator；改为接收"当前关节位置"和"时间步"
    cmd_ans update(const std::array<double, 12>& q_cur, double dt);
    void set_mode(RobotMode mode);
    RobotMode mode() const;

private:
    std::array<double, 12> q_stand_;
    std::array<double, 12> q_lie_;
    double kp_;
    double kd_;
    double clip_;
    double joint_speed_;
    std::vector<std::array<double, 12>> waypoints_{};
    RobotMode mode_ = RobotMode::DAMPING;
    std::array<double, 12> q_cmd_;
    double phase_ = 0.0;
    double freq_ = 1.2;
    double lift_ = 0.06;
    double stride_ = 0.0;
    std::array<double, 12> gait_offset(double phase) const;
};
