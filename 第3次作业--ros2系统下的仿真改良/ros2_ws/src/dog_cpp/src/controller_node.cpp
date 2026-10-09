#include "my_interfaces/msg/joint_command.hpp"
#include "my_interfaces/msg/joint_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include <array>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "controller.hpp"
#include <algorithm>
#include <iostream>
#include "my_interfaces/srv/set_mode.hpp"




class JointBridge : public rclcpp::Node
{
    std::array<double, 12> q_stand = {
        0.0,  0.6, -1.0,
        0.0, -0.6,  1.0,
        0.0, -0.6,  1.0,
        0.0,  0.6, -1.0
    };

    std::array<double, 12> q_lie = {
        0.0,  1.55, -2.45,
        0.0, -1.55,  2.45,
        0.0, -1.55,  2.45,
        0.0,  1.55, -2.45
    };

public:
  JointBridge()
  : Node("controller_node"), controller_(q_stand, q_lie)
  {
    const std::string state_topic =
      this->declare_parameter<std::string>("state_topic", "joint_states");
    const std::string command_topic =
      this->declare_parameter<std::string>("command_topic", "joint_command");

    // QoS 要跟上游状态源/下游控制器一致,否则收不到或发不出去
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    // 真机 sensor data 常用:rclcpp::QoS(rclcpp::SensorDataQoS())

    sub_ = this->create_subscription<my_interfaces::msg::JointState>(
      state_topic, qos,
      std::bind(&JointBridge::on_state, this, std::placeholders::_1));
    pub_ = this->create_publisher<my_interfaces::msg::JointCommand>(command_topic, qos);
    set_mode_service_ = this->create_service<my_interfaces::srv::SetMode>(
      "set_mode",
      [this](const std::shared_ptr<my_interfaces::srv::SetMode::Request> request,
             std::shared_ptr<my_interfaces::srv::SetMode::Response> response)
      {
          if (request->mode < 0 || request->mode > 4)
          {
                response->success = false;
                response->message = "非法模式：" + std::to_string(request->mode) + "（合法范围 0~4）";
                return;
          }
          const RobotMode mode_list[5] = {RobotMode::DAMPING, RobotMode::STAND, RobotMode::LIE, RobotMode::MARCH, RobotMode::WALK};
          controller_.set_mode(mode_list[request->mode]);
          response->success = true;
          response->message = "Mode changed successfully";
          return;
      });
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(2),
        std::bind(&JointBridge::on_timer, this));
        RCLCPP_INFO(this->get_logger(), "controller_node 已启动: 订阅 %s -> 发布 %s",
      state_topic.c_str(),command_topic.c_str() );
 }  
    
  

private:
  // 收到一条状态 -> 生成并发布一条指令
  void on_state(const my_interfaces::msg::JointState::SharedPtr state)
  {
    for (int i=0;i<12;i++)
    {
        latest_q[i]=state->q[i];
        latest_dq[i]=state->dq[i];
        latest_ddq[i]=state->ddq[i];
        latest_cur[i]=state->cur[i];
        latest_tau[i]=state->tau[i];
    } 
    
  }
  void on_timer()
    {
       my_interfaces::msg::JointCommand command;
    double dt = 0.002;
    
    cmd_ans cmd=controller_.update(latest_q,dt);
    for (int i=0;i<12;i++)
    {
        command.q[i]=cmd.q[i];
        command.w[i]=cmd.w[i];
        command.kp[i]=cmd.kp[i];
        command.tau[i]=cmd.tau[i];
        command.kd[i]=cmd.kd[i];

    } 
        command.header.stamp = this->get_clock()->now();
        command.header.frame_id = "joint_command";
        pub_->publish(command);

    if (++count_ % 100 == 0) 
    {
      RCLCPP_INFO(this->get_logger(), "已转发 %zu 条指令, q[0]=%.3f", count_, command.q[0]);
    }
    }


  rclcpp::Publisher<my_interfaces::msg::JointCommand>::SharedPtr pub_;
  rclcpp::Subscription<my_interfaces::msg::JointState>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<my_interfaces::srv::SetMode>::SharedPtr set_mode_service_;
  std::array<double, 12> latest_q{};
  std::array<double, 12> latest_dq{};
  std::array<double, 12> latest_ddq{};
  std::array<double, 12> latest_cur{};
  std::array<double, 12> latest_tau{};
  RobotController controller_;
  size_t count_ = 0;
  double clip=30.0;
  double kt=10.0;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointBridge>());
  rclcpp::shutdown();
  return 0;
}