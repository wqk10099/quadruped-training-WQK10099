#include "my_interfaces/msg/joint_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <torch/script.h>
#include "my_interfaces/msg/joint_command.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <array>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <algorithm>
#include <cmath>

class RlControllerNode : public rclcpp::Node
{
public:
  RlControllerNode()
  : Node("rl_controller")
  {
    state_topic_ = this->declare_parameter<std::string>(
      "state_topic", "joint_states");
    imu_topic_ = this->declare_parameter<std::string>(
      "imu_topic", "imu");
    command_topic_ = this->declare_parameter<std::string>(
      "command_topic", "joint_command");
    cmd_vel_topic_ = this->declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");

    this->declare_parameter<bool>("enable_publish", false);//默认步发布

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

    state_sub_ = this->create_subscription<my_interfaces::msg::JointState>(
      state_topic_, qos,
      std::bind(&RlControllerNode::on_state, this, std::placeholders::_1));

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, qos,
      std::bind(&RlControllerNode::on_imu, this, std::placeholders::_1));

    command_pub_ = this->create_publisher<my_interfaces::msg::JointCommand>(
      command_topic_, qos);

    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
       cmd_vel_topic_, qos,
      std::bind(&RlControllerNode::on_cmd_vel, this, std::placeholders::_1));


    print_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000),
      std::bind(&RlControllerNode::print_state, this));
    
    policy_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&RlControllerNode::on_policy, this));

    last_cmd_time_=this->now();
    

    RCLCPP_INFO(
      this->get_logger(),
      "rl_controller 已启动：订阅 %s 和 %s,准备发布%s",
      state_topic_.c_str(), imu_topic_.c_str(),command_topic_.c_str());
    model_path_ = this->declare_parameter<std::string>(
  "model_path", "/home/wangqikai/大作业/best.pt");

  try
  {
    model_ = torch::jit::load(model_path_);
    model_.eval();

    RCLCPP_INFO(
      this->get_logger(),
      "模型加载成功: %s",
      model_path_.c_str());
  }
  catch (const c10::Error & e)
  {
    RCLCPP_FATAL(
      this->get_logger(),
      "模型加载失败: %s",
      e.what());
    throw;
  }
  }

private:
  void on_state(const my_interfaces::msg::JointState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    for (std::size_t i = 0; i < 12; ++i)
    {
      q_[i] = msg->q[i];
      dq_[i] = msg->dq[i];
    }
    for (std::size_t i = 0; i < 12; ++i)
    {
      q_policy_[i]=q_[joint_mapping[i]];
      dq_policy_[i]=dq_[joint_mapping[i]];
    }
      for (int i=0;i<3;i++)
      {
            frame_[i]=commands_[i]*command_scale[i];
      }
      for (int i=0;i<num_dofs;i++)
      {
            frame_[9+i]=(q_policy_[i]-default_dof_pos[i])*dof_pos_scale;
            frame_[21+i]=dq_policy_[i]*dof_vel_scale;
            frame_[33+i]=actions_[i];
      }
      

    has_state_ = true;
  }

  void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    quat_[0] = msg->orientation.w;
    quat_[1] = msg->orientation.x;
    quat_[2] = msg->orientation.y;
    quat_[3] = msg->orientation.z;

    gyro_[0] = msg->angular_velocity.x;
    gyro_[1] = msg->angular_velocity.y;
    gyro_[2] = msg->angular_velocity.z;

    double gx=2*(quat_[0]*quat_[2]-quat_[1]*quat_[3]);
    double gy=-2*(quat_[0]*quat_[1]+quat_[2]*quat_[3]);
    double gz=2*(quat_[1]*quat_[1]+quat_[2]*quat_[2])-1;
    gravity_vec_={gx,gy,gz};
    ang_vel_body_={gyro_[0],gyro_[1],gyro_[2]};
    
    for (int i=0;i<3;i++)
    {
        frame_[3+i]=ang_vel_body_[i]*ang_vel_scale;
        frame_[6+i]=gravity_vec_[i];
    }
    
    has_imu_ = true;
  }
  
  void on_policy()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if ((this->now() - last_cmd_time_).seconds() > 0.5)
    {
       commands_ = {0.0, 0.0, 0.0};        //超过 0.5 秒没有收到 /cmd_vel，就自动回到零速度站立
    }

    if (!has_state_ || !has_imu_)
    {
    return;
    }
    for(int i=5;i>0;--i)
    {
      history_[i]=history_[i-1];//将整体历史帧向后退一帧
    }
    history_[0]=frame_;//记录新的一帧

    
    my_interfaces::msg::JointCommand command;
    for (int i=0;i<12;i++)
    {

      const int mj=joint_mapping[i];
      const double q_target_policy=default_dof_pos[i]+actions_[i]*action_scale[i];
      command.q[mj]=q_target_policy;
      command.w[mj]=0.0;
      command.kp[mj]=rl_kp[i];
      command.kd[mj]=rl_kd[i];
      command.tau[mj]=0.0;
    }
     command.header.stamp=this->get_clock()->now();
     command.header.frame_id="joint_command";
     bool enable_publish = false;
     this->get_parameter("enable_publish", enable_publish);

     if(enable_publish)
     {
      command_pub_->publish(command);
     }

    ++warmup_count_;//预热帧数

    if(warmup_count_<50)//最少必须等到6帧，历史输入才完整，使用50帧约等于1s，可以让actions_的历史也逐渐稳定
    {
      RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "策略预热中：%zu/50",
      warmup_count_);

      return;
    }

    for(int i=0;i<6;i++)//将历史帧数据拼成输入量
    {
      for(int j=0;j<45;j++)
      {
        obs270_[45*i+j]=history_[i][j];
      }
    }

    torch::Tensor obs_tensor = torch::from_blob(
         obs270_.data(),
         {1, 270},
         torch::TensorOptions().dtype(torch::kFloat64)).to(torch::kFloat32);//obs270_ 是 double，转成 float32 后送入模型

    torch::Tensor output;
   {
       torch::NoGradGuard no_grad;//避免推理时保存梯度
       output = model_.forward({obs_tensor}).toTensor();
   }

    if (output.dim() != 2 ||
        output.size(0) != 1 ||
        output.size(1) != 12)
    {
       RCLCPP_ERROR(this->get_logger(), "模型输出形状异常");
       return;
    }

    for (int i = 0; i < 12; ++i)//原来的actions_用于构造输入后，得到的新actions_更新
    {
      const double action = output[0][i].item<double>();

      if (!std::isfinite(action))//这样模型万一输出 NaN 或无穷大，不会把非法目标位置继续发布给仿真节点
      {
         RCLCPP_ERROR(
          this->get_logger(),
         "策略输出的第 %d 个动作不是有限值",
         i);
         return;
      }

      actions_[i] = action;
    }
   
     RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      " single_obs_size=%zu, obs270_size=%zu, "
      "frame0[0..2]=[%.3f, %.3f, %.3f] "
      "frame0[3..5]=[%.3f, %.3f, %.3f],"
      "frame0[6..8]=[%.3f, %.3f, %.3f]"
      "frame5[0..2]=[%.3f, %.3f, %.3f]"
      "frame5[3..5]=[%.3f, %.3f, %.3f]"
      "frame5[6..8]=[%.3f, %.3f, %.3f]"
      "action=[%.4f %.4f %.4f | %.4f %.4f %.4f | "
              "%.4f %.4f %.4f | %.4f %.4f %.4f]",

      frame_.size(),
      obs270_.size(),
      frame_[0], frame_[1], frame_[2],
      frame_[3], frame_[4], frame_[5],
      frame_[6], frame_[7], frame_[8],
      history_[5][0],history_[5][1],history_[5][2],
      history_[5][3],history_[5][4],history_[5][5],
      history_[5][6],history_[5][7],history_[5][8],
      actions_[0],  actions_[1],  actions_[2],
      actions_[3],  actions_[4],  actions_[5],
      actions_[6],  actions_[7],  actions_[8],
      actions_[9],  actions_[10], actions_[11]
    );

    
  }

  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
  std::lock_guard<std::mutex> lock(mutex_);

  commands_[0] = std::clamp(msg->linear.x, -0.5, 0.5);
  commands_[1] = std::clamp(msg->linear.y, -0.3, 0.3);
  commands_[2] = std::clamp(msg->angular.z, -0.5, 0.5);

  last_cmd_time_ = this->now();
  }

  void print_state()
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!has_state_ || !has_imu_)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "等待 joint_states 或 imu 数据中...");
      return;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "q=[%.3f %.3f %.3f | %.3f %.3f %.3f | %.3f %.3f %.3f | %.3f %.3f %.3f]",
      q_[0], q_[1], q_[2],
      q_[3], q_[4], q_[5],
      q_[6], q_[7], q_[8],
      q_[9], q_[10], q_[11]);

    RCLCPP_INFO(
      this->get_logger(),
      "dq=[%.3f %.3f %.3f | %.3f %.3f %.3f | %.3f %.3f %.3f | %.3f %.3f %.3f]",
      dq_[0], dq_[1], dq_[2],
      dq_[3], dq_[4], dq_[5],
      dq_[6], dq_[7], dq_[8],
      dq_[9], dq_[10], dq_[11]);

    
    RCLCPP_INFO(
      this->get_logger(),
      "quat(wxyz)=[%.3f %.3f %.3f %.3f] gyro=[%.3f %.3f %.3f]",
      quat_[0], quat_[1], quat_[2], quat_[3],
      gyro_[0], gyro_[1], gyro_[2]);
    
    RCLCPP_INFO(
      this->get_logger(),
      "obs_size=%.3i" ,45
      );

    RCLCPP_INFO(
      this->get_logger(),
      "commands=[%.3f %.3f %.3f]",
      frame_[0], frame_[1], frame_[2]
      );

    RCLCPP_INFO(
      this->get_logger(),
      "ang_vel=[%.3f %.3f %.3f]",
      frame_[3], frame_[4], frame_[5]
      );

    RCLCPP_INFO(
      this->get_logger(),
      "gravity_vec=[%.3f %.3f %.3f]",
      frame_[6], frame_[7], frame_[8]
      );

    RCLCPP_INFO(
      this->get_logger(),
      "dof_pos=[%.3f %.3f %.3f| %.3f %.3f %.3f | %.3f %.3f %.3f | %.3f %.3f %.3f]",
      frame_[9], frame_[10], frame_[11],
      frame_[12],frame_[13],frame_[14],
      frame_[15],frame_[16],frame_[17],
      frame_[18],frame_[19],frame_[20]
      );

    RCLCPP_INFO(
      this->get_logger(),
      "dof_vel=[%.3f %.3f %.3f| %.3f %.3f %.3f | %.3f %.3f %.3f | %.3f %.3f %.3f]",
      frame_[21], frame_[22], frame_[23],
      frame_[24],frame_[25],frame_[26],
      frame_[27],frame_[28],frame_[29],
      frame_[30],frame_[31],frame_[32]
      );

    RCLCPP_INFO(
      this->get_logger(),
      "actions=[%.3f %.3f %.3f| %.3f %.3f %.3f | %.3f %.3f %.3f | %.3f %.3f %.3f]",
      frame_[33], frame_[34], frame_[35],
      frame_[36],frame_[37],frame_[38],
      frame_[39],frame_[40],frame_[41],
      frame_[42],frame_[43],frame_[44]
      );
  }

  std::string state_topic_;
  std::string imu_topic_;
  std::string command_topic_;
  std::string cmd_vel_topic_;

  rclcpp::Subscription<my_interfaces::msg::JointState>::SharedPtr state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<my_interfaces::msg::JointCommand>::SharedPtr command_pub_;
  rclcpp::TimerBase::SharedPtr print_timer_;//打印计时器
  rclcpp::TimerBase::SharedPtr policy_timer_;//策略计时器
  rclcpp::Time last_cmd_time_;
  std::mutex mutex_;
  //最新状态快照，由on_tate,on_imu更新
  std::array<double, 12> q_{};
  std::array<double, 12> dq_{};
  std::array<double, 4> quat_{1.0, 0.0, 0.0, 0.0};  //四元数表示的机体姿态
  std::array<double, 3> gyro_{};                    //机体坐标系下的角速度

  bool has_state_ = false;
  bool has_imu_ = false;

  int num_dofs=12;
  std::array<int,12> joint_mapping{0,1,2,3,4,5,9,10,11,6,7,8};
  std::array<double,12> default_dof_pos{0.0, 0.8014, -1.527,
                                        0.0, -0.8014, 1.527,
                                        0.0, 0.8014, -1.527,
                                        0.0, -0.8014, 1.527}; //默认关节位置，单位 rad
  std::array<double, 3> command_scale{2.0,2.0,0.25};
  double ang_vel_scale=0.25;
  double dof_pos_scale=1.0;
  double dof_vel_scale=0.05;    
  std::array<double, 12> action_scale{
  0.25, 0.25, 0.25,
  0.25, 0.25, 0.25,
  0.25, 0.25, 0.25,
  0.25, 0.25, 0.25
  };                   //动作缩放系数，单位 rad

  std::array<double, 12> rl_kp{
  40.0, 40.0, 40.0,
  40.0, 40.0, 40.0,
  40.0, 40.0, 40.0,
  40.0, 40.0, 40.0
  };

  std::array<double, 12> rl_kd{
  1.2, 1.2, 1.2,
  1.2, 1.2, 1.2,
  1.2, 1.2, 1.2,
  1.2, 1.2, 1.2
  };
  
  std::array<double, 3> commands_{0,0,0};
  std::array<double, 3> ang_vel_body_{};  //机体坐标系下的角速度
  std::array<double, 3> gravity_vec_{};   //重力向量在机体坐标系下的表示
  std::array<double, 12> actions_{0,0,0,0,0,0,0,0,0,0,0,0};

  std::array<double, 12> q_policy_{};     //策略顺序的关节位置
  std::array<double, 12> dq_policy_{};    //策略顺序的关节速度
  std::array<double, 45> frame_{};//单帧观测
  std::array<std::array<double, 45>, 6> history_{};//六帧历史缓存
  std::array<double, 270> obs270_{};//最终模型输入
  std::size_t warmup_count_ = 0;

  std::string model_path_;
  torch::jit::script::Module model_;//接入torch模型
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RlControllerNode>());
  rclcpp::shutdown();
  return 0;
}