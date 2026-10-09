#include "my_interfaces/msg/joint_command.hpp"
#include "my_interfaces/msg/joint_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include <array>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "simulator.hpp"
#include <algorithm>
#include <iostream>
#include "sensor_msgs/msg/imu.hpp"
#include <mutex>
#include <thread>
#include <SDL2/SDL.h>
#include <SDL2/SDL_opengl.h>

// 输出当前 OpenGL 上下文的诊断信息，便于排查渲染环境问题。
void print_gl_string(const char* label, GLenum name)
{
    const GLubyte* value = glGetString(name);

    if (value == nullptr)
    {
        std::cout << label << ": null" << std::endl;
    }
    else
    {
        std::cout << label << ": "
                  << reinterpret_cast<const char*>(value)
                  << std::endl;
    }
}

class JointBridge : public rclcpp::Node
{
public:
  JointBridge(const std::string& scene_path)
  : Node("sim_node"), simulator_(scene_path)
  {
    const std::string state_topic =
      this->declare_parameter<std::string>("state_topic", "joint_states");
    const std::string command_topic =
      this->declare_parameter<std::string>("command_topic", "joint_command");

    // QoS 要跟上游状态源/下游控制器一致,否则收不到或发不出去
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    // 真机 sensor data 常用:rclcpp::QoS(rclcpp::SensorDataQoS())

    sub_ = this->create_subscription<my_interfaces::msg::JointCommand>(
      command_topic, qos,
      std::bind(&JointBridge::on_command, this, std::placeholders::_1));
    pub_1 = this->create_publisher<my_interfaces::msg::JointState>(state_topic, qos);
    pub_2=  this->create_publisher<sensor_msgs::msg::Imu>("imu", qos);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(2),
        std::bind(&JointBridge::on_timer, this));
        RCLCPP_INFO(this->get_logger(), "sim_node 已启动: 订阅 %s -> 发布 %s",
      command_topic.c_str(),state_topic.c_str() );
    
 }  
    const mjModel* model() const { return simulator_.model(); }
    mjData* data() { return simulator_.data(); }
    std::mutex& mutex() { return mtx_; }
  

private:
  // 收到一条状态 -> 生成并发布一条指令
  void on_command(const my_interfaces::msg::JointCommand::SharedPtr cmd)
  {
    for (int i=0;i<simulator_.nu();i++)
    {
        latest_q[i]=cmd->q[i];
        latest_w[i]=cmd->w[i];
        latest_kp[i]=cmd->kp[i];
        latest_kd[i]=cmd->kd[i];
        latest_tau[i]=cmd->tau[i];
    } 
    
  }
  void on_timer()
    {
      std::lock_guard<std::mutex> lock(mtx_);
      my_interfaces::msg::JointState state;
      sensor_msgs::msg::Imu imu_msg;
    mjData* d=simulator_.data();
    double tau_i;
    for (int i=0;i<simulator_.nu();i++)
    {
        tau_i=latest_kp[i]*(latest_q[i]-d->qpos[7+i])+latest_kd[i]*(latest_w[i]-d->qvel[6+i])+latest_tau[i];
        d->ctrl[i]=std::clamp(tau_i,-clip,clip);
    }
    simulator_.step();
    for (int i=0;i<simulator_.nu();i++)
    {
        state.q[i]=d->qpos[7+i];
        state.dq[i]=d->qvel[6+i];
        state.ddq[i]=d->qacc[6+i];
        state.tau[i]=d->actuator_force[i];
        state.cur[i]=d->actuator_force[i]/kt;
        
    } 
    state.header.stamp = this->get_clock()->now();
    state.header.frame_id = "joint_state";
    imu_msg.header.stamp = this->get_clock()->now();
    imu_msg.header.frame_id = "imu_link";
    imu_msg.angular_velocity.x=d->qvel[3];
    imu_msg.angular_velocity.y=d->qvel[4];
    imu_msg.angular_velocity.z=d->qvel[5];
    imu_msg.linear_acceleration.x=d->qacc[0];
    imu_msg.linear_acceleration.y=d->qacc[1];
    imu_msg.linear_acceleration.z=d->qacc[2];
    imu_msg.orientation.x=d->qpos[4];
    imu_msg.orientation.y=d->qpos[5];
    imu_msg.orientation.z=d->qpos[6];
    imu_msg.orientation.w=d->qpos[3];
        pub_1->publish(state);
        pub_2->publish(imu_msg);
    if (++count_ % 100 == 0) 
    {
      RCLCPP_INFO(this->get_logger(), "已步进 %zu 条指令, q[0]=%.3f", count_, state.q[0]);
    }
    }


  rclcpp::Publisher<my_interfaces::msg::JointState>::SharedPtr pub_1;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_2;
  rclcpp::Subscription<my_interfaces::msg::JointCommand>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::array<double, 12> latest_q{};
  std::array<double, 12> latest_w{};
  std::array<double, 12> latest_kp{};
  std::array<double, 12> latest_kd{};
  std::array<double, 12> latest_tau{};
  MuJoCoSimulator simulator_;
  std::mutex mtx_;
  size_t count_ = 0;
  double clip=30.0;
  double kt=10.0;
};

int main(int argc, char ** argv)
{
  if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <scene_path>" << std::endl;
        return 1;
    }  
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JointBridge>(argv[1]); 
  // 初始化 SDL 视频子系统。
    if (SDL_Init(SDL_INIT_VIDEO) != 0)
    {
        std::cerr << "SDL_Init failed: " << SDL_GetError() << std::endl;
        return 1;
    }

    // MuJoCo 的传统 OpenGL 渲染器依赖兼容模式上下文。
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(
        SDL_GL_CONTEXT_PROFILE_MASK,
        SDL_GL_CONTEXT_PROFILE_COMPATIBILITY
    );

    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);

    // 创建 OpenGL 窗口。
    SDL_Window* window = SDL_CreateWindow(
        "MuJoCo Viewer",
        SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED,
        1280,
        720,
        SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE
    );

    if (window == nullptr)
    {
        std::cerr << "SDL_CreateWindow failed: " << SDL_GetError() << std::endl;
        SDL_Quit();
        return 1;
    }

    // 创建 OpenGL Context。
    SDL_GLContext gl_context = SDL_GL_CreateContext(window);

    if (gl_context == nullptr)
    {
        std::cerr << "SDL_GL_CreateContext failed: "
                  << SDL_GetError() << std::endl;

        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    // 开启垂直同步，限制渲染帧率。
    if (SDL_GL_SetSwapInterval(1) != 0)
    {
        std::cerr << "VSync is not available: "
                  << SDL_GetError() << std::endl;
    }

    if (SDL_GL_MakeCurrent(window, gl_context) != 0)
    {
        std::cerr << "SDL_GL_MakeCurrent failed: "
                  << SDL_GetError() << std::endl;

        SDL_GL_DeleteContext(gl_context);
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    // 输出 OpenGL 诊断信息，确认当前环境满足 MuJoCo 渲染要求。
    print_gl_string("GL_VERSION", GL_VERSION);
    print_gl_string("GL_RENDERER", GL_RENDERER);
    print_gl_string("GL_VENDOR", GL_VENDOR);

    std::cout << "GL_ARB_framebuffer_object: "
              << (SDL_GL_ExtensionSupported("GL_ARB_framebuffer_object")
                      ? "supported"
                      : "not supported")
              << std::endl;

    std::cout << "GL_EXT_framebuffer_object: "
              << (SDL_GL_ExtensionSupported("GL_EXT_framebuffer_object")
                      ? "supported"
                      : "not supported")
              << std::endl;

    // 创建 MuJoCo 渲染所需的上下文、场景和摄像机对象。
    mjrContext render_context;
    mjvScene scene;
    mjvCamera camera;
    mjvOption option;
    mjvPerturb perturb;

    mjr_defaultContext(&render_context);
    mjv_defaultScene(&scene);
    mjv_defaultCamera(&camera);
    mjv_defaultOption(&option);
    mjv_defaultPerturb(&perturb);

    // 预留足够多的几何体空间，用于显示机器人、地面和灯光。
    mjv_makeScene(node->model(), &scene, 10000);
    mjr_makeContext(node->model(), &render_context, mjFONTSCALE_150);

    // 设置自由摄像机初始视角。
    camera.type = mjCAMERA_FREE;
    camera.distance = 3.0;
    camera.azimuth = 90.0;
    camera.elevation = -20.0;

    // 让摄像机初始观察点位于机器人基座处。
    camera.lookat[0] = node->data()->qpos[0];
    camera.lookat[1] = node->data()->qpos[1];
    camera.lookat[2] = node->data()->qpos[2];
  //ros线程
  std::thread ros_thread([node]() { rclcpp::spin(node); });
  //主线程：渲染循环
  bool running = true;
    while (running && rclcpp::ok())
    {
        SDL_Event e;
        while (SDL_PollEvent(&e))
            if (e.type == SDL_QUIT) running = false;

        int w = 0, h = 0;
        SDL_GL_GetDrawableSize(window, &w, &h);
        if (w <= 0 || h <= 0) continue;

        {
            std::lock_guard<std::mutex> lock(node->mutex());   // ← 和物理互斥
            camera.lookat[0] = node->data()->qpos[0];
            camera.lookat[1] = node->data()->qpos[1];
            camera.lookat[2] = node->data()->qpos[2];
            mjv_updateScene(node->model(), node->data(), &option, &perturb,
                            &camera, mjCAT_ALL, &scene);
        }

        mjrRect viewport{0, 0, w, h};
        mjr_render(viewport, &scene, &render_context);
        SDL_GL_SwapWindow(window);
    }
  rclcpp::shutdown();
  ros_thread.join();
  mjv_freeScene(&scene);
  mjr_freeContext(&render_context);
  SDL_GL_DeleteContext(gl_context);
  SDL_DestroyWindow(window);
  SDL_Quit();
  return 0;
}