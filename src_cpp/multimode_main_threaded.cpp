#include <array>
#include <atomic>
#include <chrono>
#include <iostream>
#include <mutex>
#include <optional>
#include <thread>

#include <SDL2/SDL.h>
#include <SDL2/SDL_opengl.h>
#include <mujoco/mujoco.h>

#include "controller.hpp"
#include "simulator.hpp"

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

int main(int argc, char** argv)
{
    // 必须提供场景 XML 路径。
    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <scene_path>" << std::endl;
        return 1;
    }

    // 创建 MuJoCo 仿真器和多模式控制器。
    MuJoCoSimulator simulator(argv[1]);

    // 12 个关节顺序为 FL, FR, RR, RL，每条腿为 hip, thigh, calf。
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

    RobotController controller(q_stand, q_lie);

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
    mjv_makeScene(simulator.model(), &scene, 10000);
    mjr_makeContext(simulator.model(), &render_context, mjFONTSCALE_150);

    // 设置自由摄像机初始视角。
    camera.type = mjCAMERA_FREE;
    camera.distance = 3.0;
    camera.azimuth = 90.0;
    camera.elevation = -20.0;

    // 让摄像机初始观察点位于机器人基座处。
    camera.lookat[0] = simulator.data()->qpos[0];
    camera.lookat[1] = simulator.data()->qpos[1];
    camera.lookat[2] = simulator.data()->qpos[2];

    // 主线程负责窗口、输入和渲染；物理线程只负责仿真推进。
    std::atomic<bool> running{true};
    std::mutex state_mutex;
    std::optional<RobotMode> pending_mode;
    std::optional<RobotMode> request_mode;

    // 物理线程每个时间步运行一次控制器和 mj_step()。
    std::thread physics_thread([&]()
    {
        while (running.load())
        {
            const auto step_start = std::chrono::steady_clock::now();

            // 控制计算、模式切换和物理步进必须作为一个整体受锁保护。
            {
                std::lock_guard<std::mutex> lock(state_mutex);

                if (pending_mode.has_value())
                {
                    controller.set_mode(pending_mode.value());
                    pending_mode.reset();
                }

                controller.update(simulator);
                simulator.step();
            }

            // 按模型时间步补齐本步消耗的时间，使仿真接近实时。
            const auto step_duration =
                std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                    std::chrono::duration<double>(simulator.timestep())
                );
            std::this_thread::sleep_until(step_start + step_duration);
        }
    });

    // 主线程只处理 SDL 事件、渲染和窗口交换。
    while (running.load())
    {
        SDL_Event event;

        while (SDL_PollEvent(&event))
        {
            if (event.type == SDL_QUIT)
            {
                running = false;
            }

            if (event.type == SDL_KEYDOWN)
            {
                // 使用 scancode 判断物理按键，避免受 Caps Lock 和 Shift 影响。
                if (event.key.keysym.scancode == SDL_SCANCODE_ESCAPE)
                {
                    running = false;
                }
                else if (event.key.keysym.scancode == SDL_SCANCODE_S)
                {
                    request_mode = RobotMode::STAND;
                }
                else if (event.key.keysym.scancode == SDL_SCANCODE_W)
                {
                    request_mode = RobotMode::WALK;
                }
                else if (event.key.keysym.scancode == SDL_SCANCODE_D)
                {
                    request_mode = RobotMode::DAMPING;
                }
                else if (event.key.keysym.scancode == SDL_SCANCODE_L)
                {
                    request_mode = RobotMode::LIE;
                }
                else if (event.key.keysym.scancode == SDL_SCANCODE_M)
                {
                    request_mode = RobotMode::MARCH;
                }
            }

            // 键盘线程只提交请求，真正的 set_mode() 由物理线程执行。
            if (request_mode.has_value())
            {
                std::lock_guard<std::mutex> lock(state_mutex);
                pending_mode = request_mode;
                request_mode.reset();
            }
        }

        // 获取实际绘制尺寸，适配窗口缩放和高 DPI 屏幕。
        int width = 0;
        int height = 0;
        SDL_GL_GetDrawableSize(window, &width, &height);

        if (width <= 0 || height <= 0)
        {
            continue;
        }

        // 构造场景时需要读取 data，因此必须与物理线程互斥。
        {
            std::lock_guard<std::mutex> lock(state_mutex);

            camera.lookat[0] = simulator.data()->qpos[0];
            camera.lookat[1] = simulator.data()->qpos[1];
            camera.lookat[2] = simulator.data()->qpos[2];

            mjv_updateScene(
                simulator.model(),
                simulator.data(),
                &option,
                &perturb,
                &camera,
                mjCAT_ALL,
                &scene
            );
        }

        // 渲染只读取已经构造好的 scene，不需要继续持有状态锁。
        mjrRect viewport;
        viewport.left = 0;
        viewport.bottom = 0;
        viewport.width = width;
        viewport.height = height;

        mjr_render(viewport, &scene, &render_context);
        SDL_GL_SwapWindow(window);
    }

    // 先停止并回收物理线程，再释放仿真器和渲染资源。
    running = false;
    physics_thread.join();

    mjv_freeScene(&scene);
    mjr_freeContext(&render_context);
    SDL_GL_DeleteContext(gl_context);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
