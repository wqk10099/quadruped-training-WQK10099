cmake_minimum_required(VERSION 3.8)
project(dog_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(my_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Python3 COMPONENTS Interpreter REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(SDL2 REQUIRED sdl2)

execute_process(
  COMMAND ${Python3_EXECUTABLE} -c
          "import mujoco, pathlib; print(pathlib.Path(mujoco.__file__).resolve().parent)"
  OUTPUT_VARIABLE MUJOCO_DIR
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
message(STATUS "MuJoCo 目录: ${MUJOCO_DIR}")

file(GLOB MUJOCO_LIBS "${MUJOCO_DIR}/libmujoco.so.*")
if(NOT MUJOCO_LIBS)
  message(FATAL_ERROR "在 ${MUJOCO_DIR} 没找到 libmujoco.so.*")
endif()
list(GET MUJOCO_LIBS 0 MUJOCO_LIB)
message(STATUS "MuJoCo 库: ${MUJOCO_LIB}")
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(sim src/sim_node.cpp src/simulator.cpp)
add_executable(controller src/controller_node.cpp src/controller.cpp)
ament_target_dependencies(sim rclcpp my_interfaces sensor_msgs)
ament_target_dependencies(controller rclcpp my_interfaces )
target_include_directories(sim PRIVATE "${MUJOCO_DIR}/include")
target_include_directories(sim PRIVATE ${SDL2_INCLUDE_DIRS})
target_link_libraries(sim ${SDL2_LIBRARIES} GL)
target_link_libraries(sim ${MUJOCO_LIBS})
set_target_properties(sim PROPERTIES
  BUILD_RPATH   "${MUJOCO_DIR}"
  INSTALL_RPATH "${MUJOCO_DIR}")
install(TARGETS
    sim
    controller
    DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
# 场景与网格模型随包安装，launch 通过 get_package_share_directory 定位，
# 这样仓库 clone 到任何路径都能直接运行，无需改绝对路径。
install(DIRECTORY scenes models
  DESTINATION share/${PROJECT_NAME})

ament_package()
