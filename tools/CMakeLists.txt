cmake_minimum_required(VERSION 3.16)
project(dog_motor_check LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(MUJOCO_ROOT "" CACHE PATH "MuJoCo root containing include/ and libmujoco")

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_path(MUJOCO_INCLUDE_DIR mujoco/mujoco.h HINTS "${MUJOCO_ROOT}/include" REQUIRED)
find_library(MUJOCO_LIBRARY NAMES mujoco libmujoco.so.3.3.7
    HINTS "${MUJOCO_ROOT}" "${MUJOCO_ROOT}/lib" REQUIRED)

add_executable(check_motor_model check_motor_model.cpp)
target_include_directories(check_motor_model PRIVATE
    "${MUJOCO_INCLUDE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/../simulation/generated_robot-mjcf_v2")
target_link_libraries(check_motor_model PRIVATE "${MUJOCO_LIBRARY}" Eigen3::Eigen)
target_compile_options(check_motor_model PRIVATE -Wall -Wextra -Wpedantic)

add_executable(check_leg_kinematics check_leg_kinematics.cpp)
target_include_directories(check_leg_kinematics PRIVATE "${MUJOCO_INCLUDE_DIR}")
target_link_libraries(check_leg_kinematics PRIVATE "${MUJOCO_LIBRARY}" Eigen3::Eigen)
target_compile_options(check_leg_kinematics PRIVATE -Wall -Wextra -Wpedantic)

set(UNITREE_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../lib/unitree_actuator_sdk"
    CACHE PATH "Local Unitree actuator SDK root")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64|ARM64)$")
    set(UNITREE_LIBRARY_NAME UnitreeMotorSDK_Arm64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64|AMD64)$")
    set(UNITREE_LIBRARY_NAME UnitreeMotorSDK_Linux64)
else()
    message(FATAL_ERROR "No bundled Unitree SDK library for ${CMAKE_SYSTEM_PROCESSOR}")
endif()
find_library(UNITREE_MOTOR_LIBRARY NAMES ${UNITREE_LIBRARY_NAME}
    HINTS "${UNITREE_SDK_ROOT}/lib" NO_DEFAULT_PATH REQUIRED)
add_executable(sdk_sim_example sdk_sim_example.cpp)
target_include_directories(sdk_sim_example PRIVATE "${MUJOCO_INCLUDE_DIR}"
    "${UNITREE_SDK_ROOT}/include" "${CMAKE_CURRENT_SOURCE_DIR}/../simulation/generated_robot-mjcf_v2")
target_link_libraries(sdk_sim_example PRIVATE "${MUJOCO_LIBRARY}" "${UNITREE_MOTOR_LIBRARY}" Eigen3::Eigen)
target_compile_options(sdk_sim_example PRIVATE -Wall -Wextra -Wpedantic)

enable_testing()
add_test(NAME motor_model COMMAND check_motor_model
    "${CMAKE_CURRENT_SOURCE_DIR}/../simulation/generated_robot-mjcf_v2/robot.xml")
add_test(NAME sdk_sim_example COMMAND sdk_sim_example
    "${CMAKE_CURRENT_SOURCE_DIR}/../simulation/generated_robot-mjcf_v2/robot.xml")
add_test(NAME leg_kinematics COMMAND check_leg_kinematics
    "${CMAKE_CURRENT_SOURCE_DIR}/../simulation/generated_robot-mjcf_v2/robot.xml")
