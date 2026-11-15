cmake_minimum_required(VERSION 3.16)
project(dog_walk_simulation LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(MUJOCO_ROOT "" CACHE PATH "MuJoCo installation or Python mujoco package directory")
option(DOG_SIM_VIEWER "Build the GLFW viewer when available" ON)

# The Python wheel also contains the complete C/C++ headers and shared library.
if(NOT MUJOCO_ROOT)
    find_package(Python3 COMPONENTS Interpreter QUIET)
    foreach(python IN ITEMS "${Python3_EXECUTABLE}" "$ENV{CONDA_PREFIX}/bin/python"
            "$ENV{HOME}/miniconda3/envs/mujoco/bin/python"
            "$ENV{HOME}/anaconda3/envs/mujoco/bin/python")
        if(EXISTS "${python}")
            execute_process(COMMAND "${python}" -B -c
                "import pathlib,mujoco; print(pathlib.Path(mujoco.__file__).parent)"
                RESULT_VARIABLE found OUTPUT_VARIABLE package_root
                OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
            if(found EQUAL 0)
                set(MUJOCO_ROOT "${package_root}")
                break()
            endif()
        endif()
    endforeach()
endif()
find_path(MUJOCO_INCLUDE_DIR mujoco/mujoco.h HINTS "${MUJOCO_ROOT}/include" REQUIRED)
find_library(MUJOCO_LIBRARY NAMES mujoco HINTS "${MUJOCO_ROOT}" "${MUJOCO_ROOT}/lib")
if(NOT MUJOCO_LIBRARY)
    file(GLOB libraries "${MUJOCO_ROOT}/libmujoco.so.*" "${MUJOCO_ROOT}/lib/libmujoco.so.*"
        "${MUJOCO_ROOT}/libmujoco.*.dylib" "${MUJOCO_ROOT}/lib/libmujoco.*.dylib")
    list(SORT libraries COMPARE NATURAL ORDER DESCENDING)
    if(libraries)
        list(GET libraries 0 MUJOCO_LIBRARY)
    else()
        message(FATAL_ERROR "MuJoCo not found. Set -DMUJOCO_ROOT=/path/to/mujoco")
    endif()
endif()
message(STATUS "MuJoCo: ${MUJOCO_LIBRARY}")
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Threads REQUIRED)

add_library(walk_core src/gait.cpp src/simulator.cpp)
target_include_directories(walk_core PUBLIC include "${MUJOCO_INCLUDE_DIR}")
target_link_libraries(walk_core PUBLIC "${MUJOCO_LIBRARY}" Eigen3::Eigen)
target_compile_options(walk_core PRIVATE -Wall -Wextra -Wpedantic)

add_executable(walk_sim src/main.cpp)
target_link_libraries(walk_sim PRIVATE walk_core Threads::Threads)
target_compile_definitions(walk_sim PRIVATE
    DOG_DEFAULT_MODEL="${CMAKE_CURRENT_SOURCE_DIR}/generated_robot-mjcf_realModel/robot.xml")
target_compile_options(walk_sim PRIVATE -Wall -Wextra -Wpedantic)
if(DOG_SIM_VIEWER)
    find_package(glfw3 QUIET)
    find_package(OpenGL QUIET)
    if(TARGET glfw AND OpenGL_FOUND)
        target_sources(walk_sim PRIVATE src/viewer.cpp)
        target_link_libraries(walk_sim PRIVATE glfw OpenGL::GL)
        target_compile_definitions(walk_sim PRIVATE DOG_SIM_HAS_VIEWER=1)
    else()
        message(STATUS "GLFW/OpenGL unavailable; use walk_sim --headless")
    endif()
endif()

include(CTest)
if(BUILD_TESTING)
    add_executable(walk_sim_check tests/check_simulation.cpp)
    target_link_libraries(walk_sim_check PRIVATE walk_core)
    target_compile_options(walk_sim_check PRIVATE -Wall -Wextra -Wpedantic)
    add_test(NAME walk_simulation COMMAND walk_sim_check
        "${CMAKE_CURRENT_SOURCE_DIR}/generated_robot-mjcf_realModel/robot.xml")
endif()
