cmake_minimum_required(VERSION 3.5)
project(rl_sar VERSION 3.0.0 LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

string(TOUPPER "${CMAKE_BUILD_TYPE}" UPPER_BUILD_TYPE)
if(UPPER_BUILD_TYPE STREQUAL "DEBUG")
    add_compile_options(-g)
endif()

# Prefer ROS1 / Noetic
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    controller_manager
    joint_state_controller
    gazebo_ros
    std_msgs
    geometry_msgs
    std_srvs
    robot_msgs
)

catkin_package(
    CATKIN_DEPENDS
    roscpp
    rospy
    controller_manager
    joint_state_controller
    gazebo_ros
    std_msgs
    geometry_msgs
    std_srvs
    robot_msgs
)

# Other dependencies
find_package(Torch REQUIRED)
find_package(TBB REQUIRED)
find_package(Threads REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)
find_package(gazebo REQUIRED)
find_package(yaml-cpp REQUIRED)

# Support both modern YAML CMake imported target (yaml-cpp::yaml-cpp)
# and older FindYAML-cpp variables. Set YAML_CPP_TARGET to a usable value.
if(TARGET yaml-cpp::yaml-cpp)
    set(YAML_CPP_TARGET yaml-cpp::yaml-cpp)
elseif(DEFINED YAML_CPP_LIBRARIES)
    set(YAML_CPP_TARGET ${YAML_CPP_LIBRARIES})
elseif(DEFINED yaml-cpp_LIBRARIES)
    set(YAML_CPP_TARGET ${yaml-cpp_LIBRARIES})
else()
    # fallback: try to find a library name
    find_library(YAML_CPP_LIB NAMES yaml-cpp yamlcpp PATHS /usr/lib /usr/local/lib /usr/lib/x86_64-linux-gnu)
    if(YAML_CPP_LIB)
        set(YAML_CPP_TARGET ${YAML_CPP_LIB})
    else()
        message(FATAL_ERROR "yaml-cpp target not found; please install yaml-cpp or update CMakeLists.")
    endif()
endif()

# Expose CMAKE source dir to C++ (so legacy code using CMAKE_CURRENT_SOURCE_DIR compiles)
# This defines a C++ macro CMAKE_CURRENT_SOURCE_DIR with a string value of the cmake source dir.
add_compile_definitions(CMAKE_CURRENT_SOURCE_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}\")

# Include directories
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/library/core/matplotlibcpp
    ${CMAKE_CURRENT_SOURCE_DIR}/library/core/observation_buffer
    ${CMAKE_CURRENT_SOURCE_DIR}/library/core/rl_sdk
    ${CMAKE_CURRENT_SOURCE_DIR}/library/core/loop
    ${CMAKE_CURRENT_SOURCE_DIR}/library/core/fsm
    ${CMAKE_CURRENT_SOURCE_DIR}/policy
)

link_directories(/usr/local/lib)

# Architecture-specific settings
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
    message(STATUS "Current system architecture: ${CMAKE_SYSTEM_PROCESSOR}")
    set(ARCH_DIR "aarch64")
    set(UNITREE_LIB "libunitree_legged_sdk_arm64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|amd64|AMD64")
    message(STATUS "Current system architecture: ${CMAKE_SYSTEM_PROCESSOR}")
    set(ARCH_DIR "x86_64")
    set(UNITREE_LIB "libunitree_legged_sdk_amd64")
else()
    message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

# unitree_legged_sdk-3.2 (imported shared library for rl_real_a1)
add_library(unitree_legged_sdk SHARED IMPORTED)
set_target_properties(unitree_legged_sdk PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/library/thirdparty/unitree_legged_sdk-3.2/include"
    IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/library/thirdparty/unitree_legged_sdk-3.2/lib/${UNITREE_LIB}.so"
)
set(UNITREE_A1_LIBS Threads::Threads unitree_legged_sdk lcm)



# Core libraries
add_library(rl_sdk library/core/rl_sdk/rl_sdk.cpp)
set_target_properties(rl_sdk PROPERTIES
    CXX_STANDARD 14
    CXX_STANDARD_REQUIRED ON
)
target_include_directories(rl_sdk PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(rl_sdk PUBLIC
    "${TORCH_LIBRARIES}"
    Python3::Python
    Python3::Module
    TBB::tbb
    ${YAML_CPP_TARGET}
    ${catkin_LIBRARIES}
)

# Optionally link NumPy if found via Python3
if(Python3_NumPy_FOUND)
    target_link_libraries(rl_sdk PUBLIC Python3::NumPy)
else()
    target_compile_definitions(rl_sdk PUBLIC WITHOUT_NUMPY)
endif()

add_library(observation_buffer library/core/observation_buffer/observation_buffer.cpp)
target_link_libraries(observation_buffer PUBLIC "${TORCH_LIBRARIES}")
set_target_properties(observation_buffer PROPERTIES
    CXX_STANDARD 14
    CXX_STANDARD_REQUIRED ON
)

# Executables
add_executable(rl_sim src/rl_sim.cpp)
target_link_libraries(rl_sim
    rl_sdk
    observation_buffer
    ${YAML_CPP_TARGET}
    Threads::Threads
    ${catkin_LIBRARIES}
)

add_executable(rl_real_a1 src/rl_real_a1.cpp)
target_link_libraries(rl_real_a1
    ${UNITREE_A1_LIBS}
    rl_sdk
    observation_buffer
    ${YAML_CPP_TARGET}
    ${catkin_LIBRARIES}
)

# Install rules (standard ROS1 layout)
install(TARGETS rl_sim rl_real_a1
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Optional: install python scripts if present
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/scripts/actuator_net.py")
    catkin_install_python(PROGRAMS scripts/actuator_net.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endif()