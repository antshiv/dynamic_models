cmake_minimum_required(VERSION 3.10)
project(DynamicModels LANGUAGES C)

set(DM_IS_TOPLEVEL OFF)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(DM_IS_TOPLEVEL ON)
endif()

option(DM_BUILD_EXAMPLES "Build dynamic_models examples" ${DM_IS_TOPLEVEL})
option(DM_BUILD_TESTS "Build dynamic_models tests" ${DM_IS_TOPLEVEL})

# Enable strict C standard enforcement
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

add_compile_definitions(_GNU_SOURCE)

# Add compile options for debugging
add_compile_options(-g -Wall -Wextra -Wpedantic)

# Project structure
set(PROJECT_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
set(INCLUDE_DIR ${PROJECT_ROOT}/include)
set(SRC_DIR ${PROJECT_ROOT}/src)
set(TEST_DIR ${PROJECT_ROOT}/tests)
set(EXAMPLES_DIR ${PROJECT_ROOT}/examples)
set(EXTERNAL_DIR ${PROJECT_ROOT}/external)

# Include directories
include_directories(${INCLUDE_DIR})

# Pull in attitude math library if the submodule is present
set(ATTITUDE_MATH_DIR ${EXTERNAL_DIR}/attitudeMathLibrary)
if(EXISTS ${ATTITUDE_MATH_DIR}/CMakeLists.txt)
    if(NOT TARGET attitude)
        add_subdirectory(${ATTITUDE_MATH_DIR} ${CMAKE_CURRENT_BINARY_DIR}/attitudeMathLibrary)
    endif()
    include_directories(${ATTITUDE_MATH_DIR}/include)
    set(HAVE_ATTITUDE_LIB ON)
else()
    set(HAVE_ATTITUDE_LIB OFF)
endif()

# Electric-machine equations remain owned and tested by motorDynamics.
set(MOTOR_DYNAMICS_DIR ${EXTERNAL_DIR}/motorDynamics)
if(EXISTS ${MOTOR_DYNAMICS_DIR}/CMakeLists.txt)
    if(NOT TARGET motor_dynamics)
        add_subdirectory(${MOTOR_DYNAMICS_DIR}
                         ${CMAKE_CURRENT_BINARY_DIR}/motorDynamics)
    endif()
    set(HAVE_MOTOR_DYNAMICS ON)
else()
    set(HAVE_MOTOR_DYNAMICS OFF)
endif()

# Add source files for subsystems
set(SRC_FILES
    ${SRC_DIR}/drone/physics_model.c
    ${SRC_DIR}/motor/motor_dynamics.c
    ${SRC_DIR}/utilities/numerical_solver.c
    ${SRC_DIR}/linear/point_mass_linear.c
    ${SRC_DIR}/linear/mass_spring_linear.c
    ${SRC_DIR}/linear/spring_pendulum_linear.c
    ${SRC_DIR}/linear/pendulum_linear.c
    ${SRC_DIR}/linear/inverted_pendulum_linear.c
    ${SRC_DIR}/linear/double_pendulum_linear.c
)

# Placeholder subsystem files remain in the tree as planned work. Do not compile
# zero-byte translation units: strict ISO C builds correctly reject them.

add_library(dynamic_models STATIC ${SRC_FILES})
target_include_directories(dynamic_models PUBLIC ${INCLUDE_DIR})

if(NOT HAVE_MOTOR_DYNAMICS)
    message(FATAL_ERROR
        "motorDynamics submodule is required; run git submodule update --init --recursive")
endif()
target_link_libraries(dynamic_models PUBLIC motorDynamics::motor_dynamics)

if(HAVE_ATTITUDE_LIB)
    target_link_libraries(dynamic_models PUBLIC attitude)
endif()

if(MSVC)
    target_compile_definitions(dynamic_models PUBLIC _USE_MATH_DEFINES)
else()
    target_link_libraries(dynamic_models PUBLIC m)
endif()

if(DM_BUILD_EXAMPLES)
    add_executable(point_mass_example ${EXAMPLES_DIR}/point_mass/main.c)
    target_link_libraries(point_mass_example PRIVATE dynamic_models)

    add_executable(spring_pendulum_example ${EXAMPLES_DIR}/spring_pendulum/main.c)
    target_link_libraries(spring_pendulum_example PRIVATE dynamic_models)

    add_executable(double_pendulum_example ${EXAMPLES_DIR}/double_pendulum/main.c)
    target_link_libraries(double_pendulum_example PRIVATE dynamic_models)

    add_executable(inverted_pendulum_example ${EXAMPLES_DIR}/inverted_pendulum/main.c)
    target_link_libraries(inverted_pendulum_example PRIVATE dynamic_models)

    add_executable(drone_hover_example ${EXAMPLES_DIR}/drone/example_physics_model.c)
    target_link_libraries(drone_hover_example PRIVATE dynamic_models)
endif()

if(DM_BUILD_TESTS AND EXISTS ${TEST_DIR}/CMakeLists.txt)
    enable_testing()
    add_subdirectory(${TEST_DIR} ${CMAKE_CURRENT_BINARY_DIR}/tests)
endif()

if(DM_BUILD_TESTS AND EXISTS ${CMAKE_CURRENT_BINARY_DIR}/CTestCustom.cmake)
    file(REMOVE ${CMAKE_CURRENT_BINARY_DIR}/CTestCustom.cmake)
endif()

# Add examples
#add_executable(dynamic_models_example ${EXAMPLES_DIR}/main.c)
#target_link_libraries(dynamic_models_example dynamic_models)
