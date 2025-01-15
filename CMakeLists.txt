cmake_minimum_required(VERSION 3.10)
project(DynamicModels LANGUAGES C)

# Enable strict C standard enforcement
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

# Add compile options for debugging
add_compile_options(-g -Wall -Wextra -Wpedantic)

# Project structure
set(PROJECT_ROOT ${CMAKE_SOURCE_DIR})
set(INCLUDE_DIR ${PROJECT_ROOT}/include)
set(SRC_DIR ${PROJECT_ROOT}/src)
set(TEST_DIR ${PROJECT_ROOT}/tests)
set(EXAMPLES_DIR ${PROJECT_ROOT}/examples)

# Include directories
include_directories(${INCLUDE_DIR})

# Add source files for subsystems
set(SRC_FILES
    ${SRC_DIR}/drone/physics_model.c
    ${SRC_DIR}/drone/neural_state_space.c
    ${SRC_DIR}/drone/system_identification.c
    ${SRC_DIR}/motor/motor_dynamics.c
    ${SRC_DIR}/battery/battery_model.c
    ${SRC_DIR}/battery/battery_management.c
    ${SRC_DIR}/utilities/numerical_solver.c
)

# Create a shared library for dynamic models
#add_library(dynamic_models ${SRC_FILES})

# Add tests
#enable_testing()
#add_subdirectory(${TEST_DIR})

# Add examples
#add_executable(dynamic_models_example ${EXAMPLES_DIR}/main.c)
#target_link_libraries(dynamic_models_example dynamic_models)
