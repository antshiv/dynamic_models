cmake_minimum_required(VERSION 3.10)
project(DynamicModels LANGUAGES C)

# Enable strict C standard enforcement
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

add_compile_definitions(_GNU_SOURCE)

# Add compile options for debugging
add_compile_options(-g -Wall -Wextra -Wpedantic)

# Project structure
set(PROJECT_ROOT ${CMAKE_SOURCE_DIR})
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
    add_subdirectory(${ATTITUDE_MATH_DIR} ${CMAKE_BINARY_DIR}/attitudeMathLibrary)
    include_directories(${ATTITUDE_MATH_DIR}/include)
    set(HAVE_ATTITUDE_LIB ON)
else()
    set(HAVE_ATTITUDE_LIB OFF)
endif()

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

add_library(dynamic_models STATIC ${SRC_FILES})
target_include_directories(dynamic_models PUBLIC ${INCLUDE_DIR})

if(HAVE_ATTITUDE_LIB)
    target_link_libraries(dynamic_models PUBLIC attitude)
endif()

if(MSVC)
    target_compile_definitions(dynamic_models PUBLIC _USE_MATH_DEFINES)
else()
    target_link_libraries(dynamic_models PUBLIC m)
endif()

# Add tests
#enable_testing()
#add_subdirectory(${TEST_DIR})

# Add examples
#add_executable(dynamic_models_example ${EXAMPLES_DIR}/main.c)
#target_link_libraries(dynamic_models_example dynamic_models)
