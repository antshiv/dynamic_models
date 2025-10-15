# Enable testing for the project
set(TEST_SOURCES
    ${TEST_DIR}/drone/test_physics_model.c
    ${TEST_DIR}/drone/test_neural_state_space.c
    ${TEST_DIR}/battery/test_battery_model.c
    ${TEST_DIR}/battery/test_battery_management.c
    ${TEST_DIR}/motor/test_motor_dynamics.c
    ${TEST_DIR}/utilities/test_numerical_solvers.c
    ${TEST_DIR}/linear/test_point_mass_linear.c
    ${TEST_DIR}/linear/test_mass_spring_linear.c
    ${TEST_DIR}/linear/test_spring_pendulum_linear.c
    ${TEST_DIR}/linear/test_pendulum_linear.c
    ${TEST_DIR}/linear/test_inverted_pendulum_linear.c
    ${TEST_DIR}/linear/test_double_pendulum_linear.c
)

# Add each test as an executable
foreach(test_source ${TEST_SOURCES})
    get_filename_component(test_name ${test_source} NAME_WE)
    add_executable(${test_name} ${test_source})
    target_link_libraries(${test_name} dynamic_models)
    add_test(NAME ${test_name} COMMAND ${test_name})
endforeach()
