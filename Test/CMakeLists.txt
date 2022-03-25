set(tests gripper_test)


foreach(test ${tests})
    add_executable(${test} ${test}.cc)
endforeach()

target_link_libraries(gripper_test Franka::Franka)