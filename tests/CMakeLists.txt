set(SOURCES
    TestRobotKinematics.cpp
)

add_executable(TestRobotKinematics ${SOURCES})

# GrindingOcc and GrindingKinematics both transitively bring in GrindingCore
target_link_libraries(TestRobotKinematics PRIVATE
    GrindingOcc
    GrindingKinematics
    Qt5::Test
)

# Place test exe alongside DLLs so Windows DLL search finds them
set_target_properties(TestRobotKinematics PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

add_test(NAME TestRobotKinematics COMMAND TestRobotKinematics)
set_tests_properties(TestRobotKinematics PROPERTIES
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)
