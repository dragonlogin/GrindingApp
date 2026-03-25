add_executable(TestRobotKinematics
    TestRobotKinematics.cpp
)

# GrindingOcc and GrindingKinematics both transitively bring in GrindingCore
target_link_libraries(TestRobotKinematics PRIVATE
    GrindingOcc
    GrindingKinematics
    Qt5::Test
)

add_test(NAME TestRobotKinematics COMMAND TestRobotKinematics)
