add_executable(TestRobotKinematics
    TestRobotKinematics.cpp
)

# GrindingAppLib already exposes all include dirs and link deps via PUBLIC
target_link_libraries(TestRobotKinematics PRIVATE
    GrindingAppLib
    Qt5::Test
)

add_test(NAME TestRobotKinematics COMMAND TestRobotKinematics)
