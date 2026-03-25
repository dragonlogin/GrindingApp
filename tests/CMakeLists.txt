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

add_test(NAME TestRobotKinematics COMMAND TestRobotKinematics)
