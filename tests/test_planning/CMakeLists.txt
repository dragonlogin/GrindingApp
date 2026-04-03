set(TestName TestPlanning)

set(Sources
    TestKdlKinematicsService.cpp
)

add_executable(${TestName} ${Sources})

target_include_directories(${TestName} PRIVATE
    ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(${TestName}
    PRIVATE
    SSPlanning
    GrindingKinematics
    Qt5::Test
)

set_target_properties(${TestName} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

add_test(NAME ${TestName} COMMAND ${TestName})
set_tests_properties(${TestName} PROPERTIES
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)
