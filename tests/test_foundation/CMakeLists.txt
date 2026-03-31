set(TestName TestFoundtion)

set(Sources
    TestFoundation.cpp
)

add_executable(${TestName} ${Sources})

target_link_libraries(${TestName} 
    PRIVATE
    SSFoundation
    GrindingUtils
    Qt5::Test
)

set_target_properties(${TestName}  PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

add_test(NAME ${TestName} COMMAND ${TestName})
set_tests_properties(${TestName} PROPERTIES
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)
