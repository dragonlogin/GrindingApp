add_library(GrindingKinematics STATIC
    RobotKinematics.h
    RobotKinematics.cpp
)

target_compile_definitions(GrindingKinematics PRIVATE GRINDINGAPPLIB_EXPORTS)

# Expose src/kdl/; inherits src/ from GrindingCore
target_include_directories(GrindingKinematics PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    $<BUILD_INTERFACE:${OpenCASCADE_INCLUDE_DIR}>
)

target_link_libraries(GrindingKinematics PUBLIC
    GrindingCore
    orocos-kdl
    Eigen3::Eigen
    TKernel TKMath TKG3d TKBRep
)
