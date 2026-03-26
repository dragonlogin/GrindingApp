set(HEADERS
    GrindingKinematicsExport.h
    IKinematicsSolver.h
    EigenSolver.h
    RobotKinematics.h
)

set(SOURCES
    EigenSolver.cpp
    RobotKinematics.cpp
)

add_library(GrindingKinematics SHARED ${HEADERS} ${SOURCES})

target_compile_definitions(GrindingKinematics PRIVATE GRINDING_KINEMATICS_BUILDING_DLL)

# Expose src/kinematics/; inherits src/ from GrindingCore
target_include_directories(GrindingKinematics PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    $<BUILD_INTERFACE:${OpenCASCADE_INCLUDE_DIR}>
)

target_link_libraries(GrindingKinematics PUBLIC
    GrindingCore
    Eigen3::Eigen
    TKernel TKMath TKG3d TKBRep
)

set_target_properties(GrindingKinematics PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
)