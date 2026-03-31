set(HEADERS
    GrindingKinematicsExport.h
    IKinematicsSolver.h
    EigenSolver.h
    KdlChainBuilder.h
    KdlSolver.h
    RobotKinematics.h
)

set(SOURCES
    EigenSolver.cpp
    KdlChainBuilder.cpp
    KdlSolver.cpp
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
    SSFoundation
    GrindingCore
    Eigen3::Eigen
    orocos-kdl
    urdfdom::urdfdom_model
    urdfdom::urdfdom_world
    urdfdom::urdfdom_sensor
    urdfdom::urdfdom_model_state
    assimp::assimp
    TKernel TKMath TKG3d TKBRep
)

set_target_properties(GrindingKinematics PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
)
