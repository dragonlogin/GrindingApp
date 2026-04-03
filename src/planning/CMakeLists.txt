set(TargetName SSPlanning)

set(Headers
    GrindingPlanningExport.h
    PlanningTypes.h
    IKinematicsService.h
    ITrajectoryPlanner.h
    ICollisionChecker.h
    CartesianTrajectoryPlanner.h
)
 

set(Sources
    CartesianTrajectoryPlanner.cpp
)
 
add_library(${TargetName} SHARED ${Headers} ${Sources})
 
target_compile_definitions(${TargetName} 
    PRIVATE 
    GRINDING_PLANNING_BUILDING_DLL
)
 
target_include_directories(${TargetName}  
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(${TargetName}
    PUBLIC
    SSFoundation
    SSDomain
    GrindingCore
    GrindingKinematics
    SSGeometry
)

 
set_target_properties(${TargetName} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
)