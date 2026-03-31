set(TargetName SSFoundation)

set(Headers
    FoundationExport.h
    Result.h
    Pose.h
    JointState.h
    Ids.h
    Error.h
    UnitTypes.h
    Conversions.h
)
 
set(Sources
    Conversions.cpp
)
 
add_library(${TargetName} SHARED "")
target_sources( ${TargetName}
    PRIVATE
    ${Headers}
    ${Sources}
)
 
target_compile_definitions(${TargetName} 
    PRIVATE 
    FOUNDATION_EXPORT_DLL
)
 
target_include_directories(${TargetName}  
    INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(${TargetName}
    PUBLIC
    GrindingUtils
    TKernel
    TKMath
    TKG3d
)

 
set_target_properties(${TargetName} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
)