set(TargetName SSFoundation)

set(Headers
    Result.h
    Pose.h
    JointState.h
    Ids.h
    Error.h
)
 
set(Sources
)
 
add_library(${TargetName} INTERFACE)
# target_sources(
#     PUBLIC
#     ${Headers}
# )
 
target_compile_definitions(${TargetName} 
    PRIVATE 
    #GRINDING_UTILS_BUILDING_DLL
)
 
target_include_directories(${TargetName}  
    INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(${TargetName}
    INTERFACE
)

 
set_target_properties(${TargetName} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
)