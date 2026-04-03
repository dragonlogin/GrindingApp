set(TargetName SSDomain)

set(Headers
    DomainExport.h
    Waypoint.h
    WaypointSet.h
    Trajectory.h
)

set(Sources
    DomainExports.cpp
)

add_library(${TargetName} SHARED ${Headers} ${Sources})

target_compile_definitions(${TargetName}
    PRIVATE
    DOMAIN_EXPORT_DLL
)

target_include_directories(${TargetName}
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(${TargetName}
    PUBLIC
    SSFoundation
)

set_target_properties(${TargetName} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
)
