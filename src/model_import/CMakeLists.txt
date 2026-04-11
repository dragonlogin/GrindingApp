set(TargetName SSModelImport)

set(Headers
    ModelImportExport.h
    RobotDefinition.h
    ToolDefinition.h
    IRobotImporter.h
    IToolImporter.h
    UrdfRobotImporter.h
    UrdfToolImporter.h
)

set(Sources
    UrdfRobotImporter.cpp
    UrdfToolImporter.cpp
)

add_library(${TargetName} SHARED ${Headers} ${Sources})

target_compile_definitions(${TargetName}
    PRIVATE
    MODEL_IMPORT_BUILDING_DLL
)

target_include_directories(${TargetName}
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(${TargetName}
    PUBLIC
    SSDomain
    GrindingCore
)

set_target_properties(${TargetName} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
)