set(TargetName SSQtDesktop)

set(Headers
    QtScenePresenter.h
    OccSceneRenderer.h
)
 
set(Sources
)
 
add_library(${TargetName} INTERFACE)
# target_sources(
#     INTERFACE 
#     ${Headers}
#     ${Sources}
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
    SSApplication
)

 
set_target_properties(${TargetName} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
)