set(HEADERS
    GrindingUtilsExport.h
    Vector3d.h
    Q.h
)
 
set(SOURCES
    Q.cpp
)
 
add_library(GrindingUtils SHARED ${HEADERS} ${SOURCES})
 
target_compile_definitions(GrindingUtils PRIVATE GRINDING_UTILS_BUILDING_DLL)
 
target_include_directories(GrindingUtils PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(GrindingUtils 
    PUBLIC
    Qt5::Core
)

 
set_target_properties(GrindingUtils PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
)