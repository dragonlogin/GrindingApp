set(HEADERS
    GrindingCoreExport.h
    RbXmlParser.h
)

set(SOURCES
    RbXmlParser.cpp
)

add_library(GrindingCore SHARED ${HEADERS} ${SOURCES})

target_compile_definitions(GrindingCore PRIVATE GRINDING_CORE_BUILDING_DLL)

# Expose src/ so all consumers can #include "RbXmlParser.h" / "*Export.h"
target_include_directories(GrindingCore PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(GrindingCore 
    PUBLIC
    GrindingUtils
    Qt5::Xml
)

set_target_properties(GrindingCore PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
)
