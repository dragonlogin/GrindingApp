add_library(GrindingCore STATIC
    GrindingAppExport.h
    RbXmlParser.h
    RbXmlParser.cpp
)

target_compile_definitions(GrindingCore PRIVATE GRINDINGAPPLIB_EXPORTS)

# Expose src/ so all consumers can #include "RbXmlParser.h" / "GrindingAppExport.h"
target_include_directories(GrindingCore PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(GrindingCore PUBLIC
    Qt5::Core
    Qt5::Xml
)
