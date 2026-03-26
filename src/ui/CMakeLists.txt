set(HEADERS
    GrindingUIExport.h
    MainWindow.h
    OcctViewWidget.h
    JogPanel.h
)

set(SOURCES
    MainWindow.cpp
    OcctViewWidget.cpp
    JogPanel.cpp
)

add_library(GrindingUI SHARED ${HEADERS} ${SOURCES})

target_compile_definitions(GrindingUI PRIVATE GRINDING_UI_BUILDING_DLL)

# Expose src/ui/; inherits src/, src/occ/, src/kdl/ from upstream deps
target_include_directories(GrindingUI PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(GrindingUI PUBLIC
    GrindingCore
    GrindingOcc
    GrindingKinematics
    Qt5::Gui
    Qt5::Widgets
    Qt5::OpenGL
)

set_target_properties(GrindingUI PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
)
