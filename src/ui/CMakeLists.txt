add_library(GrindingUI STATIC
    MainWindow.h
    OcctViewWidget.h
    MainWindow.cpp
    OcctViewWidget.cpp
)

target_compile_definitions(GrindingUI PRIVATE GRINDINGAPPLIB_EXPORTS)

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
