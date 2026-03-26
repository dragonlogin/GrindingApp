set(HEADERS
    GrindingUIExport.h
    Commands.h
    TrajectoryPlanner.h
)

set(MOC_HEADER
    MainWindow.h
    OcctViewWidget.h
    JogPanel.h
    RobotController.h
    TrajectoryPanel.h
    TrajectoryPlayer.h
)

qt5_wrap_cpp(MOC_SOURCES ${MOC_HEADER})

set(SOURCES
    MainWindow.cpp
    OcctViewWidget.cpp
    JogPanel.cpp
    RobotController.cpp
    TrajectoryPlanner.cpp
    TrajectoryPanel.cpp
    TrajectoryPlayer.cpp
    Commands.cpp
    ${MOC_SOURCES}
)

add_library(GrindingUI SHARED ${HEADERS} ${MOC_HEADER} ${SOURCES})

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
