cmake_minimum_required(VERSION 3.20)

set(CMAKE_TOOLCHAIN_FILE "C:/vcpkg/scripts/buildsystems/vcpkg.cmake")
set(CMAKE_PREFIX_PATH "C:/vcpkg/installed/x64-windows/share/cmake" CACHE PATH "")
project(GrindingApp VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# Windows MSVC compile options
if(MSVC)
    add_compile_definitions(NOMINMAX _USE_MATH_DEFINES WIN32_LEAN_AND_MEAN)
    add_compile_options(/utf-8 /nologo)
    set(CMAKE_CXX_FLAGS_DEBUG   "${CMAKE_CXX_FLAGS_DEBUG}   /utf-8")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /utf-8")
    set(ENV{CL_UTF8} 1)
endif()

# Dependencies
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets OpenGL Xml Test)
find_package(Qt5 OPTIONAL_COMPONENTS LinguistTools)
find_package(OpenCASCADE REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)

# ──────────────────────────────────────────────────────────────────
# GrindingAppLib — Shared library (DLL)
# Modules:
#   src/          — shared data types (RbXmlParser)
#   src/ui/       — Qt UI layer (MainWindow, OcctViewWidget)
#   src/occ/      — OpenCASCADE operations (StlLoader, StepImporter, RobotDisplay)
#   src/kdl/      — KDL kinematics (RobotKinematics)
# ──────────────────────────────────────────────────────────────────
set(LIB_HEADERS
    src/GrindingAppExport.h
    src/RbXmlParser.h
    src/ui/MainWindow.h
    src/ui/OcctViewWidget.h
    src/occ/StlLoader.h
    src/occ/StepImporter.h
    src/occ/RobotDisplay.h
    src/kdl/RobotKinematics.h
)

set(LIB_SOURCES
    src/RbXmlParser.cpp
    src/ui/MainWindow.cpp
    src/ui/OcctViewWidget.cpp
    src/occ/StlLoader.cpp
    src/occ/StepImporter.cpp
    src/occ/RobotDisplay.cpp
    src/kdl/RobotKinematics.cpp
)

source_group("Header Files"        FILES src/GrindingAppExport.h src/RbXmlParser.h)
source_group("Header Files\\ui"    FILES src/ui/MainWindow.h src/ui/OcctViewWidget.h)
source_group("Header Files\\occ"   FILES src/occ/StlLoader.h src/occ/StepImporter.h src/occ/RobotDisplay.h)
source_group("Header Files\\kdl"   FILES src/kdl/RobotKinematics.h)

set(TS_FILES translations/zh_CN.ts)

if(Qt5LinguistTools_FOUND)
    qt5_create_translation(QM_FILES
        ${LIB_HEADERS}
        ${LIB_SOURCES}
        ${TS_FILES}
    )
endif()

add_library(GrindingAppLib SHARED
    ${QM_FILES}
    ${LIB_HEADERS}
    ${LIB_SOURCES}
    resources/resources.qrc
)

target_compile_definitions(GrindingAppLib PRIVATE GRINDINGAPPLIB_EXPORTS)

# All module dirs exposed as PUBLIC so consumers (execute, tests) find headers
target_include_directories(GrindingAppLib PUBLIC
    src
    src/ui
    src/occ
    src/kdl
    $<BUILD_INTERFACE:${OpenCASCADE_INCLUDE_DIR}>
)

set(OCCT_LIBS
    TKernel TKMath TKG2d TKG3d TKGeomBase TKGeomAlgo TKBRep TKTopAlgo
    TKV3d TKOpenGl TKService TKMeshVS
    TKPrim TKBool TKBO TKFeat TKFillet TKOffset TKShHealing
    TKCAF TKCDF TKStd
    TKMesh TKRWMesh
    TKDESTEP TKDESTL
)

target_link_libraries(GrindingAppLib PUBLIC
    Qt5::Core
    Qt5::Gui
    Qt5::Widgets
    Qt5::OpenGL
    Qt5::Xml
    ${OCCT_LIBS}
    orocos-kdl
    Eigen3::Eigen
)

set_target_properties(GrindingAppLib PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
)

# ──────────────────────────────────────────────────────────────────
# GrindingApp — Executable
# ──────────────────────────────────────────────────────────────────
add_executable(GrindingApp
    execute/main.cpp
)

target_link_libraries(GrindingApp PRIVATE GrindingAppLib)

set_target_properties(GrindingApp PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

# ──────────────────────────────────────────────────────────────────
# Unit tests
# ──────────────────────────────────────────────────────────────────
enable_testing()
add_subdirectory(tests)
