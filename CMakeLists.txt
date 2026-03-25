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

# ── Sub-modules (STATIC libraries) ──────────────────────────────
#   GrindingCore        src/
#   GrindingOcc         src/occ/
#   GrindingKinematics  src/kdl/
#   GrindingUI          src/ui/
add_subdirectory(src)
add_subdirectory(src/occ)
add_subdirectory(src/kdl)
add_subdirectory(src/ui)

# ── Translation (optional) ───────────────────────────────────────
set(TS_FILES translations/zh_CN.ts)
if(Qt5LinguistTools_FOUND)
    set(ALL_HEADERS
        src/GrindingCoreExport.h
        src/RbXmlParser.h
        src/ui/MainWindow.h
        src/ui/OcctViewWidget.h
        src/occ/StlLoader.h
        src/occ/StepImporter.h
        src/occ/RobotDisplay.h
        src/kdl/RobotKinematics.h
    )
    set(ALL_SOURCES
        src/RbXmlParser.cpp
        src/ui/MainWindow.cpp
        src/ui/OcctViewWidget.cpp
        src/occ/StlLoader.cpp
        src/occ/StepImporter.cpp
        src/occ/RobotDisplay.cpp
        src/kdl/RobotKinematics.cpp
    )
    qt5_create_translation(QM_FILES
        ${ALL_HEADERS}
        ${ALL_SOURCES}
        ${TS_FILES}
    )
endif()

# ── GrindingApp (Executable) ─────────────────────────────────────
set(APP_SOURCES
    execute/main.cpp
    resources/resources.qrc
)

add_executable(GrindingApp ${QM_FILES} ${APP_SOURCES})

# Links all four static modules; PUBLIC deps propagate transitively
target_link_libraries(GrindingApp PRIVATE
    GrindingCore
    GrindingOcc
    GrindingKinematics
    GrindingUI
)

set_target_properties(GrindingApp PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

# ── Unit tests ───────────────────────────────────────────────────
enable_testing()
add_subdirectory(tests)
