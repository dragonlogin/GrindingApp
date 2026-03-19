cmake_minimum_required(VERSION 3.20)

set(CMAKE_TOOLCHAIN_FILE "C:/vcpkg/scripts/buildsystems/vcpkg.cmake")
set(CMAKE_PREFIX_PATH "C:/vcpkg/installed/x64-windows/share/cmake" CACHE PATH "")
project(GrindingApp VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# Windows MSVC 编译选项
if(MSVC)
    #add_compile_options(/utf-8 /W3)
    add_compile_definitions(NOMINMAX _USE_MATH_DEFINES WIN32_LEAN_AND_MEAN)
endif()

# 查找依赖
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets OpenGL)
find_package(Qt5LinguistTools REQUIRED)
find_package(OpenCASCADE REQUIRED)

# 收集源文件
set(SOURCES
    src/main.cpp
    src/MainWindow.cpp
    src/MainWindow.h
)
set(TS_FILES translations/zh_CN.ts)

# 生成/更新 .ts 文件
qt5_create_translation(QM_FILES 
    ${SOURCES}
    ${TS_FILES}
)


add_executable(GrindingApp 
    ${QM_FILES}
    ${SOURCES}
)

target_include_directories(GrindingApp PRIVATE
    src
    ${OpenCASCADE_INCLUDE_DIR}
)

target_link_libraries(GrindingApp PRIVATE
    Qt5::Core
    Qt5::Gui
    Qt5::Widgets
    Qt5::OpenGL
    TKBRep
    TKGeomBase
    TKMath
    TKG3d
    TKG2d
    TKTopAlgo
    TKPrim
)

# 设置输出目录
set_target_properties(GrindingApp PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)
