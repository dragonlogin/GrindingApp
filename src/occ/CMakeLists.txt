set(HEADERS
    GrindingOccExport.h
    StlLoader.h
    StepImporter.h
    RobotDisplay.h
)

set(SOURCES
    StlLoader.cpp
    StepImporter.cpp
    RobotDisplay.cpp
)

add_library(GrindingOcc SHARED ${HEADERS} ${SOURCES})

target_compile_definitions(GrindingOcc PRIVATE GRINDING_OCC_BUILDING_DLL)

# Expose src/occ/; inherits src/ from GrindingCore
target_include_directories(GrindingOcc PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    $<BUILD_INTERFACE:${OpenCASCADE_INCLUDE_DIR}>
)

target_link_libraries(GrindingOcc PUBLIC
    GrindingCore
    TKernel TKMath TKG2d TKG3d TKGeomBase TKGeomAlgo TKBRep TKTopAlgo
    TKV3d TKOpenGl TKService TKMeshVS
    TKPrim TKBool TKBO TKFeat TKFillet TKOffset TKShHealing
    TKCAF TKCDF TKStd TKMesh TKRWMesh
    TKDESTEP TKDESTL
)

set_target_properties(GrindingOcc PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
)
