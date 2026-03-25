add_library(GrindingOcc STATIC
    StlLoader.h
    StepImporter.h
    RobotDisplay.h
    StlLoader.cpp
    StepImporter.cpp
    RobotDisplay.cpp
)


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
