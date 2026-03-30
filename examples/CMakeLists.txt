# KDL 运动学示例
add_executable(kdl_ik_test kdl_ik_test.cpp)

# 查找依赖包
find_package(orocos_kdl CONFIG REQUIRED)
find_package(Eigen3 REQUIRED)

target_link_libraries(kdl_ik_test PRIVATE
    orocos-kdl
    Eigen3::Eigen
)

set_target_properties(kdl_ik_test PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/examples"
)
