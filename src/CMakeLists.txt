add_executable(kinematics_demo "main.cpp" "kinematicChain.cpp"
        geometryHelpers.cpp
)
target_include_directories(kinematics_demo PUBLIC "${CMAKE_SOURCE_DIR}/include")