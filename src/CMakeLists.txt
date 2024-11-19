add_executable(kinematics_demo "main.cpp"
        geometryHelpers.cpp
        sceneManager.cpp
        kinematicChain.cpp
        uiManager.cpp
        animationLoop.cpp
)
target_include_directories(kinematics_demo PUBLIC "${CMAKE_SOURCE_DIR}/include")