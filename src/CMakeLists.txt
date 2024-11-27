add_executable(kinematics_demo "main.cpp"
        chainGeometry.cpp
        sceneManager.cpp
        chainKinematics.cpp
        uiManager.cpp
        animationLoop.cpp
)
target_include_directories(kinematics_demo PUBLIC "${CMAKE_SOURCE_DIR}/include")