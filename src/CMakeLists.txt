add_library(kinematicsLib
        animationLoop.cpp
        ChainGeometry.cpp
        ChainKinematics.cpp
        SceneManager.cpp
        UiManager.cpp)
target_link_libraries(kinematicsLib PRIVATE threepp::threepp imgui::imgui)
target_include_directories(kinematicsLib PUBLIC "${CMAKE_SOURCE_DIR}/include")