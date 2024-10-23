cmake_minimum_required(VERSION 3.19)
project(Forward_Inverse_Kinematics)

set(CMAKE_CXX_STANDARD 20)

include(FetchContent)

set(THREEPP_BUILD_TESTS OFF)
set(THREEPP_BUILD_EXAMPLES OFF)
FetchContent_Declare(
        threepp
        GIT_REPOSITORY https://github.com/markaren/threepp.git
        GIT_TAG 2e2397e0b7f0e639f254ea3046b70e53a71f0387
)
FetchContent_MakeAvailable(threepp)

add_subdirectory(external)

add_executable(kinematics_demo main.cpp)
target_link_libraries(kinematics_demo PRIVATE threepp::threepp imgui::imgui)
