cmake_minimum_required(VERSION 3.19)
project(Forward_Inverse_Kinematics)

set(CMAKE_CXX_STANDARD 20)

option(PROJECT_BUILD_TESTS "test functions" ON)

include(FetchContent)

set(THREEPP_BUILD_TESTS OFF)
set(THREEPP_BUILD_EXAMPLES OFF)
FetchContent_Declare(
        threepp
        GIT_REPOSITORY https://github.com/markaren/threepp.git
        GIT_TAG 2e2397e0b7f0e639f254ea3046b70e53a71f0387
)
FetchContent_MakeAvailable(threepp)

message(PROJECT_BUILD_TESTS=${PROJECT_BUILD_TESTS})

if (PROJECT_BUILD_TESTS)
    FetchContent_Declare(
            Catch2
            GIT_REPOSITORY https://github.com/catchorg/Catch2.git
            GIT_TAG v3.7.1
    )
    FetchContent_MakeAvailable(Catch2)

    enable_testing()
    add_subdirectory(tests)
endif ()

add_subdirectory(external)
add_subdirectory(src)

target_link_libraries(kinematics_demo PRIVATE threepp::threepp imgui::imgui)


