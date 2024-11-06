add_executable(testFunctions testFunctions.cpp)
add_test(NAME testFunctions COMMAND testFunctions)
target_link_libraries(testFunctions PUBLIC Catch2::Catch2WithMain)
target_include_directories(testFunctions PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../include")