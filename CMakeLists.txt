cmake_minimum_required(VERSION 3.16)
project(ik VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(IK_BUILD_TESTS "Build unit tests" ON)
option(IK_BUILD_VIEWER "Build SFML viewer target" OFF)

add_library(ik INTERFACE)
add_library(ik::ik ALIAS ik)
target_include_directories(ik INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

if(IK_BUILD_TESTS)
    enable_testing()

    add_executable(ik_test test.cpp)
    target_link_libraries(ik_test PRIVATE ik::ik)
    target_compile_options(ik_test PRIVATE
        $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wall>
        $<$<CXX_COMPILER_ID:MSVC>:/W4>
    )
    if(UNIX)
        target_link_libraries(ik_test PRIVATE pthread)
    endif()

    add_test(NAME ik_unit_tests COMMAND ik_test)
endif()

if(IK_BUILD_VIEWER)
    find_package(SFML 2.5 COMPONENTS graphics window system REQUIRED)
    add_executable(ik_viewer viewer.cpp)
    target_link_libraries(ik_viewer PRIVATE ik::ik)
    if(TARGET SFML::Graphics)
        target_link_libraries(ik_viewer PRIVATE SFML::Graphics SFML::Window SFML::System)
    else()
        target_link_libraries(ik_viewer PRIVATE sfml-graphics sfml-window sfml-system)
    endif()
endif()
