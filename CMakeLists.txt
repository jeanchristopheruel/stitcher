cmake_minimum_required(VERSION 3.13)

#-------------------------------------------------------------------------------
# Project Definitions
#-------------------------------------------------------------------------------
project(Stitcher
        VERSION "0.1.0"
        DESCRIPTION "High performance stitching library for mutlisensor cameras based on OpenCV primitives"
        LANGUAGES CXX C)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -Wno-deprecated-declarations -Wno-unused-variable")

# Debug Config
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0")

# Do not allow to build in main repo
file(TO_CMAKE_PATH "${PROJECT_BINARY_DIR}/CMakeLists.txt" LOC_PATH)
if(EXISTS "${LOC_PATH}")
    message(FATAL_ERROR "You cannot build in a source directory (or any directory with a CMakeLists.txt file). Please make a build subdirectory. Feel free to remove CMakeCache.txt and CMakeFiles.")
endif()

# Set the default build type
set(default_build_type "Release")
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
    set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE
            STRING "Choose the type of build." FORCE)
    # Set the possible values of build type for cmake-gui
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
            "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

# project specific flags
if(DEBUG)
    add_definitions(-DDEBUG)
endif()

#-------------------------------------------------------------------------------
# Loading submodules
#-------------------------------------------------------------------------------
find_package(Git QUIET)
message(STATUS "Loading submodules")
if(GIT_FOUND AND EXISTS "${PROJECT_SOURCE_DIR}/.git")
    execute_process(COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
            RESULT_VARIABLE GIT_SUBMOD_RESULT)
    if(NOT GIT_SUBMOD_RESULT EQUAL "0")
        message(FATAL_ERROR "git submodule update --init failed with ${GIT_SUBMOD_RESULT}, please checkout submodules")
    endif()
elseif(NOT EXISTS "${PROJECT_SOURCE_DIR}/.git")
    message(FATAL_ERROR "While trying to load dependent submodules, .git directory not found")
elseif(NOT GIT_FOUND)
    message(FATAL_ERROR "While trying to load dependent submodules,
    git package not found. Install with 'apt install git'")
endif()


#-------------------------------------------------------------------------------
# CMAKE OPTIONS
#-------------------------------------------------------------------------------
option(PACKAGE_TESTS "Build the tests" OFF)
option(BUILD_APPS "Build the apps" ON)

#-------------------------------------------------------------------------------
# CONFIGURATIONS
#-------------------------------------------------------------------------------
set(JSON_BuildTests OFF CACHE INTERNAL "")

#-------------------------------------------------------------------------------
# Add components
#-------------------------------------------------------------------------------
add_subdirectory(extern/pybind11)
add_subdirectory(extern/plog)
add_subdirectory(extern/json)
add_subdirectory(modules/core)
add_subdirectory(modules/dataloader)
add_subdirectory(modules/calibration)
add_subdirectory(modules/stitching)

#-------------------------------------------------------------------------------
# Build apps
#-------------------------------------------------------------------------------
if (BUILD_APPS)
    add_subdirectory(apps)
endif()

#-------------------------------------------------------------------------------
# Tests
#-------------------------------------------------------------------------------
if(PACKAGE_TESTS)
    enable_testing()
    add_subdirectory(extern/googletest)
    add_subdirectory(tests)
endif()

#-------------------------------------------------------------------------------
# Install
#-------------------------------------------------------------------------------
if (TARGET core)
    install(TARGETS core DESTINATION lib)
endif()



