message(STATUS "Adding testing" )
set(the_description "DevOps testing")

if (NOT TARGET gtest)
    message( FATAL_ERROR "gtest could not be found")
endif()
include(GoogleTest)

#-------------------------------------------------------------------------------
# Configure
#-------------------------------------------------------------------------------
mark_as_advanced(
        BUILD_GMOCK BUILD_GTEST BUILD_SHARED_LIBS
        gmock_build_tests gtest_build_samples gtest_build_tests
        gtest_disable_pthreads gtest_force_shared_crt gtest_hide_internal_symbols
)
set_target_properties(gtest PROPERTIES FOLDER extern)
set_target_properties(gtest_main PROPERTIES FOLDER extern)
set_target_properties(gmock PROPERTIES FOLDER extern)
set_target_properties(gmock_main PROPERTIES FOLDER extern)

#-------------------------------------------------------------------------------
# Define test macro
#-------------------------------------------------------------------------------
macro(package_add_test TESTNAME FILES LIBRARIES INCLUDE_DIRS)
    add_executable(${TESTNAME} ${FILES})
    target_link_libraries(${TESTNAME} gtest gmock gtest_main ${LIBRARIES})
    target_include_directories(${TESTNAME} PUBLIC ${INCLUDE_DIRS})
    gtest_discover_tests(${TESTNAME}
            WORKING_DIRECTORY ${TEST_WORKING_DIRECTORY}
            PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY "${PROJECT_BINARY_DIR}/tests"
            )
    set_target_properties(${TESTNAME} PROPERTIES FOLDER tests)
endmacro()

#-------------------------------------------------------------------------------
# Ensure Dependencies
#-------------------------------------------------------------------------------
if (NOT TARGET core)
    message( FATAL_ERROR "core could not be found")
endif()
if (NOT TARGET calibrator)
    message( FATAL_ERROR "calibrator could not be found")
endif()
if (NOT TARGET stitcher)
    message( FATAL_ERROR "stitcher could not be found")
endif()

#-------------------------------------------------------------------------------
# Git Submodules
#-------------------------------------------------------------------------------
set(JSON_SUBMODULE ${PROJECT_SOURCE_DIR}/extern/json)

#-------------------------------------------------------------------------------
# External Libraries
#-------------------------------------------------------------------------------
find_package(OpenCV 4.0 REQUIRED core imgproc imgcodecs)
find_package(Boost 1.59 REQUIRED COMPONENTS filesystem)

#-------------------------------------------------------------------------------
# CMAKE Variables
#-------------------------------------------------------------------------------
set(ASSETSDIRPATH ${CMAKE_CURRENT_SOURCE_DIR}/assets)

#-------------------------------------------------------------------------------
# Configurations
#-------------------------------------------------------------------------------
configure_file(utils/paths.h.in ${CMAKE_BINARY_DIR}/tests/generated/utils/paths.h )

#-------------------------------------------------------------------------------
# Launch tests
#-------------------------------------------------------------------------------
set(SCALIB_LIBS calibrator ${OpenCV_LIBS} ${Boost_LIBRARIES})
set(SCALIB_DIRS
        ${OpenCV_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${JSON_SUBMODULE}/single_include/nlohmann
        ${CMAKE_BINARY_DIR}/tests/generated/utils/)
package_add_test(test_calibrate test_calibrate.cpp "${SCALIB_LIBS}" "${SCALIB_DIRS}")
