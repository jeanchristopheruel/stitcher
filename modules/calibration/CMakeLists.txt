message(STATUS "Adding Calibrator")

#-------------------------------------------------------------------------------
# External Libraries
#-------------------------------------------------------------------------------
find_package(OpenCV 4.0 REQUIRED  core calib3d features2d imgproc stitching highgui)

#-------------------------------------------------------------------------------
# CMAKE OPTIONS
#-------------------------------------------------------------------------------
# No options yet

#-------------------------------------------------------------------------------
# CMAKE VARIABLES
#-------------------------------------------------------------------------------
# No variables yet

#-------------------------------------------------------------------------------
# CMAKE CONFIGURATIONS
#-------------------------------------------------------------------------------
# No Config yet

#-------------------------------------------------------------------------------
# Build stitcher
#-------------------------------------------------------------------------------
if (NOT TARGET core)
    message( FATAL_ERROR "core could not be found")
endif()

set(calibrator_SRC src/calibrator.cpp)
add_library(calibrator SHARED ${calibrator_SRC})
target_link_libraries(calibrator core ${OpenCV_LIBS})
target_include_directories(calibrator PUBLIC ${OpenCV_INCLUDE_DIRS} include/)