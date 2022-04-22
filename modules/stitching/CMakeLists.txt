message(STATUS "Adding Stitcher")

#-------------------------------------------------------------------------------
# External Libraries
#-------------------------------------------------------------------------------
find_package(OpenCV 4.0 REQUIRED core features2d imgproc stitching)

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

file(GLOB stitcher_SRC src/*.cpp)
add_library(stitcher SHARED ${stitcher_SRC})
target_link_libraries(stitcher core ${OpenCV_LIBS})
target_include_directories(stitcher PUBLIC ${OpenCV_INCLUDE_DIRS} include/)