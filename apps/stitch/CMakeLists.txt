message(STATUS "Adding APP stitch")

#-------------------------------------------------------------------------------
# External Libraries
#-------------------------------------------------------------------------------
find_package(OpenCV 4.0 REQUIRED core imgproc highgui)

#-------------------------------------------------------------------------------
# Git Submodules
#-------------------------------------------------------------------------------
set(JSON_SUBMODULE ${PROJECT_SOURCE_DIR}/extern/json)

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
# Build app stitch
#-------------------------------------------------------------------------------
if (NOT TARGET plog)
    message( FATAL_ERROR "plog could not be found")
endif()
if (NOT TARGET core)
    message( FATAL_ERROR "core could not be found")
endif()
if (NOT TARGET dataloader)
    message( FATAL_ERROR "dataloader could not be found")
endif()
if (NOT TARGET stitcher)
    message( FATAL_ERROR "stitcher could not be found")
endif()

add_executable(stitch stitch.cpp)
target_link_libraries(stitch plog core dataloader stitcher ${OpenCV_LIBS})
target_include_directories(stitch PUBLIC ${OpenCV_INCLUDE_DIRS})


