message(STATUS "Adding APP calibrate")

#-------------------------------------------------------------------------------
# External Libraries
#-------------------------------------------------------------------------------
find_package(OpenCV 4.0 REQUIRED core imgproc highgui)
find_package(Boost 1.59 REQUIRED COMPONENTS filesystem iostreams)

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
# Build app calibrate
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
if (NOT TARGET calibrator)
    message( FATAL_ERROR "calibrator could not be found")
endif()

add_executable(calibrate calibrate.cpp)
target_link_libraries(calibrate plog core dataloader calibrator ${OpenCV_LIBS} ${Boost_LIBRARIES})
target_include_directories(calibrate PUBLIC ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})


