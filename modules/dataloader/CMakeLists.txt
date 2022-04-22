message(STATUS "Adding Dataloader")

#-------------------------------------------------------------------------------
# External Libraries
#-------------------------------------------------------------------------------
find_package(OpenCV 4.0 REQUIRED core)
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
# Build dataloader
#-------------------------------------------------------------------------------
if (NOT TARGET core)
    message( FATAL_ERROR "core could not be found")
endif()

set(dataloader_SRC src/dataloader.cpp)
add_library(dataloader SHARED ${dataloader_SRC})
target_link_libraries(dataloader core ${OpenCV_LIBS}  ${Boost_LIBRARIES})
target_include_directories(dataloader PUBLIC include/ ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})