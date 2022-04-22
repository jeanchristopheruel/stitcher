message(STATUS "Adding Core")

#-------------------------------------------------------------------------------
# External Libraries
#-------------------------------------------------------------------------------
find_package(OpenCV 4.0 REQUIRED core imgproc imgcodecs stitching)

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
# Build core
#-------------------------------------------------------------------------------
if (NOT TARGET plog)
    message( FATAL_ERROR "plog could not be found")
endif()
if (NOT TARGET nlohmann_json::nlohmann_json)
    message( FATAL_ERROR "nlohmann_json::nlohmann_json could not be found")
endif()

file(GLOB core_SRC src/*.cpp)
add_library(core SHARED ${core_SRC})
target_link_libraries(core plog ${OpenCV_LIBS} nlohmann_json::nlohmann_json)
target_include_directories(core PUBLIC ${OpenCV_INCLUDE_DIRS} include/)