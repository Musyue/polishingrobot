cmake_minimum_required(VERSION 2.8)
project( emSmarteyeAPI_demo )

# Find OpenCV, you may need to set OpenCV_DIR variable
# to the absolute path to the directory containing OpenCVConfig.cmake file
# via the command line or GUI
set(OpenCV_DIR /opt/amd-Opencv-3.3.0/share/OpenCV)
find_package(OpenCV REQUIRED)
# If the package has been found, several variables will
# be set, you can find the full list with descriptions
# in the OpenCVConfig.cmake file.
# Print some message showing some of them
message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")
# Add OpenCV headers location to your include paths
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
if (PCL_FOUND)   
#---------------------------------------------------------------------------------------
# add source files
#---------------------------------------------------------------------------------------
    set(DIR_SRCS ${DIR_SRCS} emSmarteyeAPI.cpp)
    MESSAGE(STATUS ${DIR_SRCS})
else()
#---------------------------------------------------------------------------------------
# add source files
#---------------------------------------------------------------------------------------
    set(DIR_SRCS ${DIR_SRCS} emSmarteyeAPInoPCL.cpp)
    MESSAGE(STATUS ${DIR_SRCS})
endif (PCL_FOUND)
#---------------------------------------------------------------------------------------
# basic setting
#---------------------------------------------------------------------------------------
## Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# FILE(GLOB DIR_HEADERS ../*.h)


#---------------------------------------------------------------------------------------
# SmarteyeAPI
#---------------------------------------------------------------------------------------
set(SmarteyeAPI_INCLUDE "${CMAKE_SOURCE_DIR}/../../include")
set(SmarteyeAPI_LIBS_DIR "${CMAKE_SOURCE_DIR}/../../lib")

include_directories(${SmarteyeAPI_INCLUDE})
link_directories(${SmarteyeAPI_LIBS_DIR})


#---------------------------------------------------------------------------------------
# SmarteyeAPI_Demo
#---------------------------------------------------------------------------------------
add_executable(${PROJECT_NAME} ${DIR_SRCS})

target_link_libraries(${PROJECT_NAME} emSmarteyeAPI)
target_link_libraries(${PROJECT_NAME} pthread)
# Link your application with OpenCV libraries
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
target_link_libraries (${PROJECT_NAME} ${PCL_LIBRARIES})


