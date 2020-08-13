cmake_minimum_required(VERSION 2.8.11)

project(emSmarteyeUI)

# init_qt: Let's do the CMake job for us
set(CMAKE_AUTOMOC ON) # For meta object compiler
set(CMAKE_AUTORCC ON) # Resource files
set(CMAKE_AUTOUIC ON) # UI files

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Find the QtWidgets library
set(Qt5_DIR /opt/Qt5.6.3/5.6.3/gcc_64/lib/cmake/Qt5)
find_package(Qt5 REQUIRED Widgets)

find_package(VTK REQUIRED)
find_package(PCL REQUIRED)

#---------------------------------------------------------------------------------------
# OpenCV
#---------------------------------------------------------------------------------------
set(OpenCV_DIR /opt/opencv330/share/OpenCV)
find_package(OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS})

#---------------------------------------------------------------------------------------
# ThirdtyLib
#---------------------------------------------------------------------------------------
set(INSTALL_LIB_DIR 
   "\${ORIGIN}/../../../lib/3rdParty/Applications"
   "\${ORIGIN}/../../../lib/3rdParty/xcb"
   "\${ORIGIN}/../../../lib"
   ) # 假设安装目录在编译目录的lib子目录内

set(CMAKE_SKIP_BUILD_RPATH TRUE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH "\${ORIGIN}/../../../lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# 确保链接库不在系统默认安装的目录上时更改到项目lib上
list(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES ${CMAKE_INSTALL_RPATH} isSystemDir)
if("${isSystemDir}" STREQUAL "-1")
  set(CMAKE_INSTALL_RPATH "${INSTALL_LIB_DIR}")
endif("${isSystemDir}" STREQUAL "-1")

#---------------------------------------------------------------------------------------
# emSmarteyeLib
#---------------------------------------------------------------------------------------
set(emSmarteyeLib_INCLUDE "${CMAKE_SOURCE_DIR}/../../include")
set(emSmarteyeLib_LIBS_DIR "${CMAKE_SOURCE_DIR}/../../lib")

include_directories(${emSmarteyeLib_INCLUDE})
link_directories(${emSmarteyeLib_LIBS_DIR})

# Fix a compilation bug under ubuntu 16.04 (Xenial)
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(project_SOURCES main.cpp emSmarteyeUI.cpp QxtSpanSlider.cpp)
FILE(GLOB project_HEADERS ./*.h)

add_executable(${PROJECT_NAME} ${project_SOURCES} ${project_HEADERS})
target_link_libraries(${PROJECT_NAME} optimized libemSmarteyeAPI.so)
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} Qt5::Widgets)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

#---------------------------------------------------------------------------------------
# copy dlls to the exe directory
#---------------------------------------------------------------------------------------
#add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
#    COMMAND ${CMAKE_COMMAND} -E copy_directory
#    "${CMAKE_SOURCE_DIR}/bin" $<TARGET_FILE_DIR:${PROJECT_NAME}>)

#---------------------------------------------------------------------------------------
# copy platforms to the build directory
#---------------------------------------------------------------------------------------
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND cp -rf ${CMAKE_SOURCE_DIR}/../../lib/3rdParty/platforms ${CMAKE_SOURCE_DIR}/build)    
    
