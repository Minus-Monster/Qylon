cmake_minimum_required(VERSION 3.14)

project(Qylon)
set(QYLON_SOURCES
    Qylon.cpp
    Qylon.h
    Acquisition/Camera.h
    Acquisition/Camera.cpp
    Acquisition/CameraWidget.h
    Acquisition/CameraWidget.cpp
    Acquisition/Grabber.h
    Acquisition/Grabber.cpp
    Acquisition/GrabberWidget.h
    Acquisition/GrabberWidget.cpp
    Modules/GraphicsView.h
)
add_library(${PROJECT_NAME} SHARED ${QYLON_SOURCES}
    Modules/GraphicsScene.h
    Modules/GraphicsVTKWidget.h
    Modules/GraphicsWidget.h
    Processing/vTools.h Processing/vTools.cpp
    Processing/OutputCommunication.h Processing/OutputCommunication.cpp
    Processing/SerialCommunication.h Processing/SerialCommunication.cpp
    Processing/SerialCommunicationWidget.h Processing/SerialCommunicationWidget.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
get_target_property(INCLUDE_DIRS ${PROJECT_NAME} INCLUDE_DIRECTORIES)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets SerialPort Network)
target_link_libraries(${PROJECT_NAME} PUBLIC Qt5::Core Qt5::Gui Qt5::Widgets Qt5::SerialPort Qt5::Network)

#PYLON
# Locate the cmake find module for pylon ...
find_path(pylon_cmake Findpylon.cmake
    HINTS "$ENV{PYLON_DEV_DIR}/lib/cmake"
          "$ENV{PYLON_ROOT}/lib/cmake"
          "$ENV{PYLON_ROOT}/Development/lib/cmake"
          "$ENV{ProgramFiles}/Basler/pylon 7/Development/lib/cmake"
          "$ENV{ProgramFiles}/Basler/pylon 6/Development/lib/cmake"
          "$ENV{ProgramFiles}/Basler/pylon 5/Development/lib/cmake"
          "/opt/pylon/lib/cmake"
        )
# ... and extend the cmake module path accordingly.
list(APPEND CMAKE_MODULE_PATH ${pylon_cmake})
# Locate pylon.
find_package(pylon 6.3.0 REQUIRED)

if(pylon_FOUND)
    target_link_libraries(${PROJECT_NAME} PUBLIC pylon::pylon)
    target_compile_definitions(${PROJECT_NAME} PUBLIC PYLON_ENABLED)
endif()

# vTools
find_path(pylon_vTools FindpylonDataProcessing.cmake
    HINTS "/opt/pylon/share/pylon/Samples/pylonDataProcessing/C++/cmake"
)
list(APPEND CMAKE_MODULE_PATH ${pylon_vTools})
find_package(pylonDataProcessing REQUIRED)

if(pylonDataProcessing_FOUND)
    target_link_libraries(${PROJECT_NAME} PUBLIC pylon::DataProcessing)
endif()
#FRAME GRABBER
#include(/opt/basler/cmake/modules/SisoLibDir.cmake)
#FILE(TO_CMAKE_PATH "$ENV{BASLER_FG_SDK_DIR}" BASLER_FG_SDK_DIR)
#FIND_PATH(FgLib5_INCLUDE_DIR fgrab_define.h
#        PATHS
#        "$ENV{FGLIB5}/include"
#        "${CMAKE_INSTALL_PREFIX}/include"
#        "${BASLER_FG_SDK_DIR}/include"
#        "[HKEY_LOCAL_MACHINE\\SOFTWARE\\Basler\\FGSDK;Info]/include"
#)
find_path(grabber_cmake FindFgLib5.cmake
    HINTS "$ENV{BASLER_FG_SDK_DIR}/lib/cmake"
)
list(APPEND CMAKE_MODULE_PATH ${grabber_cmake})
#SISO_LIB_DIR("${BASLER_FG_SDK_DIR}/lib" LIB_DIRS COMPILER_LIB_DIR)
find_library(FgLib5_LIBRARY NAMES fglib5
        PATHS
        "$ENV{FGLIB5}/lib"
        "$ENV{FGLIB5}"
        "${CMAKE_INSTALL_PREFIX}/lib"
        ${LIB_DIRS}
        "[HKEY_LOCAL_MACHINE\\SOFTWARE\\Basler\\FGSDK;Info]/lib/${COMPILER_LIB_DIR}"
        "[HKEY_LOCAL_MACHINE\\SOFTWARE\\Basler\\FGSDK;Info]/lib"
)
#IF (FgLib5_INCLUDE_DIR AND EXISTS "${FgLib5_INCLUDE_DIR}/fgrab_define.h")
#        FILE(STRINGS "${FgLib5_INCLUDE_DIR}/fgrab_define.h" FGLIB_H_VERSION REGEX "^#define[ \\t]*FGLIB_VERSION_STRING[ \\t]*\"[^\"]*\"")
#        IF (FGLIB_H_VERSION)
#                STRING(REGEX REPLACE "^#define FGLIB_VERSION_STRING[ \\t]*\"([^\"]*)\".*" "\\1" FgLib5_VERSION_STRING "${FGLIB_H_VERSION}")
#        ENDIF (FGLIB_H_VERSION)
#ENDIF (FgLib5_INCLUDE_DIR AND EXISTS "${FgLib5_INCLUDE_DIR}/fgrab_define.h")
#INCLUDE(FindPackageHandleStandardArgs)
#IF (CMAKE_VERSION VERSION_LESS 2.8.4)
#        FIND_PACKAGE_HANDLE_STANDARD_ARGS(FgLib5 DEFAULT_MSG FGLIB5_LIBRARY FGLIB5_INCLUDE_DIR)
#ELSE (CMAKE_VERSION VERSION_LESS 2.8.4)
#        FIND_PACKAGE_HANDLE_STANDARD_ARGS(FgLib5
#                        REQUIRED_VARS FgLib5_LIBRARY FgLib5_INCLUDE_DIR
#                        VERSION_VAR FgLib5_VERSION_STRING)
#ENDIF (CMAKE_VERSION VERSION_LESS 2.8.4)

if(FgLib5_FOUND)
        SET(FgLib5_LIBRARIES ${FgLib5_LIBRARY})
        SET(FGLIB5_FOUND ${FgLib5_FOUND})
        SET(FGLIB5_INCLUDE_DIR ${FgLib5_INCLUDE_DIR})
        SET(FGLIB5_LIBRARY ${FgLib5_LIBRARY})
        SET(FGLIB5_LIBRARIES ${FgLib5_LIBRARIES})
        target_include_directories(${PROJECT_NAME} PUBLIC /opt/basler/include)
        target_compile_definitions(${PROJECT_NAME} PUBLIC GRABBER_ENABLED)
endif(FgLib5_FOUND)


#OPENCV
find_package(OpenCV QUIET)
if(OpenCV_FOUND)
    message(STATUS "OpenCV ${OpenCV_VERSION} found")
    target_link_libraries(${PROJECT_NAME} PUBLIC ${OpenCV_LIBS})
    target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS})
    target_compile_definitions(${PROJECT_NAME} PUBLIC OPENCV_ENABLED)
    target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    )
endif()

#PCL
find_package(PCL 1.13 QUIET COMPONENTS common io filters visualization)
if(PCL_FOUND)
    message(STATUS "PCL ${PCL_VERSION} found")
    target_link_libraries(${PROJECT_NAME} PUBLIC ${PCL_LIBRARIES})
    target_include_directories(${PROJECT_NAME} PUBLIC ${PCL_INCLUDE_DIRS})
    target_compile_definitions(${PROJECT_NAME} PUBLIC PCL_ENABLED)
    #VTK
    find_package(VTK 9.2 REQUIRED)
    if(VTK_FOUND)
        message(STATUS "VTK ${VTK_VERSION} found")
        target_link_libraries(${PROJECT_NAME} PUBLIC ${VTK_LIBRARIES})
        target_include_directories(${PROJECT_NAME} PUBLIC ${VTK_INCLUDE_DIRS})
    endif()
endif()




#### RELEASE SETUP

#set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_BINARY_DIR})
## Install the Qylon library
#install(TARGETS ${PROJECT_NAME}
#    LIBRARY DESTINATION lib
#    ARCHIVE DESTINATION lib
#    RUNTIME DESTINATION bin
#    INCLUDES DESTINATION include
#)
## Install the header files
#install(FILES Qylon.h DESTINATION include)
## Install the Acquisition directory with matching header files
#install(DIRECTORY Acquisition
#    DESTINATION include
#    FILES_MATCHING PATTERN "*.h"
#    PATTERN "*CMakeFiles*" EXCLUDE
#)
#set(QYLON_INCLUDE_DIR include)
#set(QYLON_LIBRARY_DIR lib)
