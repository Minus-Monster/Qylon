cmake_minimum_required(VERSION 3.14)

project(Qylon)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(QYLON_HEADERS
    Qylon.h
    Acquisition/Camera.h
    Acquisition/CameraWidget.h
    Acquisition/Grabber.h
    Acquisition/GrabberWidget.h
    Modules/GraphicsView.h
    Modules/GraphicsScene.h
    Modules/GraphicsVTKWidget.h
    Modules/GraphicsWidget.h
    Modules/TiffReader.h
    Processing/vTools.h
    Processing/OutputCommunication.h
    Processing/SerialCommunication.h
    Processing/SerialCommunicationWidget.h

)
set(QYLON_SOURCES
    Qylon.cpp
    Acquisition/Camera.cpp
    Acquisition/CameraWidget.cpp
    Acquisition/Grabber.cpp
    Acquisition/GrabberWidget.cpp
    Processing/vTools.cpp
    Processing/OutputCommunication.cpp
    Processing/SerialCommunication.cpp
    Processing/SerialCommunicationWidget.cpp
)
set(QYLON_RESOURCES
    Resources.qrc
)
qt5_add_resources(QYLON_RESOURCES_RCC ${QYLON_RESOURCES})
add_library(${PROJECT_NAME} STATIC ${QYLON_HEADERS} ${QYLON_SOURCES} ${QYLON_RESOURCES_RCC})

set(INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR}/include)
file(MAKE_DIRECTORY ${INCLUDE_DIR})

# Copy headers to the include folder
foreach(HEADER ${QYLON_HEADERS})
    configure_file(${HEADER} ${INCLUDE_DIR}/${HEADER} COPYONLY)
endforeach()

# Add the include directory to the project
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>
)


# Qt Loading
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets SerialPort Network)
target_link_libraries(${PROJECT_NAME} PUBLIC Qt5::Core Qt5::Gui Qt5::Widgets Qt5::SerialPort Qt5::Network)

find_package(TIFF QUIET)
if(TIFF_FOUND)
    target_link_libraries(${PROJECT_NAME} PUBLIC TIFF::TIFF)
    target_compile_definitions(${PROJECT_NAME} PUBLIC TIFF_ENABLED)
endif()

# Pylon Loading
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
find_package(pylon 7.4.0 REQUIRED)

if(pylon_FOUND)
    target_link_libraries(${PROJECT_NAME} PUBLIC pylon::pylon)
    target_compile_definitions(${PROJECT_NAME} PUBLIC PYLON_ENABLED)
endif()

# vTools
find_path(pylon_vTools FindpylonDataProcessing.cmake
    HINTS "/opt/pylon/share/pylon/Samples/pylonDataProcessing/C++/cmake"
    HINTS "$ENV{PYLON_DEV_DIR}/Samples/pylonDataProcessing/C++/cmake"
)
list(APPEND CMAKE_MODULE_PATH ${pylon_vTools})
find_package(pylonDataProcessing REQUIRED)
if(pylonDataProcessing_FOUND)
    target_link_libraries(${PROJECT_NAME} PUBLIC pylon::DataProcessing)
endif()


find_path(grabber_cmake FindFgLib5.cmake
    HINTS "$ENV{BASLER_FG_SDK_DIR}/lib/cmake"
)
list(APPEND CMAKE_MODULE_PATH ${grabber_cmake})
find_library(FgLib5_LIBRARY NAMES fglib5
    PATHS
    "$ENV{FGLIB5}/lib"
    "$ENV{FGLIB5}"
    "${CMAKE_INSTALL_PREFIX}/lib"
    ${LIB_DIRS}
    "[HKEY_LOCAL_MACHINE\\SOFTWARE\\Basler\\FGSDK;Info]/lib/${COMPILER_LIB_DIR}"
    "[HKEY_LOCAL_MACHINE\\SOFTWARE\\Basler\\FGSDK;Info]/lib"
)
if(FgLib5_FOUND)
    message(STATUS "Frame grabber SDK found")
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
