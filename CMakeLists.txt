cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
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
    Modules/Graph.h
    Modules/TiffReader.h
    Modules/Console.h
    Processing/vTools.h
    Processing/vToolsWidget.h
    Processing/OutputCommunication.h
    Processing/SerialCommunication.h
    Processing/SerialCommunicationWidget.h
    Processing/ImageTools.h
    Processing/HistogramWidget.h
    Processing/LineProfileWidget.h
)
set(QYLON_SOURCES
    Qylon.cpp
    Acquisition/Camera.cpp
    Acquisition/CameraWidget.cpp
    Acquisition/Grabber.cpp
    Acquisition/GrabberWidget.cpp
    Modules/GraphicsWidget.cpp

    Modules/GraphicsView.cpp
    Modules/GraphicsScene.cpp
    Modules/GraphicsVTKWidget.cpp
    Modules/Console.cpp
    Modules/Graph.cpp
    Processing/vTools.cpp
    Processing/vToolsWidget.cpp
    Processing/OutputCommunication.cpp
    Processing/SerialCommunication.cpp
    Processing/SerialCommunicationWidget.cpp
    Processing/ImageTools.cpp
    Processing/HistogramWidget.cpp
    Processing/LineProfileWidget.cpp
)
set(QYLON_RESOURCES
    Resources.qrc
)
find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets)
if(Qt6_FOUND)
    message(STATUS "Configuring for Qt6")
    find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets SerialPort Network)
    qt_add_resources(QYLON_RESOURCES_RCC ${QYLON_RESOURCES})
    add_library(${PROJECT_NAME} STATIC ${QYLON_HEADERS} ${QYLON_SOURCES} ${QYLON_RESOURCES_RCC})
    target_link_libraries(${PROJECT_NAME} PUBLIC Qt6::Core Qt6::Gui Qt6::Widgets Qt6::SerialPort Qt6::Network)
else(Qt5_FOUND)
    message(STATUS "Configuring for Qt5")
    find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets SerialPort Network)
    qt5_add_resources(QYLON_RESOURCES_RCC ${QYLON_RESOURCES})
    add_library(${PROJECT_NAME} STATIC ${QYLON_HEADERS} ${QYLON_SOURCES} ${QYLON_RESOURCES_RCC})
    target_link_libraries(${PROJECT_NAME} PUBLIC Qt5::Core Qt5::Gui Qt5::Widgets Qt5::SerialPort Qt5::Network)
endif()

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

# TIFF Loading
find_package(TIFF QUIET)
if(TIFF_FOUND)
    message(STATUS "libTIFF found")
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
list(APPEND CMAKE_MODULE_PATH ${pylon_cmake})
find_package(pylon 7.4.0 QUIET)
if(pylon_FOUND)
    target_link_libraries(${PROJECT_NAME} PUBLIC pylon::pylon)
    target_compile_definitions(${PROJECT_NAME} PUBLIC PYLON_ENABLED)
    message(STATUS "pylon SDK ${pylon_VERSION} found")
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
    message(STATUS "pylonDataProcessing SDK found")
endif()

# Grabber Loading
file(TO_CMAKE_PATH "$ENV{BASLER_FG_SDK_DIR}" BASLER_FG_SDK_DIR)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_INSTALL_PREFIX}/cmake/modules" "${BASLER_FG_SDK_DIR}/cmake/modules")
find_package(FgLib5 QUIET)
if(FgLib5_FOUND)
    add_definitions(${SISOPLATFORM_DEFINITIONS})
    message(STATUS "Frame grabber SDK ${FgLib5_VERSION_STRING} found")
    target_include_directories(${PROJECT_NAME} PUBLIC ${FgLib5_INCLUDE_DIR})
    target_link_libraries(${PROJECT_NAME} PUBLIC ${FgLib5_LIBRARY})

    find_package(ClSerSis REQUIRED)
    if(ClSerSis_FOUND)
        message(STATUS "Frame grabber SDK ClSerSis found")
        target_include_directories(${PROJECT_NAME} PUBLIC ${ClSerSis_INCLUDE_DIR})
        target_link_libraries(${PROJECT_NAME} PUBLIC ${ClSerSis_LIBRARY})
    endif(ClSerSis_Found)

    find_package(SisoIoLib REQUIRED)
    if(SisoIoLib_FOUND)
        message(STATUS "Frame grabber SDK SisoIoLib found")
        target_include_directories(${PROJECT_NAME} PUBLIC ${SisoIoLib_INCLUDE_DIR})
        target_link_libraries(${PROJECT_NAME} PUBLIC ${SisoIoLib_LIBRARY})
    endif(SisoIoLib_FOUND)

    find_package(SisoPlatform REQUIRED)
    if(SisoPlatform_FOUND)
        message(STATUS "Frame grabber SDK SisoPlatform found")
        target_include_directories(${PROJECT_NAME} PUBLIC ${SisoPlatform_INCLUDE_DIR})
        target_link_libraries(${PROJECT_NAME} PUBLIC ${SisoPlatform_LIBRARY})
    endif(SisoPlatform_FOUND)

    find_package(SisoGenicamLib REQUIRED)
    if(SisoGenicamLib_FOUND)
        message(STATUS "Frame grabber SDK SisoGenicamLib found")
        target_include_directories(${PROJECT_NAME} PUBLIC ${SisoGenicamLib_INCLUDE_DIR})
        target_link_libraries(${PROJECT_NAME} PUBLIC ${SisoGenicamLib_LIBRARY})
    endif(SisoGenicamLib_FOUND)

    target_compile_definitions(${PROJECT_NAME} PUBLIC GRABBER_ENABLED)
endif(FgLib5_FOUND)



# # OPENCV Loading
# find_path(OpenCV_DIR OpenCVConfig.cmake
#     HINTS "C:/opencv/build"
# )
# find_package(OpenCV QUIET)
# if(OpenCV_FOUND)
#     message(STATUS "OpenCV ${OpenCV_VERSION} found")
#     target_link_libraries(${PROJECT_NAME} PUBLIC ${OpenCV_LIBS})
#     target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS})
#     target_compile_definitions(${PROJECT_NAME} PUBLIC OPENCV_ENABLED)
#     # target_include_directories(${PROJECT_NAME} PUBLIC
#     #     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#     #     $<INSTALL_INTERFACE:include>
#     # )
# endif()

# # PCL Loading
# find_package(PCL QUIET COMPONENTS common io filters visualization)
# if(PCL_FOUND)
#     message(STATUS "PCL ${PCL_VERSION} found")
#     target_link_libraries(${PROJECT_NAME} PUBLIC ${PCL_LIBRARIES})
#     target_include_directories(${PROJECT_NAME} PUBLIC ${PCL_INCLUDE_DIRS})
#     target_compile_definitions(${PROJECT_NAME} PUBLIC PCL_ENABLED)
#     #VTK
#     find_package(VTK REQUIRED)
#     if(VTK_FOUND)
#         message(STATUS "VTK ${VTK_VERSION} found")
#         target_link_libraries(${PROJECT_NAME} PUBLIC ${VTK_LIBRARIES})
#         target_include_directories(${PROJECT_NAME} PUBLIC ${VTK_INCLUDE_DIRS})
#     endif()
# endif()
