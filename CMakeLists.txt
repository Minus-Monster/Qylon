cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
project(Qylon VERSION 2.0)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
option(WITH_PYLON "Add pylon SDK for Qylon" OFF)
option(WITH_GRABBER "Add Framegrabber SDK for Qylon" OFF)

set(QYLON_HEADERS
    Qylon.h
    Camera/Camera.h
    Camera/CameraWidget.h
    Grabber/Grabber.h
    Grabber/GrabberWidget.h
    vTools/vTools.h
    vTools/vToolsWidget.h
)
set(QYLON_SOURCES
    Qylon.cpp
    Camera/Camera.cpp
    Camera/CameraWidget.cpp
    Grabber/Grabber.cpp
    Grabber/GrabberWidget.cpp
    vTools/vTools.cpp
    vTools/vToolsWidget.cpp
)
set(QYLON_RESOURCES
    Resources.qrc
)

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Core Gui Widgets SerialPort Network)
if(Qt6_FOUND)
    message(STATUS "[${PROJECT_NAME}] Configuring for Qt6")
    qt_add_resources(QYLON_RESOURCES_RCC ${QYLON_RESOURCES})
else(Qt5_FOUND)
    message(STATUS "[${PROJECT_NAME}] Configuring for Qt5")
    qt5_add_resources(QYLON_RESOURCES_RCC ${QYLON_RESOURCES})
endif()

add_library(${PROJECT_NAME} STATIC ${QYLON_HEADERS} ${QYLON_SOURCES} ${QYLON_RESOURCES_RCC})
target_link_libraries(${PROJECT_NAME} PUBLIC
    Qt${QT_VERSION_MAJOR}::Core
    Qt${QT_VERSION_MAJOR}::Gui
    Qt${QT_VERSION_MAJOR}::Widgets
    Qt${QT_VERSION_MAJOR}::SerialPort
    Qt${QT_VERSION_MAJOR}::Network
)

set(INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR}/include)
file(MAKE_DIRECTORY ${INCLUDE_DIR})

# Copy headers to the include folder
file(COPY ${QYLON_HEADERS} DESTINATION ${INCLUDE_DIR})

# Add the include directory to the project
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>
)

# Pylon Loading
# Locate the cmake find module for pylon ...
if(WITH_PYLON AND NOT pylon_FOUND)
    find_path(pylon_cmake Findpylon.cmake
        HINTS "$ENV{PYLON_DEV_DIR}/lib/cmake"
        "$ENV{PYLON_ROOT}/lib/cmake"
        "$ENV{PYLON_ROOT}/Development/lib/cmake"
        "$ENV{ProgramFiles}/Basler/pylon 8/Development/lib/cmake"
        "$ENV{ProgramFiles}/Basler/pylon 7/Development/lib/cmake"
        "/opt/pylon/lib/cmake"
    )
    list(APPEND CMAKE_MODULE_PATH ${pylon_cmake})
    find_package(pylon REQUIRED)
    if(pylon_FOUND)
        target_link_libraries(${PROJECT_NAME} PUBLIC pylon::pylon)
        target_compile_definitions(${PROJECT_NAME} PUBLIC PYLON_ENABLED)
        message(STATUS "[${PROJECT_NAME}] pylon SDK ${pylon_VERSION} found")
    endif()

    # vTools
    find_path(pylon_vTools FindpylonDataProcessing.cmake
        HINTS "/opt/pylon/share/pylon/Samples/pylonDataProcessing/C++/cmake"
        HINTS "$ENV{PYLON_DEV_DIR}/Samples/pylonDataProcessing/C++/cmake"
    )
    list(APPEND CMAKE_MODULE_PATH ${pylon_vTools})
    find_package(pylonDataProcessing QUIET)
    if(pylonDataProcessing_FOUND)
        target_link_libraries(${PROJECT_NAME} PUBLIC pylon::DataProcessing)
        target_compile_definitions(${PROJECT_NAME} PUBLIC VTOOLS_ENABLED)
        message(STATUS "[${PROJECT_NAME}] pylonDataProcessing SDK found")
    endif()

    if(OpenCV_FOUND)
        message(STATUS "[${PROJECT_NAME}] OpenCV ${OpenCV_VERSION} found")
        target_link_libraries(${PROJECT_NAME} PUBLIC ${OpenCV_LIBS})
        target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS})
        target_compile_definitions(${PROJECT_NAME} PRIVATE OPENCV_ENABLED)
    else()
        find_path(OpenCV_DIR OpenCVConfig.cmake
            HINTS "C:/dev/opencv/build"
        )
        find_package(OpenCV QUIET)
        if(OpenCV_FOUND)
            message(STATUS "[${PROJECT_NAME}] OpenCV ${OpenCV_VERSION} found")
            target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenCV_LIBS})
            target_include_directories(${PROJECT_NAME} PRIVATE ${OpenCV_INCLUDE_DIRS})
            target_compile_definitions(${PROJECT_NAME} PRIVATE OPENCV_ENABLED)
        else()
            message(WARNING "[${PROJECT_NAME}] OpenCV not found.")
        endif()
    endif()

    # PCL Loading
    find_package(PCL QUIET COMPONENTS common io filters visualization)
    if(PCL_FOUND)
        message(STATUS "PCL ${PCL_VERSION} found")
        target_link_libraries(${PROJECT_NAME} PUBLIC ${PCL_LIBRARIES})
        target_include_directories(${PROJECT_NAME} PUBLIC ${PCL_INCLUDE_DIRS})
        target_compile_definitions(${PROJECT_NAME} PUBLIC PCL_ENABLED)
        #VTK
        find_package(VTK REQUIRED)
        if(VTK_FOUND)
            message(STATUS "VTK ${VTK_VERSION} found")
            target_link_libraries(${PROJECT_NAME} PUBLIC ${VTK_LIBRARIES})
            target_include_directories(${PROJECT_NAME} PUBLIC ${VTK_INCLUDE_DIRS})
        endif()
    endif()
endif()

# Grabber Loading
if(WITH_GRABBER)
    if(NOT DEFINED BASLER_FG_SDK_DIR AND DEFINED ENV{BASLER_FG_SDK_DIR})
        set(BASLER_FG_SDK_DIR $ENV{BASLER_FG_SDK_DIR})
    endif()

    if(NOT BASLER_FG_SDK_DIR)
        find_path(BASLER_FG_SDK_DIR
            NAMES cmake/modules/FindFgLib5.cmake
            HINTS
                "/opt/Basler/FramegrabberSDK"
                "/usr/local/Basler/FramegrabberSDK"
                "C:/Program Files/Basler/FramegrabberSDK"
                "C:/Basler/FramegrabberSDK"
        )
        if(BASLER_FG_SDK_DIR)
            set(ENV{BASLER_FG_SDK_DIR} "${BASLER_FG_SDK_DIR}")
            set(Fg_cmake "${BASLER_FG_SDK_DIR}/cmake/modules")
            list(APPEND CMAKE_MODULE_PATH ${Fg_cmake})

            find_package(FgLib5 REQUIRED)
            if(FgLib5_FOUND)
                add_definitions(${SISOPLATFORM_DEFINITIONS})
                message(STATUS "[${PROJECT_NAME}] Framegrabber SDK ${FgLib5_VERSION_STRING} found")
                target_include_directories(${PROJECT_NAME} PUBLIC ${FgLib5_INCLUDE_DIR})
                target_link_libraries(${PROJECT_NAME} PUBLIC ${FgLib5_LIBRARY})

                find_package(ClSerSis REQUIRED)
                if(ClSerSis_FOUND)
                    target_include_directories(${PROJECT_NAME} PUBLIC ${ClSerSis_INCLUDE_DIR})
                    target_link_libraries(${PROJECT_NAME} PUBLIC ${ClSerSis_LIBRARY})
                endif(ClSerSis_Found)

                find_package(SisoIoLib REQUIRED)
                if(SisoIoLib_FOUND)
                    target_include_directories(${PROJECT_NAME} PUBLIC ${SisoIoLib_INCLUDE_DIR})
                    target_link_libraries(${PROJECT_NAME} PUBLIC ${SisoIoLib_LIBRARY})
                endif(SisoIoLib_FOUND)

                find_package(SisoPlatform REQUIRED)
                if(SisoPlatform_FOUND)
                    target_include_directories(${PROJECT_NAME} PUBLIC ${SisoPlatform_INCLUDE_DIR})
                    target_link_libraries(${PROJECT_NAME} PUBLIC ${SisoPlatform_LIBRARY})
                endif(SisoPlatform_FOUND)

                find_package(SisoGenicamLib REQUIRED)
                if(SisoGenicamLib_FOUND)
                    target_include_directories(${PROJECT_NAME} PUBLIC ${SisoGenicamLib_INCLUDE_DIR})
                    target_link_libraries(${PROJECT_NAME} PUBLIC ${SisoGenicamLib_LIBRARY})
                endif(SisoGenicamLib_FOUND)

                target_compile_definitions(${PROJECT_NAME} PUBLIC GRABBER_ENABLED)
            endif(FgLib5_FOUND)
        else()
            message(FATAL_ERROR "[${PROJECT_NAME}] FramegrabberSDK not found! Set BASLER_FG_SDK_DIR manually.")
        endif()
    endif()
endif(WITH_GRABBER)

