QT += core

INCLUDEPATH  += $$PWD
DEFINES += QYLON

macx{
    QMAKE_RPATHDIR += "/Library/Frameworks/"
    CONFIG-=app_bundle
    INCLUDEPATH += "/Library/Frameworks/pylon.framework/Headers/GenICam"
    QMAKE_CXXFLAGS += -F"/Library/Frameworks"
    LIBS += -F"/Library/Frameworks" -framework pylon
}
linux {
# Pylon
    QMAKE_CXXFLAGS += $$system(/opt/pylon/bin/pylon-config --cflags)
    LIBS += $$system(/opt/pylon/bin/pylon-config --libs --libs-rpath)
    DEFINES += PYLON_ENABLED

# Silicon Software
    QMAKE_CXXFLAGS += -I/opt/basler/include
    LIBS += -L/opt/basler/lib64
    DEFINES += GRABBER_ENABLED
#    LIBS += -lclsersis -lsiso-core -lsiso-lib-fglib-model -lsisoiolibrt -lfglib5 \
#         -lhaprt -lsiso_genicam -lsiso_hw_me5 -lsiso_hw_me6 -lsiso_hal -lsiso_hw

    LIBS += -lclsersis -lcommon-logging-dispatcher -lcommon-logging-siso -ldisplay \
    -lGCBase_gcc_v3_1_Basler_pylon -lGenApi_gcc_v3_1_Basler_pylon -lhaprt -llog4cpp_gcc_v3_1_Basler_pylon \
    -lLog_gcc_v3_1_Basler_pylon -lMathParser_gcc_v3_1_Basler_pylon -lNodeMapData_gcc_v3_1_Basler_pylon \
    -lsiso_auxport -lsiso-core -lsiso_genicam -lsiso_hal -lsiso_hw_me5 -lsiso_hw_me6 -lsiso_log -lsiso_hw \
    -lXmlParser_gcc_v3_1_Basler_pylon -lsisoiolibrt -lsiso-lib-debugview-qt4 -lsiso-lib-display-fglib \
    -lsiso-lib-display-qt4 -lsiso-lib-display -lsiso-lib-environment -lsiso-lib-fglib-model -lsiso-lib-frameprocessing \
    -lsiso-lib-genicam-model -lsiso-lib-gsclient -lsiso-lib-gui-core -lsiso-lib-model-qt4 -lsiso-lib-parameters-qt4 \
    -lsiso-lib-parameters -lsiso-lib-plex -lsiso-lib-processor-qt4-core -lsiso-lib-testframework -lfglib5


# PointCloud Library
    INCLUDEPATH += -I/usr/local/include/pcl-1.13 -I/usr/include/eigen3
    LIBS += -L/usr/local/lib -lpcl_common -lpcl_io -lpcl_visualization

    INCLUDEPATH += /usr/local/include/vtk-9.2
    LIBS += -lvtkCommonCore-9.2 -lvtkRenderingCore-9.2 -lvtksys-9.2
    DEFINES += PCL_ENABLED
    

# OpenCV 4
    INCLUDEPATH+= /usr/local/include/opencv4/
    LIBS += -L/usr/local/lib -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_objdetect -lopencv_imgcodecs -lopencv_videoio
    DEFINES += OPENCV_ENABLED
}
win32 {
    INCLUDEPATH += "$$(PYLON_DEV_DIR)/include"
    DEFINES += PYLON_ENABLED

    INCLUDEPATH += "$$(BASLER_FG_SDK_DIR)/include"
    DEFINES += GRABBER_ENABLED

    contains(QMAKE_TARGET.arch, x86_64) {
        LIBS += -L"$$(PYLON_DEV_DIR)/lib/x64"
        LIBS += -L"$$(BASLER_FG_SDK_DIR)/lib" -L"$$(BASLER_FG_SDK_DIR)/lib/visualc"
        LIBS += -lclsersis -lGCBase_MD_VC141_v3_1_Basler_pylon -lGenApi_MD_VC141_v3_1_Basler_pylon -llog4cpp_MD_VC141_v3_1_Basler_pylon \
        -lLog_MD_VC141_v3_1_Basler_pylon -lsiso_genicam -lsiso_log -lXmlParser_MD_VC141_v3_1_Basler_pylon -lfglib5 -lsiso_hal -llibtiff -liolibrt -ldisplay_lib
    } else {
        LIBS += -L"$$(PYLON_DEV_DIR)/lib/win32"
        LIBS += -L"$$(BASLER_FG_SDK_DIR)/lib"
    }
}

HEADERS += $$PWD/Qylon.h \
    $$PWD/Acquisition/Camera.h \
    $$PWD/Acquisition/CameraWidget.h \
    $$PWD/Acquisition/Grabber.h \
    $$PWD/Acquisition/GrabberWidget.h \
    $$PWD/Modules/DebugConsole.h \
    $$PWD/Processing/IO.h \
    $$PWD/Processing/Image.h \
    $$PWD/Processing/QDC.h \
    $$PWD/Processing/Sequence.h \
    $$PWD/Modules/GraphicsWidget.h \
    $$PWD/Modules/GraphicsVTKWidget.h \
    $$PWD/Modules/GraphicsView.h \
    $$PWD/Modules/GraphicsScene.h

SOURCES += \
    $$PWD/Acquisition/Camera.cpp \
    $$PWD/Acquisition/CameraWidget.cpp \
    $$PWD/Acquisition/Grabber.cpp \
    $$PWD/Acquisition/GrabberWidget.cpp \
    $$PWD/Processing/QDC.cpp \
    $$PWD/Qylon.cpp


RESOURCES += \
    $$PWD/Resources/Resources.qrc
