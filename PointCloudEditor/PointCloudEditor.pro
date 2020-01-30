#-------------------------------------------------
#
# Project created by QtCreator 2017-07-26T15:16:15
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = PointCloudEditor2
TEMPLATE = app
win32:LIBS += -lopengl32
win32:QMAKE_CXXFLAGS += /bigobj

SOURCES += main.cpp\
        mainwindow.cpp \
    normaldrawer.cpp \
    normalestimatorworker.cpp \
    worker.cpp \
    gaussianmapdrawer.cpp \
    pointcloudinfodialog.cpp \
    pointclouddrawer.cpp \
    simplifiedpointcloud.cpp \
    boundingboxdrawer.cpp \
    viewoptionsdialog.cpp \
    transformtranslatedialog.cpp \
    transformtranslateworker.cpp \
    transformscaledialog.cpp \
    transformscaleworker.cpp \
    transformrotatedialog.cpp \
    transformrotateworker.cpp \
    planedrawer.cpp \
    cylinderdrawer.cpp \
    selectfilter.cpp \
    pointcloudioworker.cpp \
    planedetectorworker.cpp \
    planedetectordialog.cpp

HEADERS  += mainwindow.h \
    normaldrawer.h \
    normalestimatorworker.h \
    worker.h \
    gaussianmapdrawer.h \
    pointcloudinfodialog.h \
    pointclouddrawer.h \
    simplifiedpointcloud.h \
    boundingboxdrawer.h \
    viewoptionsdialog.h \
    transformtranslatedialog.h \
    transformtranslateworker.h \
    transformscaledialog.h \
    transformscaleworker.h \
    transformrotatedialog.h \
    transformrotateworker.h \
    planedrawer.h \
    cylinderdrawer.h \
    selectfilter.h \
    pointcloudioworker.h \
    planedetectorworker.h \
    planedetectordialog.h

RESOURCES += \
    resources.qrc

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../CoreLib/release/ -lCoreLib
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../CoreLib/debug/ -lCoreLib
else:unix: LIBS += -L$$OUT_PWD/../CoreLib/ -lCoreLib

INCLUDEPATH += $$PWD/../CoreLib
DEPENDPATH += $$PWD/../CoreLib

win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../CoreLib/release/libCoreLib.a
else:win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../CoreLib/debug/libCoreLib.a
else:win32:!win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../CoreLib/release/CoreLib.lib
else:win32:!win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../CoreLib/debug/CoreLib.lib
else:unix: PRE_TARGETDEPS += $$OUT_PWD/../CoreLib/libCoreLib.a

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../GraphicsLib/release/ -lGraphicsLib
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../GraphicsLib/debug/ -lGraphicsLib
else:unix: LIBS += -L$$OUT_PWD/../GraphicsLib/ -lGraphicsLib

INCLUDEPATH += $$PWD/../GraphicsLib
DEPENDPATH += $$PWD/../GraphicsLib

win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../GraphicsLib/release/libGraphicsLib.a
else:win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../GraphicsLib/debug/libGraphicsLib.a
else:win32:!win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../GraphicsLib/release/GraphicsLib.lib
else:win32:!win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../GraphicsLib/debug/GraphicsLib.lib
else:unix: PRE_TARGETDEPS += $$OUT_PWD/../GraphicsLib/libGraphicsLib.a

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../DetectionLib/release/ -lDetectionLib
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../DetectionLib/debug/ -lDetectionLib
else:unix: LIBS += -L$$OUT_PWD/../DetectionLib/ -lDetectionLib

INCLUDEPATH += $$PWD/../DetectionLib
DEPENDPATH += $$PWD/../DetectionLib

win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../DetectionLib/release/libDetectionLib.a
else:win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../DetectionLib/debug/libDetectionLib.a
else:win32:!win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../DetectionLib/release/DetectionLib.lib
else:win32:!win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../DetectionLib/debug/DetectionLib.lib
else:unix: PRE_TARGETDEPS += $$OUT_PWD/../DetectionLib/libDetectionLib.a

INCLUDEPATH += $$PWD/../eigen3
