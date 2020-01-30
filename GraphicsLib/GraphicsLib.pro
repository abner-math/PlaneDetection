#-------------------------------------------------
#
# Project created by QtCreator 2017-08-04T17:21:30
#
#-------------------------------------------------

QT       += widgets opengl

TARGET = GraphicsLib
TEMPLATE = lib
CONFIG += staticlib
QMAKE_CXXFLAGS += -std=c++11
win32:QMAKE_CXXFLAGS += /bigobj

SOURCES += \
    affinetransform.cpp \
    camera.cpp \
    cameracontrolwindow.cpp \
    colorutils.cpp \
    framebuffer.cpp \
    scene.cpp \
    sceneobject.cpp \
    scenewidget.cpp \
    openglcontext.cpp \
    normalshader.cpp \
    drawutils.cpp \
    vertexbuffer.cpp \
    vertexbuffercreator.cpp \
    phongshader.cpp \
    shader.cpp \
    lightsource.cpp \
    filter.cpp \
    edgefilter.cpp \
    pointshader.cpp

HEADERS += \
    affinetransform.h \
    camera.h \
    cameracontrolwindow.h \
    colorutils.h \
    framebuffer.h \
    scene.h \
    sceneobject.h \
    scenewidget.h \
    openglcontext.h \
    shader.h \
    normalshader.h \
    drawutils.h \
    vertexbuffer.h \
    vertexbuffercreator.h \
    phongshader.h \
    lightsource.h \
    filter.h \
    edgefilter.h \
    pointshader.h
unix {
    target.path = /usr/lib
    INSTALLS += target
}

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

INCLUDEPATH += $$PWD/../eigen3
