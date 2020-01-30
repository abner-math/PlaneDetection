#-------------------------------------------------
#
# Project created by QtCreator 2017-08-06T08:07:03
#
#-------------------------------------------------

QT       -= gui

TARGET = DetectionLib
TEMPLATE = lib
CONFIG += staticlib
QMAKE_CXXFLAGS += -std=c++11
win32:QMAKE_CXXFLAGS += /bigobj

SOURCES += \
    primitivedetector.cpp \
    planedetector.cpp \
    planarpatch.cpp

HEADERS += \
    primitivedetector.h \
    planedetector.h \
    planarpatch.h
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
