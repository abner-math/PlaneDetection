#-------------------------------------------------
#
# Project created by QtCreator 2017-08-04T17:20:53
#
#-------------------------------------------------

QT       -= gui

TARGET = CoreLib
TEMPLATE = lib
CONFIG += staticlib
QMAKE_CXXFLAGS += -std=c++11
win32:QMAKE_CXXFLAGS += /bigobj

SOURCES += \
    nearestneighborcalculator.cpp \
    normalestimator.cpp \
    partitioner.cpp \
    point.cpp \
    pointcloud.cpp \
    rect.cpp \
    geometryutils.cpp \
    angleutils.cpp \
    line.cpp \
    pcacalculator.cpp \
    connectivitygraph.cpp \
    boundaryvolumehierarchy.cpp \
    segmentator.cpp \
    primitive.cpp \
    plane.cpp \
    circle.cpp \
    cylinder.cpp \
    geometry.cpp \
    functor.cpp \
    cylinderfunctor.cpp \
    pointcloudio.cpp \
    unionfind.cpp \
    collisiondetector.cpp \
    statisticsutils.cpp \
    connection.cpp \
    extremity.cpp

HEADERS += \
    nearestneighborcalculator.h \
    normalestimator.h \
    partitioner.h \
    point.h \
    pointcloud.h \
    rect.h \
    geometryutils.h \
    angleutils.h \
    line.h \
    pcacalculator.h \
    connectivitygraph.h \
    boundaryvolumehierarchy.h \
    segmentator.h \
    primitive.h \
    plane.h \
    circle.h \
    cylinder.h \
    geometry.h \
    functor.h \
    cylinderfunctor.h \
    cylinderfunctor.h \
    pointcloudio.h \
    unionfind.h \
    collisiondetector.h \
    statisticsutils.h \
    connection.h \
    extremity.h
unix {
    target.path = /usr/lib
    INSTALLS += target
}

INCLUDEPATH += $$PWD/../eigen3
