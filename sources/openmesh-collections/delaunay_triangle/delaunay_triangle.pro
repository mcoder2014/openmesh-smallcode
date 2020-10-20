TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

TARGET = delaunay_triangle
DESTDIR = ../bin

SOURCES += \
        DelaunayTriangulation.cpp \
        Triangle.cpp \
        Vector3D.cpp \
        main.cpp

HEADERS += \
    DelaunayTriangulation.h \
    Triangle.h \
    Vector3D.h

include(../openmesh.pri)
