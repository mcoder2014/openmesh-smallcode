TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
        BoundingBoxHelper.cpp \
        Octree.cpp \
        main.cpp \
        primitives.cpp

TARGET = octree
DESTDIR = ../bin/

HEADERS += \
    BoundingBoxHelper.h \
    Octree.h \
    Triaccel.h \
    primitives.h

include(../openmesh.pri)
