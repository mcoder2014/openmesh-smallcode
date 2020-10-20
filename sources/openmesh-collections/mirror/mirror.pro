TEMPLATE = app
CONFIG += console c++14
CONFIG -= app_bundle
CONFIG -= qt

TARGET = mirror
DESTDIR = ../bin/

SOURCES += \
        main.cpp \
        MeshAlgorithm.cpp \

HEADERS += \
        MeshAlgorithm.h

include(../openmesh.pri)

