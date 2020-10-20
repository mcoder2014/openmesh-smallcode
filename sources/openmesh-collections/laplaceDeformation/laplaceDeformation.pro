TEMPLATE = app
CONFIG += console c++17
CONFIG -= app_bundle
CONFIG -= qt

HEADERS += \
    LaplaceDeformation.h

SOURCES += \
        LaplaceDeformation.cpp \
        main.cpp

# Realize laplace Transformation algorithm

TARGET = laplaceDeformation
DESTDIR = ../bin/

include(../openmesh.pri)
