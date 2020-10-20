TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

DESTDIR=../bin
TARGET=projection

SOURCES += \
        Projection.cpp \
        main.cpp

HEADERS += \
    Projection.h

include(../openmesh.pri)
