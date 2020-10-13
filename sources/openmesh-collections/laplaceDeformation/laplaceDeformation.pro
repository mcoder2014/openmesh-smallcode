TEMPLATE = app
CONFIG += console c++17
CONFIG -= app_bundle
CONFIG -= qt

HEADERS += \
    LaplaceDeformation.h \
    Mesh.h

SOURCES += \
        LaplaceDeformation.cpp \
        main.cpp

# Realize laplace Transformation algorithm

TARGET = laplaceDeformation
DESTDIR = ../bin/

unix {
    INCLUDEPATH += /usr/include/eigen3 \

    LIBS+= \
        -L/usr/lib \
        -lOpenMeshCore -lOpenMeshTools \
}


