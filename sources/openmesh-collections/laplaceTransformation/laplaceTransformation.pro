TEMPLATE = app
CONFIG += console c++17
CONFIG -= app_bundle
CONFIG -= qt

HEADERS += \
    LaplaceTransformation.h \
    Mesh.h

SOURCES += \
        LaplaceTransformation.cpp \
        main.cpp

# Realize laplace Transformation algorithm

TARGET = laplaceTransform
DESTDIR = ../bin/

unix {
    INCLUDEPATH += /usr/include/eigen3 \

    LIBS+= \
        -L/usr/lib \
        -lOpenMeshCore -lOpenMeshTools \
}


