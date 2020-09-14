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
        MeshAlgorithm.h \
        Mesh.h \

unix {
    INCLUDEPATH += /usr/include/eigen3 \

    LIBS+= \
        -L/usr/lib \
        -lOpenMeshCore -lOpenMeshTools \
}
