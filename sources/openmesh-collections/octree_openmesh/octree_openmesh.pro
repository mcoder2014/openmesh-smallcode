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

unix {
    INCLUDEPATH += /usr/include/eigen3 \

    LIBS+= \
        -L/usr/lib \
        -lOpenMeshCore -lOpenMeshTools \
}

HEADERS += \
    BoundingBoxHelper.h \
    Mesh.h \
    Octree.h \
    Triaccel.h \
    primitives.h
