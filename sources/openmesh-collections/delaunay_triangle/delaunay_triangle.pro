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

unix {
    INCLUDEPATH += /usr/include/eigen3 \

    LIBS+= \
        -L/usr/lib \
        -lOpenMeshCore -lOpenMeshTools \
}

HEADERS += \
    DelaunayTriangulation.h \
    Mesh.h \
    Triangle.h \
    Vector3D.h
