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
    Mesh.h \
    Projection.h

unix {
    INCLUDEPATH += /usr/include/eigen3 \

    LIBS+= \
        -L/usr/lib \
        -lOpenMeshCore -lOpenMeshTools \
}
