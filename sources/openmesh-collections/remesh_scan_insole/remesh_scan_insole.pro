TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
        main.cpp

###########
# remesh 扫描鞋垫底层曲面算法
###########

TARGET = remesh-scan-insole
DESTDIR = ../bin/

unix {
    INCLUDEPATH += /usr/include/eigen3 \

    LIBS+= \
        -L/usr/lib \
        -lOpenMeshCore -lOpenMeshTools \
}
