TEMPLATE = app
CONFIG += console c++14
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
        main.cpp

###########
# remesh 扫描鞋垫底层曲面算法
# 用 pcl 尝试重建 mesh
###########

TARGET = pcl_triangle

DESTDIR = ../bin/

include(../openmesh.pri)
include(../pcl.pri)
