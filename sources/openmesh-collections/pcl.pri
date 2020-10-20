# pcl config

unix {
INCLUDEPATH +=\
    /usr/include/pcl-1.10\

LIBS += \
    -L/usr/lib \
    -lpcl_common -lpcl_features \
    -lpcl_kdtree -lpcl_octree \
    -lpcl_search \
    -lpcl_surface \
    -lvtkCommonCore \
    -lboost_filesystem -lboost_thread -lboost_iostreams -lboost_container\
    -lqhull
}

win32 {
}
