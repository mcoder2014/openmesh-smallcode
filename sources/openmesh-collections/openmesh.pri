# Openmesh Configuration

message("PWD of openmesh.pri $$PWD")

HEADERS +=\
    $$PWD/include/Mesh.h \

unix {
    INCLUDEPATH += /usr/include/eigen3 \
                $$PWD/include \

    LIBS+= \
        -L/usr/lib \
        -lOpenMeshCore -lOpenMeshTools \
}

win32 {

}
