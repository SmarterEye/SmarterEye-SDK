include(modules.pri)

CONFIG += c++11

SDKPATH =  $$PWD
INCLUDEPATH +=  $$PWD/inc $$OPENSEPATH/inc

message("sdk comm $$DESTDIR")

win32 | android {
    CONFIG(debug, debug|release) {
        BINDIR = $$BUILDDIR/DBin
        LIBDIR = $$BUILDDIR/DBin
    } else {
        BINDIR = $$BUILDDIR/Bin
        LIBDIR = $$BUILDDIR/Bin
    }
    HOST_PREFIX = $$[QT_HOST_PREFIX]
    HOST_PREFIX = $$split(HOST_PREFIX, "/")

    DESTDIR = $$BINDIR
}

unix {
    arm-xilinx-linux-gnueabi-g++{
        BINDIR = $$BUILDDIR/localfs/bin
        LIBDIR = $$BUILDDIR/localfs/lib
        equals(TEMPLATE, "app"){
            DESTDIR = $$BINDIR
            target.path = /usr/local/bin
            INSTALLS=target
        }
        equals(TEMPLATE, "lib"){
            DESTDIR = $$LIBDIR
            target.path = /usr/local/lib
            INSTALLS=target
        }
    } else {
        CONFIG(debug, debug|release) {
            BINDIR = $$BUILDDIR/DBin
            LIBDIR = $$BUILDDIR/DBin
        } else {
            BINDIR = $$BUILDDIR/Bin
            LIBDIR = $$BUILDDIR/Bin
        }

        QMAKE_LFLAGS += -Wl,-rpath,.

        DESTDIR = $$BINDIR
        target.path = /usr/lib
        INSTALLS += target

    }

}
