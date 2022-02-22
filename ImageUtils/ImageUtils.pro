TARGET = ImageUtils
TEMPLATE = lib

DEFINES += IMAGEUTILS_LIBRARY

include(../comm.pri)

SOURCES += yuv2rgb.cpp \
    disparityconvertor.cpp \
    obstaclepainter.cpp \
    roadwaypainter.cpp \
    smarterFont.cpp


