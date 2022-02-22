#-------------------------------------------------
#
# Project created by QtCreator 2018-07-06T15:42:35
#
#-------------------------------------------------

QT += network
QT -= gui

CONFIG += c++11 console
CONFIG -= app_bundle

TARGET = StereoCamera
TEMPLATE = lib
CONFIG += create_prl

include(../comm.pri)

DEFINES += STEREOCAMERA_LIBRARY

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

INCLUDEPATH +=$$PWD/../Satp $$PWD/../SatpExt $$PWD/../MessageBus $$PWD/../RealtimeDatabase

LIBS += -L$$LIBDIR -lSatp -lSatpExt -lMessageBus -lRealtimeDatabase


SOURCES += \
    stereocamera.cpp \
    stereocameraimpl.cpp \
    framereceiver.cpp \
    rtdbreceiver.cpp \
    firmwareupdater.cpp \
    autoconnector.cpp \
    devicestatus.cpp \
    sedevicestate.cpp

HEADERS += \
    ../inc/stereocamera.h \
    stereocameraimpl.h \
    stereocamera_global.h \
    framereceiver.h \
    rtdbareceiver.h \
    firmwareupdater.h \
    autoconnector.h \
    devicestatus.h \

