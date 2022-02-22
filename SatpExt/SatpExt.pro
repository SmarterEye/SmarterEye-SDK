#-------------------------------------------------
#
# Project created by QtCreator 2016-11-03T15:45:20
#
#-------------------------------------------------

QT       += network

#QT       -= gui

TARGET = SatpExt
TEMPLATE = lib
!win32:VERSION = 1.0.2
CONFIG += create_prl

DEFINES += SATPEXT_LIBRARY

include(../comm.pri)

INCLUDEPATH += $$PWD/../Satp $$PWD/../MessageBus $$PWD/../RealtimeDatabase $$PWD/../RtdbService

LIBS += -L$$LIBDIR -lMessageBus -lRealtimeDatabase -lSatp -lRtdbService

SOURCES += \
    filereceiver.cpp \
    filesender.cpp \
    messageadapter.cpp \
    rtdbsender.cpp

HEADERS +=\
    satpext_global.h \
    filereceiver.h \
    filesender.h \
    messageadapter.h \
    rtdbsender.h
