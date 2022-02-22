#-------------------------------------------------
#
# Project created by QtCreator 2016-11-03T15:45:20
#
#-------------------------------------------------

QT       += network

QT       -= gui

TARGET = Satp
TEMPLATE = lib
!win32:VERSION = 1.0.2
CONFIG += create_prl

DEFINES += SATP_LIBRARY

include(../comm.pri)
INCLUDEPATH += $$PWD/../3rdParty/QAsioSocket/include
LIBS += -L$$LIBDIR -lQAsioSocket

SOURCES += \
    acceptor.cpp \
    recvthread.cpp \
    tcpprotocol.cpp

HEADERS +=\
    connector.h \
    recvthread.h \
    satp_global.h \
    acceptor.h \
    tcpprotocol.h

    SOURCES += \
        transmitthread.cpp
    HEADERS +=\
        transmitthread.h
    SOURCES += \
        connector.cpp
