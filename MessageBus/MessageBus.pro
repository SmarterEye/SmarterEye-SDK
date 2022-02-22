#-------------------------------------------------
#
# Project created by QtCreator 2015-12-17T11:36:43
#
#-------------------------------------------------

QT       -= gui
QT      += network

TARGET = MessageBus
TEMPLATE = lib
CONFIG += create_prl

DEFINES += MESSAGEBUS_LIBRARY

include(../comm.pri)

SOURCES += messagebus.cpp \
    ipcserver.cpp \
    slaveconnection.cpp \
    masterconnection.cpp \
    messageutils.cpp \
    service.cpp \
    messageslot.cpp \
    packet.cpp \
    packetmessageadapter.cpp \
    serviceutils.cpp

HEADERS += messagebus.h\
    messagebus_global.h \
    ipcserver.h \
    slaveconnection.h \
    masterconnection.h \
    messageutils.h \
    messageslot.h \
    service.h \
    messagerouter.h \
    packetrouter.h \
    packet.h \
    packethandler.h \
    packetmessageadapter.h \
    serviceutils.h

INCLUDEPATH += $$PWD/../3rdParty

