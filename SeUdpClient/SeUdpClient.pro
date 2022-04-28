#-------------------------------------------------
#
# Project created by QtCreator 2015-12-17T11:36:43
#
#-------------------------------------------------

QT       -= gui
QT      += network

TARGET = SeUdpClient
TEMPLATE = lib
CONFIG += create_prl

DEFINES += SEUDPCLIENT_LIBRARY
DEFINES += ASIO_STANDALONE

include(../comm.pri)

INCLUDEPATH += $$PWD/../3rdParty/QAsioSocket/include $$PWD/../3rdParty/QAsioSocket/src/include_asio
LIBS += -L$$LIBDIR -lQAsioSocket

SOURCES += seudpprotocolclient.cpp

HEADERS += se_udp_protocol.h\
           seudpprotocolclient.h \
           qasioudpsocket.h

