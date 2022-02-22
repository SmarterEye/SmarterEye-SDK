#-------------------------------------------------
#
# Project created by QtCreator 2015-12-11T12:51:06
#
#-------------------------------------------------

QT       -= gui

TARGET = RealtimeDatabase
TEMPLATE = lib
CONFIG += create_prl

DEFINES += REALTIMEDATABASE_LIBRARY

include(../comm.pri)


SOURCES += realtimedatabase.cpp

HEADERS += realtimedatabase.h\
    realtimedatabase_global.h
