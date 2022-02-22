QT       += core
QT       -= gui

TEMPLATE = lib
DEFINES += QASIOSOCKET_LIBRARY
DESTDIR   = $$PWD/../lib
DEFINES += _WIN32_WINNT=0x0501
CONFIG += create_prl

TARGET  = QAsioSocket

include($$PWD/../QAsioSocket.pri)

