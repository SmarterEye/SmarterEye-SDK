TEMPLATE = subdirs

include(comm.pri)

SUBDIRS  += MessageBus \
            RealtimeDatabase \
            RtdbService \
            3rdParty/QAsioSocket \
            Satp \
            SatpExt \
            ImageUtils \
            SeUdpClient

!arm-xilinx-linux-gnueabi-g++{
    SUBDIRS  += StereoCamera \
}

CONFIG += ordered

