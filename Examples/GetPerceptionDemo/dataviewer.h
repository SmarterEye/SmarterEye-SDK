#ifndef DATAVIEWER_H
#define DATAVIEWER_H

#include "seudpprotocolclient.h"

class DataViewer
{
public:
    DataViewer();

    void initUdpClient();
    void showObs(const SEObstacles &obs);
    void showFreespace(const SEFreeSpace &fs);
    void showFlatness(const SEFlatness &flatness);
    void showLaneLine(const SELane &line);
    void showTSR(const SETSR &tsr);
    void showTFL(const SETFL &tfl);
    static void newDeviceFoundCallback();
    static void disconnectedFromServerCallback();
    static void perceptionReceivedCallback(uint64_t tm);
private:
    SEUdpProtocolClient *mUdpClient;
    static DataViewer *mStaticDataViewer;
};

#endif // DATAVIEWER_H
