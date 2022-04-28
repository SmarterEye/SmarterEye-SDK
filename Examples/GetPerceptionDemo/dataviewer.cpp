#include "dataviewer.h"
#include <iostream>

DataViewer * DataViewer::mStaticDataViewer = nullptr;

DataViewer::DataViewer()
    : mUdpClient(nullptr)
{
    initUdpClient();
}

void DataViewer::initUdpClient()
{
    ClientCallback clientCallback;
    //clientCallback.newDeviceFound = newDeviceFoundCallback;
    clientCallback.disconnectedFromServer = disconnectedFromServerCallback;
    clientCallback.perceptionReceived = perceptionReceivedCallback;
    mUdpClient = SEUdpProtocolClient::instance();
    mUdpClient->init(&clientCallback);
    mStaticDataViewer = this;
}

void DataViewer::showObs(const SEObstacles &obs)
{
    std::cout << "Got obstacle: ";
    for (auto i = 0; i < obs.obs_num; i++) {
        std::cout << "{ID: " << (int)obs.obstacles[i].id;
        std::cout << " type: " << obs.obstacles[i].type;
        std::cout << " size: " << obs.obstacles[i].length << "x" << obs.obstacles[i].width << "x" << obs.obstacles[i].height;
        std::cout << " position: " << obs.obstacles[i].position.x << "x" << obs.obstacles[i].position.y << "x" << obs.obstacles[i].position.z;
        std::cout << " source: " << (int)obs.obstacles[i].source;
        std::cout << "}";
        std::cout << std::endl;
    }
}

void DataViewer::showFreespace(const SEFreeSpace &fs)
{
    std::cout << "Got freespace: ";
    for (int i = 0; i < fs.point_num; i++) {
        std::cout << " " << fs.points[i].longitudinal << "x" << fs.points[i].lateral;
    }
    std::cout << std::endl;
}

void DataViewer::showFlatness(const SEFlatness &flatness)
{
    std::cout << "Got flatness: {" << std::endl;
    std::cout << "samplingInterval: " << flatness.samplingInterval << std::endl;
          std::cout << "road type: " << flatness.type <<std::endl;
    std::cout << "road type confidence: " << flatness.confidence << std::endl;
    std::cout << "left: " << "right: " << std::endl;
    for (int i = 0; i < flatness.point_num; i++) {
        std::cout << "(" << flatness.points[i].leftWheelHightPoint.x << ", " << flatness.points[i].leftWheelHightPoint.y << ") ";
        std::cout << "(" << flatness.points[i].rightWheelHightPoint.x << ", " << flatness.points[i].rightWheelHightPoint.y << ")" << std::endl;
    }
    std::cout << "}"<< std::endl;
}

void DataViewer::showLaneLine(const SELane &lane)
{
    std::cout << "Got Lane:";
    for (int i = 0; i < lane.line_num; i++) {
        std::cout << "{Confidence:" << lane.lines[i].confidence;
        std::cout << " Color:" << (int)lane.lines[i].color;
        std::cout << " Width:" << lane.lines[i].width;
        std::cout << " Type:" << (int)lane.lines[i].type;
        std::cout << " Position:" << (int)lane.lines[i].side_position;
        std::cout << " Start:" << lane.lines[i].start_point.x;
        std::cout << " End:" << lane.lines[i].end_point.x;
        std::cout << " coeffs:{" << lane.lines[i].coeffs[0] << "," << lane.lines[i].coeffs[1] << "," << lane.lines[i].coeffs[2] << "," << lane.lines[i].coeffs[3] << "}}";
    }
    std::cout << std::endl;
}

void DataViewer::showTSR(const SETSR &tsr)
{
    std::cout << "Got TSR:";
    for (int i = 0; i < tsr.sign_num; i++) {
        std::cout << "{ID:" << (int)tsr.signs[i].id;
        std::cout << " Traffic Sign Type:" << (int)tsr.signs[i].sign_type;
        std::cout << " Position:" << tsr.signs[i].position.x << "x" << tsr.signs[i].position.y << "x" << tsr.signs[i].position.z;
        std::cout << "}";
    }
    std::cout << std::endl;
}

void DataViewer::showTFL(const SETFL &tfl)
{
    std::cout << "Got TFL:";
    for (int i = 0; i < tfl.light_num; i++) {
        std::cout << "{ID:" << (int)tfl.lights[i].id;
        std::cout << " Color:" << (int)tfl.lights[i].color;
        std::cout << " Shape:" << (int)tfl.lights[i].shapeType;
        std::cout << " Position:" << tfl.lights[i].position.x << "x" << tfl.lights[i].position.y << "x" << tfl.lights[i].position.z;
        std::cout << "}";
    }
    std::cout << std::endl;
}

void DataViewer::newDeviceFoundCallback()
{
    std::cout << "new device found" << std::endl;
}

void DataViewer::disconnectedFromServerCallback()
{
    std::cout <<"disconnected from server"<<std::endl;
}

void DataViewer::perceptionReceivedCallback(uint64_t tm)
{
    std::cout << "perception received " << tm <<std::endl;
    SEUdpProtocolClient *inst = SEUdpProtocolClient::instance();
    while (inst->hasPerceptionData())
    {
        std::shared_ptr<SEPercetion> pack = inst->getOnePerceptionData();
        switch (pack->type)
        {
        case SEPercetionType_Obstacle: {
            uint16_t parsedLen = sizeof(SEPercetion);
            SEObstacles *obsPtr = reinterpret_cast<SEObstacles*>(pack->data);
            if (mStaticDataViewer) {
                mStaticDataViewer->showObs(*obsPtr);
            }
        }
            break;
        case SEPercetionType_Lane: {
            SELane *lane = reinterpret_cast<SELane*>(pack->data);
            if (mStaticDataViewer) {
                mStaticDataViewer->showLaneLine(*lane);
            }
        }
            break;
        case SEPercetionType_Freespace: {
            SEFreeSpace *fs = reinterpret_cast<SEFreeSpace*>(pack->data);
            if (mStaticDataViewer) {
                mStaticDataViewer->showFreespace(*fs);
            }
        }
            break;
        case SEPercetionType_Flatness: {
            SEFlatness *fs = reinterpret_cast<SEFlatness*>(pack->data);
            if (mStaticDataViewer) {
                mStaticDataViewer->showFlatness(*fs);
            }
        }
            break;
        case SEPercetionType_TSR: {
            SETSR *tsr = reinterpret_cast<SETSR*>(pack->data);
            if (mStaticDataViewer) {
                mStaticDataViewer->showTSR(*tsr);
            }
        }
            break;
        case SEPercetionType_TFL: {
            SETFL *tfl = reinterpret_cast<SETFL*>(pack->data);
            if (mStaticDataViewer) {
                mStaticDataViewer->showTFL(*tfl);
            }
        }
            break;
        default:
            break;
        }
    }
}
