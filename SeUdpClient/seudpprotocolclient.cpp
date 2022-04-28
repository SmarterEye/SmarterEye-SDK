#include <iostream>
#include <signal.h>
#include <string.h>
#include <assert.h>
#include <functional>
#include <thread>
#include "qasioudpsocket.h"
#include "seudpprotocolclient.h"


static std::map<std::string, SEBroadcastDeviceInfo> mDevices;
static const int kHeartBeatInterval(30 * 1000);
static const int kAckInterval(10 * 1000);
static const int kBufferSize(65536);

SEUdpProtocolClient::SEUdpProtocolClient() :
    mContext(0),
    mWorkGuard(asio::make_work_guard(mContext))
{
    qAsioUdpSocketMonitor_ = nullptr;
    qAsioUdpSocketCommand_ = nullptr;
    qAsioUdpSocketPerception_ = nullptr;
    mHeartbeatTimer = nullptr;
    mAckTimer = nullptr;

    registerSystemSignalHandler();

    mThread = new asio::thread([this](){
        mContext.run();
    });
}

SEUdpProtocolClient::~SEUdpProtocolClient()
{
    deinit();
}

SEUdpProtocolClient* SEUdpProtocolClient::instance()
{
    static SEUdpProtocolClient* client = nullptr;
    if (client == nullptr) {
        client = new SEUdpProtocolClient();
        assert(client);
    }
    return client;
}

void SEUdpProtocolClient::signalExitHandler(int sig)
{
    std::cout << "recv system signal: " << sig <<std::endl;
    SEUdpProtocolClient::instance()->disconnectFromServer();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    exit(0);
}

void SEUdpProtocolClient::registerSystemSignalHandler()
{
    signal(SIGTERM, signalExitHandler);
    signal(SIGINT, signalExitHandler);
    //signal(SIGHUP, signalExitHandler);
    //signal(SIGQUIT, signalExitHandler);
    //signal(SIGKILL, signalExitHandler);
}

void SEUdpProtocolClient::handleHeartBeatTimeout(const asio::error_code &error)
{
    if(!error){
        sendCommand(SECommandID_Heartbeat, nullptr, 0);
        mAckTimer->expires_after(asio::chrono::milliseconds(kAckInterval));
        mAckTimer->async_wait(std::bind(&SEUdpProtocolClient::handleAckTimerTimeout, this, std::placeholders::_1));
    }
}

void SEUdpProtocolClient::handleAckTimerTimeout(const asio::error_code &error)
{
    if(!error){
            deinit();
            if (mClientCallback.disconnectedFromServer) {
                mClientCallback.disconnectedFromServer();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            init();
        }
}

void SEUdpProtocolClient::init(ClientCallback *clientcallback)
{
    if (clientcallback != nullptr) {
        mClientCallback = *clientcallback;
    }

    if (qAsioUdpSocketMonitor_ == nullptr) 
    {
        qAsioUdpSocketMonitor_ = new QAsioUdpsocket(4096, SE_MONITOR_PORT);
        qAsioUdpSocketMonitor_->setAsyncReadDataHandleCb((void *)SEUdpProtocolClient::handleBroadcastData);

        qAsioUdpSocketMonitor_->StartAsyncReadData();
    }

    if (mHeartbeatTimer == nullptr) {
        mHeartbeatTimer = new asio::steady_timer(mContext);
    }
    if (mAckTimer == nullptr) {
        mAckTimer = new asio::steady_timer(mContext);
    }

}

void SEUdpProtocolClient::handleBroadcastData(const char *pData, size_t dataSize, 
        const std::string remoteIp, uint16_t remotePort)
{
    if (dataSize <= sizeof(SECommandPacket)) {
        printf("[file: %s func: %s linenum: %d] invalid buffer size\n", __FILE__, __FUNCTION__, __LINE__);
        return;
    }

    const SECommandPacket *package = reinterpret_cast<const SECommandPacket *>(pData);
    if (package->cmd_type != SECommandType_Command) {
        printf("[file: %s func: %s linenum: %d] Command type valid!\n", __FILE__, __FUNCTION__, __LINE__);
        return;
    }

    if (package->cmd_id == SECommandID_Broadcast) {
        printf("[file: %s func: %s linenum: %d] recv broadcast command ID!\n", __FILE__, __FUNCTION__, __LINE__);
        const SEBroadcastDeviceInfo *info = reinterpret_cast<const SEBroadcastDeviceInfo*>(package->data);

        if(mDevices.count(remoteIp) <= 0){
            printf("[file: %s func: %s linenum: %d] this is a new device. ip : %s port : %d.\n", __FILE__, __FUNCTION__, __LINE__,
                    remoteIp.c_str(), remotePort);
            mDevices.insert(std::pair<std::string, SEBroadcastDeviceInfo>(remoteIp, *info));

            /* connect to device */
            SEUdpProtocolClient::instance()->connectTo(remoteIp);
        }
        else
        {
            printf("[file: %s func: %s linenum: %d] this is device already exit. ip : %s port : %d.\n", __FILE__, __FUNCTION__, __LINE__,
                    remoteIp.c_str(), remotePort);
        }
    }
}

void SEUdpProtocolClient::handleCommandData(const char *pData, size_t dataSize, 
        const std::string remoteIp, uint16_t remotePort)
{
    printf("[file: %s func: %s linenum: %d] handle command data \n", __FILE__, __FUNCTION__, __LINE__);

    if(dataSize <= sizeof(SECommandPacket)) {
        printf("[file: %s func: %s linenum: %d] invalid command data size\n", __FILE__, __FUNCTION__, __LINE__);
        return;
    }

    const SECommandPacket *package = reinterpret_cast<const SECommandPacket *>(pData);
    if (package->cmd_type != SECommandType_ACK) {
        printf("[file: %s func: %s linenum: %d] only need command is ACK\n", __FILE__, __FUNCTION__, __LINE__);
        return;
    }

    switch (package->cmd_id) {
        case SECommandID_Sync:
            {
                printf("[file: %s func: %s linenum: %d] got sync command ack from %s\n", __FILE__, __FUNCTION__, __LINE__,
                        remoteIp.c_str());
                const int32_t *ret = reinterpret_cast<const int32_t*>(package->data);
                if(*ret == 0){
                    SEUdpProtocolClient::instance()->mHeartbeatTimer->expires_after(asio::chrono::milliseconds(kHeartBeatInterval));
                    SEUdpProtocolClient::instance()->mHeartbeatTimer->async_wait(std::bind(&SEUdpProtocolClient::handleHeartBeatTimeout,
                                                                               SEUdpProtocolClient::instance(), std::placeholders::_1));
                    uint32_t req = 1;
                    SEUdpProtocolClient::instance()->sendCommand(SECommandID_RequirePerception, &req, sizeof (req));
                }
                break;
            }
        case SECommandID_Heartbeat:
            {
                printf("[file: %s func: %s linenum: %d] got heart beat command ack from %s\n", __FILE__, __FUNCTION__, __LINE__,
                        remoteIp.c_str());

                const SEHeartbeatResponse *resp = reinterpret_cast<const SEHeartbeatResponse *>(package->data);
                if (resp->error_code != 0) {
                    printf("[file: %s func: %s linenum: %d] got device error code: %d\n", __FILE__, __FUNCTION__, __LINE__,
                            resp->error_code);
                }
                SEUdpProtocolClient::instance()->mAckTimer->cancel();
                SEUdpProtocolClient::instance()->mHeartbeatTimer->expires_after(asio::chrono::milliseconds(kHeartBeatInterval));
                SEUdpProtocolClient::instance()->mHeartbeatTimer->async_wait(std::bind(&SEUdpProtocolClient::handleHeartBeatTimeout,
                                                                           SEUdpProtocolClient::instance(), std::placeholders::_1));

                break;
            }
        case SECommandID_Disconnect:
            {
                printf("[file: %s func: %s linenum: %d] got disconnect command ack from %s\n", __FILE__, __FUNCTION__, __LINE__,
                        remoteIp.c_str());
                SEUdpProtocolClient::instance()->mHeartbeatTimer->cancel();
                break;
            }
        case SECommandID_RequirePerception:
            {
                printf("[file: %s func: %s linenum: %d] got require perception command ack from %s\n", __FILE__, __FUNCTION__, __LINE__,
                        remoteIp.c_str());
                break;
            }
        default:
            break;
    }

}

void SEUdpProtocolClient::handlePerceptionData(const char *pData, size_t dataSize, 
        const std::string remoteIp, uint16_t remotePort)
{
    printf("[file: %s func: %s linenum: %d] handle perception data \n", __FILE__, __FUNCTION__, __LINE__);

     if (dataSize <= sizeof(SEDataPacket)) {
        printf("[file: %s func: %s linenum: %d] invalid perception data size\n", __FILE__, __FUNCTION__, __LINE__);
            return;
        }

        const SEDataPacket *package = reinterpret_cast<const SEDataPacket *>(pData);
        if (package->data_type != SEDataType_Perception) {
            printf("[file: %s func: %s linenum: %d] Only perception data is valid and this is not perception data\n", __FILE__, __FUNCTION__, __LINE__);
            return;
        }

        uint16_t parsedLen = 0;
        while (parsedLen < package->length){
            const SEPercetion *result = reinterpret_cast<const SEPercetion*>(package->data + parsedLen);
            parsedLen += result->length;
            std::shared_ptr<SEPercetion> perception(reinterpret_cast<SEPercetion*>(
                                                        new char[result->length]));
            memcpy(perception.get(), result, result->length);
            SEUdpProtocolClient::instance()->mPerceptionDatas.push_back(perception);
        }

        if (SEUdpProtocolClient::instance()->mClientCallback.perceptionReceived) {
            SEUdpProtocolClient::instance()->mClientCallback.perceptionReceived(package->timestamp);
        }

}

bool SEUdpProtocolClient::connectTo(const std::string &ip)
{
    std::string peerIp = ip;

    printf("[file: %s func: %s linenum: %d] connect to ip : %s.\n", __FILE__, __FUNCTION__, __LINE__,
            peerIp.c_str());

    if(mDevices.count(peerIp) > 0){
        printf("[file: %s func: %s linenum: %d] found ip : %s.\n", __FILE__, __FUNCTION__, __LINE__,
                peerIp.c_str());

        /* creat command socket */
        if(qAsioUdpSocketCommand_ == nullptr) 
        {
            qAsioUdpSocketCommand_ = new QAsioUdpsocket(4096, 0);
            qAsioUdpSocketCommand_->setRemotePoint(peerIp, SE_BROADCAST_PORT);
            qAsioUdpSocketCommand_->setAsyncReadDataHandleCb((void *)SEUdpProtocolClient::handleCommandData);

            qAsioUdpSocketCommand_->StartAsyncReadData();

            printf("[file: %s func: %s linenum: %d] creat command socket. peer ip : %s peer port : %d.\n", __FILE__, __FUNCTION__, __LINE__,
                    peerIp.c_str(), SE_BROADCAST_PORT);
        }

        /* creat perception socket */
        if(qAsioUdpSocketPerception_ == nullptr) 
        {
            qAsioUdpSocketPerception_ = new QAsioUdpsocket(4096, 0);
            qAsioUdpSocketPerception_->setAsyncReadDataHandleCb((void *)SEUdpProtocolClient::handlePerceptionData);

            qAsioUdpSocketPerception_->StartAsyncReadData();

            printf("[file: %s func: %s linenum: %d] creat perception socket.\n", __FILE__, __FUNCTION__, __LINE__);
        }

        if(qAsioUdpSocketCommand_ && qAsioUdpSocketPerception_){
            printf("[file: %s func: %s linenum: %d] send command sync to %s\n", __FILE__, __FUNCTION__, __LINE__,
                    peerIp.c_str());

            SESyncRequest req;
            req.ip_addr = 0;
            req.cmd_port = qAsioUdpSocketCommand_->getLocalPort();
            req.perception_port = qAsioUdpSocketPerception_->getLocalPort();
            printf("[file: %s func: %s linenum: %d] command socket port : %d perception port : %d monitor port : %d\n", __FILE__, __FUNCTION__, __LINE__,
                    req.cmd_port, req.perception_port, qAsioUdpSocketMonitor_->getLocalPort());
            //req.image_port = mImageRecvr->getUsedPort();
            req.rsvd[0] = 0;
            sendCommand(SECommandID_Sync, &req, sizeof (req));
            return true;
        }
    }

    return false;
}

void SEUdpProtocolClient::sendCommand(uint8_t cmdId, void *data, uint16_t len)
{
    if (qAsioUdpSocketCommand_ == nullptr) {
        printf("[file: %s func: %s linenum: %d] invalid command socket\n", __FILE__, __FUNCTION__, __LINE__);
        return;
    }

    char* reply = new char[sizeof(SECommandPacket) + len];
    SECommandPacket *package = reinterpret_cast<SECommandPacket *>(reply);
    generateCommandPackage(*package, SECommandType_Command,
                           cmdId, data, len);
    size_t sendSize = qAsioUdpSocketCommand_->syncSendData(reply, sizeof(SECommandPacket) + len);
    printf("[file: %s func: %s linenum: %d] send command data size : %d\n", __FILE__, __FUNCTION__, __LINE__,
            sendSize);

    delete[] reply;
}

void SEUdpProtocolClient::generateCommandPackage(SECommandPacket &pack,
        uint8_t type,
        uint8_t id,
                                           void *data,
                                           uint16_t len)
{
    pack.version = 1;
    pack.rsvd = 0;
    pack.length = len;
    //pack.seq_num = mCommandSeq.fetch_add(1);
    pack.cmd_type = type;
    pack.cmd_id = id;
    if(data)memcpy(pack.data, data, len);
}

void SEUdpProtocolClient::getAllDevices(std::vector<std::string> &keys)
{
    std::map<std::string, SEBroadcastDeviceInfo>::iterator iter = mDevices.begin();
    for (; iter != mDevices.end(); iter++) {
        keys.push_back(iter->first);
    }
}

std::string SEUdpProtocolClient::getDeviceDescript(const std::string &ip)
{
    if(mDevices.count(ip) > 0){
        return mDevices[ip].device_info;
    }
    return std::string("");
}

void SEUdpProtocolClient::disconnectFromServer()
{
    sendCommand(SECommandID_Disconnect, nullptr, 0);
}

void SEUdpProtocolClient::deinit()
{
    if(mHeartbeatTimer){
        mHeartbeatTimer->cancel();
        delete mHeartbeatTimer;
        mHeartbeatTimer = nullptr;
    }
    if(mAckTimer){
        mAckTimer->cancel();
        delete mAckTimer;
        mAckTimer = nullptr;
    }

    if(qAsioUdpSocketMonitor_ == nullptr)
    {
        delete qAsioUdpSocketMonitor_;
        qAsioUdpSocketMonitor_ = nullptr;
    }

    if(qAsioUdpSocketCommand_ == nullptr)
    {
        delete qAsioUdpSocketCommand_;
        qAsioUdpSocketCommand_ = nullptr;
    }
    if(qAsioUdpSocketPerception_ == nullptr)
    {
        delete qAsioUdpSocketPerception_;
        qAsioUdpSocketPerception_ = nullptr;
    }

    mPerceptionDatas.clear();

    mDevices.clear();
    mWorkGuard.reset();
}

