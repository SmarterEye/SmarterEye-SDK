#ifndef SEUDPPROTOCOLCLIENT_H
#define SEUDPPROTOCOLCLIENT_H
#include <memory>
#include <atomic>
#include <mutex>
#include <thread>
#include <map>
#include <vector>
#include "se_udp_protocol.h"
#include "asio.hpp"

#  ifdef _MSC_VER
#    define SE_DECL_EXPORT     __declspec(dllexport)
#    define SE_DECL_IMPORT     __declspec(dllimport)
#  else
#    define SE_DECL_EXPORT     __attribute__((visibility("default")))
#    define SE_DECL_IMPORT     __attribute__((visibility("default")))
#    define SE_DECL_HIDDEN     __attribute__((visibility("hidden")))
#  endif

#if defined(SEUDPCLIENT_LIBRARY)
#  define SEUDPCLIENT_API SE_DECL_EXPORT
#else
#  define SEUDPCLIENT_API SE_DECL_IMPORT
#endif

class QAsioUdpsocket;
typedef struct
{
    void (*newDeviceFound)() = nullptr;
    void (*disconnectedFromServer)() = nullptr;
    void (*perceptionReceived)(uint64_t ) = nullptr;
} ClientCallback;

class SEUDPCLIENT_API SEUdpProtocolClient
{
public:
    SEUdpProtocolClient();
    virtual ~SEUdpProtocolClient();
    static SEUdpProtocolClient* instance();

    void init(ClientCallback *clientcallback = nullptr);
    void deinit();

public:

    void disconnectFromServer();
    void generateCommandPackage(SECommandPacket &pack, uint8_t type,
                                uint8_t id, void *data, uint16_t len);

    void sendCommand(uint8_t cmdId, void *data, uint16_t len);

    static void handleBroadcastData(const char *pData, size_t dataSize,
        const std::string remoteIp, uint16_t remotePort);

    static void handleCommandData(const char *pData, size_t dataSize,
        const std::string remoteIp, uint16_t remotePort);

    static void handlePerceptionData(const char *pData, size_t dataSize,
        const std::string remoteIp, uint16_t remotePort);

    bool connectTo(const std::string &ip);

    void getAllDevices(std::vector<std::string> &keys);

    std::string getDeviceDescript(const std::string &ip);

    std::shared_ptr< SEPercetion> getOnePerceptionData() {
        std::shared_ptr<SEPercetion> perception = mPerceptionDatas[0];
        mPerceptionDatas.erase(mPerceptionDatas.begin());
        return perception;
	}

	bool hasPerceptionData() {
        return !mPerceptionDatas.empty();
	}

protected:

    static void signalExitHandler(int sig);
    void registerSystemSignalHandler();
    void handleHeartBeatTimeout(const asio::error_code& error);
    void handleAckTimerTimeout(const asio::error_code& error);

private:
    QAsioUdpsocket *qAsioUdpSocketMonitor_;
    QAsioUdpsocket *qAsioUdpSocketCommand_;
    QAsioUdpsocket *qAsioUdpSocketPerception_;

    ClientCallback mClientCallback;

    std::vector<std::shared_ptr<SEPercetion>> mPerceptionDatas;
    asio::steady_timer *mHeartbeatTimer;
    asio::steady_timer *mAckTimer;
    asio::io_context mContext;
    typedef asio::io_context::executor_type ExecutorType;
    asio::executor_work_guard<ExecutorType> mWorkGuard;
    asio::thread *mThread;
};
#endif // SEUDPPROTOCOLCLIENT_H
