#ifndef QSOCKRTCONNECTION_H
#define QSOCKRTCONNECTION_H

#include <atomic>
#include <functional>
#include "ioserviceclass.h"

class QAsioUdpsocket;

class QUdpSocketConnection :
    public std::enable_shared_from_this<QUdpSocketConnection> ,
    public asio::noncopyable
{
    public :
        union Sockets {
            asio::ip::udp::socket *nossl;
        };
    public:
        QUdpSocketConnection(asio::io_service & service, size_t size, uint16_t localPort):
            stand_(new asio::io_service::strand(service))
        {
            asio::ip::udp::endpoint localPoint(asio::ip::udp::v4(), localPort);
            socket_.nossl = new asio::ip::udp::socket(service, localPoint);

            size_ = size;
            buffer_ = nullptr;
            pRemotPoint = nullptr;
            asyncReadDataHandleCb = nullptr;
        }

        virtual ~QUdpSocketConnection() {
            if (socket().is_open()) {
                socket().close();
            }
        }

        inline void setAsyncReadDataHandleCb(void *ptr)
        {
            asyncReadDataHandleCb = (void(*)(const char *, size_t, const std::string, uint16_t))ptr;
        }

        void setUdpSocketRemotePoint(std::string peerIp, uint16_t peerPort){
            printf("[file: %s func: %s linenum: %d] set udp socket remote point\n", __FILE__, __FUNCTION__, __LINE__);
            pRemotPoint = new asio::ip::udp::endpoint(asio::ip::address_v4::from_string(peerIp), peerPort);
        }

        inline uint16_t getLocalPort()
        {
            return  socket().local_endpoint().port();
        }

        void wirteData(const QByteArray &data) {
            if (is_open() && nullptr != pRemotPoint) {
                printf("[file: %s func: %s linenum: %d] write data\n", __FILE__, __FUNCTION__, __LINE__);
                QByteArray * tdata = new QByteArray(data);
            
                socket().async_send_to(asio::buffer(tdata->data(),tdata->size()), *pRemotPoint,
                        stand_->wrap(std::bind(&QUdpSocketConnection::writeHandler, shared_from_this(),
                                std::placeholders::_1, std::placeholders::_2, tdata)));
            }
            else
            {
                printf("[file: %s func: %s linenum: %d] write data failed\n", __FILE__, __FUNCTION__, __LINE__);
            }
        }

        void doAsyncRead() {
            if(!buffer_){
                buffer_ = new QByteArray();
                buffer_->reserve(size_);
            }
            
            socket().async_receive_from(asio::buffer(buffer_->data(),buffer_->capacity()), senderPoint,
                    stand_->wrap(std::bind(&QUdpSocketConnection::readHandler,shared_from_this(),
                            std::placeholders::_1,std::placeholders::_2)));
        }

        size_t syncWirteData(const char *pData, size_t dataSize) {
            int sendindex = 0;
            while(sendindex < dataSize){
                if (is_open()) {
                    printf("[file: %s func: %s linenum: %d] sync write data\n", __FILE__, __FUNCTION__, __LINE__);
                    sendindex += socket().send_to(asio::buffer(pData + sendindex, dataSize - sendindex), *pRemotPoint);
                } else {
                    printf("[file: %s func: %s linenum: %d] sync write data failed\n", __FILE__, __FUNCTION__, __LINE__);
                    return -1;
                }
            }
            return sendindex;
        }

        size_t syncReadData(char *data, size_t maxLen, bool peek = false) {
            if (is_open()) {
                size_t recvSize = 0;
                asio::ip::udp::endpoint senderPoint;
                recvSize = socket().receive_from(asio::buffer(data, maxLen), senderPoint);
                if(recvSize < 0)
                {
                    printf("[file: %s func: %s linenum: %d] sync read data failed\n", __FILE__, __FUNCTION__, __LINE__);
                }
                else
                {
                    printf("[file: %s func: %s linenum: %d] sync read ip : %s port : %d data size : %d\n", __FILE__, __FUNCTION__, __LINE__,
                            senderPoint.address().to_string().c_str(), senderPoint.port(), recvSize);
                }
                return recvSize;
            } else {
                return -1;
            }
        }

        void disconnectFromHost() {
            if (socket().is_open()) {
                socket().cancel();
                socket().shutdown(asio::ip::udp::socket::shutdown_both,erro_code);
            }
        }

        bool is_open() {
            return socket().is_open();
        }

        int socketDescriptor() {
            return static_cast<int>(socket().native_handle());
        }

        void setSocketOption(uint option, bool isEnble, uint value);

        std::pair<bool, int> getSocketOption(uint option);

        inline asio::ip::udp::socket & socket(){
            return *(socket_.nossl);
        }
    
    protected:
        /* 数据读取的回调函数 */
        void readHandler(const asio::error_code& error, std::size_t bytes_transferred);
        /* 数据写入的回调函数 */
        void writeHandler(const asio::error_code& error, std::size_t bytes_transferred, QByteArray * sdata);
    
    protected:
        void setError(const asio::error_code & erro, QString && erro_string){
            erro_code = erro;
            erro_string.append(erro_code.message().c_str());
            disconnectFromHost();
        }

        template <typename SettableSocketOption>
            void  set_option(const SettableSocketOption& option){
                socket().set_option(option,erro_code);
            }

        template <typename SettableSocketOption>
            void get_option(SettableSocketOption & option) {
                socket().get_option(option,erro_code);
            }


    protected :
        asio::error_code erro_code;
        QByteArray *buffer_;
        asio::io_service::strand *stand_;
        Sockets socket_;
        asio::ip::udp::endpoint senderPoint;
        asio::ip::udp::endpoint *pRemotPoint;
        size_t size_;
        void (*asyncReadDataHandleCb)(const char *, size_t, const std::string, uint16_t);
    private:
        QUdpSocketConnection(const QUdpSocketConnection &) = delete;
};

struct UdpSession
{
    std::shared_ptr<QUdpSocketConnection> connection;
    asio::ip::udp::socket & socket() {return connection->socket();}
    UdpSession(asio::io_service & server, size_t size = 4096, uint16_t localPort = 0) :
        connection(new QUdpSocketConnection(server, size, localPort))
    {
    }
    
    ~UdpSession() {
    }
};

#endif // QSOCKRTCONNECTION_H
