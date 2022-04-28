#include <QtCore/QString>

#include "tcpabstractsocket.h"
#include "qasioudpsocket.h"
#include "qudpsocketconnection.h"

void QUdpSocketConnection::setSocketOption(uint option, bool isEnble, uint value)
{
    switch (option) {
        case TcpAbstractSocket::Multicast_Loopback :
            {
                asio::ip::multicast::enable_loopback option(isEnble);
                set_option(option);
            }
            break;
        case TcpAbstractSocket::Multicast_TTL :
            {
                asio::ip::multicast::hops option(value);
                set_option(option);
            }
            break;
        case TcpAbstractSocket::IP_TCP_NODelay :
            {
                asio::ip::tcp::no_delay option(isEnble);
                set_option(option);
            }
            break;
        case TcpAbstractSocket::Broadcast :
            {
                asio::socket_base::broadcast option(isEnble);
                set_option(option);
            }
            break;
        case TcpAbstractSocket::Linger :
            {
                asio::socket_base::linger option(isEnble, value);
                set_option(option);
            }
            break;
        case TcpAbstractSocket::Keep_Live :
            {
                asio::socket_base::keep_alive option(isEnble);
                set_option(option);
            }
            break;
        case TcpAbstractSocket::Receive_Buffer_Size :
            {
                asio::socket_base::receive_buffer_size option(value);
                set_option(option);
            }
            break;
        case TcpAbstractSocket::Receive_Low_Watermark :
            {
                asio::socket_base::receive_low_watermark option(value);
                set_option(option);
            }
            break;
        case TcpAbstractSocket::Reuse_Address :
            {
                asio::socket_base::reuse_address option(isEnble);
                set_option(option);
            }
            break;
        case TcpAbstractSocket::Send_Buffer_Size :
            {
                asio::socket_base::send_buffer_size option(value);
                set_option(option);
            }
            break;
        case TcpAbstractSocket::Send_Low_Watermark :
            {
                asio::socket_base::send_low_watermark option(value);
                set_option(option);
            }
            break;
        default:
            break;
    }
}


std::pair<bool,int> QUdpSocketConnection::getSocketOption(uint option)
{
    switch (option) {
        case TcpAbstractSocket::Multicast_Loopback :
            {
                asio::ip::multicast::enable_loopback option;
                get_option(option);
                bool is_set = option.value();
                return std::make_pair(is_set,0);
            }
            break;
        case TcpAbstractSocket::Multicast_TTL :
            {
                asio::ip::multicast::hops option;
                get_option(option);
                int ttl = option.value();
                return std::make_pair(false,ttl);
            }
            break;
        case TcpAbstractSocket::IP_TCP_NODelay :
            {
                asio::ip::tcp::no_delay option;
                get_option(option);
                bool is_set = option.value();
                return std::make_pair(is_set,0);
            }
            break;
        case TcpAbstractSocket::Broadcast :
            {
                asio::socket_base::broadcast option;
                get_option(option);
                bool is_set = option.value();
                return std::make_pair(is_set,0);
            }
            break;
        case TcpAbstractSocket::Linger :
            {
                asio::socket_base::linger option;
                get_option(option);
                bool is_set = option.enabled();
                int timeout = option.timeout();
                return std::make_pair(is_set,timeout);
            }
            break;
        case TcpAbstractSocket::Keep_Live :
            {
                asio::socket_base::keep_alive option;
                get_option(option);
                bool is_set = option.value();
                return std::make_pair(is_set,0);
            }
            break;
        case TcpAbstractSocket::Receive_Buffer_Size :
            {
                asio::socket_base::receive_buffer_size option;
                get_option(option);
                int size = option.value();
                return std::make_pair(false,size);
            }
            break;
        case TcpAbstractSocket::Receive_Low_Watermark :
            {
                asio::socket_base::receive_low_watermark option;
                get_option(option);
                int size = option.value();
                return std::make_pair(false,size);
            }
            break;
        case TcpAbstractSocket::Reuse_Address :
            {
                asio::socket_base::reuse_address option;
                get_option(option);
                bool is_set = option.value();
                return std::make_pair(is_set,0);

            }
            break;
        case TcpAbstractSocket::Send_Buffer_Size :
            {
                asio::socket_base::send_buffer_size option;
                get_option(option);
                int size = option.value();
                return std::make_pair(false,size);
            }
            break;
        case TcpAbstractSocket::Send_Low_Watermark :
            {
                asio::socket_base::send_low_watermark option;
                get_option(option);
                int size = option.value();
                return std::make_pair(false,size);
            }
            break;
        default:
            return std::make_pair(false,0);
            break;
    }
}


void QUdpSocketConnection::readHandler(const asio::error_code& error, std::size_t bytes_transferred)
{
    try{
        if(!error){
            printf("[file: %s func: %s linenum: %d] sync read data size : %d\n", __FILE__, __FUNCTION__, __LINE__,
                    bytes_transferred);
            
            /* handle data */
            if(nullptr != asyncReadDataHandleCb)
            {
                asyncReadDataHandleCb(buffer_->data(), bytes_transferred, 
                        senderPoint.address().to_string(), senderPoint.port());
            }

            doAsyncRead();
        }
        else {
            setError(error, QString("Read Data Erro : "));
        }
    }
    catch (std::exception& e){
        return;
    }
}

void QUdpSocketConnection::writeHandler(const asio::error_code& error, std::size_t bytes_transferred, QByteArray * sdata)
{
    int size = sdata->size();
    delete sdata;
    if (!error && bytes_transferred > 0){
    } 
    else {
        setError(error,QString("Write Data Erro : "));
    }
}
