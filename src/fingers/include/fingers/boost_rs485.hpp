
/**
 *  @file       boost_rs485.hpp
 *  @brief
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#pragma once

/* Includes ------------------------------------------------------------------*/
#include "i_transport.hpp"
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <iostream>
#include <cstring>
#include <chrono>
#include <thread>
#include <mutex>

using namespace std;
using namespace boost::asio;

namespace boost_rs485
{
    static const uint32_t proto_max_buff    = 32;

    class Boost_RS485_Master : public i_transport::ITransport
    {
    private:
        boost::asio::io_service    m_ioService;
        boost::asio::serial_port   m_port;
        int m_fd;
        uint8_t m_recvdData[proto_max_buff] = {0};
        uint8_t m_copyRecvdData[proto_max_buff] = {0};
        bool m_recvd = false;
        uint32_t m_sendCount = 0;
        uint32_t m_recvdCount = 0;
        std::mutex channel__access;
        size_t recvd_bytes_RS = 0;

        // static std::size_t completion_condition( const boost::system::error_code& error, std::size_t bytes_transferred){
        //     if(error){
        //         std::cout << error.what();
        //         exit(-1);
        //     }
        //     /* Ждем */
        //     std::cout << "bytes_transferred = " << bytes_transferred << std::endl;
        //     std::this_thread::sleep_for(std::chrono::microseconds(1000));
        //     return 0;
        // }

        void read_handler(const boost::system::error_code& error,size_t bytes_transferred)
        {
            if(!error && bytes_transferred > 0){
                std::chrono::microseconds mcs = std::chrono::duration_cast< std::chrono::microseconds >
                    (std::chrono::system_clock::now().time_since_epoch());
                //std::cout << "\nread from rs microseconds = " << mcs.count();
                m_recvdCount++;
                m_recvd = true;
                recvd_bytes_RS = bytes_transferred;
                std::memset(m_copyRecvdData, 0, sizeof(m_copyRecvdData));
                memcpy(m_copyRecvdData, m_recvdData, sizeof(m_copyRecvdData));
                // std::cout << "\nport read returns: " + error.message();
                // printf("\n[M RECEIVED]:\n"
                // "[%u][%u][%u][%u\t][%u][%u][%u][%u][%u][%u][%u][%u][%u][%u][%u][%u]\n"
                // "m_recvdCount = %u\n"
                // "m_sendCount = %u\n",
                // m_copyRecvdData[0], m_copyRecvdData[1], m_copyRecvdData[2], m_copyRecvdData[3],
                // m_copyRecvdData[4], m_copyRecvdData[5], m_copyRecvdData[6], m_copyRecvdData[7], 
                // m_copyRecvdData[8], m_copyRecvdData[9], m_copyRecvdData[10], m_copyRecvdData[11],
                // m_copyRecvdData[12], m_copyRecvdData[13], m_copyRecvdData[14], m_copyRecvdData[15], m_recvdCount, m_sendCount);
                // cout << "bytes_transferred: "<< bytes_transferred << endl;
            } else {
                std::cout << "\n[ERROR RESEIVED FROM RS485]\n";
            }
            getData();
        }

    public: 
        Boost_RS485_Master(string dev_Port, uint32_t baudrate):m_ioService(),m_port(m_ioService, dev_Port)
        {
            termios t;
            m_fd = m_port.native_handle();
            if (tcgetattr(m_fd, &t) < 0) { /* handle error */ }
            if (cfsetspeed(&t, baudrate) < 0) { /* handle error */ }
            if (tcsetattr(m_fd, TCSANOW, &t) < 0) { /* handle error */ }
            //m_port.set_option(boost::asio::serial_port_base::baud_rate(baudrate));
            m_port.set_option(boost::asio::serial_port_base::character_size(8));
            m_port.set_option(boost::asio::serial_port_base::stop_bits(serial_port_base::stop_bits::one));
            m_port.set_option(boost::asio::serial_port_base::parity(serial_port_base::parity::none));
            m_port.set_option(boost::asio::serial_port_base::flow_control(serial_port_base::flow_control::none));

            boost::thread td(boost::bind(&boost::asio::io_service::run, &m_ioService));
            //td.join();
            getData();

        }

        bool sendData(const uint8_t* ptrData, uint32_t len)
        {
            std::chrono::microseconds mcs = std::chrono::duration_cast< std::chrono::microseconds >
                (std::chrono::system_clock::now().time_since_epoch());
            std::cout << "\nsend to rs microseconds = " << mcs.count();
            boost::system::error_code error;
            size_t sendBytes = m_port.write_some(boost::asio::buffer(ptrData, len), error);
            if(!error && sendBytes > 0){
                m_sendCount++;
                std::cout << "\nport write returns: " + error.message();
                printf("\n[I SEND]:\n"
                "[%u][%u][%u][%u\t][%u][%u][%u][%u][%u][%u][%u][%u][%u][%u][%u][%u]\n"
                "m_recvdCount = %u\n"
                "m_sendCount = %u\n",
                ptrData[0], ptrData[1], ptrData[2], ptrData[3],
                ptrData[4], ptrData[5], ptrData[6], ptrData[7], 
                ptrData[8], ptrData[9], ptrData[10], ptrData[11],
                ptrData[12], ptrData[13], ptrData[14], ptrData[15], m_recvdCount, m_sendCount);
                cout << "sendBytes: "<< sendBytes << endl;
                return true;
            } else {
                //std::cerr << error.what();
                return false;
            }
        }

        bool getData(uint8_t* ptrData, uint32_t* lenInOut)
        {
            if (!m_recvd) return false;
            *lenInOut = recvd_bytes_RS;
            std::memcpy(ptrData, m_copyRecvdData, *lenInOut);
            recvd_bytes_RS = 0;
            m_recvd = false;
            return true;
        }

        void getData(){
            std::memset(m_recvdData, 0, sizeof(m_recvdData));
            m_port.async_read_some(boost::asio::buffer(m_recvdData, sizeof(m_recvdData)),
                    boost::bind(&Boost_RS485_Master::read_handler,this,
                            boost::asio::placeholders::error,
                            boost::asio::placeholders::bytes_transferred));
        }

        bool transportReset() {return true;}

        ~Boost_RS485_Master() override {
            m_port.close();
        };
    };
    
/////////////////////////////////////////////////////////////////
    class Boost_RS485_Slave_async : public i_transport::ITransport
    {
    private:
        boost::asio::io_service    s_ioService;
        boost::asio::serial_port   async_port;
        int async_fd;
        uint8_t s_recvdData[proto_max_buff] = {0};
        bool s_recvd = false;
        uint32_t s_sendCount = 0;
        uint32_t s_recvdCount = 0;

        void read_handler(const boost::system::error_code& error,size_t bytes_transferred)
        {
            if(!error && bytes_transferred > 0){
                s_recvdCount++;
                std::cout << "\nport read returns: " + error.message();
                s_recvd = true;
                printf("\n[I RECEIVED]:\n"
                "[%u][%u][%u][%u\t][%u][%u][%u][%u][%u][%u][%u][%u][%u][%u][%u][%u]\n"
                "s_recvdCount = %u\n"
                "s_sendCount = %u\n",
                s_recvdData[0], s_recvdData[1], s_recvdData[2], s_recvdData[3],
                s_recvdData[4], s_recvdData[5], s_recvdData[6], s_recvdData[7], 
                s_recvdData[8], s_recvdData[9], s_recvdData[10], s_recvdData[11],
                s_recvdData[12], s_recvdData[13], s_recvdData[14], s_recvdData[15], s_recvdCount, s_sendCount);
                cout << "bytes_transferred: "<< bytes_transferred << endl;
            }
            getData();
        }

    public:
        Boost_RS485_Slave_async(string dev_Port, uint32_t baudrate):s_ioService(),async_port(s_ioService, dev_Port)
        {
            termios t;
            async_fd = async_port.native_handle();
            if (tcgetattr(async_fd, &t) < 0) { /* handle error */ }
            if (cfsetspeed(&t, baudrate) < 0) { /* handle error */ }
            if (tcsetattr(async_fd, TCSANOW, &t) < 0) { /* handle error */ }
            //async_port.set_option(boost::asio::serial_port_base::baud_rate(baudrate));
            async_port.set_option(boost::asio::serial_port_base::character_size(8));
            async_port.set_option(boost::asio::serial_port_base::stop_bits(serial_port_base::stop_bits::one));
            async_port.set_option(boost::asio::serial_port_base::parity(serial_port_base::parity::none));
            async_port.set_option(boost::asio::serial_port_base::flow_control(serial_port_base::flow_control::none));

            boost::thread td(boost::bind(&boost::asio::io_service::run, &s_ioService));
            //td.join();
            getData();
        }

        bool sendData(const uint8_t* ptrData, uint32_t len)
        {
            boost::system::error_code error;
            size_t sendBytes = async_port.write_some(boost::asio::buffer(ptrData, len), error);
            if(!error && sendBytes > 0){
                s_sendCount++;
                std::cout << "\nport write returns: " + error.message();
                printf("\n[I SEND]:\n"
                "[%u][%u][%u][%u\t][%u][%u][%u][%u][%u][%u][%u][%u][%u][%u][%u][%u]\n"
                "s_recvdCount = %u\n"
                "s_sendCount = %u\n",
                ptrData[0], ptrData[1], ptrData[2], ptrData[3],
                ptrData[4], ptrData[5], ptrData[6], ptrData[7], 
                ptrData[8], ptrData[9], ptrData[10], ptrData[11],
                ptrData[12], ptrData[13], ptrData[14], ptrData[15], s_recvdCount, s_sendCount);
                cout << "sendBytes: "<< sendBytes << endl;
                return true;
            } else {
                //std::cerr << error.what();
                return false;
            }
        }

        bool getData(uint8_t* ptrData, uint32_t* lenInOut)
        {
            if (!s_recvd) return false;
            std::memcpy(ptrData, s_recvdData, sizeof(ptrData));
            s_recvd = false;
            return true;
        }

        void getData(){
            std::memset(s_recvdData, 0, sizeof(s_recvdData));
            async_port.async_read_some(boost::asio::buffer(s_recvdData, sizeof(s_recvdData)),
                    boost::bind(&Boost_RS485_Slave_async::read_handler,this,
                            boost::asio::placeholders::error,
                            boost::asio::placeholders::bytes_transferred));
        }

        bool transportReset() {return true;}

        ~Boost_RS485_Slave_async() override {
            async_port.close();
        };
    };

/////////////////////////////////////////////////////////////////

    class Boost_RS485_Slave_sync : public i_transport::ITransport
    {
    public:
        Boost_RS485_Slave_sync(string dev_Port, uint32_t baudrate):sync_ioService(),sync_port(sync_ioService, dev_Port)
        {
            termios t;
            sync_fd = sync_port.native_handle();
            if (tcgetattr(sync_fd, &t) < 0) { /* handle error */ }
            if (cfsetspeed(&t, baudrate) < 0) { /* handle error */ }
            if (tcsetattr(sync_fd, TCSANOW, &t) < 0) { /* handle error */ }
            //sync_port.set_option(boost::asio::serial_port_base::baud_rate(baudrate));
            sync_port.set_option(boost::asio::serial_port_base::character_size(8));
            sync_port.set_option(boost::asio::serial_port_base::stop_bits(serial_port_base::stop_bits::one));
            sync_port.set_option(boost::asio::serial_port_base::parity(serial_port_base::parity::none));
            sync_port.set_option(boost::asio::serial_port_base::flow_control(serial_port_base::flow_control::none));

            boost::thread td(boost::bind(&boost::asio::io_service::run, &sync_ioService));
            //td.join();
        }
    
        bool sendData(const uint8_t* ptrData, uint32_t len)
        {
            boost::system::error_code error;
            size_t sendBytes = sync_port.write_some(boost::asio::buffer(ptrData, len), error);
            if(!error && sendBytes > 0){
                std::chrono::microseconds mcs = std::chrono::duration_cast< std::chrono::microseconds >
                    (std::chrono::system_clock::now().time_since_epoch());
                std::cout << "\nsend to rs microseconds = " << mcs.count();
                sync_sendCount++;
                std::cout << "\nport write returns: " + error.message();
                printf("\n[I SEND]:\n"
                "[%u][%u][%u][%u\t][%u][%u][%u][%u][%u][%u][%u][%u][%u][%u][%u][%u]\n"
                "sync_recvdCount = %u\n"
                "sync_sendCount = %u\n",
                ptrData[0], ptrData[1], ptrData[2], ptrData[3],
                ptrData[4], ptrData[5], ptrData[6], ptrData[7], 
                ptrData[8], ptrData[9], ptrData[10], ptrData[11],
                ptrData[12], ptrData[13], ptrData[14], ptrData[15], sync_recvdCount, sync_sendCount);
                cout << "sendBytes: "<< sendBytes << endl;
                return true;
            } else {
                //std::cerr << error.what();
                return false;
            }
        }

        bool getData(uint8_t* ptrData, uint32_t* lenInOut)
        {
            boost::system::error_code error;
            size_t recvdBytes = sync_port.read_some(boost::asio::buffer(ptrData, *lenInOut), error);
            //size_t recvdBytes = boost::asio::read(sync_port, boost::asio::buffer(ptrData, *lenInOut), error);
            if(!error && recvdBytes > 0){
                std::chrono::microseconds mcs = std::chrono::duration_cast< std::chrono::microseconds >
                    (std::chrono::system_clock::now().time_since_epoch());
                std::cout << "\nread from rs microseconds = " << mcs.count();
                sync_recvdCount++;
                std::cout << "\nport read returns: " + error.message();
                printf("\n[I RECEIVED]:\n"
                "[%u][%u][%u][%u\t][%u][%u][%u][%u][%u][%u][%u][%u][%u][%u][%u][%u]\n"
                "sync_recvdCount = %u\n"
                "sync_sendCount = %u\n",
                ptrData[0], ptrData[1], ptrData[2], ptrData[3],
                ptrData[4], ptrData[5], ptrData[6], ptrData[7], 
                ptrData[8], ptrData[9], ptrData[10], ptrData[11],
                ptrData[12], ptrData[13], ptrData[14], ptrData[15], sync_recvdCount, sync_sendCount);
                cout << "recvdBytes: "<< recvdBytes << endl;
                return true;
            } else {
                //std::cerr << error.what();
                return false;
            }
        }

        bool transportReset() {return true;}

        ~Boost_RS485_Slave_sync() override {
            sync_port.close();
        };

    private:
        boost::asio::io_service    sync_ioService;
        boost::asio::serial_port   sync_port;
        int sync_fd;
        uint32_t sync_sendCount = 0;
        uint32_t sync_recvdCount = 0;
    };
}

