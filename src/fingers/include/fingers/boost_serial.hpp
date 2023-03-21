/*
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

namespace boost_serial
{
    static const uint32_t proto_max_buff    = 32;

    class Boost_Serial_Async : public i_transport::ITransport
    {
    private:
        boost::asio::io_service    m_ioService;
        boost::asio::serial_port   m_port;
        int m_fd;
        uint8_t m_recvdData[proto_max_buff] = {0};
        std::vector<uint8_t> m_copyRecvdData;
        uint32_t m_sendCount = 0;
        uint32_t m_recvdCount = 0;
        boost::mutex my_mytex;
        bool get_bytes = false;
        
        void read_handler(const boost::system::error_code& error,size_t bytes_transferred)
        {
            my_mytex.lock();
            if(!error && bytes_transferred > 0){
                m_recvdCount++;
                
                if(send_error) {
                    m_copyRecvdData.clear();
                    send_error = false;
                }
                
                for (size_t i = 0; i < bytes_transferred; i++){
                    m_copyRecvdData.push_back(m_recvdData[i]);
                }
                printf("\n[RECEIVED]:\n");
                for (size_t i = 0; i < m_copyRecvdData.size(); i++)
                {
                printf("[%u]", m_copyRecvdData[i]);
                }
                std::cout << std::endl;
                  
                // "[%u][%u][%u][%u][%u][%u][%u][%u]\n"
                // "m_recvdCount = %u\n"
                // "m_sendCount = %u\n",
                // m_recvdData[0], m_recvdData[1], m_recvdData[2], m_recvdData[3],
                // m_recvdData[4], m_recvdData[5], m_recvdData[6], m_recvdData[7], m_recvdCount, m_sendCount);
                cout << "bytes_transferred: "<< bytes_transferred << endl;
            } else {
                std::cout << "\n\033[1;31m[ERROR RESEIVED FROM SERIAL]\033[0m\n";
            }
            my_mytex.unlock();
            read_msg_serial();
        }

    public: 
        
        Boost_Serial_Async(string dev_Port, uint32_t baudrate):m_ioService(),m_port(m_ioService, dev_Port)
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
            read_msg_serial();

        }

        bool sendData(const uint8_t* ptrData, uint32_t len)
        {
            boost::system::error_code error;
            size_t sendBytes = m_port.write_some(boost::asio::buffer(ptrData, len), error);
            if(!error && sendBytes > 0){
                // m_sendCount++;
                // std::cout << "\nport write returns: " + error.message();
                // printf("\n[I SEND]:\n"
                // "[%u][%u][%u][%u\t][%u][%u][%u][%u][%u][%u][%u][%u][%u][%u][%u][%u]\n"
                // "m_recvdCount = %u\n"
                // "m_sendCount = %u\n",
                // ptrData[0], ptrData[1], ptrData[2], ptrData[3],
                // ptrData[4], ptrData[5], ptrData[6], ptrData[7], 
                // ptrData[8], ptrData[9], ptrData[10], ptrData[11],
                // ptrData[12], ptrData[13], ptrData[14], ptrData[15], m_recvdCount, m_sendCount);
                // cout << "sendBytes: "<< sendBytes << endl;
                return true;
            } else {
                //std::cerr << error.what();
                return false;
            }
        }

        bool getData(uint8_t* ptrData, uint32_t* lenInOut)
        {
            my_mytex.lock();

            //std::cout << "getData(...)\n";

            if (m_copyRecvdData.size() < 2){
                std::cout << "m_copyRecvdData.size() < 2\n";
                my_mytex.unlock();
                return false;
            }
            
            uint32_t packageLen = m_copyRecvdData[1];

            if (packageLen > 8 || packageLen == 0){
                std::cout << "packageLen > 8 || packageLen == 0\n";
                m_copyRecvdData.clear();
                my_mytex.unlock();
                return false;
            }

            if (m_copyRecvdData.size() < packageLen){
                std::cout << "packageLen > 8 || packageLen == 0\n";
                my_mytex.unlock();
                return false;
            }

            std::cout << "m_copyRecvdData:\n";
            for (int i = 0; i < packageLen; i++){
                printf("[%u]", m_copyRecvdData[i]);
                ptrData[i] = m_copyRecvdData[i];
            }
            std::cout << std::endl;        

            *lenInOut = packageLen;
            std::cout << "*lenInOut = " << *lenInOut << std::endl;

            std::vector <uint8_t> tmp_vec = m_copyRecvdData;
            m_copyRecvdData.clear();
            for (size_t i = 0; i < tmp_vec.size() - packageLen; i++){
                m_copyRecvdData.push_back(tmp_vec[i + packageLen]);
            }

            my_mytex.unlock();
            return true;
        }

        void read_msg_serial(){
            std::memset(m_recvdData, 0, sizeof(m_recvdData));
            m_port.async_read_some(boost::asio::buffer(m_recvdData, sizeof(m_recvdData)),
                    boost::bind(&Boost_Serial_Async::read_handler,this,
                            boost::asio::placeholders::error,
                            boost::asio::placeholders::bytes_transferred));
        }

        bool transportReset() {return true;}

        ~Boost_Serial_Async() override {
            m_port.close();
        };

    };

/////////////////////////////////////////////////////////////////

    // class Boost_RS485_Sync : public i_transport::ITransport
    // {
    // public:
    //     Boost_RS485_Sync(string dev_Port, uint32_t baudrate):sync_ioService(),sync_port(sync_ioService, dev_Port)
    //     {
    //         termios t;
    //         sync_fd = sync_port.native_handle();
    //         if (tcgetattr(sync_fd, &t) < 0) { /* handle error */ }
    //         if (cfsetspeed(&t, baudrate) < 0) { /* handle error */ }
    //         if (tcsetattr(sync_fd, TCSANOW, &t) < 0) { /* handle error */ }
    //         //sync_port.set_option(boost::asio::serial_port_base::baud_rate(baudrate));
    //         sync_port.set_option(boost::asio::serial_port_base::character_size(8));
    //         sync_port.set_option(boost::asio::serial_port_base::stop_bits(serial_port_base::stop_bits::one));
    //         sync_port.set_option(boost::asio::serial_port_base::parity(serial_port_base::parity::none));
    //         sync_port.set_option(boost::asio::serial_port_base::flow_control(serial_port_base::flow_control::none));

    //         boost::thread td(boost::bind(&boost::asio::io_service::run, &sync_ioService));
    //         // td.join();
    //     }
    
    //     bool sendData(const uint8_t* ptrData, uint32_t len)
    //     {
    //         boost::system::error_code error;
    //         size_t sendBytes = sync_port.write_some(boost::asio::buffer(ptrData, len), error);
    //         if(!error && sendBytes > 0){
    //             // std::chrono::microseconds mcs = std::chrono::duration_cast< std::chrono::microseconds >
    //             //     (std::chrono::system_clock::now().time_since_epoch());
    //             // std::cout << "\nsend to rs microseconds = " << mcs.count();
    //             // sync_sendCount++;
    //             // std::cout << "\nport write returns: " + error.message();
    //             // printf("\n[I SEND]:\n"
    //             // "[%u][%u][%u][%u\t][%u][%u][%u][%u][%u][%u][%u][%u][%u][%u][%u][%u]\n"
    //             // "sync_recvdCount = %u\n"
    //             // "sync_sendCount = %u\n",
    //             // ptrData[0], ptrData[1], ptrData[2], ptrData[3],
    //             // ptrData[4], ptrData[5], ptrData[6], ptrData[7], 
    //             // ptrData[8], ptrData[9], ptrData[10], ptrData[11],
    //             // ptrData[12], ptrData[13], ptrData[14], ptrData[15], sync_recvdCount, sync_sendCount);
    //             // cout << "sendBytes: "<< sendBytes << endl;
    //             return true;
    //         } else {
    //             //std::cerr << error.what();
    //             return false;
    //         }
    //     }

    //     bool getData(uint8_t* ptrData, uint32_t* lenInOut)
    //     {
    //         boost::system::error_code error;
    //         uint32_t ptrDataSize = 256;
    //         size_t recvdBytes = sync_port.read_some(boost::asio::buffer(ptrData, ptrDataSize), error);
    //         //size_t recvdBytes = boost::asio::read(sync_port, boost::asio::buffer(ptrData, *lenInOut), error);
            
    //         if(!error && recvdBytes > 0){
    //             *lenInOut = recvdBytes; 
    //             // std::cout << "\n!!!!!!!I AM HERE!!!!!!\n";
    //             // std::chrono::microseconds mcs = std::chrono::duration_cast< std::chrono::microseconds >
    //             //     (std::chrono::system_clock::now().time_since_epoch());
    //             // std::cout << "\nread from rs microseconds = " << mcs.count();
    //             // sync_recvdCount++;
    //             // std::cout << "\nport read returns: " + error.message();
    //             // printf("\n[I RECEIVED]:\n"
    //             // "[%u][%u][%u][%u\t][%u][%u][%u][%u][%u][%u][%u][%u][%u][%u][%u][%u]\n"
    //             // "sync_recvdCount = %u\n"
    //             // "sync_sendCount = %u\n",
    //             // ptrData[0], ptrData[1], ptrData[2], ptrData[3],
    //             // ptrData[4], ptrData[5], ptrData[6], ptrData[7], 
    //             // ptrData[8], ptrData[9], ptrData[10], ptrData[11],
    //             // ptrData[12], ptrData[13], ptrData[14], ptrData[15], sync_recvdCount, sync_sendCount);
    //             // cout << "recvdBytes: "<< recvdBytes << endl;
    //             return true;
    //         } else {
    //             //std::cerr << error.what();
    //             return false;
    //         }
    //     }

    //     bool transportReset() {return true;}

    //     ~Boost_RS485_Sync() override {
    //         sync_port.close();
    //     };

    // private:
    //     boost::asio::io_service    sync_ioService;
    //     boost::asio::serial_port   sync_port;
    //     int sync_fd;
    //     uint32_t sync_sendCount = 0;
    //     uint32_t sync_recvdCount = 0;
    // };
}

