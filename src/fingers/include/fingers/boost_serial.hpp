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
        std::mutex my_mytex;
        uint32_t bytesGet = 0;
        
        void read_handler(const boost::system::error_code& error, size_t bytes_transferred)
        {
            std::cout << "\n\n!!!!!!!!!read_handler!!!!!!!!!\n\n";
            my_mytex.lock();
            if(!error && bytes_transferred > 0){
                m_recvdCount++;
                
                for (size_t i = 0; i < bytes_transferred; i++){
                    m_copyRecvdData.push_back(m_recvdData[i]);
                }
                printf("\n[RECEIVED]:\n");
                for (size_t i = 0; i < m_copyRecvdData.size(); i++)
                {
                    printf("[%u]", m_copyRecvdData[i]);
                }
                std::cout << std::endl;
                cout << "bytes_transferred: "<< bytes_transferred << endl;
            } else {
                std::cout << "\n\033[1;31m[ERROR RESEIVED FROM SERIAL]\033[0m\n";
            }
            my_mytex.unlock();
            read_msg_serial();
        }
        
        void read_msg_serial()
        {
            memset(m_recvdData, 0, sizeof(m_recvdData));
            m_port.async_read_some(boost::asio::buffer(m_recvdData,sizeof(m_recvdData)),
                    boost::bind(&Boost_Serial_Async::read_handler,this,
                            boost::asio::placeholders::error,
                            boost::asio::placeholders::bytes_transferred));
        }

    public: 
        
        Boost_Serial_Async(std::string port_, std::string boudrate):m_ioService(),m_port(m_ioService, port_)
            {
                // Configure basic serial port parameters
                termios t;
                int m_fd;
                m_fd = m_port.native_handle();
                if (tcgetattr(m_fd, &t) < 0) { /* handle error */ }
                if (cfsetspeed(&t, std::stoi(boudrate)) < 0) { /* handle error */ }
                if (tcsetattr(m_fd, TCSANOW, &t) < 0) { /* handle error */ }
                m_port.set_option(boost::asio::serial_port_base::baud_rate(std::stoi(boudrate)));
                m_port.set_option(boost::asio::serial_port_base::character_size(8 /* data bits */));
                m_port.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
                m_port.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
                m_port.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

                boost::thread td(boost::bind(&boost::asio::io_service::run, &m_ioService));
                read_msg_serial();
            }

            bool sendData(const uint8_t* ptrData, uint32_t len)
            {
                boost::system::error_code error;
                size_t sendBytes = m_port.write_some(boost::asio::buffer(ptrData, len), error);
                if(!error && sendBytes > 0){
                    m_sendCount++;
                    std::cout << "\nport write returns: " + error.message();
                    
                    printf("\n[SEND]:\n");
                    for (size_t i = 0; i < sendBytes; i++)
                    {
                        printf("[%u]", ptrData[i]);
                    }
                    std::cout << std::endl;
                    cout << "sendBytes: "<< sendBytes << endl;
                    return true;
                } else {
                    std::cout << "error.what()\n";
                    return false;
                }
            }

        bool getData(uint8_t* ptrData, uint32_t* lenInOut)
        {
            my_mytex.lock();

            // std::cout << "bytesGet = " << bytesGet << std::endl;
            // std::cout << "m_copyRecvdData.size() = " << m_copyRecvdData.size() << std::endl;

            if (m_copyRecvdData.size() != bytesGet || m_copyRecvdData.empty()){
                //std::cout << "empty or not BytesGet\n";
                bytesGet = m_copyRecvdData.size();
                my_mytex.unlock();
                return false;
            }

            if (m_copyRecvdData.size() < 2 || m_copyRecvdData[1] > 9 || m_copyRecvdData[1] == 0){
                //std::cout << "non-valid m_copyRecvdData length\n";
                m_copyRecvdData.clear();
                my_mytex.unlock();
                return true;
            }
            
            uint32_t packageLen           = m_copyRecvdData[1];
            uint32_t m_copyRecvdData_size = m_copyRecvdData.size();

            if (packageLen > m_copyRecvdData_size) {
                my_mytex.unlock();
                return false;
            }            

            std::cout << "m_copyRecvdData:\n";
            for (int i = 0; i < packageLen; i++){
                ptrData[i] = m_copyRecvdData[i];
                printf("[%u]", m_copyRecvdData[i]);
            }
            std::cout << std::endl;        

            *lenInOut = packageLen;
            std::cout << "*lenInOut = " << *lenInOut << std::endl;

            auto position = m_copyRecvdData.begin();
            auto begin = m_copyRecvdData.begin() + packageLen;
            auto end = begin + m_copyRecvdData.size() - packageLen;
            // std::cout << "\ninsert data: ";
            // for (auto iter = begin; iter != end; iter++) {
            //     printf("[%u]",*iter);
            // }
            
            m_copyRecvdData.insert(position, begin, end);
            

            //std::cout << "\npackageLen = " << packageLen << std::endl;
            //std::cout << "m_copyRecvdData.size() = " <<  m_copyRecvdData.size() << std::endl;

            m_copyRecvdData.erase(m_copyRecvdData.begin(), m_copyRecvdData.begin() + m_copyRecvdData_size);

            //std::cout << "after erase() m_copyRecvdData.size() = " << m_copyRecvdData.size() << std::endl;

            std::cout << "m_copyRecvdData:\n";
            for (int i = 0; i < m_copyRecvdData.size(); i++){
                printf("[%u]", m_copyRecvdData[i]);
            }
            std::cout << std::endl;    

            bytesGet = 0;

            my_mytex.unlock();
            return true;
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

