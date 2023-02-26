#include <cstdlib>
#include <iostream>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <stdio.h>
#include <string.h>
#include <ros/ros.h>
#include "umba_crc_table.h"

using boost::asio::ip::udp;
using boost::asio::ip::address;

#define PORT 1234
#define IP "127.0.0.1"

class UDPClient
{
public:
    UDPClient(boost::asio::io_context& io_context)
    : io_context_(io_context), socket_(io_context){
        std::cout << "UDP CLIENT IS RUNNING\n";
        sender_endpoint_ = udp::endpoint(address::from_string(IP), PORT);
        socket_.open(udp::v4());
    }

    void sendMsg() {
        auto start = std::chrono::high_resolution_clock::now();

        uint8_t msg[] =       {0xAA, 0xBB, 42,          //header
                               0x1, 0x2, 0x3, 0x4, 0x5, //data to fingers
                               0x1, 0x2, 0x3, 0x4, 0x5, //data to fingers
                               0x1, 0x2, 0x3, 0x4, 0x5, //data to fingers
                               0x1, 0x2, 0x3, 0x4, 0x5, //data to fingers 
                               0x1, 0x2, 0x3, 0x4, 0x5, //data to fingers
                               0x1, 0x2, 0x3, 0x4, 0x5, //data to fingers
                               0x1,                     //hand_mount
                               0x1,                     //hold_position
                               0x1,                     //camera_from_bat_cam
                               0x1,                     //relay_state
                               0x1,                     //keepalive
                               0x1,                     //keepalive
                               0x1,                     //keepalive
                               0x1                      //keepalive
                               };

        uint8_t crc8 = umba_crc8_table(msg, sizeof(msg));

        uint8_t msgToSend[] = {0xAA, 0xBB, 42,          //header
                               0x1, 0x2, 0x3, 0x4, 0x5, //data to fingers
                               0x1, 0x2, 0x3, 0x4, 0x5, //data to fingers
                               0x1, 0x2, 0x3, 0x4, 0x5, //data to fingers
                               0x1, 0x2, 0x3, 0x4, 0x5, //data to fingers 
                               0x1, 0x2, 0x3, 0x4, 0x5, //data to fingers
                               0x1, 0x2, 0x3, 0x4, 0x5, //data to fingers
                               0x1,                     //hand_mount
                               0x1,                     //hold_position
                               0x1,                     //camera_from_bat_cam
                               0x1,                     //relay_state
                               0x1,                     //keepalive
                               0x1,                     //keepalive
                               0x1,                     //keepalive
                               0x1,                     //keepalive
                               crc8                     //crc8
                               };

        uint8_t msg2[] =      {0xAA, 0xBB, 42,          //header
                               0x1, 0x2, 0x3, 0x4, 0x5, //data to fingers
                               0x1, 0x2, 0x3, 0x4, 0x5, //data to fingers
                               0x1, 0x2, 0x3, 0x4, 0x5, //data to fingers
                               0x1, 0x2, 0x3, 0x4, 0x5, //data to fingers 
                               0x1, 0x2, 0x3, 0x4, 0x5, //data to fingers
                               0x1, 0x2, 0x3, 0x4, 0x5, //data to fingers
                               0x2,                     //hand_mount
                               0x3,                     //hold_position
                               0x4,                     //camera_from_bat_cam
                               0x5,                     //relay_state
                               0x1,                     //keepalive
                               0x1,                     //keepalive
                               0x1,                     //keepalive
                               0x1                      //keepalive
                               };

        uint8_t crc8_2 = umba_crc8_table(msg2, sizeof(msg2));

        uint8_t msgToSend2[] = {0xAA, 0xBB, 42,         //header
                               0x1, 0x2, 0x3, 0x4, 0x5, //data to fingers
                               0x1, 0x2, 0x3, 0x4, 0x5, //data to fingers
                               0x1, 0x2, 0x3, 0x4, 0x5, //data to fingers
                               0x1, 0x2, 0x3, 0x4, 0x5, //data to fingers 
                               0x1, 0x2, 0x3, 0x4, 0x5, //data to fingers
                               0x1, 0x2, 0x3, 0x4, 0x5, //data to fingers
                               0x2,                     //hand_mount
                               0x3,                     //hold_position
                               0x4,                     //camera_from_bat_cam
                               0x5,                     //relay_state
                               0x1,                     //keepalive
                               0x1,                     //keepalive
                               0x1,                     //keepalive
                               0x1,                     //keepalive
                               crc8_2                   //crc8_2
                               };


        static uint32_t send_count = 0;
        boost::system::error_code err;
        if (send_count & 1){
            auto sent = socket_.send_to(boost::asio::buffer(msgToSend), sender_endpoint_, 0, err);
            if (!err && sent > 0){
                std::cout << "SEND TO UDP: ";
                for (int i = 0; i < sizeof(msgToSend); i++){
                    printf("[%u]", msgToSend[i]);
                }
                std::cout << std::endl;
                send_count++;
                std::cout << "Sent Payload = " << sent << "\n";
                std::cout << "send_count = " << send_count << "\n";
                //printf("[crc8 = %u\n]", crc8);
                std::this_thread::sleep_for(std::chrono::microseconds(10000));
            }
        } else {
            auto sent = socket_.send_to(boost::asio::buffer(msgToSend2), sender_endpoint_, 0, err);
            if (!err && sent > 0){
                std::cout << "SEND TO UDP: ";
                for (int i = 0; i < sizeof(msgToSend2); i++){
                    printf("[%u]", msgToSend2[i]);
                }
                std::cout << std::endl;
                send_count++;
                std::cout << "Sent Payload = " << sent << "\n";
                std::cout << "send_count = " << send_count << "\n";
                //printf("[crc8 = %u\n]", crc8);
                std::this_thread::sleep_for(std::chrono::microseconds(10000));
            }
        }

        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<float> duration = end - start;
        float mls = duration.count() * 1000;
        std::cout << "mls: " << mls << std::endl;
    }

    ~UDPClient() {
        socket_.close();
    }

private:
    boost::asio::io_context& io_context_;
    udp::socket socket_;
    udp::endpoint sender_endpoint_;
};

int main(int argc, char* argv[])
{
    // uint8_t msg1[] = {17, 9, 3, 1, 2, 3, 4, 5};
    // uint8_t crc8_ = umba_crc8_table(msg1, sizeof(msg1));
    // std::cout << "crc8_ = ";
    // printf("%u\n", crc8_);
    // while(1){;}

    try{
      std::cout << "udp_client is running!" << std::endl;
      ros::init(argc, argv, "udp_client");
      boost::asio::io_context io_context;
      UDPClient udpClient(io_context);
      while(1){
          udpClient.sendMsg();
          ros::spinOnce();
      }
    } catch (std::exception e){
		  //std::cerr << "Exeption: " << e.what() << std::endl;
    }
    return 0;//
}