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

#define PORT 20002
//#define IP "192.168.88.16"
//#define IP "192.168.43.212"
#define IP "127.0.0.1"

#define RAW_UDP_DATA 1
#define COMPLETE_UDP_DATA 2

#define CODE_PART COMPLETE_UDP_DATA

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

#if CODE_PART == RAW_UDP_DATA
#define UDP_MSG_SIZE 42
        uint8_t msg[] =       {0xAA, 0xBB, UDP_MSG_SIZE,    //header
                               0x1, 0x2, 0x3, 0x4, 0x5      //data to fingers
                               0x1, 0x2, 0x3, 0x4, 0x5      //data to fingers
                               0x1, 0x2, 0x3, 0x4, 0x5      //data to fingers
                               0x1, 0x2, 0x3, 0x4, 0x5      //data to fingers 
                               0x1, 0x2, 0x3, 0x4, 0x5      //data to fingers
                               0x1, 0x2, 0x3, 0x4, 0x5      //data to fingers
                               0x1,                         //hand_mount
                               0x1,                         //hold_position
                               0x1,                         //camera
                               0x1,                         //relay_state
                               0x1,                         //keepalive
                               0x1,                         //keepalive
                               0x1,                         //keepalive
                               0x1                          //keepalive
                               };

        uint8_t crc8 = umba_crc8_table(msg, sizeof(msg));

        uint8_t msgToSend[] = {0xAA, 0xBB, UDP_MSG_SIZE,              //header
                               0x1, 0x2, 0x3, 0x4, 0x5      //data to fingers
                               0x1, 0x2, 0x3, 0x4, 0x5      //data to fingers
                               0x1, 0x2, 0x3, 0x4, 0x5      //data to fingers
                               0x1, 0x2, 0x3, 0x4, 0x5      //data to fingers 
                               0x1, 0x2, 0x3, 0x4, 0x5      //data to fingers
                               0x1, 0x2, 0x3, 0x4, 0x5      //data to fingers
                               0x1,                         //hand_mount
                               0x1,                         //hold_position
                               0x1,                         //camera
                               0x1,                         //relay_state
                               0x1,                         //keepalive
                               0x1,                         //keepalive
                               0x1,                         //keepalive
                               0x1,                         //keepalive
                               crc8                         //crc8
                               };

        uint8_t msg2[] =      {0xAA, 0xBB, UDP_MSG_SIZE,              //header
                               0x1, 0x2, 0x3, 0x4, 0x5      //data to fingers
                               0x1, 0x2, 0x3, 0x4, 0x5      //data to fingers
                               0x1, 0x2, 0x3, 0x4, 0x5      //data to fingers
                               0x1, 0x2, 0x3, 0x4, 0x5      //data to fingers 
                               0x1, 0x2, 0x3, 0x4, 0x5      //data to fingers
                               0x1, 0x2, 0x3, 0x4, 0x5      //data to fingers
                               0x6,                         //hand_mount
                               0x7,                         //hold_position
                               0x8,                         //camera
                               0x9,                         //relay_state
                               0x1,                         //keepalive
                               0x1,                         //keepalive
                               0x1,                         //keepalive
                               0x1                          //keepalive
                               };

        uint8_t crc8_2 = umba_crc8_table(msg2, sizeof(msg2));

        uint8_t msgToSend2[] = {0xAA, 0xBB, UDP_MSG_SIZE,              //header
                               0x1, 0x2, 0x3, 0x4, 0x5      //data to fingers
                               0x1, 0x2, 0x3, 0x4, 0x5      //data to fingers
                               0x1, 0x2, 0x3, 0x4, 0x5      //data to fingers
                               0x1, 0x2, 0x3, 0x4, 0x5      //data to fingers 
                               0x1, 0x2, 0x3, 0x4, 0x5      //data to fingers
                               0x1, 0x2, 0x3, 0x4, 0x5      //data to fingers
                               0x6,                         //hand_mount
                               0x7,                         //hold_position
                               0x8,                         //camera
                               0x9,                         //relay_state
                               0x1,                         //keepalive
                               0x1,                         //keepalive
                               0x1,                         //keepalive
                               0x1,                         //keepalive
                               crc8_2                       //crc8_2
                               };

        uint8_t msg3[] =      {0xAA, 0xBB, UDP_MSG_SIZE,              //header
                               0x1, 0x2, 0x3, 0x4, 0x5      //data to fingers
                               0x1, 0x2, 0x3, 0x4, 0x5      //data to fingers
                               0x1, 0x2, 0x3, 0x4, 0x5      //data to fingers
                               0x1, 0x2, 0x3, 0x4, 0x5      //data to fingers 
                               0x1, 0x2, 0x3, 0x4, 0x5      //data to fingers
                               0x1, 0x2, 0x3, 0x4, 0x5      //data to fingers
                               0x6,                         //hand_mount
                               0x7,                         //hold_position
                               0x2,                         //camera
                               0x9,                         //relay_state
                               0x1,                         //keepalive
                               0x1,                         //keepalive
                               0x1,                         //keepalive
                               0x1                          //keepalive
                               };

        uint8_t crc8_3 = umba_crc8_table(msg3, sizeof(msg3));

        uint8_t msgToSend3[] = {0xAA, 0xBB, UDP_MSG_SIZE,              //header
                               0x1, 0x2, 0x3, 0x4, 0x5      //data to fingers
                               0x1, 0x2, 0x3, 0x4, 0x5      //data to fingers
                               0x1, 0x2, 0x3, 0x4, 0x5      //data to fingers
                               0x1, 0x2, 0x3, 0x4, 0x5      //data to fingers 
                               0x1, 0x2, 0x3, 0x4, 0x5      //data to fingers
                               0x1, 0x2, 0x3, 0x4, 0x5      //data to fingers
                               0x6,                         //hand_mount
                               0x7,                         //hold_position
                               0x2,                         //camera
                               0x9,                         //relay_state
                               0x1,                         //keepalive
                               0x1,                         //keepalive
                               0x1,                         //keepalive
                               0x1,                         //keepalive
                               crc8_3                       //crc8_2
                               };

#elif CODE_PART == COMPLETE_UDP_DATA
#define UDP_MSG_SIZE 36

        uint8_t msg[] =       {0xAA, 0xBB, UDP_MSG_SIZE,              //header
                               0x21, 0x04, 0x10, 0xB4,      //data to fingers
                               0x22, 0x04, 0x10, 0x7E,      //data to fingers
                               0x23, 0x04, 0x10, 0x38,      //data to fingers
                               0x24, 0x04, 0x10, 0xDB,      //data to fingers 
                               0x25, 0x04, 0x10, 0x9D,      //data to fingers
                               0x26, 0x04, 0x10, 0x57,      //data to fingers
                               0x1,                         //hand_mount
                               0x0,                         //hold_position
                               0x1,                         //camera
                               0x1,                         //relay_state
                               0x1,                         //keepalive
                               0x1,                         //keepalive
                               0x1,                         //keepalive
                               0x1                          //keepalive
                               };

        uint8_t crc8 = umba_crc8_table(msg, sizeof(msg));

        uint8_t msgToSend[] = {0xAA, 0xBB, UDP_MSG_SIZE,              //header
                               0x21, 0x04, 0x10, 0xB4,      //data to fingers
                               0x22, 0x04, 0x10, 0x7E,      //data to fingers
                               0x23, 0x04, 0x10, 0x38,      //data to fingers
                               0x24, 0x04, 0x10, 0xDB,      //data to fingers 
                               0x25, 0x04, 0x10, 0x9D,      //data to fingers
                               0x26, 0x04, 0x10, 0x57,      //data to fingers
                               0x1,                         //hand_mount
                               0x0,                         //hold_position
                               0x1,                         //camera
                               0x1,                         //relay_state
                               0x1,                         //keepalive
                               0x1,                         //keepalive
                               0x1,                         //keepalive
                               0x1,                         //keepalive
                               crc8                         //crc8
                               };

        uint8_t msg2[] =      {0xAA, 0xBB, UDP_MSG_SIZE,              //header
                               0x21, 0x04, 0x10, 0xB4,      //data to fingers
                               0x22, 0x04, 0x10, 0x7E,      //data to fingers
                               0x23, 0x04, 0x10, 0x38,      //data to fingers
                               0x24, 0x04, 0x10, 0xDB,      //data to fingers 
                               0x25, 0x04, 0x10, 0x9D,      //data to fingers
                               0x26, 0x04, 0x10, 0x57,      //data to fingers
                               0x6,                         //hand_mount
                               0x7,                         //hold_position
                               0x1,                         //camera
                               0x1,                         //relay_state
                               0x1,                         //keepalive
                               0x1,                         //keepalive
                               0x1,                         //keepalive
                               0x1                          //keepalive
                               };

        uint8_t crc8_2 = umba_crc8_table(msg2, sizeof(msg2));

        uint8_t msgToSend2[] = {0xAA, 0xBB, UDP_MSG_SIZE,             //header
                               0x21, 0x04, 0x10, 0xB4,      //data to fingers
                               0x22, 0x04, 0x10, 0x7E,      //data to fingers
                               0x23, 0x04, 0x10, 0x38,      //data to fingers
                               0x24, 0x04, 0x10, 0xDB,      //data to fingers 
                               0x25, 0x04, 0x10, 0x9D,      //data to fingers
                               0x26, 0x04, 0x10, 0x57,      //data to fingers
                               0x6,                         //hand_mount
                               0x7,                         //hold_position
                               0x1,                         //camera
                               0x1,                         //relay_state
                               0x1,                         //keepalive
                               0x1,                         //keepalive
                               0x1,                         //keepalive
                               0x1,                         //keepalive
                               crc8_2                       //crc8_2
                               };

        uint8_t msg3[] =      {0xAA, 0xBB, UDP_MSG_SIZE,              //header
                               0x21, 0x04, 0x10, 0xB4,      //data to fingers
                               0x22, 0x04, 0x10, 0x7E,      //data to fingers
                               0x23, 0x04, 0x10, 0x38,      //data to fingers
                               0x24, 0x04, 0x10, 0xDB,      //data to fingers 
                               0x25, 0x04, 0x10, 0x9D,      //data to fingers
                               0x26, 0x04, 0x10, 0x57,      //data to fingers
                               0x7,                         //hand_mount
                               0x8,                         //hold_position
                               0x1,                         //camera
                               0x1,                         //relay_state
                               0x1,                         //keepalive
                               0x1,                         //keepalive
                               0x1,                         //keepalive
                               0x1                          //keepalive
                               };

        uint8_t crc8_3 = umba_crc8_table(msg3, sizeof(msg3));

        uint8_t msgToSend3[] = {0xAA, 0xBB, UDP_MSG_SIZE,             //header
                               0x21, 0x04, 0x10, 0xB4,      //data to fingers
                               0x22, 0x04, 0x10, 0x7E,      //data to fingers
                               0x23, 0x04, 0x10, 0x38,      //data to fingers
                               0x24, 0x04, 0x10, 0xDB,      //data to fingers 
                               0x25, 0x04, 0x10, 0x9D,      //data to fingers
                               0x26, 0x04, 0x10, 0x57,      //data to fingers
                               0x7,                         //hand_mount
                               0x8,                         //hold_position
                               0x1,                         //camera
                               0x1,                         //relay_state
                               0x1,                         //keepalive
                               0x1,                         //keepalive
                               0x1,                         //keepalive
                               0x1,                         //keepalive
                               crc8_3                       //crc8_2
                               };

#else 

#error "INCORRECT CODE_PART"

#endif

        std::vector <uint8_t*> msg_vec = {msgToSend, msgToSend2, msgToSend3};

        static uint32_t send_count = 0;
        static uint32_t send_count_chanched = 0;
        boost::system::error_code err;

        for (size_t i = 0; i < msg_vec.size(); i++)
        {
            if (send_count % 200 == 0 || send_count_chanched) {
                send_count_chanched++;
                //msg_vec[i][29] = 0x02;
                msg_vec[i][30] = 0x02;
                msg_vec[i][UDP_MSG_SIZE - sizeof(uint8_t)] = umba_crc8_table(msg_vec[i], UDP_MSG_SIZE - sizeof(uint8_t));
                std::cout << "\n\n\n!!!!!!!!!!!!!!SEND TO CAM_BAT!!!!!!!!!!!!!\n\n\n";
                if (send_count_chanched == 100) {
                    send_count_chanched = 0;
                }   
            }
            auto start = std::chrono::high_resolution_clock::now();
            auto sent = socket_.send_to(boost::asio::buffer(msg_vec[i], UDP_MSG_SIZE), sender_endpoint_, 0, err);
            if (!err && sent > 0){
                std::cout << "SEND TO UDP: ";   
                for (int k = 0; k < UDP_MSG_SIZE; k++){
                    printf("[%u]", msg_vec[i][k]);
                }
                std::cout << std::endl;
                send_count++;
                std::cout << "Sent Payload = " << sent << "\n";
                std::cout << "send_count = " << send_count << "\n";
                //printf("[crc8 = %u\n]", crc8);
                std::this_thread::sleep_for(std::chrono::microseconds(10000));
            }
            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<float> duration = end - start;
            float mls = duration.count() * 1000;
            std::cout << "mls: " << mls << std::endl;
        }

        
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
      uint32_t count = 0;
      while(count < 10000){
          udpClient.sendMsg();
          ros::spinOnce();
          count++;
      }
    } catch (std::exception e){
		  //std::cerr << "Exeption: " << e.what() << std::endl;
    }
    return 0;
}