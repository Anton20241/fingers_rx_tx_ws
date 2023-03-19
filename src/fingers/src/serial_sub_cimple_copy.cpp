#include <boost/asio.hpp>
#include <iostream>
#include "umba_crc_table.h"
#include <ros/ros.h>
#include <std_msgs/ByteMultiArray.h>
#include "boost_serial.hpp"
#include <boost/chrono.hpp>
#include "protocol.hpp"
#include <mutex>

#define BUFSIZE 256

void collectPkg(uint8_t* resvdData, size_t resvdBytes, uint8_t* dataUart, uint32_t& dataUartSize){
    if (resvdBytes == 0) return;
    if (dataUartSize >= 8) return;
    if (dataUartSize >= 5 && dataUart[1] == 5) return;
    memcpy(dataUart + dataUartSize, resvdData, resvdBytes);
    dataUartSize += resvdBytes;
}

bool parserOk(uint8_t* dataUart, uint32_t& dataUartSize){
    if (dataUartSize != 8 && dataUartSize != 5)                                                     return false;
    if (dataUart[0] != 0)                                                                           return false;
    if (dataUartSize == 8 && dataUart[1] != 8 || dataUartSize == 5 && dataUart[1] != 5)             return false;
    if (dataUartSize == 8 && dataUart[2] != 0 || dataUartSize == 5 && dataUart[2] != 0x70)          return false;
    uint8_t crc_8 = umba_crc8_table(dataUart, dataUartSize - sizeof(uint8_t));
    if (dataUart[dataUartSize - sizeof(uint8_t)] != crc_8)                                          return false;
    return true;
}

void updateDataUart(uint8_t* dataUart, uint32_t& dataUartSize){
    if (dataUartSize <= 2)                                                                          return;
    if (dataUartSize < 8 && dataUart[1] == 8 || dataUartSize < 5 && dataUart[1] == 5)               return;
    memset(dataUart, 0, dataUartSize);
    dataUartSize = 0;
}

int main(int argc, char *argv[]) {

     if(argc < 4){
         std::cout << "[./serial_pub_simple][/dev/ttyX][baudrate][waiting time (microseconds)]\n";
         return -1;
     }

    std::string port = argv[1];
    std::string baudrate = argv[2];
    std::string timeToWait = argv[3];

    ros::init(argc, argv, "uart_node");


    boost::asio::io_service io;
    // Open serial port
    boost::asio::serial_port serial(io, "/dev/tty" + port);

    // Configure basic serial port parameters: 115.2kBaud, 8N1
    serial.set_option(boost::asio::serial_port_base::baud_rate(std::stoi(baudrate)));
    serial.set_option(boost::asio::serial_port_base::character_size(8 /* data bits */));
    serial.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    serial.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
    serial.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));

    std::cout << "\nserial_sub_simple is running.\n";
    uint8_t data[BUFSIZE] = {0};
    uint8_t dataUart[BUFSIZE] = {0};
    uint32_t dataUartSize = 0;
    uint32_t pkgCount = 0;
    uint32_t count = 0;
    
    while(true) {
    	count ++;
        std::memset(data, 0, sizeof(data));
        size_t resvdBytes = serial.read_some(boost::asio::buffer(data, BUFSIZE));
        collectPkg(data, resvdBytes, dataUart, dataUartSize);

        if (!parserOk(dataUart, dataUartSize)){
            updateDataUart(dataUart, dataUartSize);
        } else{
            //resvd pack to std::cout 
            pkgCount++;
            printf("\nresvd new pack\n");
            for(int i = 0; i < dataUartSize; i++){
                printf("[%u]", dataUart[i]);
            }
            std::cout << std::endl;
            printf("count = %u\n", count);
            printf("pkgCount = %u\n", pkgCount);
        
            memset(dataUart, 0, dataUartSize);
            dataUartSize = 0;
            count = 0;
        }
        std::this_thread::sleep_for (std::chrono::microseconds(std::stoi(timeToWait)));
    }
}