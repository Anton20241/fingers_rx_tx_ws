#include "boost_rs485.hpp"
#include "protocol.hpp"
#include <tabl_reg_config.hpp>
#include <iostream>
#include <ros/ros.h>

class Boost_RS485_Client
{
public:
    Boost_RS485_Client(protocol::Protocol& protocol_, tabl_reg::TablReg& tabl_)
    : m_protocol(protocol_), m_tabl(tabl_){};

    void pollingSensors(){
        size_t count = 0;

        while(1){
            uint16_t angle = 15;
            uint16_t force = 20;
            uint16_t current = 20;
            uint8_t angle_l = angle >> 8;
            uint8_t angle_h = angle & 0xFF;
            uint8_t force_l = force>>8;
            uint8_t force_h = force & 0xFF;
            uint8_t current_l = current >> 8;
            uint8_t current_h = current & 0xFF;
            uint8_t calibrate = 0x01;
            uint8_t data[5] = {angle_l, angle_h, force_l, force_h, calibrate };
            uint16_t offSet = 1; //???
            m_tabl.setRegRaw(data, offSet, 6);
            m_protocol.process();
            //std::cout << "\n<------[count] = " << count << "------>\n" << std::endl;
            count++;
        } 
    }
private:
    protocol::Protocol& m_protocol;
    tabl_reg::TablReg& m_tabl;
};

int main(int argc, char** argv) {

	if(argc != 4) {
		std::cerr << "[program_name][devAddr(1, 2...)][devPort(1, 2...)][baudrate]" << std::endl;
		return -1;
	}

    std::string devAddr = argv[1];
    std::string devPort = argv[2];
    std::string baudrate = argv[3];

    printf("(uint32_t)std::stoi(baudrate) = %u\n", (uint32_t)std::stoi(baudrate));

	try{
        std::cout << "finger is running!" << std::endl;
        ros::init(argc, argv, "finger");
        boost_rs485::Boost_RS485_Async boostRS485_transp("/dev/ttyUSB" + devPort, (uint32_t)std::stoi(baudrate));
        tabl_reg::TablReg m_tabl(tabl_reg_cfg::tablRegConfig);
        protocol::Protocol boostRS485_prot(boostRS485_transp, m_tabl, (uint8_t)std::stoi(devAddr));
        Boost_RS485_Client client(boostRS485_prot, m_tabl);
        while(ros::ok()){
        client.pollingSensors();
        ros::spinOnce();
        }
	} catch (std::exception e){
		std::cerr << "Exeption: " << e.what() << std::endl;
	}
  return 0;
    
}