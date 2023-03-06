#include "boost_rs485.hpp"
#include "protocol.hpp"
#include <tabl_reg_config.hpp>
#include <iostream>
#include <ros/ros.h>

class Boost_RS485_Server
{
public:
    Boost_RS485_Server(protocol_master::ProtocolMaster& protocol_)
    : m_protocol(protocol_){};

    void polling(){
        size_t count = 0;
        std::vector <bool> answRecvd(device.size(), 0);
        std::vector <size_t> answRecvdFailedCount(device.size(), 0);
        int low_C_S_Time = 0;
        int high_C_S_Time = 3800;
        //create trackbars
        //cv::namedWindow("Trackbars");
        //cv::createTrackbar("C_S_Time", "Trackbars", &low_C_S_Time, high_C_S_Time);
        

        while(1){
            //cv::waitKey(1);
            
            uint16_t angle = 15;
            uint16_t force = 20;
            uint16_t current = 20;
            uint8_t angle_l = angle >> 8;
            uint8_t angle_h = angle & 0xFF;
            uint8_t force_l = force>>8;
            uint8_t force_h = force & 0xFF;
            uint8_t current_l = current >> 8;
            uint8_t current_h = current & 0xFF;
            uint8_t data[6] = {angle_l, angle_h, force_l, force_h, current_l, current_h };

            for (int i = 0; i < device.size(); i++){
                auto start = std::chrono::high_resolution_clock::now();
                answRecvd[i] = m_protocol.sendCmdNOP(device[i]);

                if (!answRecvd[i])
                    answRecvdFailedCount[i]++;
                else 
                    answRecvdFailedCount[i] = 0;

                if (answRecvdFailedCount[i] == 10) {
                    std::cout << "\n\n\n!!!DEVICE[" << i << "] CONNECT ERROR!!!\n\n\n";
                }
                std::this_thread::sleep_for(std::chrono::microseconds(1000));

                auto end = std::chrono::high_resolution_clock::now();
                std::chrono::duration<float> duration = end - start;
                float mls = duration.count() * 1000;
                std::cout << "mls: " << mls << std::endl;
            }
            //std::cout << "\n<------[count] = " << count << "------>\n" << std::endl;
            count++; 
        }
    }
private:
    protocol_master::ProtocolMaster& m_protocol;
    std::vector<uint8_t> device = {0x11, 0x12};
};

int main(int argc, char** argv) {

	if(argc != 3) {
		std::cerr << "[program_name][devPort(1, 2...)][baudrate]" << std::endl;
		return -1;
	}

    std::string devPort = argv[1];
    std::string baudrate = argv[2];

    std::cout << "rs_master start.\n";

    printf("(uint32_t)std::stoi(baudrate) = %u\n", (uint32_t)std::stoi(baudrate));

	try{
        boost_rs485::Boost_RS485_Master boostRS485_transp("/dev/ttyUSB" + devPort, (uint32_t)std::stoi(baudrate));
        protocol_master::ProtocolMaster boostRS485_prot_master(boostRS485_transp);
        Boost_RS485_Server raspbPi(boostRS485_prot_master);
        raspbPi.polling();
        // while(ros::ok()){
        //     raspbPi.polling();
        //     ros::spinOnce();
        // }
	} catch (std::exception e){
		std::cerr << "Exeption: " << e.what() << std::endl;
	}
    ros::spin();
  return 0;
    
}



