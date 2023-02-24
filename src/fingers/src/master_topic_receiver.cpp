#include <iostream>
#include <ros/ros.h>
#include <boost/asio.hpp>
#include <std_msgs/ByteMultiArray.h>
#include "boost_rs485.hpp"
#include "protocol.hpp"
#include "umba_crc_table.h"
#include <chrono>
#include <thread>
#include <chrono>
#include <ctime>

class Boost_RS485_Server
{
public:
    Boost_RS485_Server(protocol_master::ProtocolMaster& protocol_)
    : m_protocol(protocol_){
        toFingersSub = node.subscribe<std_msgs::ByteMultiArray>("toFingersTopic", 10, &Boost_RS485_Server::topic_handle_receive, this);
        fromFingersPub = node.advertise<std_msgs::ByteMultiArray>("fromFingersTopic", 10);
    };

    void nodeFromTopicProcess(){
        if (getMsgFromTopic){
            update_brush_holder();
            toEachFinger();
            sendMsgToTopic();
            getMsgFromTopic = false;
        }
    }

private:
    protocol_master::ProtocolMaster& m_protocol;
    uint8_t dataFromTopic[31] = {0};
    uint8_t dataToTopic[31] = {0};
    uint8_t dataToFinger[5] = {0};
    uint8_t dataFromFinger[5] = {0};
    ros::NodeHandle node;
    ros::Publisher fromFingersPub;
    ros::Subscriber toFingersSub;
    uint32_t recvd_count_topic = 0;
    uint32_t send_count_rs = 0;
    uint32_t recvd_count_rs = 0;
    uint32_t send_count_topic = 0;
    bool getMsgFromTopic = false;
    std::vector<uint8_t> fingersAddrs = {0x11, 0x12, 0x13, 0x14, 0x15, 0x16};
    uint8_t brush_holder_addr = 0x31;
    uint8_t to_brush_holder_status = 0;
    uint8_t from_brush_holder_status = 0;

    void update_brush_holder(){
        if (to_brush_holder_status == dataFromTopic[30]) return;
        to_brush_holder_status = dataFromTopic[30];
        std::cout << "update brush holder = ";
        printf("%u", to_brush_holder_status);
        m_protocol.sendCmdReadWrite(brush_holder_addr, &to_brush_holder_status, sizeof(uint8_t), 
                                                &from_brush_holder_status, sizeof(uint8_t));
        memcpy(dataToTopic + 30, &from_brush_holder_status, sizeof(uint8_t));
    }

    void topic_handle_receive(const std_msgs::ByteMultiArray::ConstPtr& recvdMsg) {
        getMsgFromTopic = true;
        recvd_count_topic++;
        std::cout << "RECVD FROM TOPIC toFingersTopic recvdMsg->data.size() = " << recvdMsg->data.size() << std::endl;
        std::cout << "recvd_count_topic = " << recvd_count_topic << std::endl;
        memset(dataFromTopic, 0, sizeof(dataFromTopic));
        for (int i = 0; i < recvdMsg->data.size(); i++){
            dataFromTopic[i] = recvdMsg->data[i];
            printf("[%u]", dataFromTopic[i]);
        }
        std::cout << endl;
    }

	void toEachFinger(){
		for (int i = 0; i < fingersAddrs.size(); i++){
			memset(dataToFinger, 0, sizeof(dataToFinger));
            memset(dataToTopic, 0, sizeof(dataToTopic));
			memcpy(dataToFinger, dataFromTopic + i * sizeof(dataToFinger), sizeof(dataToFinger));
            std::cout << "\ndataToFinger = ";
            for (int i = 0; i < sizeof(dataToFinger); i++){
                printf("[%u]", dataToFinger[i]);
            }
			m_protocol.sendCmdReadWrite(fingersAddrs[i], dataToFinger, sizeof(dataToFinger), 
                                                    dataFromFinger, sizeof(dataFromFinger));
			memcpy(dataToTopic + i * sizeof(dataFromFinger), dataFromFinger, sizeof(dataFromFinger));
		}
	}

    void sendMsgToTopic(){
        std_msgs::ByteMultiArray sendMsgFromFingersTopic;
        sendMsgFromFingersTopic.layout.dim.push_back(std_msgs::MultiArrayDimension());
        sendMsgFromFingersTopic.layout.dim[0].size = 1;
        sendMsgFromFingersTopic.layout.dim[0].stride = sizeof(dataToTopic);
        sendMsgFromFingersTopic.data.clear();
        std::cout << "\nSEND MSG TO TOPIC = ";
        for (int i = 0; i < sizeof(dataToTopic); i++){
            sendMsgFromFingersTopic.data.push_back(dataToTopic[i]);
            printf("[%u]", dataToTopic[i]);
        }
        fromFingersPub.publish(sendMsgFromFingersTopic);
        std::cout << endl;
    }
};

int main(int argc, char** argv){
	std::string devPort = "";
	if(argc == 2) {
		devPort = argv[1];
	} else {
		std::cerr << "[program_name][devPort(1, 2...)]" << std::endl;
		return -1;
	}

	try{
		std::cout << "master_topic_receiver is running!" << std::endl;
		ros::init(argc, argv, "master_topic_receiver");
		boost_rs485::Boost_RS485_Master boostRS485_transp("/dev/ttyUSB" + devPort);
		protocol_master::ProtocolMaster boostRS485_prot_master(boostRS485_transp);
		Boost_RS485_Server raspbPi(boostRS485_prot_master);
        while(ros::ok()){
            raspbPi.nodeFromTopicProcess();
            ros::spinOnce();
        }
	} catch (std::exception e){
		//std::cerr << "Exeption: " << e.what() << std::endl;
	}
  return 0;
};
