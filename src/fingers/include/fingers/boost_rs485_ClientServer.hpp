/**
 *  @file       boost_rs485_ClientServer.hpp
 *  @brief
 */

#pragma once
#include "boost_rs485.hpp"
#include "protocol.hpp"
#include "umba_crc_table.h"
#include <chrono>
#include <thread>
#include <chrono>
#include <ctime>
#include <ros/ros.h>
#include <std_msgs/ByteMultiArray.h>

class Boost_RS485_Server
{
public:
    Boost_RS485_Server(protocol_master::ProtocolMaster& protocol_)
    : m_protocol(protocol_){
        toFingersSub = node.subscribe<std_msgs::ByteMultiArray>("toFingersTopic", 10, &Boost_RS485_Server::recvd_callback, this);
        fromFingersPub = node.advertise<std_msgs::ByteMultiArray>("fromFingersTopic", 10);
    };
    
private:
    protocol_master::ProtocolMaster& m_protocol;
    std::vector<uint8_t> device = {0x01, 0x02};
    uint8_t dataFromTopic[32] = {0};
    uint8_t dataFromFingers[32] = {0};
    uint32_t dataFromFingersSize = 0;
    ros::NodeHandle node;
    ros::Publisher fromFingersPub;
    ros::Subscriber toFingersSub;
    uint32_t recvd_count_topic = 0;
    uint32_t send_count_rs = 0;
    uint32_t recvd_count_rs = 0;
    uint32_t send_count_topic = 0;

    void recvd_callback(const std_msgs::ByteMultiArray::ConstPtr& recvdMsg) {
        recvd_count_topic++;
        std::cout << "RECVD FROM TOPIC toFingersTopic recvdMsg->data.size() = " << recvdMsg->data.size() << std::endl;
        std::cout << "recvd_count_topic = " << recvd_count_topic << std::endl;
        memset(dataFromTopic, 0, sizeof(dataFromTopic));
        for (int i = 0; i < recvdMsg->data.size(); i++){
            dataFromTopic[i] = recvdMsg->data[i];
            printf("[%u]", dataFromTopic[i]);
        }
        std::cout << endl;
        m_protocol.sendSomeCmd(dataFromTopic, recvdMsg->data.size(), dataFromFingers, dataFromFingersSize);

        std_msgs::ByteMultiArray sendMsgFromFingersTopic;
        sendMsgFromFingersTopic.layout.dim.push_back(std_msgs::MultiArrayDimension());
        sendMsgFromFingersTopic.layout.dim[0].size = 1;
        sendMsgFromFingersTopic.layout.dim[0].stride = dataFromFingersSize;
        sendMsgFromFingersTopic.data.clear();

        for (int i = 0; i < dataFromFingersSize; i++){
            sendMsgFromFingersTopic.data.push_back(dataFromFingers[i]);
        }
        fromFingersPub.publish(sendMsgFromFingersTopic);
    }
};

class Boost_RS485_Client
{
public:
    Boost_RS485_Client(protocol::Protocol& protocol_, tabl_reg::TablReg& tabl_)
    : m_protocol(protocol_), m_tabl(tabl_){};

    void pollingSensors(){
        size_t count = 0;

        while(1){
            m_protocol.process();
            //std::cout << "\n<------[count] = " << count << "------>\n" << std::endl;
            count++;
        } 
    }
private:
    protocol::Protocol& m_protocol;
    tabl_reg::TablReg& m_tabl;
};