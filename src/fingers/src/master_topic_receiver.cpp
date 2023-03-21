#include <iostream>
#include <ros/ros.h>
#include <boost/asio.hpp>
#include <std_msgs/ByteMultiArray.h>
#include "boost_serial.hpp"
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
        toFingersSub = node.subscribe<std_msgs::ByteMultiArray>("toFingersTopic", 100, &Boost_RS485_Server::topic_handle_receive, this);
        fromFingersPub = node.advertise<std_msgs::ByteMultiArray>("fromFingersTopic", 100);
    };

    void nodeFromTopicProcess(){
        if (getMsgFromTopic){
            update_hand_mount();
            toEachFinger();
            sendMsgToTopic();
            getMsgFromTopic = false;
        }
        //std::cout << "[WAIT MSG FROM TOPIC toFingersTopic]\n";
    }

private:
    protocol_master::ProtocolMaster& m_protocol;
    uint8_t dataFromTopic[31] = {0};
    uint8_t dataToTopic[56] = {0};
    uint8_t dataToFinger[5] = {0};
    uint8_t dataFromFinger[9] = {0};
    uint32_t dataFromFingerSize = sizeof(dataFromFinger);
    ros::NodeHandle node;
    ros::Publisher fromFingersPub;
    ros::Subscriber toFingersSub;
    uint32_t recvd_count_topic = 0;
    uint32_t send_count_rs = 0;
    uint32_t recvd_count_rs = 0;
    uint32_t send_count_topic = 0;
    bool getMsgFromTopic = false;
    std::vector<uint8_t> fingersAddrs = {0x11, 0x12, 0x13, 0x14, 0x15, 0x16};       //5 пальцев + модуль отведения
    uint8_t hand_mount_addr = 0x31;                                                 //устройство отсоединения схвата
    uint8_t to_hand_mount = 0;
    uint8_t from_hand_mount = 0;
    bool start_communication = true;
    uint8_t resvdFromAllDev = 0;
    uint32_t size8t = 1;

    enum fingersOK{                  //ок, если ответ пришел
        bigFinger      =     1,      //большой палец
        foreFinger     =     2,      //указательный палец
        middleFinger   =     4,      //средный палец
        ringFinger     =     8,      //безымянный палец
        pinkyFinger    =     16,     //мизинец
        leadModule     =     32,     //модуль отведения
        handMount      =     64      //устройство отсоединения схвата
    };

    uint8_t fingers_OK[7] = {1, 2, 4, 8, 16, 32, 64}; ////ок, если ответ пришел

    void update_hand_mount(){
        to_hand_mount = dataFromTopic[sizeof(dataFromTopic) - sizeof(uint8_t)];
        std::cout << "\nupdate brush holder = ";
        printf("%u", to_hand_mount);
        if(m_protocol.sendCmdReadWrite(hand_mount_addr, 0x3, &to_hand_mount, sizeof(uint8_t), 
                                                &from_hand_mount, &size8t)){
            resvdFromAllDev |= fingers_OK[6]; //ответ пришел
        }
        dataToTopic[sizeof(dataToTopic) - 2 * sizeof(uint8_t)] = from_hand_mount;
    }

    void topic_handle_receive(const std_msgs::ByteMultiArray::ConstPtr& recvdMsg) {
        getMsgFromTopic = true;
        recvd_count_topic++;
        std::cout << "\033[1;34mRECVD FROM TOPIC toFingersTopic recvdMsg->data.size() = \033[0m" << recvdMsg->data.size() << std::endl;
        std::cout << "recvd_count_topic = " << recvd_count_topic << std::endl;
        memset(dataFromTopic, 0, sizeof(dataFromTopic));
        memset(dataToTopic, 0, sizeof(dataToTopic));
        resvdFromAllDev = 0;
        for (int i = 0; i < recvdMsg->data.size(); i++){
            dataFromTopic[i] = recvdMsg->data[i];
            printf("[%u]", dataFromTopic[i]);
        }
        std::cout << endl;
    }

	void toEachFinger(){
		for (int i = 0; i < fingersAddrs.size(); i++){
			memset(dataToFinger, 0, sizeof(dataToFinger));
            memset(dataFromFinger, 0, sizeof(dataFromFinger));
			memcpy(dataToFinger, dataFromTopic + i * sizeof(dataToFinger), sizeof(dataToFinger));
            std::cout << "\ndataToFinger ";
            printf("%u = ", fingersAddrs[i]);
            for (int i = 0; i < sizeof(dataToFinger); i++){
                printf("[%u]", dataToFinger[i]);
            }
			if (m_protocol.sendCmdReadWrite(fingersAddrs[i], 0x3, dataToFinger, sizeof(dataToFinger), 
                                                    dataFromFinger, &dataFromFingerSize)) {
                resvdFromAllDev |= fingers_OK[i]; //ответ пришел
            }
			memcpy(dataToTopic + i * sizeof(dataFromFinger), dataFromFinger, sizeof(dataFromFinger));
		}
        dataToTopic[sizeof(dataToTopic) - sizeof(uint8_t)] = resvdFromAllDev;
	}

    void sendMsgToTopic(){
        //отправка пакета в топик "fromFingersTopic" 
        std_msgs::ByteMultiArray sendMsgFromFingersTopic;
        sendMsgFromFingersTopic.layout.dim.push_back(std_msgs::MultiArrayDimension());
        sendMsgFromFingersTopic.layout.dim[0].size = 1;
        sendMsgFromFingersTopic.layout.dim[0].stride = sizeof(dataToTopic);
        sendMsgFromFingersTopic.data.clear();
        std::cout << "\n\033[1;34mSEND MSG TO TOPIC fromFingersTopic = \033[0m";
        for (int i = 0; i < sizeof(dataToTopic); i++){
            sendMsgFromFingersTopic.data.push_back(dataToTopic[i]);
            printf("[%u]", dataToTopic[i]);
        }
        fromFingersPub.publish(sendMsgFromFingersTopic);
        std::cout << endl;
    }
};

int main(int argc, char** argv){

	// if(argc != 3) {
	// 	std::cerr << "[program_name][devPort(1, 2...)][baudrate]" << std::endl;
	// 	return -1;
	// }
    // std::string devPort = argv[1];
    // std::string baudrate = argv[2];

    std::string devPort = "";
    std::string baudrate = "";

    ros::param::param<std::string> ("~_devPortForFingers", devPort, "USB0");
    ros::param::param<std::string> ("~_baudrateForFingers", baudrate, "256000");


    printf("(uint32_t)std::stoi(baudrate) = %u\n", (uint32_t)std::stoi(baudrate));
	try{
		std::cout << "master_topic_receiver is running!" << std::endl;
		ros::init(argc, argv, "master_topic_receiver");
		boost_serial::Boost_Serial_Async boostRS485_transp("/dev/tty" + devPort, (uint32_t)std::stoi(baudrate));
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
