#include <iostream>
#include <boost/asio.hpp>
#include <ros/ros.h>
#include <std_msgs/ByteMultiArray.h>
#include "boost_serial.hpp"
#include <boost/chrono.hpp>
#include "protocol.hpp"
#include "umba_crc_table.h"
#include <mutex>
#include <QCoreApplication>

#define CAM_TOPIC_NAME "camera_topic"
#define BAT_CAM_TOPIC_NAME "bat_cam_topic"

class UART_Node
{
public:
    UART_Node(protocol_master::ProtocolMaster& protocol_)
    : m_protocol(protocol_){
        bat_cam_sub = node.subscribe<std_msgs::ByteMultiArray>(CAM_TOPIC_NAME, 10, &UART_Node::cam_callback,this);
        bat_cam_pub = node.advertise<std_msgs::ByteMultiArray>(BAT_CAM_TOPIC_NAME, 10);
        // tp_first = boost::chrono::system_clock::now();
    };

  void UART_process(){
    static uint32_t fail_count = 0;
    if (msg_sent_relay){
      bool getResponse = false;
      if (m_protocol.sendCmdReadUART(0x01, from_board_data, &from_board_dataSize, getResponse, msg_sent_relay, cam_status)){
        resvdFromDev |= 128;
        fail_count = 0;
        pub_board_data();
      } else {
        std::cout << "[msg SEND and failed]\n";
        sendError();
      }
      msg_sent_relay = false;

    } else if (msg_sent_cam){
      bool getResponse = false;
      if (m_protocol.sendCmdReadUART(0x01, from_board_data, &from_board_dataSize, getResponse, msg_sent_cam, cam_status)){
        resvdFromDev |= 128;
        fail_count = 0;
        pub_board_data();
      } else {
        std::cout << "[msg SEND and failed]\n";
        sendError();
      }
      msg_sent_cam = false;

    } else {
      bool getResponse = false;
      if (m_protocol.sendCmdReadUART(0x01, from_board_data, &from_board_dataSize, getResponse, false, cam_status)){
        resvdFromDev |= 128;
        fail_count = 0;
        pub_board_data();
      } else {
        if (!getResponse || fail_count >= 70000){
          fail_count = 0;
          std::cout << "[msg NOT SEND and failed]\n";
          sendError();
        }
        fail_count++;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }
  }

private:
  protocol_master::ProtocolMaster& m_protocol;
  ros::NodeHandle node;
  ros::Subscriber bat_cam_sub;
  ros::Publisher bat_cam_pub;
  uint32_t recvd_count_topic = 0;
  uint8_t from_board_data[8] = {0}; //<--from board
  uint32_t from_board_dataSize = 0;
  uint8_t to_bat_cam_topic[5] = {0}; //<--to_bat_cam_topic
  uint8_t resvdFromDev = 0;
  uint8_t cam_status = 0;
  uint8_t relay_state = 0;
  uint8_t cam_status_prev = 0;
  uint8_t relay_state_prev = 0;
  bool msg_sent_relay = false;
  bool msg_sent_cam = false;
  uint32_t sendToTopicCount = 0;

  void sendError(){
    m_protocol.m_transport.send_error = true;
    static uint32_t failCount = 0;
    failCount++;
    std::cout << "\nfailCount = " << failCount << std::endl;
    std::cout << "\033[1;31m[RECEIVE ERROR FROM UART]\033[0m\n";
    std::cout << "\033[1;31m[NON-Valid MESSAGE]:\033[0m\n";
    
    for (int i = 0; i < from_board_dataSize; i++){
      printf("[%u]", from_board_data[i]);
    }
    std::cout << std::endl;
    resvdFromDev = 0;
    memset(from_board_data, 0, from_board_dataSize);
    from_board_dataSize = 1;
    from_board_data[0] = 0;
    pub_board_data();
  }

  void pub_board_data() //create and pub ros message 
  {
    uint8_t bytesToSendCount = 0;
    if (from_board_dataSize == 5){
      //пакет в топик "bat_cam_topic"
      to_bat_cam_topic[0] = from_board_data[2]; //CMD
      to_bat_cam_topic[1] = from_board_data[3]; //TIME_DOWN
      to_bat_cam_topic[2] = resvdFromDev;       //all_ok
      bytesToSendCount = 3;
    }
    else if(from_board_dataSize == 8) {
      //пакет в топик "bat_cam_topic"
      to_bat_cam_topic[0] = from_board_data[3]; //BAT_24V
      to_bat_cam_topic[1] = from_board_data[4]; //BAT_48V
      to_bat_cam_topic[2] = from_board_data[5]; //VIDEO_SWITCH
      to_bat_cam_topic[3] = from_board_data[6]; //relay_state
      to_bat_cam_topic[4] = resvdFromDev;       //all_ok
      bytesToSendCount = 5;
    } else if(from_board_dataSize == 1) {
      to_bat_cam_topic[0] = resvdFromDev;       //all_ok
      bytesToSendCount = 1;
    } else {
      return;
    }
    std_msgs::ByteMultiArray toBatCamTopicMsg;
    toBatCamTopicMsg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    toBatCamTopicMsg.layout.dim[0].size = 1;
    toBatCamTopicMsg.layout.dim[0].stride = bytesToSendCount;
    toBatCamTopicMsg.data.clear();

    std::cout << "\033[1;34mSEND TO TOPIC bat_cam_topic: \033[0m";
    for (size_t i = 0; i < bytesToSendCount; i++)
    {
      toBatCamTopicMsg.data.push_back(to_bat_cam_topic[i]);
      printf("[%u]", to_bat_cam_topic[i]);
    }
    std::cout << std::endl;
    sendToTopicCount++;
    std::cout << "sendToTopicCount = " << sendToTopicCount << std::endl;
    bat_cam_pub.publish(toBatCamTopicMsg);
    memset(to_bat_cam_topic, 0, sizeof(to_bat_cam_topic));
    memset(from_board_data, 0, from_board_dataSize);
    from_board_dataSize = 0;
    resvdFromDev = 0;
  }

  void cam_callback(const std_msgs::ByteMultiArray::ConstPtr& camStatus)
  {
    recvd_count_topic++;
    cam_status = camStatus->data[0];
    relay_state = camStatus->data[1];
    std::cout << "\n[cam_callback]\n" << std::endl;
    std::cout << "\nrecvd_count_topic = " << recvd_count_topic << std::endl;
    std::cout << "CAMERA STATUS ";
    printf("%u\n", cam_status);
    std::cout << "RELAY STATUS ";
    printf("%u\n", relay_state);
    uint8_t toRelaySet[2] = {1, relay_state};

    if (relay_state != relay_state_prev){
      relay_state_prev = relay_state;
      std::cout << "\n\033[1;35m[send UART msg with new relay_state]\033[0m\n";
      m_protocol.sendCmdWrite(0x01, 0x20, toRelaySet, sizeof(toRelaySet));
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      msg_sent_relay = true;
    }
    if (cam_status != cam_status_prev){
      cam_status_prev = cam_status;
      std::cout << "\n\033[1;35m[send UART msg with new cam_status]\033[0m\n";
      m_protocol.sendCmdWrite(0x01, 0x10, &cam_status, sizeof(uint8_t));
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      msg_sent_cam = true;
    }
  }
};

int main(int argc, char** argv)
{
  QCoreApplication coreApplication(argc, argv);
  std::string devPort = "0";
  std::string baudrate = "19200"; 

  ros::param::param<std::string> ("~_UART_baudrate", baudrate, "19200");
  try{
        std::cout << "\n\033[1;32m╔═══════════════════════════════════════╗\033[0m"
                  << "\n\033[1;32m║UART Node is running!                  ║\033[0m" 
                  << "\n\033[1;32m║Baud rate: " << baudrate << ", Port: /dev/ttyS" << devPort << "\t║\033[0m"
                  << "\n\033[1;32m╚═══════════════════════════════════════╝\033[0m\n";
    ros::init(argc, argv, "uart_node");
    boost_serial::Boost_Serial_Async boostRS485_transp("/dev/ttyUSB" + devPort, (uint32_t)std::stoi(baudrate));
    protocol_master::ProtocolMaster boostRS485_prot_master(boostRS485_transp, &coreApplication);
    UART_Node uartNode(boostRS485_prot_master);
    while(ros::ok()){
      uartNode.UART_process();
      ros::spinOnce();
    }
  } catch (std::exception e){
      std::cerr << "\nExeption: " << e.what() << std::endl;
  }
  return 0;
};
