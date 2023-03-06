#include <iostream>
#include <boost/asio.hpp>
#include <ros/ros.h>
#include <std_msgs/ByteMultiArray.h>
#include "boost_rs485.hpp"
#include <boost/chrono.hpp>
#include "protocol.hpp"
#include "umba_crc_table.h"
#include <mutex>

#define CAM_TOPIC_NAME "camera_topic"
#define BAT_CAM_TOPIC_NAME "bat_cam_topic"
#define BAUDRATE 19200

ros::Subscriber bat_cam_sub;
ros::Publisher bat_cam_pub;

class UART_Node
{
public:
    UART_Node(protocol_master::ProtocolMaster& protocol_)
    : m_protocol(protocol_){
        bat_cam_sub = node.subscribe<std_msgs::ByteMultiArray>(CAM_TOPIC_NAME, 10, &UART_Node::cam_callback,this);
        bat_cam_pub = node.advertise<std_msgs::ByteMultiArray>(BAT_CAM_TOPIC_NAME, 10);
        tp_first = boost::chrono::system_clock::now();
    };

  void UART_process(){
    if(m_protocol.sendCmdReadUART(0x01, from_board_data, &from_board_dataSize))
    {
      resvdFromDev |= 128;
      tp_last = boost::chrono::system_clock::now();
      cam_bat_time = tp_last - tp_first;
      std::cout << "\nReceived BOARD DATA:\n";
      for (size_t i = 0; i < from_board_dataSize; i++){
        printf("[%u]",from_board_data[i]);
      }
      std::cout << "\nTime since previos msg:" << cam_bat_time << '\n';
      tp_first = boost::chrono::system_clock::now();
      pub_board_data();
    };
    std::this_thread::sleep_for(std::chrono::microseconds(50));
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
    boost::chrono::system_clock::time_point tp_first;// = boost::chrono::system_clock::now();
    boost::chrono::system_clock::time_point tp_last;
    boost::chrono::duration<double> cam_bat_time;
    uint8_t resvdFromDev = 0;
    std::mutex my_mytex;

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
    } else {
      return;
    }
    std_msgs::ByteMultiArray toBatCamTopicMsg;
    toBatCamTopicMsg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    toBatCamTopicMsg.layout.dim[0].size = 1;
    toBatCamTopicMsg.layout.dim[0].stride = bytesToSendCount;
    toBatCamTopicMsg.data.clear();

    std::cout << "SEND TO TOPIC bat_cam_topic: ";
    for (size_t i = 0; i < bytesToSendCount; i++)
    {
        toBatCamTopicMsg.data.push_back(to_bat_cam_topic[i]);
        printf("[%u]", to_bat_cam_topic[i]);
    }
    std::cout << std::endl;
    bat_cam_pub.publish(toBatCamTopicMsg);
    memset(to_bat_cam_topic, 0, sizeof(to_bat_cam_topic));
    std::memset(from_board_data, 0, from_board_dataSize);
    from_board_dataSize = 0;
    resvdFromDev = 0;
}

void cam_callback(const std_msgs::ByteMultiArray::ConstPtr& camStatus)
{
  recvd_count_topic++;
  uint8_t cam_status = camStatus->data[0];
  uint8_t relay_state = camStatus->data[1];

  std::cout << "recvd_count_topic = " << recvd_count_topic << std::endl;
  std::cout << "\nCAMERA STATUS ";
  printf("%u\n", cam_status);
  std::cout << "\nRELAY STATUS ";
  printf("%u\n", relay_state);
  uint8_t toRelaySet[2] = {1, relay_state};
  if (m_protocol.sendCmdWrite(0x01, 0x20, toRelaySet, sizeof(toRelaySet))){
    resvdFromDev |= 128;
  }
  if (m_protocol.sendCmdReadWriteUART(0x01, 0x10, &cam_status, sizeof(uint8_t), from_board_data, &from_board_dataSize)){
    resvdFromDev |= 128;
  }
  pub_board_data();
  std::cout << endl;  
}
};

int main(int argc, char** argv)
{
    std::string devPort = "0";
    std::string baudrate = "19200"; 
        // if(argc == 3) {
        //      devPort = argv[1];
  //  baudrate = argv[2];
        // } else {
        //      std::cerr << "[program_name][/dev/ttyS(0, 1...)][baudrate]" << std::endl;
        //      return -1;
        // }
        try{
          std::cout << "\nUART Node is running!\n" << "Baud rate: " << baudrate << ", Port: /dev/ttyS" << devPort << '\n';
          ros::init(argc, argv, "uart_node");
          boost_rs485::Boost_RS485_Master boostRS485_transp("/dev/ttyS" + devPort, (uint32_t)std::stoi(baudrate));
          protocol_master::ProtocolMaster boostRS485_prot_master(boostRS485_transp);
          UART_Node uartNode(boostRS485_prot_master);
          while(ros::ok()){
              uartNode.UART_process();
              ros::spinOnce();
          }
        } catch (std::exception e){
                std::cerr << "Exeption: " << e.what() << std::endl;
        }
  return 0;
};
