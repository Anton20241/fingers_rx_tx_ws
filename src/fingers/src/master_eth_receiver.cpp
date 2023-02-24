#include <iostream>
#include <ros/ros.h>
#include <boost/asio.hpp>
#include <std_msgs/ByteMultiArray.h>
#include "umba_crc_table.h"
#include <mutex>

#define PORT 1234

using boost::asio::ip::udp;
using boost::asio::ip::address;

class UDPServer{
public:
	UDPServer(boost::asio::io_service& io_service): socket_(io_service, udp::endpoint(udp::v4(), PORT)){
    toFingersPub = node.advertise<std_msgs::ByteMultiArray>("toFingersTopic", 100);
    fromFingersSub = node.subscribe<std_msgs::ByteMultiArray>("fromFingersTopic", 100, &UDPServer::topic_handle_receive, this);
    boost::bind(&UDPServer::udp_handle_receive, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred);
    read_msg_udp();
  }

	void read_msg_udp(){
    socket_.async_receive_from(boost::asio::buffer(dataFromUDP, sizeof(dataFromUDP)), sender_endpoint_,
        boost::bind(&UDPServer::udp_handle_receive, this, boost::asio::placeholders::error, 
        boost::asio::placeholders::bytes_transferred));
	}
  
  void nodeFromUDPProcess(){
    //получаем данные с ост устройств
    //...
    //...
    //...
    //получаем данные с ост устройств
    if (getMsgFromUDP) sendMsgToTopic();
    getMsgFromUDP = false;
    if (getMsgFromTopic) sendMsgToUDP();
    getMsgFromTopic = false;
  }


private:
	udp::socket socket_;
	udp::endpoint sender_endpoint_;
	uint8_t dataFromUDP[41] = {0};
	uint8_t dataToUDP[49] = {0};
  uint8_t dataToTopic[30] = {0};
  uint8_t dataFromTopic[30] = {0};
  ros::NodeHandle node;
  ros::Publisher toFingersPub;
  ros::Subscriber fromFingersSub;
  uint32_t recvd_count_udp = 0;
  uint32_t send_count_topic = 0;
  uint32_t recvd_count_topic = 0;
  uint32_t send_count_udp = 0;
  bool getMsgFromTopic = false;
  bool getMsgFromUDP = false;
  std::mutex lock;

  void topic_handle_receive(const std_msgs::ByteMultiArray::ConstPtr& recvdMsg) {
    getMsgFromTopic = true;
    recvd_count_topic++;
    std::cout << "RECVD FROM TOPIC fromFingersTopic recvdMsg->data.size() = " << recvdMsg->data.size() << std::endl;
    std::cout << "recvd_count_topic = " << recvd_count_topic << std::endl;
    memset(dataFromTopic, 0, sizeof(dataFromTopic));
    for (int i = 0; i < recvdMsg->data.size(); i++){
        dataFromTopic[i] = recvdMsg->data[i];
        printf("[%u]", dataFromTopic[i]);
    }
    std::cout << std::endl;
  }

  void udp_handle_receive(const boost::system::error_code& error, size_t bytes_transferred) {
    getMsgFromUDP = true;
    if (!error && bytes_transferred > 0){
      // if(!parserUDP(dataFromUDP)){
      //   std::cout << "UDP data not valid\n";
      //   return;
      // }
      memset(dataToTopic, 0, sizeof(dataToTopic));
      recvd_count_udp++;
      std::cout << "RECVD FROM UDP bytes_transferred = " << bytes_transferred << std::endl;
      std::cout << "recvd_count_udp = " << recvd_count_udp << std::endl;
      for (int i = 0; i < bytes_transferred; i++){
        printf("[%u]", dataFromUDP[i]);
      }
      memcpy(dataToTopic, dataFromUDP + 3, sizeof(dataToTopic));
      read_msg_udp();
    } else {
      std::cerr << error.what();
    }
  }

  void sendMsgToTopic(){
    //отправка пакета в топик "toFingersTopic"
    std::cout << std::endl;
    std_msgs::ByteMultiArray sendMsgToFingersTopic;
    sendMsgToFingersTopic.layout.dim.push_back(std_msgs::MultiArrayDimension());
    sendMsgToFingersTopic.layout.dim[0].size = 1;
    sendMsgToFingersTopic.layout.dim[0].stride = sizeof(dataToTopic);
    sendMsgToFingersTopic.data.clear();
    std::cout << "SEND TO toFingersTopic: ";
    for (int i = 0; i < sizeof(dataToTopic); i++){
      printf("[%u]", dataToTopic[i]);
      sendMsgToFingersTopic.data.push_back(dataToTopic[i]);
    }
    toFingersPub.publish(sendMsgToFingersTopic);
    std::cout << std::endl;
    
  }

  void sendMsgToUDP(){
    dataToUDP[0] = 0xBB;
    dataToUDP[1] = 0xAA;
    dataToUDP[2] = sizeof(dataToUDP);
    memcpy(dataToUDP + 3, dataFromTopic, sizeof(dataFromTopic));
    dataToUDP[33] = 0x00; //пустышка
    dataToUDP[34] = 0x00; //пустышка
    dataToUDP[35] = 0x00; //пустышка
    dataToUDP[36] = 0x00; //пустышка
    dataToUDP[37] = 0x00; //пустышка
    dataToUDP[38] = 0x00; //пустышка
    dataToUDP[39] = 0x00; //пустышка
    dataToUDP[40] = 0x00; //пустышка
    dataToUDP[41] = 0x00; //пустышка
    dataToUDP[42] = 0x00; //пустышка
    dataToUDP[43] = 0x00; //пустышка
    dataToUDP[44] = 0x00; //пустышка
    dataToUDP[45] = 0x00; //пустышка
    dataToUDP[46] = 0x00; //пустышка
    dataToUDP[47] = 0x00; //пустышка
    dataToUDP[48] = umba_crc8_table(dataToUDP, sizeof(dataToUDP) - sizeof(uint8_t));
    boost::system::error_code error;
    auto sent = socket_.send_to(boost::asio::buffer(dataFromTopic), sender_endpoint_, 0, error);
    if (!error && sent > 0){
      send_count_udp++;
      std::cout << "SEND TO UDP sent = " << sent << std::endl;
      std::cout << "send_count_udp = " << send_count_udp << std::endl;
      for (int i = 0; i < sent; i++){
        printf("[%u]", dataFromTopic[i]);
      }
    } else {
      std::cerr << error.what();
    }
  }

  bool parserUDP(uint8_t* dataFromUDP){
    if (dataFromUDP == nullptr) return false;
    if (dataFromUDP[0] != 0xAA) return false;
    if (dataFromUDP[1] != 0xBB) return false;
    //if (dataFromUDP[2] != 0xBB) return false; //add crc8
    //TODO!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  }
};

int main(int argc, char** argv){
  try{
    std::cout << "master_eth_receiver is running!" << std::endl;
		ros::init(argc, argv, "master_eth_receiver");
		boost::asio::io_service io_service;
		UDPServer udpServer(io_service);
    while(ros::ok()){
      udpServer.nodeFromUDPProcess();
      io_service.poll_one();
      ros::spinOnce();
    }
	} catch (std::exception e){
		std::cerr << "Exeption: " << e.what() << std::endl;
	}
  return 0;
};
