#include <iostream>
#include <ros/ros.h>
#include <boost/asio.hpp>
#include <std_msgs/ByteMultiArray.h>
#include <std_msgs/Byte.h>
#include "umba_crc_table.h"
#include <mutex>

#define PORT 1234

using boost::asio::ip::udp;
using boost::asio::ip::address;

class UDPServer{
public:
	UDPServer(boost::asio::io_service& io_service): socket_(io_service, udp::endpoint(udp::v4(), PORT)){
    toFingersPub = node.advertise<std_msgs::ByteMultiArray>("toFingersTopic", 100);
    toCamPub = node.advertise<std_msgs::Byte>("camera_topic", 100);
    fromFingersSub = node.subscribe<std_msgs::ByteMultiArray>("fromFingersTopic", 100, &UDPServer::from_finger_handle_receive, this);
    fromCamBatSub = node.subscribe<std_msgs::ByteMultiArray>("bat_cam_topic", 100, &UDPServer::from_cam_bat_handle_receive, this);
    boost::bind(&UDPServer::udp_handle_receive, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred);
    read_msg_udp();
  }

	void read_msg_udp(){
    socket_.async_receive_from(boost::asio::buffer(dataFromUDP, sizeof(dataFromUDP)), sender_endpoint_,
        boost::bind(&UDPServer::udp_handle_receive, this, boost::asio::placeholders::error, 
        boost::asio::placeholders::bytes_transferred));
	}
  
  void nodeFromUDPProcess(){
    if (getMsgFromUDP){
      sendMsgToFingers();
      sendMsgToCameras();
      sendMsgToUDP();
      getMsgFromUDP = false;
    }
  }

private:
	udp::socket socket_;
	udp::endpoint sender_endpoint_;
	uint8_t dataFromUDP[41] = {0};
	uint8_t dataToUDP[68] = {0};
  uint8_t dataToTopic[31] = {0};
  uint8_t dataFromTopic[56] = {0};
  ros::NodeHandle node;
  ros::Publisher toFingersPub;
  ros::Subscriber fromFingersSub;
  ros::Publisher toCamPub;
  ros::Subscriber fromCamBatSub;
  uint32_t send_count_udp = 0;
  uint32_t recvd_count_udp = 0;
  uint32_t send_count_topic_fingers = 0;
  uint32_t recvd_count_topic_fingers = 0;
  uint32_t send_count_topic_camera = 0;
  uint32_t recvd_count_topic_cam_bat = 0;
  bool start_communication = true;
  uint8_t resvdFromAllDev = 0;
  
  struct dataNotFromFingers{
    uint8_t hand_mount = 0;
    uint8_t hold_position = 0;
    uint8_t camera_from_udp = 0;
    uint8_t camera_from_bat_cam = 0;
    uint8_t bat_24V = 0;
    uint8_t bat_48V = 0;
    uint8_t keepalive[4] = {0};
  };
  
  dataNotFromFingers dataFingersFromNot;

  bool getMsgFromFingers = false;
  bool getMsgFromUDP = false;
  std::mutex lock;

  void from_finger_handle_receive(const std_msgs::ByteMultiArray::ConstPtr& recvdMsg) {
    getMsgFromFingers = true;
    recvd_count_topic_fingers++;
    memset(dataFromTopic, 0, sizeof(dataFromTopic));
    std::cout << "RECVD FROM TOPIC fromFingersTopic recvdMsg->data.size() = " << recvdMsg->data.size() << std::endl;
    std::cout << "recvd_count_topic_fingers = " << recvd_count_topic_fingers << std::endl;
    for (int i = 0; i < recvdMsg->data.size(); i++){
        dataFromTopic[i] = recvdMsg->data[i];
        printf("[%u]", dataFromTopic[i]);
    }
    std::cout << std::endl;
    dataFingersFromNot.hand_mount = dataFromTopic[sizeof(dataFromTopic) - 2 * sizeof(uint8_t)];
    resvdFromAllDev |= dataFromTopic[sizeof(dataFromTopic) - sizeof(uint8_t)]; //get answers from all fingers
  }

  void from_cam_bat_handle_receive(const std_msgs::ByteMultiArray::ConstPtr& recvdMsg) {
    recvd_count_topic_cam_bat++;
    dataFingersFromNot.camera_from_bat_cam = 0;
    dataFingersFromNot.bat_24V = 0;
    dataFingersFromNot.bat_48V = 0;
    std::cout << "RECVD FROM TOPIC bat_cam_topic recvdMsg->data.size() = " << recvdMsg->data.size() << std::endl;
    std::cout << "recvd_count_topic_cam_bat = " << recvd_count_topic_cam_bat << std::endl;
    for (int i = 0; i < recvdMsg->data.size(); i++){
        printf("[%u]", recvdMsg->data[i]);
    }
    resvdFromAllDev |= recvdMsg->data[recvdMsg->data.size() - sizeof(uint8_t)];

    //обновляем данные
    dataFingersFromNot.bat_24V = recvdMsg->data[0];
    dataFingersFromNot.bat_48V = recvdMsg->data[1];
    dataFingersFromNot.camera_from_bat_cam = recvdMsg->data[2];
  }

  void udp_handle_receive(const boost::system::error_code& error, size_t bytes_transferred) {
    getMsgFromUDP = true;
    if (!error && bytes_transferred > 0){
      if(!parserUDP(dataFromUDP)){
        std::cout << "UDP data not valid\n";
        return;
      }
      memset(dataToTopic, 0, sizeof(dataToTopic));
      memset(dataFingersFromNot.keepalive, 0, sizeof(dataFingersFromNot.keepalive));
      uint8_t resvdFromAllDev = 0;
      dataFingersFromNot.hand_mount = 0;
      dataFingersFromNot.hold_position = 0;
      dataFingersFromNot.camera_from_udp = 0;
      recvd_count_udp++;
      std::cout << "\nRECVD FROM UDP bytes_transferred = " << bytes_transferred << std::endl;
      std::cout << "recvd_count_udp = " << recvd_count_udp << std::endl;
      for (int i = 0; i < bytes_transferred; i++){
        printf("[%u]", dataFromUDP[i]);
      }
      std::cout << std::endl;
      memcpy(dataToTopic, dataFromUDP + 3, sizeof(dataToTopic));
      memcpy(dataFingersFromNot.keepalive, dataFromUDP + 36, sizeof(dataFingersFromNot.keepalive));
      dataFingersFromNot.hand_mount = dataFromUDP[33];
      dataFingersFromNot.hold_position = dataFromUDP[34];
      read_msg_udp();
    } else {
      //std::cerr << error.what();
    }
  }

  void sendMsgToFingers(){
    if (dataFingersFromNot.hold_position == 1){
      printf("dataFingersFromNot.hold_position = %u. MSG NOT TO SEND TO FINGERS.\n", dataFingersFromNot.hold_position);
      return;
    }
    //отправка пакета в топик "toFingersTopic"
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

  void sendMsgToCameras(){
    if (start_communication) start_communication = false;
    else if (dataFingersFromNot.camera_from_udp == dataFromUDP[35]) return;
    dataFingersFromNot.camera_from_udp = dataFromUDP[35];
    //отправка пакета в топик "camera_topic"
    std_msgs::Byte sendMsgToCameraTopic;
    sendMsgToCameraTopic.data = dataFingersFromNot.camera_from_udp;
    std::cout << "SEND TO camera_topic: ";
    printf("[%u]\n", dataFingersFromNot.camera_from_udp);
    toCamPub.publish(sendMsgToCameraTopic);
  }

  void sendMsgToUDP(){
    //формируем пакет
    dataToUDP[0] = 0xBB;                                                                          //header 1b
    dataToUDP[1] = 0xAA;                                                                          //header 1b
    dataToUDP[2] = sizeof(dataToUDP);                                                             //data length 1b
    memcpy(dataToUDP + 3, dataFromTopic, sizeof(dataFromTopic) - sizeof(uint8_t));                //data from fingers 9*6b
    dataToUDP[sizeof(dataToUDP) - 11 * sizeof(uint8_t)] = dataFingersFromNot.hand_mount;          //hand_mount 1b
    dataToUDP[sizeof(dataToUDP) - 10 * sizeof(uint8_t)] = dataFingersFromNot.hold_position;        //hold_position 1b
    dataToUDP[sizeof(dataToUDP) - 9 * sizeof(uint8_t)] = dataFingersFromNot.camera_from_bat_cam;  //camera_from_bat_cam 1b
    dataToUDP[sizeof(dataToUDP) - 8 * sizeof(uint8_t)] = dataFingersFromNot.bat_24V;              //bat_24V 1b
    dataToUDP[sizeof(dataToUDP) - 7 * sizeof(uint8_t)] = dataFingersFromNot.bat_48V;              //bat_48V 1b
    dataToUDP[sizeof(dataToUDP) - 6 * sizeof(uint8_t)] = resvdFromAllDev;                         //allDevOk 1b
    memcpy(dataToUDP + sizeof(dataToUDP) - 5 * sizeof(uint8_t), dataFingersFromNot.keepalive, 
        sizeof(dataFingersFromNot.keepalive));                                                    //keepalive 4b
    dataToUDP[sizeof(dataToUDP) - sizeof(uint8_t)] = 
        umba_crc8_table(dataToUDP, sizeof(dataToUDP) - sizeof(uint8_t));                          //crc8

    //отправляем пакет в UDP
    boost::system::error_code error;
    auto sent = socket_.send_to(boost::asio::buffer(dataToUDP), sender_endpoint_, 0, error);
    if (!error && sent > 0){
      send_count_udp++;
      std::cout << "SEND TO UDP sent = " << sent << std::endl;
      std::cout << "send_count_udp = " << send_count_udp << std::endl;
      for (int i = 0; i < sent; i++){
        printf("[%u]", dataToUDP[i]);
      }
      std::cout << std::endl;
    } else {
      //std::cerr << error.what();
    }
  }

  bool parserUDP(uint8_t* dataFromUDP){
    if (dataFromUDP[0] != 0xAA) return false;
    if (dataFromUDP[1] != 0xBB) return false;
    return true;
  }

  static inline uint8_t getLen(uint8_t* ptrBuff)
  {
    return ptrBuff[2];
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
