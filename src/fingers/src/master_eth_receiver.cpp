#include <iostream>
#include <ros/ros.h>
#include <boost/asio.hpp>
#include <std_msgs/ByteMultiArray.h>

#define PORT 1234

using boost::asio::ip::udp;
using boost::asio::ip::address;

class UDPServer{
public:
	UDPServer(boost::asio::io_service& io_service): socket_(io_service, udp::endpoint(udp::v4(), PORT)){
    toFingersPub = node.advertise<std_msgs::ByteMultiArray>("toFingersTopic", 100);
    fromFingersSub = node.subscribe<std_msgs::ByteMultiArray>("fromFingersTopic", 100, &UDPServer::topic_handle_receive, this);
    boost::bind(&UDPServer::udp_handle_receive, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
    read_msg_udp();
  }

	void read_msg_udp(){
    socket_.async_receive_from(boost::asio::buffer(dataFromUDP, sizeof(dataFromUDP)), sender_endpoint_,
        boost::bind(&UDPServer::udp_handle_receive, this, boost::asio::placeholders::error, 
        boost::asio::placeholders::bytes_transferred));
	}

  void nodeFromUDPProcess() {
     
  }
  
private:
	udp::socket socket_;
	udp::endpoint sender_endpoint_;
	uint8_t dataFromUDP[60] = {0};
	uint8_t dataToUDP[60] = {0};
  uint8_t dataToFingers[30] = {0};
  uint8_t dataFromFingers[30] = {0};
  ros::NodeHandle node;
  ros::Publisher toFingersPub;
  ros::Subscriber fromFingersSub;
  uint32_t recvd_count_udp = 0;
  uint32_t send_count_topic = 0;
  uint32_t recvd_count_topic = 0;
  uint32_t send_count_udp = 0;

  void topic_handle_receive(const std_msgs::ByteMultiArray::ConstPtr& recvdMsg) {
    recvd_count_topic++;
    std::cout << "RECVD FROM TOPIC fromFingersTopic recvdMsg->data.size() = " << recvdMsg->data.size() << std::endl;
    std::cout << "recvd_count_topic = " << recvd_count_topic << std::endl;
    memset(dataFromFingers, 0, sizeof(dataFromFingers));
    for (int i = 0; i < recvdMsg->data.size(); i++){
        dataFromFingers[i] = recvdMsg->data[i];
        printf("[%u]", recvdMsg->data[i]);
    }
    std::cout << std::endl;
    boost::system::error_code error;
    auto sent = socket_.send_to(boost::asio::buffer(dataFromFingers), sender_endpoint_, 0, error);
    if (!error && sent > 0){
      send_count_udp++;
      std::cout << "SEND TO UDP sent = " << sent << std::endl;
      std::cout << "send_count_udp = " << send_count_udp << std::endl;
      for (int i = 0; i < sent; i++){
        printf("[%u]", dataFromFingers[i]);
      }
    } else {
      std::cerr << error.what();
    }
  }

  void udp_handle_receive(const boost::system::error_code& error, size_t bytes_transferred) {
    if (!error && bytes_transferred > 0){
      if(!parserUDP(dataFromUDP)){
        std::cout << "UDP data not valid\n";
        return;
      }
      memset(dataFromUDP, 0, sizeof(dataFromUDP));
      memset(dataToFingers, 0, sizeof(dataToFingers));
      recvd_count_udp++;
      std::cout << "RECVD FROM UDP bytes_transferred = " << bytes_transferred << std::endl;
      std::cout << "recvd_count_udp = " << recvd_count_udp << std::endl;
      for (int i = 0; i < bytes_transferred; i++){
        printf("[%u]", dataFromUDP[i]);
      }
      memcpy(dataToFingers, dataFromUDP + 3, sizeof(dataToFingers));

      //отправка пакета в топик "toFingersTopic"
      std::cout << std::endl;
      std_msgs::ByteMultiArray sendMsgToFingersTopic;
      sendMsgToFingersTopic.layout.dim.push_back(std_msgs::MultiArrayDimension());
      sendMsgToFingersTopic.layout.dim[0].size = 1;
      sendMsgToFingersTopic.layout.dim[0].stride = sizeof(dataToFingers);
      sendMsgToFingersTopic.data.clear();
      for (int i = 0; i < sizeof(dataToFingers); i++){
        sendMsgToFingersTopic.data.push_back(dataToFingers[i]);
      }
      toFingersPub.publish(sendMsgToFingersTopic);
      read_msg_udp();
    } else {
      std::cerr << error.what();
    }
  }

  bool parserUDP(uint8_t* dataFromUDP){
    if (dataFromUDP[0] != 0xAA) return false;
    if (dataFromUDP[1] != 0xBB) return false;
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
    }
		//io_service.run();
	} catch (std::exception e){
		std::cerr << "Exeption: " << e.what() << std::endl;
	}
  ros::spin();
  return 0;
};
