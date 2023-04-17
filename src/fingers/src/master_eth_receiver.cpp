#include <iostream>
#include <ros/ros.h>
#include <boost/asio.hpp>
#include <std_msgs/ByteMultiArray.h>
#include "umba_crc_table.h"
#include <mutex>

#define PORT 20002

#define RAW_UDP_DATA 1
#define COMPLETE_UDP_DATA 2

/*DATA TYPE*/
#define CODE_PART RAW_UDP_DATA
/////////////

#if CODE_PART == RAW_UDP_DATA

#define DATA_FROM_UDP_SIZE 42
#define DATA_TO_UDP_SIZE 69
#define DATA_FROM_FINGERS_TOPIC_SIZE 56
#define DATA_TO_FINGERS_TOPIC_SIZE 31

#elif CODE_PART == COMPLETE_UDP_DATA

#define DATA_FROM_UDP_SIZE 36
#define DATA_TO_UDP_SIZE 51
#define DATA_FROM_FINGERS_TOPIC_SIZE 38
#define DATA_TO_FINGERS_TOPIC_SIZE 25

#else 

#error "INCORRECT CODE_PART"

#endif


using boost::asio::ip::udp;
using boost::asio::ip::address;

class UDPServer{
public:
	UDPServer(boost::asio::io_service& io_service): socket_(io_service, udp::endpoint(udp::v4(), PORT)){
    toFingersPub = node.advertise<std_msgs::ByteMultiArray>("toFingersTopic", 0);
    toCamPub = node.advertise<std_msgs::ByteMultiArray>("camera_topic", 0);
    fromFingersSub = node.subscribe<std_msgs::ByteMultiArray>("fromFingersTopic", 0, &UDPServer::from_finger_handle_receive, this);
    fromCamBatSub = node.subscribe<std_msgs::ByteMultiArray>("bat_cam_topic", 0, &UDPServer::from_cam_bat_handle_receive, this);
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
      sendMsgToCamBat();
      sendMsgToUDP();
      getMsgFromUDP = false;
    }
  }

private:
	udp::socket socket_;
	udp::endpoint sender_endpoint_;
	uint8_t dataFromUDP[DATA_FROM_UDP_SIZE] = {0};
	uint8_t dataToUDP[DATA_TO_UDP_SIZE] = {0};
  uint8_t dataToFingersTopic[DATA_TO_FINGERS_TOPIC_SIZE] = {0};
  uint8_t dataToFingersTopic_OLD[DATA_TO_FINGERS_TOPIC_SIZE] = {0};
  uint8_t dataFromFingersTopic[DATA_FROM_FINGERS_TOPIC_SIZE] = {0};
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
  uint8_t resvdFromAllDev = 0;
  bool dataGetFromBatCam = false;

  
  struct currentState_{
    uint8_t hand_mount = 0;
    uint8_t hold_position = 0;
    uint8_t camera_from_udp = 0;
    uint8_t camera_from_bat_cam = 0;
    uint8_t bat_24V = 0;
    uint8_t bat_48V = 0;
    uint8_t relay_state_from_udp = 0;
    uint8_t relay_state_from_bat_cam = 0;
    uint8_t camera_from_udp_prev = 0;
    uint8_t relay_state_from_udp_prev = 0;
    uint8_t keepalive[4] = {0};
    uint8_t cmdBatCamTopic = 0;
    uint8_t time_down = 0;
  };
  
  currentState_ currentState;

  bool getMsgFromFingers = false; 
  bool getMsgFromUDP = false;

  void show_cur_state() {
    std::cout << "b24V:" << currentState.bat_24V << " rly: " << currentState.relay_state_from_bat_cam << " cm: " << currentState.camera_from_bat_cam << std::endl;
  }

  void from_finger_handle_receive(const std_msgs::ByteMultiArray::ConstPtr& recvdMsg) {
    getMsgFromFingers = true;
    recvd_count_topic_fingers++;
    memset(dataFromFingersTopic, 0, sizeof(dataFromFingersTopic));
    std::cout << "\n\033[1;34mRECVD FROM TOPIC fromFingersTopic recvdMsg->data.size() = \033[0m" << recvdMsg->data.size() << std::endl;
    std::cout << "recvd_count_topic_fingers = " << recvd_count_topic_fingers << std::endl;
    for (int i = 0; i < recvdMsg->data.size(); i++){
        dataFromFingersTopic[i] = recvdMsg->data[i];
        printf("[%u]", dataFromFingersTopic[i]);
    }
    std::cout << std::endl;
    currentState.hand_mount = dataFromFingersTopic[sizeof(dataFromFingersTopic) - 2 * sizeof(uint8_t)];
    resvdFromAllDev        |= dataFromFingersTopic[sizeof(dataFromFingersTopic) - 1 * sizeof(uint8_t)]; //get answers from all fingers
  }

  void from_cam_bat_handle_receive(const std_msgs::ByteMultiArray::ConstPtr& recvdMsg) {
    //dataGetFromBatCam = true;
    if(recvdMsg->data.size() == 5){
      //обновляем данные
      currentState.bat_24V                    = recvdMsg->data[0];
      currentState.bat_48V                    = recvdMsg->data[1];
      currentState.camera_from_bat_cam        = recvdMsg->data[2];
      currentState.relay_state_from_bat_cam   = recvdMsg->data[3];
      resvdFromAllDev                        |= recvdMsg->data[4];
      recvd_count_topic_cam_bat++;
    } else if (recvdMsg->data.size() == 3){
      currentState.cmdBatCamTopic             = recvdMsg->data[0];
      currentState.time_down                  = recvdMsg->data[1];
      resvdFromAllDev                        |= recvdMsg->data[2];
      recvd_count_topic_cam_bat++;
      startShutDownProcess();
    } else if (recvdMsg->data.size() == 1){
      resvdFromAllDev                        |= recvdMsg->data[0];
    } else return;
    //recvd_count_topic_cam_bat++;
    std::cout << "\033[1;34mRECVD FROM TOPIC bat_cam_topic recvdMsg->data.size() = \033[0m" << recvdMsg->data.size() << std::endl;
    std::cout << "recvd_count_topic_cam_bat = " << recvd_count_topic_cam_bat << std::endl;
    for (int i = 0; i < recvdMsg->data.size(); i++){
      printf("[%u]", (uint8_t)recvdMsg->data[i]);
    }
    std::cout << std::endl;
  }

  void startShutDownProcess(){
    std::cout << "\n\033[1;31m╔═══════════════╗\033[0m";
    std::cout << "\n\033[1;31m║   SHUTDOWN    ║\033[0m";
    std::cout << "\n\033[1;31m╚═══════════════╝\033[0m\n";
    system("shutdown -h now");
  }

  void udp_handle_receive(const boost::system::error_code& error, size_t bytes_transferred) {

    if (!error && bytes_transferred > 0){
      std::cout << "\n\033[1;36mRECVD FROM UDP bytes_transferred = \033[0m" << bytes_transferred << std::endl;
      std::cout << "recvd_count_udp = " << recvd_count_udp << std::endl;
      for (int i = 0; i < bytes_transferred; i++){
        printf("[%u]", dataFromUDP[i]);
      }
      std::cout << std::endl;
      if(!parserUDP(dataFromUDP)){
        std::cout << "\033[1;31mUDP data not valid\033[0m\n";
        memset(dataFromUDP, 0, sizeof(dataFromUDP));
        read_msg_udp();
        return;
      }
      getMsgFromUDP = true;
      memset(dataToFingersTopic, 0, sizeof(dataToFingersTopic));
      memset(currentState.keepalive, 0, sizeof(currentState.keepalive));
      recvd_count_udp++;
      memcpy(dataToFingersTopic_OLD, dataToFingersTopic, sizeof(dataToFingersTopic_OLD));
      memcpy(dataToFingersTopic, dataFromUDP + 3, sizeof(dataToFingersTopic)); //fingers + hand_mount
      currentState.hold_position        = dataFromUDP[sizeof(dataFromUDP) - 8 * sizeof(uint8_t)];
      currentState.camera_from_udp      = dataFromUDP[sizeof(dataFromUDP) - 7 * sizeof(uint8_t)];
      currentState.relay_state_from_udp = dataFromUDP[sizeof(dataFromUDP) - 6 * sizeof(uint8_t)];
      memcpy(currentState.keepalive,    dataFromUDP + sizeof(dataFromUDP) - 5 * sizeof(uint8_t), sizeof(currentState.keepalive));
      read_msg_udp();
    } else {
      //std::cerr << error.what();
    }
  }

  bool equal(uint8_t* arr1, uint32_t arr1Size, uint8_t* arr2, uint32_t arr2Size){
    if (arr1Size != arr2Size) return false;
    for (size_t i = 0; i < arr1Size; i++) {
      if (arr1[i] != arr2[i]) return false;
    }
    return true;
  }

  void sendMsgToFingers(){
    if (currentState.hold_position == 1){
      printf("\033[1;33mcurrentState.hold_position = %u.\033[0m\n", currentState.hold_position);
      if(!equal(dataToFingersTopic_OLD, sizeof(dataToFingersTopic_OLD), dataToFingersTopic, sizeof(dataToFingersTopic))){
        memcpy(dataToFingersTopic, dataToFingersTopic_OLD, sizeof(dataToFingersTopic));
      }
    }
    //отправка пакета в топик "toFingersTopic"
    std_msgs::ByteMultiArray sendMsgToFingersTopic;
    sendMsgToFingersTopic.layout.dim.push_back(std_msgs::MultiArrayDimension());
    sendMsgToFingersTopic.layout.dim[0].size = 1;
    sendMsgToFingersTopic.layout.dim[0].stride = sizeof(dataToFingersTopic);
    sendMsgToFingersTopic.data.clear();
    std::cout << "\033[1;34mSEND TO toFingersTopic: \033[0m";
    for (int i = 0; i < sizeof(dataToFingersTopic); i++){
      printf("[%u]", dataToFingersTopic[i]);
      sendMsgToFingersTopic.data.push_back(dataToFingersTopic[i]);
    }
    toFingersPub.publish(sendMsgToFingersTopic);
    std::cout << std::endl;
  }

  void sendMsgToCamBat(){
    if (currentState.camera_from_udp == currentState.camera_from_udp_prev &&
          currentState.relay_state_from_udp == currentState.relay_state_from_udp_prev) return;
    currentState.camera_from_udp_prev = currentState.camera_from_udp;
    currentState.relay_state_from_udp_prev = currentState.relay_state_from_udp;
    //отправка пакета в топик "camera_topic"
    std_msgs::ByteMultiArray sendMsgToCameraTopic;
    sendMsgToCameraTopic.layout.dim.push_back(std_msgs::MultiArrayDimension());
    sendMsgToCameraTopic.layout.dim[0].size = 1;
    sendMsgToCameraTopic.layout.dim[0].stride = sizeof(dataToFingersTopic);
    sendMsgToCameraTopic.data.clear();
    std::cout << "\033[1;34mSEND TO camera_topic:\033[0m\n";
    printf("camera_from_udp = [%u]\n", currentState.camera_from_udp);
    printf("relay_state = [%u]\n", currentState.relay_state_from_udp);
    sendMsgToCameraTopic.data.push_back(currentState.camera_from_udp);
    sendMsgToCameraTopic.data.push_back(currentState.relay_state_from_udp);
    toCamPub.publish(sendMsgToCameraTopic);
    std::cout << std::endl;
  }

  void sendMsgToUDP(){
    std::cout << "recvd_count_topic_cam_bat = " << recvd_count_topic_cam_bat << std::endl;
    //формируем пакет
    dataToUDP[0] = 0xBB;                                                                            //header 1b
    dataToUDP[1] = 0xAA;                                                                            //header 1b
    dataToUDP[2] = sizeof(dataToUDP);                                                               //data length 1b
    memcpy(dataToUDP + 3, dataFromFingersTopic, sizeof(dataFromFingersTopic) - sizeof(uint8_t));    //data from fingers
    dataToUDP[sizeof(dataToUDP) - 12 * sizeof(uint8_t)] = currentState.hand_mount;                  //hand_mount 1b
    dataToUDP[sizeof(dataToUDP) - 11 * sizeof(uint8_t)] = currentState.hold_position;               //hold_position 1b
    dataToUDP[sizeof(dataToUDP) - 10 * sizeof(uint8_t)] = currentState.camera_from_bat_cam;         //camera_from_bat_cam 1b
    dataToUDP[sizeof(dataToUDP) - 9  * sizeof(uint8_t)] = currentState.bat_24V;                     //bat_24V 1b
    dataToUDP[sizeof(dataToUDP) - 8  * sizeof(uint8_t)] = currentState.bat_48V;                     //bat_48V 1b
    dataToUDP[sizeof(dataToUDP) - 7  * sizeof(uint8_t)] = resvdFromAllDev;                          //allDevOk 1b
    dataToUDP[sizeof(dataToUDP) - 6  * sizeof(uint8_t)] = currentState.relay_state_from_bat_cam;    //relay_state 1b
    memcpy(dataToUDP + sizeof(dataToUDP) - 5 * sizeof(uint8_t), currentState.keepalive, 
        sizeof(currentState.keepalive));                                                            //keepalive 4b
    dataToUDP[sizeof(dataToUDP) - 1 * sizeof(uint8_t)] = 
        umba_crc8_table(dataToUDP, sizeof(dataToUDP) - sizeof(uint8_t));                            //crc8 1b
    show_cur_state();
    //отправляем пакет в UDP
    boost::system::error_code error;
    auto sent = socket_.send_to(boost::asio::buffer(dataToUDP), sender_endpoint_, 0, error);
    if (!error && sent > 0){
      send_count_udp++;
      std::cout << "\033[1;36mSEND TO UDP bytes_transferred = \033[0m" << sent << std::endl;
      std::cout << "send_count_udp = " << send_count_udp << std::endl;
      for (int i = 0; i < sent; i++){
        printf("[%u]", dataToUDP[i]);
        if (i==2 || i==11 || i== 20 || i==29 || i==38 || i==47 || i==56 || i==57 || i==58 || i==59 || i==60 || i== 61 || i==62 || i==63 || i==67){
          printf("\033[1;31m|\033[0m");
        }
      }
      std::cout << std::endl;
      //resvdFromAllDev = 0;
    } else {
      //std::cerr << error.what();
    }
  }

  static inline uint8_t getLen(uint8_t* ptrBuff)
  {
    return ptrBuff[2];
  }

  static inline uint8_t getCrc8(uint8_t* ptrBuff, uint32_t len)
  {
    return ptrBuff[len - sizeof(uint8_t)];
  }

  bool parserUDP(uint8_t* dataFromUDP){
    if (dataFromUDP[0] != 0xAA) return false;
    if (dataFromUDP[1] != 0xBB) return false;
    /* Если длина пакета не валидная, ошибка */
    if (getLen(dataFromUDP) != DATA_FROM_UDP_SIZE) {
      return false;
    }
    /* Если контрольная сумма не совпадает, приняли муссор, ошибка */
    if (umba_crc8_table(dataFromUDP, DATA_FROM_UDP_SIZE - sizeof(uint8_t)) != getCrc8(dataFromUDP, DATA_FROM_UDP_SIZE)) {
      return false;
    }
    return true;
  }
};

int main(int argc, char** argv){
  try{
    std::cout << "\n\033[1;32m╔═══════════════════════════════╗\033[0m"
              << "\n\033[1;32m║master_eth_receiver is running!║\033[0m" 
              << "\n\033[1;32m╚═══════════════════════════════╝\033[0m\n";
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
