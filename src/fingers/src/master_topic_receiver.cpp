#include <iostream>
#include <ros/ros.h>
#include <boost/asio.hpp>
#include <std_msgs/ByteMultiArray.h>
#include "qt_serial.hpp"
#include "protocol.hpp"
#include "umba_crc_table.h"
#include <chrono>
#include <thread>
#include <chrono>
#include <ctime>
#include <QCoreApplication>

#define HMOUNT_DATA_SIZE 5

#define RAW_UDP_DATA 1
#define COMPLETE_UDP_DATA 2

#define CODE_PART COMPLETE_UDP_DATA

#if CODE_PART == RAW_UDP_DATA

#define DATA_FROM_FINGER_SIZE 9
#define DATA_TO_FINGER_SIZE 5
#define DATA_FROM_TOPIC_SIZE 31
#define DATA_TO_TOPIC_SIZE 56

#elif CODE_PART == COMPLETE_UDP_DATA

#define DATA_FROM_FINGER_SIZE 6
#define DATA_TO_FINGER_SIZE 4
#define DATA_FROM_TOPIC_SIZE 25
#define DATA_TO_TOPIC_SIZE 38

#else 

#error "INCORRECT CODE_PART"

#endif

class RS_Server
{
public:
  RS_Server(protocol_master::ProtocolMaster& protocol_)
  : m_protocol(protocol_){
    toFingersSub = node.subscribe<std_msgs::ByteMultiArray>("toFingersTopic", 0, &RS_Server::topic_handle_receive, this);
    fromFingersPub = node.advertise<std_msgs::ByteMultiArray>("fromFingersTopic", 0);
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
  uint8_t dataFromHandMount[HMOUNT_DATA_SIZE] = {0};
  uint8_t dataFromTopic[DATA_FROM_TOPIC_SIZE] = {0};
  uint8_t dataToTopic[DATA_TO_TOPIC_SIZE] = {0};
  uint8_t dataToFinger[DATA_TO_FINGER_SIZE] = {0};
  uint8_t dataFromFinger[DATA_FROM_FINGER_SIZE] = {0};
  uint32_t dataFromFingerSize = 0;
  ros::NodeHandle node;
  ros::Publisher fromFingersPub;
  ros::Subscriber toFingersSub;
  uint32_t recvd_count_topic = 0;
  uint32_t fail_cnt = 0;
  uint32_t send_count_rs = 0;
  uint32_t recvd_count_rs = 0;
  uint32_t send_count_topic = 0;
  bool getMsgFromTopic = false;
  std::vector<uint8_t> fingersAddrs = {0x11, 0x12, 0x13, 0x14, 0x15, 0x16};       //5 пальцев + модуль отведения
  uint8_t hand_mount_addr = 0x31;                                                 //устройство отсоединения схвата
  uint8_t dataToHandMount = 0;
  bool start_communication = true;
  uint8_t resvdFromAllDev = 0;
  uint32_t dataFromHandMountSize = 0;

  enum fingersOK{                  //ок, если ответ пришел
    bigFinger      =     1,        //большой палец
    foreFinger     =     2,        //указательный палец
    middleFinger   =     4,        //средный палец
    ringFinger     =     8,        //безымянный палец
    pinkyFinger    =     16,       //мизинец
    leadModule     =     32,       //модуль отведения
    handMount      =     64        //устройство отсоединения схвата
  };

  uint8_t fingers_OK[7] = {1, 2, 4, 8, 16, 32, 64}; ////ок, если ответ пришел

  void update_hand_mount(){
    if(dataToHandMount == dataFromTopic[sizeof(dataFromTopic) - sizeof(uint8_t)]) return;
    dataToHandMount = dataFromTopic[sizeof(dataFromTopic) - sizeof(uint8_t)];
    std::cout << "\ndataToHandMount = ";
    printf("%u\n", dataToHandMount);

    #if CODE_PART == RAW_UDP_DATA

    if(m_protocol.sendCmdReadWrite(hand_mount_addr, 0x3, &dataToHandMount, sizeof(uint8_t), 
                                            dataFromHandMount, &dataFromHandMountSize)){
      resvdFromAllDev |= fingers_OK[6]; //ответ пришел
      std::cout << "\nOk\n";
    } else {
      std::cout << "\nFail\n";
      memset(dataFromHandMount, 0, dataFromHandMountSize);
      dataFromHandMountSize = 0;
    }

    #elif CODE_PART == COMPLETE_UDP_DATA

    uint8_t dataToHMount[HMOUNT_DATA_SIZE] = {0};
    dataToHMount[0] = hand_mount_addr;
    dataToHMount[1] = 0x05;
    dataToHMount[2] = 0x3;
    dataToHMount[3] = dataToHandMount;
    dataToHMount[4] = umba_crc8_table(dataToHMount, HMOUNT_DATA_SIZE - sizeof(uint8_t));
    if(m_protocol.sendSomeCmd(dataToHMount, sizeof(dataToHMount), dataFromHandMount, &dataFromHandMountSize)){
      resvdFromAllDev |= fingers_OK[6]; //ответ пришел
      std::cout << "\nOk\n";
      recvd_count_rs++;
      printf("\nrecvd_count_rs = %u\n", recvd_count_rs);
      printf("\nfail_cnt = %u\n", fail_cnt);
    } else {
      std::cout << "\nFail\n";
      memset(dataFromHandMount, 0, dataFromHandMountSize);
      dataFromHandMountSize = 0;
      fail_cnt++;
      printf("\nfail_cnt = %u\n", fail_cnt);
    }

    #else 

    #error "INCORRECT CODE_PART"

    #endif

    dataToTopic[sizeof(dataToTopic) - 2 * sizeof(uint8_t)] = dataFromHandMount[3];
  }

  void topic_handle_receive(const std_msgs::ByteMultiArray::ConstPtr& recvdMsg) {
      getMsgFromTopic = true;
      recvd_count_topic++;
      std::cout << "\033\n[1;34mRECVD FROM TOPIC toFingersTopic recvdMsg->data.size() = \033[0m" << recvdMsg->data.size() << std::endl;
      std::cout << "recvd_count_topic = " << recvd_count_topic << std::endl;
      memset(dataFromTopic, 0, sizeof(dataFromTopic));
      memset(dataToTopic, 0, sizeof(dataToTopic));
      resvdFromAllDev = 0;
      for (int i = 0; i < recvdMsg->data.size(); i++){
        dataFromTopic[i] = recvdMsg->data[i];
        printf("[%u]", dataFromTopic[i]);
      }
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

      #if CODE_PART == RAW_UDP_DATA

      //
      if (m_protocol.sendCmdReadWrite(fingersAddrs[i], 0x3, dataToFinger, sizeof(dataToFinger), 
                                                    dataFromFinger, &dataFromFingerSize)) {
        resvdFromAllDev |= fingers_OK[i]; //ответ пришел
        std::cout << "\nOk\n";
      } else {
        std::cout << "\nFail\n";
        memset(dataFromHandMount, 0, dataFromHandMountSize);
        dataFromHandMountSize = 0;
      }
      //

      #elif CODE_PART == COMPLETE_UDP_DATA

      //
      if (m_protocol.sendSomeCmd(dataToFinger, sizeof(dataToFinger), dataFromFinger, &dataFromFingerSize)) {
        resvdFromAllDev |= fingers_OK[i]; //ответ пришел
        std::cout << "\nOk\n";
        recvd_count_rs++;
        printf("\nrecvd_count_rs = %u\n", recvd_count_rs);
        printf("\nfail_cnt = %u\n", fail_cnt);
      } else {
        std::cout << "\nFail\n";
        memset(dataFromHandMount, 0, dataFromHandMountSize);
        dataFromHandMountSize = 0;
        fail_cnt++;
        printf("\nfail_cnt = %u\n", fail_cnt);
      }
      //

      #else 

      #error "INCORRECT CODE_PART"

      #endif

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
    std::cout << "\n\033[1;34mSEND MSG TO TOPIC fromFingersTopic: \n\033[0m";
    for (int i = 0; i < sizeof(dataToTopic); i++){
      sendMsgFromFingersTopic.data.push_back(dataToTopic[i]);
      printf("[%u]", dataToTopic[i]);
    }
    fromFingersPub.publish(sendMsgFromFingersTopic);
  }
};

int main(int argc, char** argv)
{
  QCoreApplication coreApplication(argc, argv);

  std::string devPort = "USB0";
  std::string baudrate = "256000";

  ros::param::param<std::string> ("~_devPortForFingers", devPort, "USB0");
  ros::param::param<std::string> ("~_baudrateForFingers", baudrate, "256000");
  try{
    std::cout << "\n\033[1;32m╔═══════════════════════════════════════╗\033[0m"
              << "\n\033[1;32m║       MTOPIC_RECIEVER is running!     ║\033[0m" 
              << "\n\033[1;32m║Baud rate: " << baudrate << ", Port: /dev/tty" << devPort << "\t║\033[0m"
              << "\n\033[1;32m╚═══════════════════════════════════════╝\033[0m\n";
    ros::init(argc, argv, "master_topic_receiver");
    
    qt_serial::Qt_Serial_Async qt_RS_transp("/dev/tty" + devPort, 256000);
    protocol_master::ProtocolMaster rs_prot_master(qt_RS_transp, &coreApplication);
    RS_Server raspbPi(rs_prot_master);

    while(ros::ok()){
      raspbPi.nodeFromTopicProcess();
      ros::spinOnce();
    }
  } catch (std::exception e){
    std::cerr << "\nExeption: " << e.what() << std::endl;
  }
  return 0;
};