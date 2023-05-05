#include <iostream>
#include <unistd.h>
#include <ros/ros.h>
#include <boost/asio.hpp>
#include <boost/chrono.hpp>
#include <std_msgs/ByteMultiArray.h>
#include <fingers/From_Finger.h>
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

/*DATA TYPE*/
#define CODE_PART RAW_UDP_DATA
/////////////

#if CODE_PART == RAW_UDP_DATA

#define DATA_FROM_FINGER_SIZE 13
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

  int debugBigFinger   = 0;
  int debugIndexFinger = 0;
  int debugMidFinger   = 0;
  int debugRingFinger  = 0;
  int debugPinky       = 0;
  int debugModulOtv    = 0;
  int debugBatCam      = 0;
  int debugAllFingers  = 0;

class RS_Server
{
public:
  RS_Server(protocol_master::ProtocolMaster& protocol_)
  : m_protocol(protocol_){
    toFingersSub = node.subscribe<std_msgs::ByteMultiArray>("toFingersTopic", 0, &RS_Server::topic_handle_receive, this);

    debugFromBigFingerPub        = node.advertise<fingers::From_Finger>("debugFromBigFingerTopic", 0);
    debugFromIndexFingerPub      = node.advertise<fingers::From_Finger>("debugFromIndexFingerTopic", 0);
    debugFromMidFingerPub        = node.advertise<fingers::From_Finger>("debugFromMidFingerTopic", 0);
    debugFromRingFingerPub       = node.advertise<fingers::From_Finger>("debugFromRingFingerTopic", 0);
    debugFromPinkyPub            = node.advertise<fingers::From_Finger>("debugFromPinkyTopic", 0);
    debugFromModulOtvPub         = node.advertise<fingers::From_Finger>("debugFromModulOtvTopic", 0);

    // ros::param::param<int>("~_debugBigFinger", debugBigFinger, 0);
    // ros::param::param<int>("~_debugIndexFinger", debugIndexFinger, 0);
    // ros::param::param<int>("~_debugMidFinger", debugMidFinger, 0);
    // ros::param::param<int>("~_debugRingFinger", debugRingFinger, 0);
    // ros::param::param<int>("~_debugPinky", debugPinky, 0);
    // ros::param::param<int>("~_debugModulOtv", debugModulOtv, 0);
    // ros::param::param<int>("~_debugBatCam", debugBatCam, 0);

    // node.getParam("/fingers/_debugBigFinger",   debugBigFinger);
    // node.getParam("/fingers/_debugIndexFinger", debugIndexFinger);
    // node.getParam("/fingers/_debugMidFinger",   debugMidFinger);
    // node.getParam("/fingers/_debugRingFinger",  debugRingFinger);
    // node.getParam("/fingers/_debugPinky",       debugPinky);
    // node.getParam("/fingers/_debugModulOtv",    debugModulOtv);
    // node.getParam("/fingers/_debugBatCam",      debugBatCam);

    fromFingersPub = node.advertise<std_msgs::ByteMultiArray>("fromFingersTopic", 0);
  };

  void nodeFromTopicProcess(){
    if (getMsgFromTopic){
      update_hand_mount();
      toEachFinger();              
      sendMsgToTopic();
      getMsgFromTopic = false;
    }

    // while (count < 1000) {
    //   ROS_INFO("");
    //   boost::chrono::system_clock::time_point cur_tp = boost::chrono::system_clock::now();
    //   boost::chrono::duration<double> ex_time = cur_tp - first_tp;
    //   std::cout << "\033\n[1;32mExecution time: \033\n[0m" << ex_time.count() * 1000000 << "\n";
    //   std::cout << "count = " << count << "\n";
    //   first_tp = boost::chrono::system_clock::now();
    //   toEachFinger();
    //   sendMsgToTopic();
    //   count++;
    //   ROS_INFO("count = %d\n", count);
    // }
  
  }
  
private:
  ros::NodeHandle node;
  protocol_master::ProtocolMaster& m_protocol;
  uint8_t dataFromHandMount[HMOUNT_DATA_SIZE] = {0};
  uint8_t dataFromTopic[DATA_FROM_TOPIC_SIZE] = {0};
  uint8_t dataToTopic[DATA_TO_TOPIC_SIZE] = {0};
  uint8_t dataToFinger[DATA_TO_FINGER_SIZE] = {0};
  uint8_t dataFromFinger_new[6][DATA_FROM_FINGER_SIZE] = {0};
  uint8_t dataFromFinger_old[6][DATA_FROM_FINGER_SIZE] = {0};

  ros::Publisher fromFingersPub;
  ros::Subscriber toFingersSub;

  ros::Publisher debugFromBigFingerPub;
  ros::Publisher debugFromIndexFingerPub;
  ros::Publisher debugFromMidFingerPub;
  ros::Publisher debugFromRingFingerPub;
  ros::Publisher debugFromPinkyPub;
  ros::Publisher debugFromModulOtvPub;

  uint32_t recvd_count_topic = 0;
  uint32_t fail_cnt = 0;
  uint32_t fail_cnt_f[6] = {0};
  uint32_t rcvd_cnt_f[6] = {0};

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
  boost::chrono::system_clock::time_point first_tp = boost::chrono::system_clock::now();
  uint32_t count = 0;


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
    memset(dataFromHandMount, 0, sizeof(dataFromHandMount));
    dataFromHandMountSize = 0;
    dataToHandMount = dataFromTopic[sizeof(dataFromTopic) - sizeof(uint8_t)];
    // std::cout << "\ndataToHandMount = ";
    // printf("%u\n", dataToHandMount);

    #if CODE_PART == RAW_UDP_DATA

    if(m_protocol.sendCmdReadWrite(hand_mount_addr, 0x30, &dataToHandMount, sizeof(uint8_t), 
                                            dataFromHandMount, &dataFromHandMountSize)){
      resvdFromAllDev |= fingers_OK[6]; //ответ пришел
      // std::cout << "\nOk\n";
      recvd_count_rs++;
      // printf("\nrecvd_count_rs = %u\n", recvd_count_rs);
      // printf("\nfail_cnt = %u\n", fail_cnt);
      
    } else {
      resvdFromAllDev &= ~fingers_OK[6]; //ответ НЕ пришел
      // std::cout << "\033\n[1;31mNO DATA FROM HAND_MOUNT\033\n[0m";
      memset(dataFromHandMount, 0, dataFromHandMountSize);
      dataFromHandMountSize = 0;
      fail_cnt++;
      // printf("\nfail_cnt = %u\n", fail_cnt);
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
      resvdFromAllDev &= ~fingers_OK[6]; //ответ НЕ пришел
      std::cout << "\033\n[1;31mNO DATA FROM HAND_MOUNT\033\n[0m";
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
      // std::cout << "\033\n[1;34mSIZE OF RECVD DATA FROM TOPIC toFingersTopic = \033[0m" << recvdMsg->data.size() << std::endl;
      // std::cout << "recvd_count_topic = " << recvd_count_topic << std::endl;
      memset(dataFromTopic, 0, sizeof(dataFromTopic));
      memset(dataToTopic, 0, sizeof(dataToTopic));
      resvdFromAllDev = 0;
      for (int i = 0; i < recvdMsg->data.size(); i++){
        dataFromTopic[i] = recvdMsg->data[i];
        // printf("[%u]", dataFromTopic[i]);
      }
  }

  void toEachFinger(){
    // boost::chrono::system_clock::time_point cur_tp = boost::chrono::system_clock::now();
    // boost::chrono::duration<double> ex_time = cur_tp - first_tp;
    // std::cout << "Execution time: " << ex_time.count() << std::endl;
    // first_tp = boost::chrono::system_clock::now();

    //отправка запроса каждому пальцу без ожидания ответа
    for (int i = 0; i < fingersAddrs.size(); i++){
      memset(dataToFinger, 0, sizeof(dataToFinger));
      memcpy(dataToFinger, dataFromTopic + i * sizeof(dataToFinger), sizeof(dataToFinger));
      // std::cout << "\ndataToFinger ";
      // printf("%u = ", fingersAddrs[i]);
      // for (int i = 0; i < sizeof(dataToFinger); i++){
      //   printf("[%u]", dataToFinger[i]);
      // }
      m_protocol.sendCmdWrite(fingersAddrs[i], 0x30, dataToFinger, sizeof(dataToFinger));
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    //ожидание ответа от каждого пальца
    if (m_protocol.RSRead(dataFromFinger_new, &resvdFromAllDev)){

      for (int i = 0; i < fingersAddrs.size(); i++){
        if((resvdFromAllDev & fingers_OK[i]) != 0){
          // std::cout << "\033\n[1;32mOk\033\n[0m";
          recvd_count_rs++;
          rcvd_cnt_f[fingersAddrs[i]- 0x11]++;  
          // printf("\nrecvd_count_rs = %u\n", recvd_count_rs);
          // std::cout << "FAIL CNT OF " << fingersAddrs[i] << "device = " << fail_cnt_f[fingersAddrs[i] - 0x11] << std::endl;
          // std::cout << "RCVD CNT OF " << fingersAddrs[i] << "device = " << rcvd_cnt_f[fingersAddrs[i] - 0x11] << std::endl;
          memset(dataFromFinger_old[i], 0, 13);
          memcpy(dataFromFinger_old[i], dataFromFinger_new[i], 13);
        }else{
          // std::cout << "\033\n[1;31mNO DATA FROM DEVICE\033\n[0m";
          // std::cout << "FAIL CNT OF " << fingersAddrs[i] << "device = " << fail_cnt_f[fingersAddrs[i] - 0x11] << std::endl;
          // std::cout << "RCVD CNT OF " << fingersAddrs[i] << "device = " << rcvd_cnt_f[fingersAddrs[i] - 0x11] << std::endl;
          fail_cnt_f[fingersAddrs[i] - 0x11]++;
          memset(dataFromFinger_new[i], 0, 13);
          memcpy(dataFromFinger_new[i], dataFromFinger_old[i], 13);
          fail_cnt++;
          // printf("\nfail_cnt = %u\n", fail_cnt);
        }
        memcpy(dataToTopic + i * (13 - 4 * sizeof(uint8_t)), 
            &dataFromFinger_new[i][3 * sizeof(uint8_t)], 13 - 4 * sizeof(uint8_t));
      }
    }
    dataToTopic[sizeof(dataToTopic) - sizeof(uint8_t)] = resvdFromAllDev;
  }

  void sendMsgToDebugTopic(const fingers::From_Finger msgsArrToDebugFingers[]){
    if (debugAllFingers) {
      debugFromBigFingerPub.publish(msgsArrToDebugFingers[0]);
      debugFromIndexFingerPub.publish(msgsArrToDebugFingers[1]);
      debugFromMidFingerPub.publish(msgsArrToDebugFingers[2]);
      debugFromRingFingerPub.publish(msgsArrToDebugFingers[3]);
      debugFromPinkyPub.publish(msgsArrToDebugFingers[4]);
      debugFromModulOtvPub.publish(msgsArrToDebugFingers[5]);
      return;
    }
    if (debugBigFinger == 1)   debugFromBigFingerPub.publish(msgsArrToDebugFingers[0]);
    if (debugIndexFinger == 1) debugFromIndexFingerPub.publish(msgsArrToDebugFingers[1]);
    if (debugMidFinger == 1)   debugFromMidFingerPub.publish(msgsArrToDebugFingers[2]);
    if (debugRingFinger == 1)  debugFromRingFingerPub.publish(msgsArrToDebugFingers[3]);
    if (debugPinky == 1)       debugFromPinkyPub.publish(msgsArrToDebugFingers[4]);
    if (debugModulOtv == 1)    debugFromModulOtvPub.publish(msgsArrToDebugFingers[5]);
  }

  void setMsgsToDebugTopic(fingers::From_Finger msgsArrToDebugFingers[], uint8_t dataToTopic[]){
    for (size_t i = 0; i < 6; i++){
      msgsArrToDebugFingers[i].current = (dataToTopic[i * 9 + 1] << 8) + dataToTopic[i * 9];
      msgsArrToDebugFingers[i].pressure = (dataToTopic[i * 9 + 3] << 8) + dataToTopic[i * 9 + 2];
      msgsArrToDebugFingers[i].angle = (dataToTopic[i * 9 + 5] << 8) + dataToTopic[i * 9 + 4];
      msgsArrToDebugFingers[i].count = (dataToTopic[i * 9 + 7] << 8) + dataToTopic[i * 9 + 6];
      msgsArrToDebugFingers[i].mask = dataToTopic[i * 9 + 8];
    }
  }


  void sendMsgToTopic(){
    //отправка пакета в топик "fromFingersTopic" 
    std_msgs::ByteMultiArray sendMsgFromFingersTopic;
    sendMsgFromFingersTopic.layout.dim.push_back(std_msgs::MultiArrayDimension());
    sendMsgFromFingersTopic.layout.dim[0].size = 1;
    sendMsgFromFingersTopic.layout.dim[0].stride = sizeof(dataToTopic);
    sendMsgFromFingersTopic.data.clear();
    // std::cout << "\n\033[1;34mSEND MSG TO TOPIC fromFingersTopic: \n\033[0m";
    for (int i = 0; i < sizeof(dataToTopic); i++){
      sendMsgFromFingersTopic.data.push_back(dataToTopic[i]);
      // printf("[%u]", dataToTopic[i]);
    }
    // std::cout << "\nresvdFromAllDev = " << resvdFromAllDev;
    fromFingersPub.publish(sendMsgFromFingersTopic);

    fingers::From_Finger msgsArrToDebugFingers[6];
    setMsgsToDebugTopic(msgsArrToDebugFingers, dataToTopic);
    sendMsgToDebugTopic(msgsArrToDebugFingers);
  }
  
};

int main(int argc, char** argv)
{
  QCoreApplication coreApplication(argc, argv);

  std::string devPort = "USB0";
  std::string baudrate = "256000";

  ros::param::get("/_devPortForFingers", devPort);
  ros::param::get("/_baudrateForFingers", baudrate);
  try{

    std::cout << "\n\033[1;32m╔═══════════════════════════════════════╗\033[0m"
              << "\n\033[1;32m║       MTOPIC_RECIEVER is running!     ║\033[0m" 
              << "\n\033[1;32m║Baud rate: " << baudrate << ", Port: /dev/tty" << devPort << "\t║\033[0m"
              << "\n\033[1;32m╚═══════════════════════════════════════╝\033[0m\n";
    ros::init(argc, argv, "master_topic_receiver");
    
    qt_serial::Qt_Serial_Async qt_RS_transp("/dev/tty" + devPort, baudrate);
    protocol_master::ProtocolMaster rs_prot_master(qt_RS_transp, &coreApplication);
    RS_Server raspbPi(rs_prot_master);

    ros::param::get("/_debugBigFinger", debugBigFinger);
    ros::param::get("/_debugIndexFinger", debugIndexFinger);
    ros::param::get("/_debugMidFinger", debugMidFinger);
    ros::param::get("/_debugRingFinger", debugRingFinger);
    ros::param::get("/_debugPinky", debugPinky);
    ros::param::get("/_debugModulOtv", debugModulOtv);
    ros::param::get("/_debugBatCam", debugBatCam);
    ros::param::get("/_debugAllFingers", debugAllFingers);

    std::cout << "debugBatCam " << debugBigFinger << std::endl;
    std::cout << "debugIndexFinger " << debugIndexFinger << std::endl;
    std::cout << "debugMidFinger "<< debugMidFinger << std::endl;
    std::cout << "debugRingFinger "<< debugRingFinger << std::endl;
    std::cout << "debugPinky "<< debugPinky << std::endl;
    std::cout << "debugModulOtv "<< debugModulOtv << std::endl;

    while(ros::ok()){
      raspbPi.nodeFromTopicProcess();
      ros::spinOnce();
    }
  } catch (std::exception e){
    std::cerr << "\nExeption: " << e.what() << std::endl;
  }
  return 0;
};