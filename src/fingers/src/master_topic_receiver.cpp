/*
  Задачи данной программы:
  1) Прием данных из ROS-топика(1) для последующей отправки на Кисть и устройство отсоединения схвата
  2) Отправка  полученных данных на Кисть и устройство отсоединения схвата
  3) Получение актуальных данных  с Кисти и устройства отсоединения схвата
  4) Отправка  полученных данных  с Кисти и устройства отсоединения схвата в ROS-топик(2) для последующей отправки по UDP

  ROS-топик(1) -> тек. программа -> Кисть, устройство отсоединения схвата -> тек. программа -> ROS-топик(2)
*/

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

// размер пакета устройства отсоединения схвата
// количество пальцев
#define HMOUNT_DATA_SIZE              5  // WRITE_DATA 1б + OTHER 4б
#define FINGERS_COUNT                 6

// размеры пакетов данных в нормальном режиме
#define DATA_FROM_FINGER_SIZE         13 // READ_DATA  9б + OTHER 4б
#define DATA_TO_FINGER_SIZE           6  // CMD 1b + WRITE_DATA 5б
#define DATA_FROM_TOPIC_SIZE          37 // [6f*6b][HM]
#define DATA_TO_TOPIC_SIZE            62 // [6f*10b][HM][OK]

// размеры пакетов данных в калибровочном режиме
#define DATA_SETTING_FROM_FINGER_SIZE 12 // READ_DATA  8б + OTHER 4б
#define DATA_SETTING_TO_FINGER_SIZE   9  // CMD 1b + WRITE_DATA 8б
#define DATA_SETTING_FROM_TOPIC_SIZE  55 // [6f*9b][HM]
#define DATA_SETTING_TO_TOPIC_SIZE    56 // [6f*9b][HM][OK]

// 0 - debug off, 1 - debug on : возможность отслеживать данные с Кисти в удобочитаемом виде
int debugBigFinger   = 0;
int debugIndexFinger = 0;
int debugMidFinger   = 0;
int debugRingFinger  = 0;
int debugPinky       = 0;
int debugModulOtv    = 0;
int debugBatCam      = 0;
int debugAllFingers  = 0;

/*
  Класс RS_Server предназначен для реализации функционала обмена данными между ROS-топиками, Кистью и устройством отсоединения схвата
  Класс RS_Server включает следующие методы:
    * nodeFromTopicProcess()  - основной цикл программы, который состоит из последовательности действий:
        1) отправка данных на устройство отсоединения схвата                              - метод sendToHandMount()  или sendSettingToHandMount()
        2) отправка данных на Кисть каждому пальцу                                        - метод sendToEachFinger() или sendSettingToEachFinger()
        3) ожидание и получение ответа от каждого пальца и устройства отсоединения схвата - метод getResponses()     или getSettingResponses()
        4) отправка полученных ответов в ROS-топик(2) для последующей отправки по UDP     - метод sendMsgToTopic()   или sendSettingMsgToTopic()
    * topic_handle_receive()                    - обработчик сообщений от ROS-топика(1) на Кисть
    * sendToHandMount()                         - отправка данных устройству отсоединения схвата без ожидания ответа в нормальном режиме
    * sendSettingToHandMount()                  - отправка данных устройству отсоединения схвата без ожидания ответа в калибровочном режиме
    * sendToEachFinger()                        - отправка данных каждому пальцу без ожидания ответа в нормальном режиме
    * sendSettingToEachFinger()                 - отправка данных каждому пальцу без ожидания ответа в калибровочном режиме
    * sendReadToAllDev()                        - отправка команды на чтение каждому пальцу
    * fingersResponsesProcessing()              - обработка ответов от каждого пальца в нормальном режиме
    * handMountResponsesProcessing()            - обработка ответа от устройства отсоединения схвата в нормальном режиме
    * fingersSettingResponsesProcessing()       - обработка ответов от каждого пальца в калибровочном режиме
    * handMountSettingResponsesProcessing()     - обработка ответа от устройства отсоединения схвата в калибровочном режиме
    * getResponses()                            - ожидание и обработка ответов от каждого пальца и устройства отсоединения схвата в нормальном режиме
    * getSettingResponses()                     - ожидание и обработка ответов от каждого пальца и устройства отсоединения схвата в калибровочном режиме
    * sendMsgToTopic()                          - отправка данных в ROS-топик(2) от Кисти в нормальном режиме
    * sendSettingMsgToTopic()                   - отправка данных в ROS-топик(2) от Кисти в калибровочном режиме
    * sendMsgToDebugTopic()                     - перенаправление сообщений от Кисти в отдельный ROS-топик(3) в удобочитаемом виде
    * setMsgsToDebugTopic()                     - получение удобочитаемых сообщений от Кисти для отправки в отдельный ROS-топик(3)
*/
class RS_Server
{
public:

  // конструктор класса RS_Server, в котором происходит подключение к ROS-топикам 
  RS_Server(protocol_master::ProtocolMaster& protocol_)
  : m_protocol(protocol_){
    toFingersSub = node.subscribe<std_msgs::ByteMultiArray>("toFingersTopic", 0, &RS_Server::topic_handle_receive, this);

    debugFromBigFingerPub   = node.advertise<fingers::From_Finger>     ("debugFromBigFingerTopic",    0);
    debugFromIndexFingerPub = node.advertise<fingers::From_Finger>     ("debugFromIndexFingerTopic",  0);
    debugFromMidFingerPub   = node.advertise<fingers::From_Finger>     ("debugFromMidFingerTopic",    0);
    debugFromRingFingerPub  = node.advertise<fingers::From_Finger>     ("debugFromRingFingerTopic",   0);
    debugFromPinkyPub       = node.advertise<fingers::From_Finger>     ("debugFromPinkyTopic",        0);
    debugFromModulOtvPub    = node.advertise<fingers::From_Finger>     ("debugFromModulOtvTopic",     0);
    fromFingersPub          = node.advertise<std_msgs::ByteMultiArray> ("fromFingersTopic",           0);
  };

  // основной цикл программы
  void nodeFromTopicProcess(){

    // порядок действий при нормальном режиме 
    if (getMsgFromTopic && dataFromTopicSize == DATA_FROM_TOPIC_SIZE){
      getMsgFromTopic = false;
      sendToHandMount();
      sendToEachFinger();
      getResponses();
      sendMsgToTopic();
    }
    // порядок действий при калибровочном режиме 
    else if (getMsgFromTopic && dataFromTopicSize == DATA_SETTING_FROM_TOPIC_SIZE){
      getMsgFromTopic = false;
      sendSettingToHandMount();
      sendSettingToEachFinger();
      getSettingResponses();
      sendSettingMsgToTopic();
    }
  }
  
private:
  ros::NodeHandle                  node;
  protocol_master::ProtocolMaster& m_protocol;

  // данные от устройства отсоединения схвата
  uint8_t dataFromHandMount_new[HMOUNT_DATA_SIZE]                                 = {0};
  uint8_t dataFromHandMount_old[HMOUNT_DATA_SIZE]                                 = {0};

  // данные при нормальном режиме 
  uint8_t dataFromTopic[DATA_FROM_TOPIC_SIZE]                                     = {0};
  uint8_t dataToTopic[DATA_TO_TOPIC_SIZE]                                         = {0};
  uint8_t dataToFinger[DATA_TO_FINGER_SIZE]                                       = {0};
  uint8_t dataFromFinger_new[FINGERS_COUNT][DATA_FROM_FINGER_SIZE]                = {0};
  uint8_t dataFromFinger_old[FINGERS_COUNT][DATA_FROM_FINGER_SIZE]                = {0};
  
  // данные при калибровочном режиме 
  uint8_t dataSettingFromTopic[DATA_SETTING_FROM_TOPIC_SIZE]                      = {0};
  uint8_t dataSettingToTopic[DATA_SETTING_TO_TOPIC_SIZE]                          = {0};
  uint8_t dataSettingToFinger[DATA_SETTING_TO_FINGER_SIZE]                        = {0};
  uint8_t dataSettingFromFinger_new[FINGERS_COUNT][DATA_SETTING_FROM_FINGER_SIZE] = {0};
  uint8_t dataSettingFromFinger_old[FINGERS_COUNT][DATA_SETTING_FROM_FINGER_SIZE] = {0};

  // остальные переменные
  ros::Subscriber toFingersSub;
  ros::Publisher  fromFingersPub;
  ros::Publisher  debugFromBigFingerPub;
  ros::Publisher  debugFromIndexFingerPub;
  ros::Publisher  debugFromMidFingerPub;
  ros::Publisher  debugFromRingFingerPub;
  ros::Publisher  debugFromPinkyPub;
  ros::Publisher  debugFromModulOtvPub;

  uint32_t recvd_count_topic        = 0;
  uint32_t resvdFailCount           = 0;
  uint32_t sendFailCount            = 0;
  uint32_t fail_cnt_f[7]            = {0};
  uint32_t rcvd_cnt_f[7]            = {0};
  uint8_t  dataFromTopicSize        = 0;
  uint8_t  dataToHandMount          = 0;
  uint8_t  resvdFromAllDev          = 0;
  uint8_t  fingers_OK[7]            = {1, 2, 4, 8, 16, 32, 64};                   // ок, если ответ пришел
  uint8_t  hand_mount_addr          = 0x31;                                       // устройство отсоединения схвата
  std::vector<uint8_t> fingersAddrs = {0x11, 0x12, 0x13, 0x14, 0x15, 0x16};       // 5 пальцев + модуль отведения
  bool getMsgFromTopic              = false;

  boost::chrono::system_clock::time_point first_tp_recv = boost::chrono::system_clock::now();
  boost::chrono::system_clock::time_point first_tp_send = boost::chrono::system_clock::now();

  // обработчик сообщений от ROS-топика(1) на Кисть
  // полученные данные с ROS-топика(1) "расфасовываются" в переменные для дальнейшей отправки каждому пальцу и устройству отсоединения схвата
  void topic_handle_receive(const std_msgs::ByteMultiArray::ConstPtr& recvdMsg) {
    if (getMsgFromTopic) return;
    getMsgFromTopic = true;
    recvd_count_topic++;

    memset(dataFromTopic,        0, sizeof(dataFromTopic));
    memset(dataToTopic,          0, sizeof(dataToTopic));
    memset(dataSettingFromTopic, 0, sizeof(dataSettingFromTopic));
    memset(dataSettingToTopic,   0, sizeof(dataSettingToTopic));
    resvdFromAllDev            = 0;
    dataFromTopicSize          = recvdMsg->data.size();

    boost::chrono::system_clock::time_point cur_tp_recv = boost::chrono::system_clock::now();
    boost::chrono::duration<double> ex_time_recv = cur_tp_recv - first_tp_recv;
    std::cout << "\033\n[1;32mex_time_recv time: \033\n[0m" << ex_time_recv.count() * 1000000 << "\n";
    first_tp_recv = boost::chrono::system_clock::now();
    if ((ex_time_recv.count() * 1000) > 100) {
      resvdFailCount++;
      printf("\n\n\n\033[1;31mRECV ERR\nRECV ERR\nRECV ERR\nRECV ERR\n\n\n\n\033[0m");
    }
    
    printf("\n\033[1;31msendFailCount  = %u\033[0m\n", sendFailCount);
    printf("\n\033[1;31mresvdFailCount = %u\033[0m\n", resvdFailCount);

    printf("\033\n[1;34mSIZE OF RECVD DATA FROM TOPIC toFingersTopic = %u \033[0m\n", dataFromTopicSize);
    std::cout << "recvd_count_topic = " << recvd_count_topic << std::endl;

    // при нормальном режиме полученные данные записываем в массив dataFromTopic
    if (dataFromTopicSize == DATA_FROM_TOPIC_SIZE){               
      for (int i = 0; i < dataFromTopicSize; i++){
        dataFromTopic[i] = recvdMsg->data[i];
      }

      for (int i = 0; i < sizeof(dataFromTopic); i++){
        if (i==6 || i==12 || i== 18 || i==24 || i==30 || i==36){
          printf("\033[1;31m|\033[0m");
        }
        printf("[%u]", dataFromTopic[i]);
      }

    // при калибровочном режиме полученные данные записываем в массив dataSettingFromTopic
    } else if (dataFromTopicSize == DATA_SETTING_FROM_TOPIC_SIZE){
      for (int i = 0; i < dataFromTopicSize; i++){
        dataSettingFromTopic[i] = recvdMsg->data[i];
      }

      for (int i = 0; i < sizeof(dataSettingFromTopic); i++){
        if (i==9 || i==18 || i== 27 || i==36 || i==45 || i==54){
          printf("\033[1;31m|\033[0m");
        }
        printf("\033[1;33m[%u]\033[0m", dataSettingFromTopic[i]);
      }
    }
  }

  // отправка данных устройству отсоединения схвата без ожидания ответа в нормальном режиме
  void sendToHandMount(){
    dataToHandMount = dataFromTopic[sizeof(dataFromTopic) - sizeof(uint8_t)];
    printf("\ndataToHandMount %u: [%u]\n", hand_mount_addr, dataToHandMount);
    uint8_t dataToHandMountSet[2] = {0x30, dataToHandMount};
    m_protocol.sendCmdWriteRS(hand_mount_addr, dataToHandMountSet, sizeof(dataToHandMountSet));
    std::this_thread::sleep_for(std::chrono::microseconds(1000));
  }

  // отправка данных устройству отсоединения схвата без ожидания ответа в калибровочном режиме
  void sendSettingToHandMount(){
    dataToHandMount = dataSettingFromTopic[sizeof(dataSettingFromTopic) - sizeof(uint8_t)];
    printf("\ndataToHandMount %u: [%u]\n", hand_mount_addr, dataToHandMount);
    uint8_t dataToHandMountSet[2] = {0x30, dataToHandMount};
    m_protocol.sendCmdWriteRS(hand_mount_addr, dataToHandMountSet, sizeof(dataToHandMountSet));
    std::this_thread::sleep_for(std::chrono::microseconds(1000));
  }

  // отправка данных каждому пальцу без ожидания ответа в нормальном режиме
  void sendToEachFinger(){
    for (int i = 0; i < fingersAddrs.size(); i++){
      memset(dataToFinger, 0, sizeof(dataToFinger));
      memcpy(dataToFinger, dataFromTopic + i * sizeof(dataToFinger), sizeof(dataToFinger));
      printf("dataToFinger %u: ", fingersAddrs[i]);
      for (int i = 0; i < sizeof(dataToFinger); i++){
        printf("[%u]", dataToFinger[i]);
      }
      printf("\n");
      m_protocol.sendCmdWriteRS(fingersAddrs[i], dataToFinger, sizeof(dataToFinger));
      if (fingersAddrs[i] == 0x16) std::this_thread::sleep_for(std::chrono::microseconds(1300));
      else std::this_thread::sleep_for(std::chrono::microseconds(1000));
    }
  }

  // отправка данных каждому пальцу без ожидания ответа в калибровочном режиме
  void sendSettingToEachFinger(){
    for (int i = 0; i < fingersAddrs.size(); i++){
      memset(dataSettingToFinger, 0, sizeof(dataSettingToFinger));
      memcpy(dataSettingToFinger, dataSettingFromTopic + i * sizeof(dataSettingToFinger), sizeof(dataSettingToFinger));
      printf("dataSettingToFinger %u: ", fingersAddrs[i]);
      for (int i = 0; i < sizeof(dataSettingToFinger); i++){
        printf("[%u]", dataSettingToFinger[i]);
      }
      printf("\n");
      if (dataSettingToFinger[0] < 0x60){
        printf("send cmdRead to %u", fingersAddrs[i]);
        m_protocol.sendCmdRead(fingersAddrs[i], dataSettingToFinger[0]);
      } else{
        m_protocol.sendCmdWriteRS(fingersAddrs[i], dataSettingToFinger, sizeof(dataSettingToFinger));
      }
      std::this_thread::sleep_for(std::chrono::microseconds(1300));
    }
  }

  // отправка команды на чтение каждому пальцу
  void sendReadToAllDev(){
    for (int i = 0; i < fingersAddrs.size(); i++){
      if (dataSettingToFinger[0] < 0x60) continue;
      m_protocol.sendCmdRead(fingersAddrs[i], 0x50);
      std::this_thread::sleep_for(std::chrono::microseconds(1300));
    }
  }

  // обработка ответов от каждого пальца в нормальном режиме
  void fingersResponsesProcessing(){
    for (int i = 0; i < fingersAddrs.size(); i++){
      if((resvdFromAllDev & fingers_OK[i]) != 0){                               //если палец на связи
        rcvd_cnt_f[fingersAddrs[i]- 0x11]++;
        printf("\033[1;31mFAIL CNT OF %u device = %u\033[0m\n", fingersAddrs[i], fail_cnt_f[fingersAddrs[i] - 0x11]);
        printf("\033[1;32mRCVD CNT OF %u device = %u\033[0m\n", fingersAddrs[i], rcvd_cnt_f[fingersAddrs[i] - 0x11]);
        memset(dataFromFinger_old[i], 0, DATA_FROM_FINGER_SIZE);
        memcpy(dataFromFinger_old[i], dataFromFinger_new[i], DATA_FROM_FINGER_SIZE);
      }else{                                                                    //если палец НЕ на связи
        fail_cnt_f[fingersAddrs[i] - 0x11]++;
        printf("\033[1;31mFAIL CNT OF %u device = %u\033[0m\n", fingersAddrs[i], fail_cnt_f[fingersAddrs[i] - 0x11]);
        printf("\033[1;32mRCVD CNT OF %u device = %u\033[0m\n", fingersAddrs[i], rcvd_cnt_f[fingersAddrs[i] - 0x11]);
        memset(dataFromFinger_new[i], 0, DATA_FROM_FINGER_SIZE);
        memcpy(dataFromFinger_new[i], dataFromFinger_old[i], DATA_FROM_FINGER_SIZE);
      }
      memcpy(dataToTopic + i * 10, &dataFromFinger_new[i][2], 10);              // 10b for 1 finger - cmd + data
    }
  }

  // обработка ответа от устройства отсоединения схвата в нормальном режиме
  void handMountResponsesProcessing(){
    if((resvdFromAllDev & fingers_OK[6]) != 0){                                 // если hand_mount на связи
      rcvd_cnt_f[6]++;
      printf("\033[1;31mFAIL CNT OF %u device (HAND MOUNT) = %u\033[0m\n", hand_mount_addr, fail_cnt_f[6]);
      printf("\033[1;32mRCVD CNT OF %u device (HAND MOUNT) = %u\033[0m\n", hand_mount_addr, rcvd_cnt_f[6]);
      memset(dataFromHandMount_old, 0, HMOUNT_DATA_SIZE);
      memcpy(dataFromHandMount_old, dataFromHandMount_new, HMOUNT_DATA_SIZE);

    } else {                                                                    // если hand_mount НЕ на связи
      fail_cnt_f[6]++;
      printf("\033[1;31mFAIL CNT OF %u device (HAND MOUNT) = %u\033[0m\n", hand_mount_addr, fail_cnt_f[6]);
      printf("\033[1;32mRCVD CNT OF %u device (HAND MOUNT) = %u\033[0m\n", hand_mount_addr, rcvd_cnt_f[6]);
      memset(dataFromHandMount_new, 0, HMOUNT_DATA_SIZE);
      memcpy(dataFromHandMount_new, dataFromHandMount_old, HMOUNT_DATA_SIZE);
    }
    dataToTopic[sizeof(dataToTopic) - 2 * sizeof(uint8_t)] = dataFromHandMount_new[3];
  }

  // обработка ответов от каждого пальца в калибровочном режиме
  void fingersSettingResponsesProcessing(){
    for (int i = 0; i < fingersAddrs.size(); i++){
      if((resvdFromAllDev & fingers_OK[i]) != 0){     // если палец на связи
        rcvd_cnt_f[fingersAddrs[i]- 0x11]++;
        printf("\033[1;31mFAIL CNT OF %u device = %u\033[0m\n", fingersAddrs[i], fail_cnt_f[fingersAddrs[i] - 0x11]);
        printf("\033[1;32mRCVD CNT OF %u device = %u\033[0m\n", fingersAddrs[i], rcvd_cnt_f[fingersAddrs[i] - 0x11]);
        memset(dataSettingFromFinger_old[i], 0, DATA_SETTING_FROM_FINGER_SIZE);
        memcpy(dataSettingFromFinger_old[i], dataSettingFromFinger_new[i], DATA_SETTING_FROM_FINGER_SIZE);
      }else{                                          // если палец НЕ на связи
        fail_cnt_f[fingersAddrs[i] - 0x11]++;
        printf("\033[1;31mFAIL CNT OF %u device = %u\033[0m\n", fingersAddrs[i], fail_cnt_f[fingersAddrs[i] - 0x11]);
        printf("\033[1;32mRCVD CNT OF %u device = %u\033[0m\n", fingersAddrs[i], rcvd_cnt_f[fingersAddrs[i] - 0x11]);
        memset(dataSettingFromFinger_new[i], 0, DATA_SETTING_FROM_FINGER_SIZE);
        memcpy(dataSettingFromFinger_new[i], dataSettingFromFinger_old[i], DATA_SETTING_FROM_FINGER_SIZE);
      }
      memcpy(dataSettingToTopic + i * 9, &dataSettingFromFinger_new[i][2], 9);  //  9b for 1 finger - cmd + data
    }
  }

  // обработка ответа от устройства отсоединения схвата в калибровочном режиме
  void handMountSettingResponsesProcessing(){
    if((resvdFromAllDev & fingers_OK[6]) != 0){                                 // если hand_mount на связи
      rcvd_cnt_f[6]++;
      printf("\033[1;31mFAIL CNT OF %u device (HAND MOUNT) = %u\033[0m\n", hand_mount_addr, fail_cnt_f[6]);
      printf("\033[1;32mRCVD CNT OF %u device (HAND MOUNT) = %u\033[0m\n", hand_mount_addr, rcvd_cnt_f[6]);
      memset(dataFromHandMount_old, 0, HMOUNT_DATA_SIZE);
      memcpy(dataFromHandMount_old, dataFromHandMount_new, HMOUNT_DATA_SIZE);

    } else {                                                                    // если hand_mount НЕ на связи
      fail_cnt_f[6]++;
      printf("\033[1;31mFAIL CNT OF %u device (HAND MOUNT) = %u\033[0m\n", hand_mount_addr, fail_cnt_f[6]);
      printf("\033[1;32mRCVD CNT OF %u device (HAND MOUNT) = %u\033[0m\n", hand_mount_addr, rcvd_cnt_f[6]);
      memset(dataFromHandMount_new, 0, HMOUNT_DATA_SIZE);
      memcpy(dataFromHandMount_new, dataFromHandMount_old, HMOUNT_DATA_SIZE);
    }
    dataSettingToTopic[sizeof(dataSettingToTopic) - 2 * sizeof(uint8_t)] = dataFromHandMount_new[3];
  }

  // ожидание и обработка ответов от каждого пальца и устройства отсоединения схвата в нормальном режиме
  void getResponses(){
    memset(dataFromFinger_new,           0, FINGERS_COUNT * DATA_FROM_FINGER_SIZE);
    memset(dataFromHandMount_new,        0, HMOUNT_DATA_SIZE);
    memset(dataSettingFromFinger_new,    0, FINGERS_COUNT * DATA_SETTING_FROM_FINGER_SIZE);
    m_protocol.RSRead(dataFromFinger_new, dataFromHandMount_new, dataSettingFromFinger_new, &resvdFromAllDev);
    fingersResponsesProcessing();
    handMountResponsesProcessing();
    dataToTopic[sizeof(dataToTopic) - sizeof(uint8_t)] = resvdFromAllDev;
    printf("resvdFromAllDevBeforeSend = %u\n", resvdFromAllDev);
  }

  // ожидание и обработка ответов от каждого пальца и устройства отсоединения схвата в калибровочном режиме
  void getSettingResponses(){
    memset(dataFromFinger_new,           0, FINGERS_COUNT * DATA_FROM_FINGER_SIZE);
    memset(dataFromHandMount_new,        0, HMOUNT_DATA_SIZE);
    memset(dataSettingFromFinger_new,    0, FINGERS_COUNT * DATA_SETTING_FROM_FINGER_SIZE);
    sendReadToAllDev();
    m_protocol.RSRead(dataFromFinger_new, dataFromHandMount_new, dataSettingFromFinger_new, &resvdFromAllDev);
    fingersSettingResponsesProcessing();
    handMountSettingResponsesProcessing();
    dataSettingToTopic[sizeof(dataSettingToTopic) - sizeof(uint8_t)] = resvdFromAllDev;
    printf("resvdFromAllDevBeforeSend = %u\n", resvdFromAllDev);
  }

  // перенаправление сообщений от Кисти в отдельный ROS-топик(3) в удобочитаемом виде
  void sendMsgToDebugTopic(const fingers::From_Finger msgsArrToDebugFingers[]){
    if (debugAllFingers) {
      debugFromBigFingerPub.  publish(msgsArrToDebugFingers[0]);
      debugFromIndexFingerPub.publish(msgsArrToDebugFingers[1]);
      debugFromMidFingerPub.  publish(msgsArrToDebugFingers[2]);
      debugFromRingFingerPub. publish(msgsArrToDebugFingers[3]);
      debugFromPinkyPub.      publish(msgsArrToDebugFingers[4]);
      debugFromModulOtvPub.   publish(msgsArrToDebugFingers[5]);
      return;
    }
    if (debugBigFinger    == 1)   debugFromBigFingerPub.publish(msgsArrToDebugFingers[0]);
    if (debugIndexFinger  == 1) debugFromIndexFingerPub.publish(msgsArrToDebugFingers[1]);
    if (debugMidFinger    == 1)   debugFromMidFingerPub.publish(msgsArrToDebugFingers[2]);
    if (debugRingFinger   == 1)  debugFromRingFingerPub.publish(msgsArrToDebugFingers[3]);
    if (debugPinky        == 1)       debugFromPinkyPub.publish(msgsArrToDebugFingers[4]);
    if (debugModulOtv     == 1)    debugFromModulOtvPub.publish(msgsArrToDebugFingers[5]);
  }

  // получение удобочитаемых сообщений от Кисти для отправки в отдельный ROS-топик(3)
  void setMsgsToDebugTopic(fingers::From_Finger msgsArrToDebugFingers[], uint8_t dataToTopic[]){
    for (uint32_t i = 0; i < 6; i++){
      msgsArrToDebugFingers[i].current  = (dataToTopic[i * 9 + 1] << 8) + dataToTopic[i * 9];
      msgsArrToDebugFingers[i].pressure = (dataToTopic[i * 9 + 3] << 8) + dataToTopic[i * 9 + 2];
      msgsArrToDebugFingers[i].angle    = (dataToTopic[i * 9 + 5] << 8) + dataToTopic[i * 9 + 4];
      msgsArrToDebugFingers[i].count    = (dataToTopic[i * 9 + 7] << 8) + dataToTopic[i * 9 + 6];
      msgsArrToDebugFingers[i].mask     =  dataToTopic[i * 9 + 8];
    }
  }

  // отправка данных в ROS-топик(2) и ROS-топик(3) (при включенной отладке) от Кисти и устройства отсоединения схвата в нормальном режиме
  void sendMsgToTopic(){
    std_msgs::ByteMultiArray sendMsgFromFingersTopic;
    sendMsgFromFingersTopic.layout.dim.push_back(std_msgs::MultiArrayDimension());
    sendMsgFromFingersTopic.layout.dim[0].size = 1;
    std::cout << "\n\033[1;34mSEND MSG TO TOPIC fromFingersTopic: \n\033[0m";
    sendMsgFromFingersTopic.layout.dim[0].stride = sizeof(dataToTopic);
    sendMsgFromFingersTopic.data.clear();
    for (int i = 0; i < sizeof(dataToTopic); i++){
      sendMsgFromFingersTopic.data.push_back(dataToTopic[i]);
      if (i==10 || i==20 || i==30 || i==40 || i==50 || i==60 || i==61){
        printf("\033[1;31m|\033[0m");
      }
      printf("[%u]", dataToTopic[i]);
    }
    printf("\n");
    fromFingersPub.publish(sendMsgFromFingersTopic);
    fingers::From_Finger msgsArrToDebugFingers[6];
    setMsgsToDebugTopic(msgsArrToDebugFingers, dataToTopic);
    sendMsgToDebugTopic(msgsArrToDebugFingers);

    boost::chrono::system_clock::time_point cur_tp_send = boost::chrono::system_clock::now();
    boost::chrono::duration<double> ex_time_send = cur_tp_send - first_tp_send;
    std::cout << "\033\n[1;32mex_time_send time: \033\n[0m" << ex_time_send.count() * 1000000 << "\n";
    first_tp_send = boost::chrono::system_clock::now();
    if ((ex_time_send.count() * 1000) > 100) {
      sendFailCount++;
      printf("\n\n\n\033[1;31mSEND ERR\nSEND ERR\nSEND ERR\nSEND ERR\n\n\n\n\033[0m");
    }

  }

  // отправка данных в ROS-топик(2), полученных от Кисти и устройства отсоединения схвата в калибровочном режиме
  void sendSettingMsgToTopic(){
    std_msgs::ByteMultiArray sendMsgFromFingersTopic;
    sendMsgFromFingersTopic.layout.dim.push_back(std_msgs::MultiArrayDimension());
    sendMsgFromFingersTopic.layout.dim[0].size = 1;
    std::cout << "\n\033[1;34mSEND MSG TO TOPIC fromFingersTopic: \n\033[0m";
    sendMsgFromFingersTopic.layout.dim[0].stride = sizeof(dataSettingToTopic);
    sendMsgFromFingersTopic.data.clear();
    for (int i = 0; i < sizeof(dataSettingToTopic); i++){
      sendMsgFromFingersTopic.data.push_back(dataSettingToTopic[i]);
      if (i==9 || i==18 || i== 27 || i==36 || i==45 || i==54 || i==55){
        printf("\033[1;31m|\033[0m");
      }
      printf("[%u]", dataSettingToTopic[i]);
    }
    printf("\n");
    fromFingersPub.publish(sendMsgFromFingersTopic);
  }
  
};

int main(int argc, char** argv)
{
  QCoreApplication coreApplication(argc, argv);

  std::string devPort  = "USB0";
  std::string baudrate = "256000";

  ros::param::get("/_devPortForFingers",    devPort);
  ros::param::get("/_baudrateForFingers",   baudrate);
  try{

    std::cout << "\n\033[1;32m╔═══════════════════════════════════════╗\033[0m"
              << "\n\033[1;32m║       MTOPIC_RECIEVER is running!     ║\033[0m" 
              << "\n\033[1;32m║Baud rate: " << baudrate << ", Port: /dev/tty" << devPort << "\t║\033[0m"
              << "\n\033[1;32m╚═══════════════════════════════════════╝\033[0m\n";
    ros::init(argc, argv, "master_topic_receiver");
    
    qt_serial::Qt_Serial_Async qt_RS_transp("/dev/tty" + devPort, baudrate);
    protocol_master::ProtocolMaster rs_prot_master(qt_RS_transp, &coreApplication);
    RS_Server raspbPi(rs_prot_master);

    ros::param::get("/_debugBigFinger",   debugBigFinger);
    ros::param::get("/_debugIndexFinger", debugIndexFinger);
    ros::param::get("/_debugMidFinger",   debugMidFinger);
    ros::param::get("/_debugRingFinger",  debugRingFinger);
    ros::param::get("/_debugPinky",       debugPinky);
    ros::param::get("/_debugModulOtv",    debugModulOtv);
    ros::param::get("/_debugBatCam",      debugBatCam);
    ros::param::get("/_debugAllFingers",  debugAllFingers);

    std::cout << "debugBigFinger "    <<  debugBigFinger   << std::endl;
    std::cout << "debugIndexFinger "  <<  debugIndexFinger << std::endl;
    std::cout << "debugMidFinger "    <<  debugMidFinger   << std::endl;
    std::cout << "debugRingFinger "   <<  debugRingFinger  << std::endl;
    std::cout << "debugPinky "        <<  debugPinky       << std::endl;
    std::cout << "debugModulOtv "     <<  debugModulOtv    << std::endl;

    while(ros::ok()){
      raspbPi.nodeFromTopicProcess();
      ros::spinOnce();
    }
  } catch (std::exception e){
    std::cerr << "\nExeption: " << e.what() << std::endl;
  }
  return 0;
};//z
