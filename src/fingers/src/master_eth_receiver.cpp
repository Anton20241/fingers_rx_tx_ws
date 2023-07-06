/*
  Задачи данной программы:
  1) Прием данных по протоколу UDP от пользователя
  2) Отправка  полученных данных в ROS-топики  на Кисть и Плату Управления (АКБ, камеры)
  3) Получение актуальных данных с ROS-топиков от Кисти и Платы Управления (АКБ, камеры)
  4) Отправка  полученных данных от Кисти и Платы Управления (АКБ, камеры) по протоколу UDP

  UDP -> тек. программа -> ROS-топики на Кисть и Плату Управления (АКБ, камеры) -> тек. программа -> UDP
*/

#include <iostream>
#include <ros/ros.h>
#include <unistd.h>
#include <boost/asio.hpp>
#include <std_msgs/ByteMultiArray.h>
#include <fingers/To_Finger.h>
#include <fingers/To_Bat_Cam.h>
#include <std_msgs/UInt32.h>
#include "umba_crc_table.h"
#include <mutex>

using boost::asio::ip::udp;
using boost::asio::ip::address;

#define PORT                                   20002

// размеры пакетов данных в нормальном режиме
#define DATA_FROM_UDP_SIZE                     49 // [AA][BB][LEN][6f*6b][HM][HPos][CAM][UR][REL][4b KeepAL][CRC]
#define DATA_TO_UDP_SIZE                       76 // [BB][AA][LEN][6f*10b][HM][HPos][CAM][24v][48v][Ok][UR][RL][4b KeepAL][CRC]
#define DATA_TO_FINGERS_TOPIC_SIZE             37 // [6f*6b][HM]
#define DATA_FROM_FINGERS_TOPIC_SIZE           62 // [6f*10b][HM][OK]

// размеры пакетов данных в калибровочном режиме
#define DATA_FROM_UDP_SIZE_SETTINGS            67 // [AA][BB][LEN][6f*9b][HM][HPos][CAM][UR][REL][4b KeepAL][CRC]
#define DATA_TO_UDP_SIZE_SETTINGS              70 // [BB][AA][LEN][6f*9b][HM][HPos][CAM][24v][48v][Ok][UR][RL][4b KeepAL][CRC]
#define DATA_TO_FINGERS_TOPIC_SIZE_SETTINGS    55 // [6f*9b][HM]
#define DATA_FROM_FINGERS_TOPIC_SIZE_SETTINGS  56 // [6f*9b][HM][OK]

// имена ROS-топиков для обмена данными между программами
#define TO_FINGERS_TOPIC_NAME       "toFingersTopic"
#define TO_BAT_CAM_TOPIC_NAME       "toBatCamTopic"
#define FROM_BAT_CAM_TOPIC_NAME     "fromBatCamTopic"
#define FROM_FINGERS_TOPIC_NAME     "fromFingersTopic"
#define TO_KEEP_ALIVE_TOPIC_NAME    "toKeepAliveTopic"

// 0 - debug off, 1 - debug on : возможность отслеживать данные на Кисть в удобочитаемом виде
int debugBigFinger   = 0;
int debugIndexFinger = 0;
int debugMidFinger   = 0;
int debugRingFinger  = 0;
int debugPinky       = 0;
int debugModulOtv    = 0;
int debugBatCam      = 0;
int debugAllFingers  = 0;

/* 
  Класс UDPServer предназначен для реализации функционала обмена данными между UDP, Кистью и Платой Управления.
  Обмен происходит с использованием сокета для приема/отправки сообщений по протоколу UDP 
  и ROS-топиков для общения с Кистью и Платой Управления.

  Класс UDPServer включает следующие методы:
    * read_msg_udp()        - вызов асинхронной функции для считывания данных с UDP
    * nodeFromUDPProcess()  - основной цикл программы, который состоит из последовательности действий: 
        1) отправка в ROS-топик данных на Кисть                             - метод sendMsgToFingers()
        2) отправка в ROS-топик данных на Плату Управления (АКБ, камеры)    - метод sendMsgToBatCam()
        3) отправка по протоколу UDP данных пользователю                    - метод sendMsgToUDP()
    * from_finger_handle_receive()  - обработчик сообщений от ROS-топика с Кисти
    * from_cam_bat_handle_receive() - обработчик сообщений от ROS-топика с Платы Управления (АКБ, камеры)
    * startShutDownProcess()        - метод для выключения устройства
    * udp_handle_receive()          - обработчик сообщений от сети по протоколу UDP
    * sendMsgToDebugTopic()         - перенаправление сообщений на пальцы в отдельный топик в удобочитаемом виде
    * setMsgsToDebugTopic()         - получение удобочитаемых сообщений на пальцы для отправки в отдельный топик
    * setAnglesOLD()                - установка старых значений углов в данные для отправки в ROS-топик на Кисть
    * sendMsgToUDP()                - отправка  полученных данных с ROS-топиков с Кисти и Платы Управления (АКБ, камеры) по протоколу UDP
    * getLen()                      - возвращает длину пакета
    * getCrc8()                     - возвращает контрольную сумму пакета
    * parserUDP()                   - проверка принятого пакета по протоколу UDP на валидность
*/

class UDPServer{
public:

  // конструктор класса UDPServer, в котором происходит подключение к сети по протоколу UDP, и подключение к ROS-топикам 
	UDPServer(boost::asio::io_service& io_service): socket_(io_service, udp::endpoint(udp::v4(), PORT)){
    toFingersPub    = node.advertise<std_msgs::ByteMultiArray>(TO_FINGERS_TOPIC_NAME,   0);
    toBatCamPub     = node.advertise<std_msgs::ByteMultiArray>(TO_BAT_CAM_TOPIC_NAME,   0);
    toKeepAlivePub  = node.advertise<std_msgs::UInt32>(TO_KEEP_ALIVE_TOPIC_NAME,        0);
    fromFingersSub  = node.subscribe<std_msgs::ByteMultiArray>(FROM_FINGERS_TOPIC_NAME, 0,  
        &UDPServer::from_finger_handle_receive, this);
    fromCamBatSub   = node.subscribe<std_msgs::ByteMultiArray>(FROM_BAT_CAM_TOPIC_NAME, 0,  
        &UDPServer::from_cam_bat_handle_receive, this);

    debugToBigFingerPub   = node.advertise<fingers::To_Finger> ("debugToBigFingerTopic",   0);
    debugToIndexFingerPub = node.advertise<fingers::To_Finger> ("debugToIndexFingerTopic", 0);
    debugToMidFingerPub   = node.advertise<fingers::To_Finger> ("debugToMidFingerTopic",   0);
    debugToRingFingerPub  = node.advertise<fingers::To_Finger> ("debugToRingFingerTopic",  0);
    debugToPinkyPub       = node.advertise<fingers::To_Finger> ("debugToPinkyTopic",       0);
    debugToModulOtvPub    = node.advertise<fingers::To_Finger> ("debugToModulOtvTopic",    0);
    debugToBatCamPub      = node.advertise<fingers::To_Bat_Cam>("debugToBatCamTopic",      0);

    boost::bind(&UDPServer::udp_handle_receive, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred);
    read_msg_udp();
  }
  
  // основной цикл программы
  void nodeFromUDPProcess(){
    if (getMsgFromUDP){
      getMsgFromUDP = false;
      sendMsgToFingers();
      sendMsgToBatCam();
      sendMsgToUDP();
    }
  }

private:
  ros::NodeHandle node;
	udp::socket     socket_;
	udp::endpoint   sender_endpoint_;

  // данные от сети по протоколу UDP
	uint8_t dataFromUDP[DATA_FROM_UDP_SIZE_SETTINGS]                                 = {0};

  // данные при нормальном режиме работы
	uint8_t dataToUDP[DATA_TO_UDP_SIZE]                                              = {0};
  uint8_t dataToFingersTopic[DATA_TO_FINGERS_TOPIC_SIZE]                           = {0};
  uint8_t dataToFingersTopic_OLD[DATA_TO_FINGERS_TOPIC_SIZE]                       = {0};
  uint8_t dataFromFingersTopic[DATA_FROM_FINGERS_TOPIC_SIZE]                       = {0};
  uint8_t dataFromFingersTopic_OLD[DATA_FROM_FINGERS_TOPIC_SIZE]                   = {0};
  
  // данные при калибровочном режиме работы
  uint8_t dataToUDP_setting[DATA_TO_UDP_SIZE_SETTINGS]                             = {0};
  uint8_t dataToFingersTopic_setting[DATA_TO_FINGERS_TOPIC_SIZE_SETTINGS]          = {0};
  uint8_t dataToFingersTopic_OLD_setting[DATA_TO_FINGERS_TOPIC_SIZE_SETTINGS]      = {0};
  uint8_t dataFromFingersTopic_setting[DATA_FROM_FINGERS_TOPIC_SIZE_SETTINGS]      = {0};
  uint8_t dataFromFingersTopic_OLD_setting[DATA_FROM_FINGERS_TOPIC_SIZE_SETTINGS]  = {0};
  
  // остальные переменные
  ros::Publisher  toFingersPub;
  ros::Publisher  debugToBigFingerPub;
  ros::Publisher  debugToIndexFingerPub;
  ros::Publisher  debugToMidFingerPub;
  ros::Publisher  debugToRingFingerPub;
  ros::Publisher  debugToPinkyPub;
  ros::Publisher  debugToModulOtvPub;
  ros::Publisher  debugToBatCamPub;
  ros::Publisher  toBatCamPub;
  ros::Publisher  toKeepAlivePub;
  ros::Subscriber fromFingersSub;
  ros::Subscriber fromCamBatSub;

  uint32_t send_count_udp             = 0;
  uint32_t recvd_count_udp            = 0;
  uint32_t send_count_topic_fingers   = 0;
  uint32_t recvd_count_topic_fingers  = 0;
  uint32_t send_count_topic_camera    = 0;
  uint32_t recvd_count_topic_cam_bat  = 0;
  uint32_t resvdFailCount             = 0;
  uint32_t sendFailCount              = 0;
  uint8_t  resvdBytesFromFingers      = 0;
  uint8_t  boardOk                    = 0;
  uint8_t  fingersOk                  = 0;
  uint8_t  allDevOk                   = 0;

  bool dataGetFromBatCam              = false;
  bool settings                       = false;
  bool getMsgFromUDP                  = false;
  bool holdPositionProcessStart       = true;

  struct currentState_{
    uint8_t hand_mount                = 0;
    uint8_t hold_position             = 0;
    uint8_t camera_from_udp           = 0;
    uint8_t camera_from_bat_cam       = 0;
    uint8_t ur5_from_udp              = 0;
    uint8_t ur5_from_bat_cam          = 0;
    uint8_t relay_from_udp            = 0;
    uint8_t relay_from_bat_cam        = 0;
    uint8_t bat_24V                   = 0;
    uint8_t bat_48V                   = 0;
    uint8_t keepalive[4]              = {0};
    uint8_t cmdBatCamTopic            = 0;
    uint8_t time_down                 = 0;
  };
  
  currentState_ currentState;

  boost::chrono::system_clock::time_point first_tp_recv = boost::chrono::system_clock::now();
  boost::chrono::system_clock::time_point first_tp_send = boost::chrono::system_clock::now();

  // вызов асинхронной функции "socket_.async_receive_from" для считывания данных с UDP
	void read_msg_udp(){
    socket_.async_receive_from(boost::asio::buffer(dataFromUDP, sizeof(dataFromUDP)), sender_endpoint_,
        boost::bind(&UDPServer::udp_handle_receive, this, boost::asio::placeholders::error, 
        boost::asio::placeholders::bytes_transferred));
	}

  // обработчик сообщений от ROS-топика с Кисти. 
  // полученные данные с ROS-топика "расфасовываются" в переменные для дальнейшей отправки по UDP
  void from_finger_handle_receive(const std_msgs::ByteMultiArray::ConstPtr& recvdMsg){

    recvd_count_topic_fingers++;
    resvdBytesFromFingers = recvdMsg->data.size();

    std::cout << "\n\033[1;34mRECVD FROM TOPIC fromFingersTopic recvdMsg->data.size() = \033[0m" << recvdMsg->data.size() << std::endl;
    std::cout << "recvd_count_topic_fingers = " << recvd_count_topic_fingers << std::endl;

    if (recvdMsg->data.size() == DATA_FROM_FINGERS_TOPIC_SIZE){
      for (int i = 0; i < recvdMsg->data.size(); i++){
        dataFromFingersTopic[i] = recvdMsg->data[i];
        printf("[%u]", dataFromFingersTopic[i]);
        if (i==9 || i==19 || i==29 || i==39 || i==49 || i==59 || i==60){
          printf("\033[1;31m|\033[0m");
        }
      }
      std::cout << std::endl;
      currentState.hand_mount = dataFromFingersTopic[sizeof(dataFromFingersTopic) - 2 * sizeof(uint8_t)];
      fingersOk               = dataFromFingersTopic[sizeof(dataFromFingersTopic) - 1 * sizeof(uint8_t)];

    } else if (recvdMsg->data.size() == DATA_FROM_FINGERS_TOPIC_SIZE_SETTINGS){
      for (int i = 0; i < recvdMsg->data.size(); i++){
        dataFromFingersTopic_setting[i] = recvdMsg->data[i];
        printf("[%u]", dataFromFingersTopic_setting[i]);
        if (i==8 || i==17 || i==26 || i==35 || i==44 || i==53 || i==54){
          printf("\033[1;31m|\033[0m");
        }
      }
      std::cout << std::endl;
      currentState.hand_mount = dataFromFingersTopic_setting[sizeof(dataFromFingersTopic_setting) - 2 * sizeof(uint8_t)];
      fingersOk               = dataFromFingersTopic_setting[sizeof(dataFromFingersTopic_setting) - 1 * sizeof(uint8_t)];
      
    }

  }

  // обработчик сообщений от ROS-топика с Платы Управления (АКБ, камеры). 
  // полученные данные с ROS-топика "расфасовываются" в переменные для дальнейшей отправки по UDP
  void from_cam_bat_handle_receive(const std_msgs::ByteMultiArray::ConstPtr& recvdMsg) {
    recvd_count_topic_cam_bat++;
    std::cout << "\033[1;34mRECVD FROM TOPIC bat_cam_topic recvdMsg->data.size() = \033[0m" << recvdMsg->data.size() << std::endl;
    std::cout << "recvd_count_topic_cam_bat = " << recvd_count_topic_cam_bat << std::endl;
    for (int i = 0; i < recvdMsg->data.size(); i++){
      printf("[%u]", (uint8_t)recvdMsg->data[i]);
    }
    std::cout << std::endl;

    if(recvdMsg->data.size() == 6){
      //обновляем данные
      currentState.bat_24V                    = recvdMsg->data[0];
      currentState.bat_48V                    = recvdMsg->data[1];
      currentState.camera_from_bat_cam        = recvdMsg->data[2];
      currentState.relay_from_bat_cam         = recvdMsg->data[3];
      currentState.ur5_from_bat_cam           = recvdMsg->data[4];
      boardOk                                 = recvdMsg->data[5];
      recvd_count_topic_cam_bat++;
    } else if (recvdMsg->data.size() == 3){
      currentState.cmdBatCamTopic             = recvdMsg->data[0];
      currentState.time_down                  = recvdMsg->data[1];
      boardOk                                 = recvdMsg->data[2];
      recvd_count_topic_cam_bat++;
      startShutDownProcess();
    } else if (recvdMsg->data.size() == 1){
      boardOk                                 = recvdMsg->data[0];
    } else return;

  }

  // метод для выключения устройства
  void startShutDownProcess(){
    std::cout << "\n\033[1;31m╔═══════════════╗\033[0m";
    std::cout << "\n\033[1;31m║   SHUTDOWN    ║\033[0m";
    std::cout << "\n\033[1;31m╚═══════════════╝\033[0m\n";
    system("shutdown -h now");
  }

  /*
    обработчик сообщений от сети по протоколу UDP.
    полученные данные от сети по протоколу UDP "расфасовываются" в переменные 
    для дальнейшей отправки ROS-топики на Кисть и Плату Управления (АКБ, камеры)
  */
  void udp_handle_receive(const boost::system::error_code& error, uint32_t bytes_transferred) {

    if(!parserUDP(dataFromUDP)){
      std::cout << "\033[1;31mUDP data not valid\033[0m\n";
      memset(dataFromUDP, 0, sizeof(dataFromUDP));
      read_msg_udp();
      return;
    }

    boost::chrono::system_clock::time_point cur_tp_recv = boost::chrono::system_clock::now();
    boost::chrono::duration<double> ex_time_recv = cur_tp_recv - first_tp_recv;
    std::cout << "\033\n[1;32mex_time_recv time: \033\n[0m" << ex_time_recv.count() * 1000000 << "\n";
    first_tp_recv = boost::chrono::system_clock::now();
    if ((ex_time_recv.count() * 1000) > 100) {
      resvdFailCount++;
      printf("\n\n\n\033[1;31mRECV ERR\nRECV ERR\nRECV ERR\nRECV ERR\n\n\n\n\033[0m");
    }

    recvd_count_udp++;
    
    printf("\n\033[1;31msendFailCount  = %u\033[0m\n", sendFailCount);
    printf("\n\033[1;31mresvdFailCount = %u\033[0m\n", resvdFailCount);

    std::cout << "\n\033[1;36mRECVD FROM UDP bytes_transferred = \033[0m" << bytes_transferred << std::endl;
    std::cout << "recvd_count_udp = " << recvd_count_udp << std::endl;

    if (bytes_transferred == DATA_FROM_UDP_SIZE_SETTINGS) { // Get new settings
      std::cout << "\n\033[1;93mUDP PACKAGE WITH NEW SETTINGS  \033[0m" << std::endl;
      for (int i = 0; i < bytes_transferred; i++){
        printf("[%u]", dataFromUDP[i]);
        if (i==2 || i==11 || i==20 || i==29 || i==38 || i==47 || i==56 || i==57 || i==58 || i==59 || i==60 || i== 61 || i==65 ){
          printf("\033[1;31m|\033[0m");
        }
      }
      printf("\n");
    } else if (bytes_transferred == DATA_FROM_UDP_SIZE) { //Get regular data
      for (int i = 0; i < bytes_transferred; i++) { 
        printf("[%u]", dataFromUDP[i]);
        if (i==2 || i==8 || i==14 || i==20 || i==26 || i==32 || i==38 || i==39 || i==40 || i==41 || i==42 || i== 43 || i==47 ){
          printf("\033[1;31m|\033[0m");
        }
      }
      printf("\n");
    } else {
      memset(dataFromUDP, 0, sizeof(dataFromUDP));
      read_msg_udp();
      return;
    }
        
    getMsgFromUDP = true;
    
    currentState.hold_position        = dataFromUDP[dataFromUDP[2] - 9 * sizeof(uint8_t)];
    currentState.camera_from_udp      = dataFromUDP[dataFromUDP[2] - 8 * sizeof(uint8_t)];
    currentState.ur5_from_udp         = dataFromUDP[dataFromUDP[2] - 7 * sizeof(uint8_t)];
    currentState.relay_from_udp       = dataFromUDP[dataFromUDP[2] - 6 * sizeof(uint8_t)];
    memcpy(currentState.keepalive,    dataFromUDP + dataFromUDP[2] - 5 * sizeof(uint8_t), sizeof(currentState.keepalive));
    
    if (settings) {
      memcpy(dataToFingersTopic_setting, dataFromUDP + 3, sizeof(dataToFingersTopic_setting));  //fingers + hand_mount
    } else {
      memcpy(dataToFingersTopic, dataFromUDP + 3, sizeof(dataToFingersTopic));                  //fingers + hand_mount
      printf("\nCLEAR\n");
      for (int i = 0; i < sizeof(dataToFingersTopic); i++){
        printf("[%u]", dataToFingersTopic[i]);
        if (i==5 || i==11 || i==17 || i==23 || i==29 || i==35){
          printf("\033[1;31m|\033[0m");
        }
      }
      std::cout << std::endl;
      
    }

    //отправка пакета в топик "toKeepAliveTopic"
    std_msgs::UInt32 sendMsgToKeepAliveTopic;
    uint32_t keepAlive32 = (currentState.keepalive[3] << 24) + (currentState.keepalive[2] << 16) + (currentState.keepalive[1] << 8) + currentState.keepalive[0];
    sendMsgToKeepAliveTopic.data = keepAlive32;
    toKeepAlivePub.publish(sendMsgToKeepAliveTopic);

    read_msg_udp();
  }

  // переонаправление сообщений на пальцы в отдельный топик в удобочитаемом виде
  void sendMsgToDebugTopic(const fingers::To_Finger msgsArrToDebugFingers[]){
    if (debugAllFingers) {
      debugToBigFingerPub.  publish(msgsArrToDebugFingers[0]);
      debugToIndexFingerPub.publish(msgsArrToDebugFingers[1]);
      debugToMidFingerPub.  publish(msgsArrToDebugFingers[2]);
      debugToRingFingerPub. publish(msgsArrToDebugFingers[3]);
      debugToPinkyPub.      publish(msgsArrToDebugFingers[4]);
      debugToModulOtvPub.   publish(msgsArrToDebugFingers[5]);
      return;
    }

    if (debugBigFinger)     debugToBigFingerPub.publish(msgsArrToDebugFingers[0]);
    if (debugIndexFinger) debugToIndexFingerPub.publish(msgsArrToDebugFingers[1]);
    if (debugMidFinger)     debugToMidFingerPub.publish(msgsArrToDebugFingers[2]);
    if (debugRingFinger)   debugToRingFingerPub.publish(msgsArrToDebugFingers[3]);
    if (debugPinky)             debugToPinkyPub.publish(msgsArrToDebugFingers[4]);
    if (debugModulOtv)       debugToModulOtvPub.publish(msgsArrToDebugFingers[5]);
  }

  // получение удобочитаемых сообщений на Кисть для отправки в отдельный ROS-топик
  void setMsgsToDebugTopic(fingers::To_Finger msgsArrToDebugFingers[], uint8_t dataToFingersTopic[]){
    for (uint32_t i = 0; i < 6; i++){
      msgsArrToDebugFingers[i].angle    = (dataToFingersTopic[i * 5 + 1] << 8) + dataToFingersTopic[i * 5];
      msgsArrToDebugFingers[i].reserve  = (dataToFingersTopic[i * 5 + 3] << 8) + dataToFingersTopic[i * 5 + 2];
      msgsArrToDebugFingers[i].mask     =  dataToFingersTopic[i * 5 + 4];
    }
  }

  // установка старых значений углов в данные для отправки в ROS-топик на Кисть
  void setAnglesOLD(uint8_t* dataToFingersTopic_, uint8_t* dataFromFingersTopic_){
    for (int i = 0; i < 6; i++){
      memcpy(&dataToFingersTopic_[i * 6 + 1], &dataFromFingersTopic_[i * 10 + 5], 2);
    }
  }
  
  // отправка в ROS-топик данных на Кисть
  void sendMsgToFingers(){
    
    // если Пользователь отправляет hold_position, мы запоминаем старые значения углов, и их же отправляем на Кисть
    if (currentState.hold_position == 1 && !settings){
      printf("\n\033[1;33mcurrentState.hold_position = %u\033[0m\n", currentState.hold_position);

      if (holdPositionProcessStart){
        holdPositionProcessStart = false;
        memcpy(dataFromFingersTopic_OLD, dataFromFingersTopic, sizeof(dataFromFingersTopic_OLD));
      }
      
      setAnglesOLD(dataToFingersTopic, dataFromFingersTopic_OLD);
    } else {
      holdPositionProcessStart = true;
    }
    
    //отправка пакета в топик "toFingersTopic"
    std_msgs::ByteMultiArray sendMsgToFingersTopic;
    sendMsgToFingersTopic.layout.dim.push_back(std_msgs::MultiArrayDimension());
    sendMsgToFingersTopic.layout.dim[0].size = 1;
    
    if (settings){
      sendMsgToFingersTopic.layout.dim[0].stride = sizeof(dataToFingersTopic_setting);
    } else {
      sendMsgToFingersTopic.layout.dim[0].stride = sizeof(dataToFingersTopic);
    }

    sendMsgToFingersTopic.data.clear();
    std::cout << "\033[1;34mSEND T0 toFingersTopic: \033[0m";

    if (!settings) {  // сообщение - команды для пальцев
      for (int i = 0; i < sizeof(dataToFingersTopic); i++){
        printf("[%u]", dataToFingersTopic[i]);
        if (i==5 || i==11 || i==17 || i==23 || i==29 || i==35){
          printf("\033[1;31m|\033[0m");
        }
        sendMsgToFingersTopic.data.push_back(dataToFingersTopic[i]);
      }
      std::cout << std::endl;

    } else {          // сообщение - настройки
      for (int i = 0; i < sizeof(dataToFingersTopic_setting); i++){
        printf("[%u]", dataToFingersTopic_setting[i]);
        if (i==8 || i==17 || i==26 || i==35 || i==44 || i==53){
          printf("\033[1;31m|\033[0m");
        }
        sendMsgToFingersTopic.data.push_back(dataToFingersTopic_setting[i]);
      }
      std::cout << std::endl;

    }
    
    toFingersPub.publish(sendMsgToFingersTopic);
    
    // установка и перенаправление сообщений на Кисть в отдельный ROS-топик в удобочитаемом виде
    if (!settings){
      fingers::To_Finger msgsArrToDebugFingers[6];
      setMsgsToDebugTopic(msgsArrToDebugFingers, dataToFingersTopic);
      sendMsgToDebugTopic(msgsArrToDebugFingers);
    }

  }

  // отправка в ROS-топик данных на Плату Управления (АКБ, камеры)
  void sendMsgToBatCam(){

    //отправка пакета в ROS-топик "toBatCamTopic"
    std_msgs::ByteMultiArray sendMsgToCameraTopic;
    sendMsgToCameraTopic.layout.dim.push_back(std_msgs::MultiArrayDimension());
    sendMsgToCameraTopic.layout.dim[0].size   = 1;
    sendMsgToCameraTopic.layout.dim[0].stride = 3;
    sendMsgToCameraTopic.data.clear();

    std::cout << "\033[1;34mSEND TO toBatCamTopic:\033[0m\n";
    printf("camera_from_udp = [%u]\n",  currentState.camera_from_udp);
    printf("relay_from_udp  = [%u]\n",  currentState.relay_from_udp);
    printf("ur5_from_udp    = [%u]\n",  currentState.ur5_from_udp);
    sendMsgToCameraTopic.data.push_back(currentState.camera_from_udp);
    sendMsgToCameraTopic.data.push_back(currentState.relay_from_udp);
    sendMsgToCameraTopic.data.push_back(currentState.ur5_from_udp);
    
    toBatCamPub.publish(sendMsgToCameraTopic);
    
    // установка и перенаправление сообщений на Плату Управления (АКБ, камеры) в отдельный ROS-топик в удобочитаемом виде
    fingers::To_Bat_Cam msgToDebugBatCam;
    msgToDebugBatCam.camera     = currentState.camera_from_udp;
    msgToDebugBatCam.relay      = currentState.relay_from_udp;
    msgToDebugBatCam.ur5_state  = currentState.ur5_from_udp;
    if (debugBatCam) debugToBatCamPub.publish(msgToDebugBatCam);
  }
  
  // отправка полученных данных с ROS-топиков от Кисти и Платы Управления (АКБ, камеры) пользователю по протоколу UDP
  void sendMsgToUDP(){
    allDevOk |= fingersOk;
    allDevOk |= boardOk;

    // формируем пакет данных для отправки по протоколу UDP
    if (resvdBytesFromFingers == DATA_FROM_FINGERS_TOPIC_SIZE){
      dataToUDP[0] = 0xBB;                                                                                    //header                   1b
      dataToUDP[1] = 0xAA;                                                                                    //header                   1b
      dataToUDP[2] = sizeof(dataToUDP);                                                                       //data length              1b
      memcpy(dataToUDP + 3, dataFromFingersTopic, sizeof(dataFromFingersTopic) - 2 * sizeof(uint8_t));        //data from fingers (6f * 10b)
      dataToUDP[sizeof(dataToUDP) - 13 * sizeof(uint8_t)] = currentState.hand_mount;                          //hand_mount               1b
      dataToUDP[sizeof(dataToUDP) - 12 * sizeof(uint8_t)] = currentState.hold_position;                       //hold_position            1b
      dataToUDP[sizeof(dataToUDP) - 11 * sizeof(uint8_t)] = currentState.camera_from_bat_cam;                 //camera_from_bat_cam      1b
      dataToUDP[sizeof(dataToUDP) - 10 * sizeof(uint8_t)] = currentState.bat_24V;                             //bat_24V                  1b
      dataToUDP[sizeof(dataToUDP) - 9  * sizeof(uint8_t)] = currentState.bat_48V;                             //bat_48V                  1b
      dataToUDP[sizeof(dataToUDP) - 8  * sizeof(uint8_t)] = allDevOk;                                         //allDevOk                 1b
      dataToUDP[sizeof(dataToUDP) - 7  * sizeof(uint8_t)] = currentState.ur5_from_bat_cam;                    //ur5_state                1b
      dataToUDP[sizeof(dataToUDP) - 6  * sizeof(uint8_t)] = currentState.relay_from_bat_cam;                  //relay_state              1b
      memcpy(dataToUDP + sizeof(dataToUDP) - 5 * sizeof(uint8_t), currentState.keepalive,                     //keepalive                4b
          sizeof(currentState.keepalive));
      dataToUDP[sizeof(dataToUDP) - 1 * sizeof(uint8_t)]  =                                                   //crc8                     1b     
          umba_crc8_table(dataToUDP, sizeof(dataToUDP) - sizeof(uint8_t));                                                                                

    } else if (resvdBytesFromFingers == DATA_FROM_FINGERS_TOPIC_SIZE_SETTINGS){
      dataToUDP_setting[0] = 0xBB;                                                                            //header                  1b
      dataToUDP_setting[1] = 0xAA;                                                                            //header                  1b
      dataToUDP_setting[2] = sizeof(dataToUDP_setting);                                                       //data length             1b
      memcpy(dataToUDP_setting + 3, dataFromFingersTopic_setting,                                             //data from fingers (6f * 9b)
          sizeof(dataFromFingersTopic_setting) - 2 * sizeof(uint8_t));    
      dataToUDP_setting[sizeof(dataToUDP_setting) - 13 * sizeof(uint8_t)] = currentState.hand_mount;          //hand_mount              1b
      dataToUDP_setting[sizeof(dataToUDP_setting) - 12 * sizeof(uint8_t)] = currentState.hold_position;       //hold_position           1b
      dataToUDP_setting[sizeof(dataToUDP_setting) - 11 * sizeof(uint8_t)] = currentState.camera_from_bat_cam; //camera_from_bat_cam     1b
      dataToUDP_setting[sizeof(dataToUDP_setting) - 10 * sizeof(uint8_t)] = currentState.bat_24V;             //bat_24V                 1b
      dataToUDP_setting[sizeof(dataToUDP_setting) - 9  * sizeof(uint8_t)] = currentState.bat_48V;             //bat_48V                 1b
      dataToUDP_setting[sizeof(dataToUDP_setting) - 8  * sizeof(uint8_t)] = allDevOk;                         //allDevOk                1b
      dataToUDP_setting[sizeof(dataToUDP_setting) - 7  * sizeof(uint8_t)] = currentState.ur5_from_bat_cam;    //ur5_state               1b
      dataToUDP_setting[sizeof(dataToUDP_setting) - 6  * sizeof(uint8_t)] = currentState.relay_from_bat_cam;  //relay_state             1b
      memcpy(dataToUDP_setting + sizeof(dataToUDP_setting) - 5 * sizeof(uint8_t), currentState.keepalive,     //keepalive               4b
          sizeof(currentState.keepalive));
      dataToUDP_setting[sizeof(dataToUDP_setting) - 1 * sizeof(uint8_t)]  =                                   //crc8                    1b     
          umba_crc8_table(dataToUDP_setting, sizeof(dataToUDP_setting) - sizeof(uint8_t));                                          

    }

    // отправляем пакет данных по протоколу UDP
    boost::system::error_code error;
    if (resvdBytesFromFingers == DATA_FROM_FINGERS_TOPIC_SIZE){
      auto sent = socket_.send_to(boost::asio::buffer(dataToUDP), sender_endpoint_, 0, error);
      if (!error && sent > 0){

        boost::chrono::system_clock::time_point cur_tp_send = boost::chrono::system_clock::now();
        boost::chrono::duration<double> ex_time_send = cur_tp_send - first_tp_send;
        std::cout << "\033\n[1;32mex_time_send time: \033\n[0m" << ex_time_send.count() * 1000000 << "\n";
        first_tp_send = boost::chrono::system_clock::now();
        if ((ex_time_send.count() * 1000) > 100) {
          // while (ros::ok()) {;}
          sendFailCount++;
          printf("\n\n\n\033[1;31mSEND ERR\nSEND ERR\nSEND ERR\nSEND ERR\n\n\n\n\033[0m");
        }

        allDevOk = 0;
        send_count_udp++;
        std::cout << "\033[1;36mSEND TO UDP bytes_transferred = \033[0m" << sent << std::endl;
        std::cout << "send_count_udp = " << send_count_udp << std::endl;
        for (int i = 0; i < sent; i++){
          printf("[%u]", dataToUDP[i]);
          if (i==2 || i==12 || i== 22 || i==32 || i==42 || i==52 || i==62 || i==63 || i==64 || i==65 || i==66 || i==67 || i==68 || i==69 || i==70 || i==74){
            printf("\033[1;31m|\033[0m");
          }
        }
        std::cout << std::endl;
      }

    } else if (resvdBytesFromFingers == DATA_FROM_FINGERS_TOPIC_SIZE_SETTINGS){
      auto sent = socket_.send_to(boost::asio::buffer(dataToUDP_setting), sender_endpoint_, 0, error);
      if (!error && sent > 0){
        allDevOk = 0;
        send_count_udp++;
        std::cout << "\033[1;36mSEND TO UDP bytes_transferred = \033[0m" << sent << std::endl;
        std::cout << "send_count_udp = " << send_count_udp << std::endl;
        for (int i = 0; i < sent; i++){
          printf("[%u]", dataToUDP_setting[i]);
          if (i==2 || i==11 || i== 20 || i==29 || i==38 || i==47 || i==56 || i==57 || i==58 || i==59 || i==60 || i==61 || i==62 || i==63 || i==64 || i==68){
            printf("\033[1;31m|\033[0m");
          }
        }
        std::cout << std::endl;
      }

    }
    resvdBytesFromFingers = 0;
  
  }

  // возвращает длину пакета
  static inline uint8_t getLen(uint8_t* ptrBuff)
  {
    return ptrBuff[2];
  }

  // возвращает контрольную сумму пакета
  static inline uint8_t getCrc8(uint8_t* ptrBuff, uint32_t len)
  {
    return ptrBuff[len - sizeof(uint8_t)];
  }

  // проверка принятого пакета по протоколу UDP на валидность
  bool parserUDP(uint8_t* dataFromUDP){
    if (dataFromUDP[0] != 0xAA) return false;
    if (dataFromUDP[1] != 0xBB) return false;
    /* Если длина пакета не валидная, ошибка */
    if (getLen(dataFromUDP) != DATA_FROM_UDP_SIZE && getLen(dataFromUDP) != DATA_FROM_UDP_SIZE_SETTINGS) {
      return false;
    }
    /* Если контрольная сумма не совпадает, приняли муссор, ошибка */
    if ((umba_crc8_table(dataFromUDP, DATA_FROM_UDP_SIZE          - sizeof(uint8_t)) != getCrc8(dataFromUDP, DATA_FROM_UDP_SIZE)) &&
        (umba_crc8_table(dataFromUDP, DATA_FROM_UDP_SIZE_SETTINGS - sizeof(uint8_t)) != getCrc8(dataFromUDP, DATA_FROM_UDP_SIZE_SETTINGS))) {
      return false;
    }
    settings = (getLen(dataFromUDP) == DATA_FROM_UDP_SIZE_SETTINGS);

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

    ros::param::get("/_debugBigFinger",   debugBigFinger);
    ros::param::get("/_debugIndexFinger", debugIndexFinger);
    ros::param::get("/_debugMidFinger",   debugMidFinger);
    ros::param::get("/_debugRingFinger",  debugRingFinger);
    ros::param::get("/_debugPinky",       debugPinky);
    ros::param::get("/_debugModulOtv",    debugModulOtv);
    ros::param::get("/_debugBatCam",      debugBatCam);
    ros::param::get("/_debugAllFingers",  debugAllFingers);

    std::cout << "debugBigFinger "    << debugBigFinger   << std::endl;
    std::cout << "debugIndexFinger "  << debugIndexFinger << std::endl;
    std::cout << "debugMidFinger "    << debugMidFinger   << std::endl;
    std::cout << "debugRingFinger "   << debugRingFinger  << std::endl;
    std::cout << "debugPinky "        << debugPinky       << std::endl;
    std::cout << "debugModulOtv "     << debugModulOtv    << std::endl; 
    std::cout << "debugBatCam "       << debugBatCam      << std::endl; 

    while(ros::ok()){
      udpServer.nodeFromUDPProcess();
      io_service.poll_one();
      ros::spinOnce();
    }
	} catch (std::exception e){
		std::cerr << "Exeption: " << e.what() << std::endl;
	}
  return 0;
};//z