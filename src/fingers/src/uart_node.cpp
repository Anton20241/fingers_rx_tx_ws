/*
  Задачи данной программы:
  1) Прием данных из ROS-топика(1) для последующей отправки на Плату Управления (АКБ, камеры), далее просто Плата Управления
  2) Отправка  полученных данных на Плату Управления
  3) Получение актуальных данных  с Платы Управления
  4) Отправка  полученных данных  с Платы Управления в ROS-топик(3) для последующей отправки по UDP

  ROS-топик(1) -> тек. программа -> Плата Управления (АКБ, камеры) -> тек. программа -> ROS-топик(3)
*/

#include <iostream>
#include <boost/asio.hpp>
#include <ros/ros.h>
#include <std_msgs/ByteMultiArray.h>
#include <fingers/From_Bat_Cam_Norm_Work.h>
#include <fingers/From_Bat_Cam_Shutdown.h>
#include <fingers/To_Bat_Cam.h>
#include "qt_serial.hpp"
#include "boost_serial.hpp"
#include <boost/chrono.hpp>
#include "protocol.hpp"
#include "umba_crc_table.h"
#include <mutex>

// размеры пакетов данных
#define DATA_FROM_BOARD_NORM_SIZE               9
#define DATA_FROM_BOARD_SHUTDOWN_SIZE           5
#define DATA_FROM_BATCAM_TOPIC_SIZE             6

// имена ROS-топиков для обмена данными между программами
#define TO_BAT_CAM_TOPIC_NAME                   "toBatCamTopic"
#define FROM_BAT_CAM_TOPIC_NAME                 "fromBatCamTopic"
#define DEBUG_TO_BAT_CAM_TOPIC_NAME             "debugToBatCamTopic"
#define DEBUG_FROM_BAT_CAM_NORM_TOPIC_NAME      "debugFromBatCamNormTopic"
#define DEBUG_FROM_BAT_CAM_SHUTDOWN_TOPIC_NAME  "debugFromBatCamShutdownTopic"

// 0 - debug off, 1 - debug on : возможность отслеживать данные с Платы Управления в удобочитаемом виде
int debugBatCam = 0;

/*
  Класс UART_Node предназначен для реализации функционала обмена данными между ROS-топиками и Платой Управления.
  Класс UART_Node включает следующие методы:
    * send_init_cmd()               - первоначальное подключение к Плате Управления
    * UART_process()                - основной цикл программы
    * sendError()                   - отправка сообщения об ошибке в ROS-топик(3), если Плата Управления не отвечает
    * pub_board_data()              - отправка данных в ROS-топик(3), полученных от Платы Управления
    * showToBoardData()             - вывод данных, отправляемых на Плату Управления
    * sendDataToBoard()             - отправка данных на Плату Управления
    * to_bat_cam_callback()         - обработчик сообщений от ROS-топика(1) на Плату Управления
    * debug_to_bat_cam_callback()   - обработчик сообщений от ROS-топика(2) на Плату Управления
*/
class UART_Node
{
public:

  // конструктор класса UART_Node, в котором происходит подключение к ROS-топикам 
  UART_Node(protocol_master::ProtocolMaster& protocol_)
  : m_protocol(protocol_){
      bat_cam_sub                 = node.subscribe<std_msgs::ByteMultiArray>(TO_BAT_CAM_TOPIC_NAME,   0, &UART_Node::to_bat_cam_callback,this);
      debug_bat_cam_sub           = node.subscribe<fingers::To_Bat_Cam>(DEBUG_TO_BAT_CAM_TOPIC_NAME,  0, &UART_Node::debug_to_bat_cam_callback,this);
      bat_cam_pub                 = node.advertise<std_msgs::ByteMultiArray>(FROM_BAT_CAM_TOPIC_NAME,                       0);
      debugFromBatCamNormPub      = node.advertise<fingers::From_Bat_Cam_Norm_Work>(DEBUG_FROM_BAT_CAM_NORM_TOPIC_NAME,     0);
      debugFromBatCamShutdownPub  = node.advertise<fingers::From_Bat_Cam_Shutdown>(DEBUG_FROM_BAT_CAM_SHUTDOWN_TOPIC_NAME,  0);
  };

  // первоначальное подключение к Плате Управления
  void send_init_cmd() {

    std::cout << "!!!START send_init_cmd!!!" << std::endl;
    uint32_t requestToBoardCount = 0;
    bool getResponse             = false;

    while(!getDataFromBoard && requestToBoardCount < 10){
      m_protocol.sendCmdWriteUART(0x01, 0x10, &cam_status, sizeof(cam_status));
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      requestToBoardCount++;

      memset(from_board_data, 0, sizeof(from_board_data));
      from_board_dataSize = 0;

      if(m_protocol.sendCmdReadUART(0x01, from_board_data, &from_board_dataSize, getResponse, true, cam_status)){
        getDataFromBoard = true;
        resvdFromDev    |= 128;
        pub_board_data();
      }
    }
    std::cout << "!!!END send_init_cmd!!!" << std::endl;
  }

  // основной цикл программы
  void UART_process(){
    
    static uint32_t fail_count = 0;

    // при отсутствии сообщения на Плату Управления, ожидание от нее сообщения в течение 70 секунд
    if (!msg_sent_relay && !msg_sent_cam && !msg_sent_ur5){
      bool getResponse = false;

      memset(from_board_data, 0, sizeof(from_board_data));
      from_board_dataSize = 0;

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

    /*
      если была отправка сообщения на Плату Управления для UR-5,
      ожидание, получение ответа от Платы Управления и отправка данного ответа в ROS-топик(3)
    */
    if (msg_sent_ur5){
      bool getResponse = false;

      memset(from_board_data, 0, sizeof(from_board_data));
      from_board_dataSize = 0;

      if (m_protocol.sendCmdReadUART(0x01, from_board_data, &from_board_dataSize, getResponse, msg_sent_ur5, ur5_state)){
        resvdFromDev |= 128;
        fail_count    = 0;
        pub_board_data();
      } else {
        std::cout << "[msg_sent_ur5 and failed]\n";
        sendError();
      }
      msg_sent_ur5 = false;
    }

    /*
      если была отправка сообщения на Плату Управления для Реле,
      ожидание, получение ответа от Платы Управления и отправка данного ответа в ROS-топик(3)
    */
    if (msg_sent_relay){
      bool getResponse = false;

      memset(from_board_data, 0, sizeof(from_board_data));
      from_board_dataSize = 0;

      if (m_protocol.sendCmdReadUART(0x01, from_board_data, &from_board_dataSize, getResponse, msg_sent_relay, relay_state)){
        resvdFromDev |= 128;
        fail_count    = 0;
        pub_board_data();
      } else {
        std::cout << "[msg_sent_relay and failed]\n";
        sendError();
      }
      msg_sent_relay = false;
    }

    /*
      если была отправка сообщения на Плату Управления для Камеры,
      ожидание, получение ответа от Платы Управления и отправка данного ответа в ROS-топик(3)
    */
    if (msg_sent_cam){
      bool getResponse = false;
      if (m_protocol.sendCmdReadUART(0x01, from_board_data, &from_board_dataSize, getResponse, msg_sent_cam, cam_status)){
        resvdFromDev |= 128;
        fail_count    = 0;
        pub_board_data();
      } else {
        std::cout << "[msg_sent_cam and failed]\n";
        sendError();
      }
      msg_sent_cam = false;
    } 
  }

private:
  protocol_master::ProtocolMaster& m_protocol;
  ros::NodeHandle node;
  ros::Subscriber bat_cam_sub;
  ros::Subscriber debug_bat_cam_sub;
  ros::Publisher  bat_cam_pub;
  ros::Publisher  debugFromBatCamNormPub;
  ros::Publisher  debugFromBatCamShutdownPub;
  uint8_t from_board_data[DATA_FROM_BOARD_NORM_SIZE]      = {0};
  uint8_t from_bat_cam_topic[DATA_FROM_BATCAM_TOPIC_SIZE] = {0};
  uint8_t resvdFromDev                                    = 0;
  uint8_t cam_status                                      = 3;
  uint8_t cam_status_prev                                 = 3;
  uint8_t relay_state                                     = 0;
  uint8_t relay_state_prev                                = 0;
  uint8_t ur5_state                                       = 0;
  uint8_t ur5_state_prev                                  = 0;
  uint32_t recvd_count_topic                              = 0;
  uint32_t from_board_dataSize                            = 0;
  bool msg_sent_relay                                     = false;
  bool msg_sent_cam                                       = false;
  bool msg_sent_ur5                                       = false;
  bool getDataFromBoard                                   = false;

  // отправка сообщения об ошибке в ROS-топик(3), если Плата Управления не отвечает
  void sendError(){
    m_protocol.m_transport.send_error = true;
    static uint32_t failCount         = 0;
    failCount++;
    std::cout << "\nfailCount = " << failCount << std::endl;
    std::cout << "\033[1;31m[RECEIVE ERROR FROM UART]\033[0m\n";
    std::cout << "\033[1;31m[NON-Valid MESSAGE]:\033[0m\n";
    
    for (int i = 0; i < from_board_dataSize; i++){
      printf("[%u]", from_board_data[i]);
    }
    std::cout << std::endl;
    resvdFromDev = 0;
    memset(from_board_data, 0, sizeof(from_board_data));
    from_board_dataSize = 1;
    from_board_data[0] = 0;
    pub_board_data();
  }

  // заполнение и отправка данных в ROS-топик(3), полученных от Платы Управления
  void pub_board_data()
  {
    uint8_t bytesToSendCount = 0;

    fingers::From_Bat_Cam_Norm_Work msgFromBatCamNorm;
    fingers::From_Bat_Cam_Shutdown  msgFromBatCamShutdown;
    memset(from_bat_cam_topic, 0, sizeof(from_bat_cam_topic));
    
    // если от Платы Управления пришли данные на выключение, заполняем данные для отправки в ROS-топик(3) на выключение устройства
    if (from_board_dataSize == DATA_FROM_BOARD_SHUTDOWN_SIZE){
      from_bat_cam_topic[0] = from_board_data[2];             // CMD
      from_bat_cam_topic[1] = from_board_data[3];             // TIME_DOWN
      from_bat_cam_topic[2] = resvdFromDev;                   // all_ok
      bytesToSendCount = 3;

      // заполнение и перенаправление данных с Платы Управления в отдельный ROS-топик(4) в удобочитаемом виде
      msgFromBatCamShutdown.cmdBatCamTopic = from_bat_cam_topic[0];   // CMD
      msgFromBatCamShutdown.time_down      = from_bat_cam_topic[1];   // TIME_DOWN
      if (debugBatCam) debugFromBatCamShutdownPub.publish(msgFromBatCamShutdown);
      
    }

    // если от Платы Управления пришли штатные данные, заполняем данные для отправки в ROS-топик(3)
    else if(from_board_dataSize == DATA_FROM_BOARD_NORM_SIZE) {
      from_bat_cam_topic[0] = from_board_data[3];             // BAT_24V
      from_bat_cam_topic[1] = from_board_data[4];             // BAT_48V
      from_bat_cam_topic[2] = from_board_data[5];             // VIDEO_SWITCH
      from_bat_cam_topic[3] = from_board_data[6];             // relay_state
      from_bat_cam_topic[4] = from_board_data[7];             // ur5_state
      from_bat_cam_topic[5] = resvdFromDev;                   // all_ok
      bytesToSendCount = 6;

      // заполнение и перенаправление данных с Платы Управления в отдельный ROS-топик(5) в удобочитаемом виде
      msgFromBatCamNorm.bat_24V     = from_bat_cam_topic[0];  // BAT_24V
      msgFromBatCamNorm.bat_48V     = from_bat_cam_topic[1];  // BAT_48V
      msgFromBatCamNorm.camera      = from_bat_cam_topic[2];  // VIDEO_SWITCH
      msgFromBatCamNorm.relay       = from_bat_cam_topic[3];  // relay_state
      msgFromBatCamNorm.ur5_state   = from_bat_cam_topic[4];  // ur5_state
      if (debugBatCam) debugFromBatCamNormPub.publish(msgFromBatCamNorm);

    // если от Платы Управления данные не пришли, формируем сообщение об ошибке для отправки в ROS-топик(3) 
    } else if(from_board_dataSize == 1) {
      from_bat_cam_topic[0] = resvdFromDev;       // all_ok
      bytesToSendCount = 1;

    } else return;

    // непосредственно сама отправка данных в ROS-топик(3)
    std_msgs::ByteMultiArray fromBatCamTopicMsg;
    fromBatCamTopicMsg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    fromBatCamTopicMsg.layout.dim[0].size   = 1;
    fromBatCamTopicMsg.layout.dim[0].stride = bytesToSendCount;
    fromBatCamTopicMsg.data.clear();

    std::cout << "\033[1;34mSEND TO TOPIC fromBatCamTopic: \033[0m";
    for (uint32_t i = 0; i < bytesToSendCount; i++){
      printf("[%u]", from_bat_cam_topic[i]);
    }
    std::cout << std::endl;

    for (uint32_t i = 0; i < bytesToSendCount; i++){
      fromBatCamTopicMsg.data.push_back(from_bat_cam_topic[i]);
    }

    bat_cam_pub.publish(fromBatCamTopicMsg);
  }

  // вывод данных, отправляемых на Плату Управления
  void showToBoardData(){
    printf("\n");
    std::cout << "UR5 STATUS ";
    printf("%u\n", ur5_state);
    std::cout << "CAMERA STATUS ";
    printf("%u\n", cam_status);
    std::cout << "RELAY STATUS ";
    printf("%u\n", relay_state);
  }

  // отправка данных на Плату Управления
  void sendDataToBoard(){

    // если пришли новые данные для UR-5, отправляем эти данные на Плату Управления
    if (ur5_state != ur5_state_prev){
      showToBoardData();
      ur5_state_prev = ur5_state;
      std::cout << "\033[1;35m[send UART msg with new ur5_state]\033[0m\n";
      m_protocol.sendCmdWriteUART(0x01, 0x30, &ur5_state, sizeof(ur5_state));
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      msg_sent_ur5 = true;
    }

    // если пришли новые данные для Реле, отправляем эти данные на Плату Управления
    if (relay_state != relay_state_prev){
      showToBoardData();
      relay_state_prev = relay_state;
      std::cout << "\033[1;35m[send UART msg with new relay_state]\033[0m\n";
      uint8_t toRelaySet[2] = {1, relay_state};
      m_protocol.sendCmdWriteUART(0x01, 0x20, toRelaySet, sizeof(toRelaySet));
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      msg_sent_relay = true;
    }

    // если пришли новые данные для Камеры, отправляем эти данные на Плату Управления
    if (cam_status != cam_status_prev){
      showToBoardData();
      cam_status_prev = cam_status;
      std::cout << "\033[1;35m[send UART msg with new cam_status]\033[0m\n";
      m_protocol.sendCmdWriteUART(0x01, 0x10, &cam_status, sizeof(cam_status));
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      msg_sent_cam = true;
    }
  }

  // обработчик сообщений от ROS-топика(2) на Плату Управления
  void debug_to_bat_cam_callback(const fingers::To_Bat_Cam::ConstPtr& camRelStatus){
    recvd_count_topic++;
    std::cout << "\n[debug_to_bat_cam_callback]\n";
    std::cout << "recvd_count_topic = " << recvd_count_topic << std::endl;
    cam_status  = camRelStatus->camera;
    relay_state = camRelStatus->relay;
    ur5_state   = camRelStatus->ur5_state;

    sendDataToBoard();
  }

  // обработчик сообщений от ROS-топика(1) на Плату Управления
  void to_bat_cam_callback(const std_msgs::ByteMultiArray::ConstPtr& camRelStatus){
    resvdFromDev = 0;
    recvd_count_topic++;
    cam_status  = camRelStatus->data[0];
    relay_state = camRelStatus->data[1];
    ur5_state   = camRelStatus->data[2];
    sendDataToBoard();
  }
};

int main(int argc, char** argv)
{
  QCoreApplication coreApplication(argc, argv);

  std::string port = "AMA0";
  std::string baudrate = "19200"; 

  ros::param::get("/_UART_baudrate", baudrate);
  ros::param::get("/_debugBatCam",   debugBatCam);
  
  try{
    std::cout << "\n\033[1;32m╔═══════════════════════════════════════╗\033[0m"
              << "\n\033[1;32m║UART Node is running!                  ║\033[0m" 
              << "\n\033[1;32m║Baud rate: " << baudrate << ", Port: /dev/tty" << port << "\t║\033[0m"
              << "\n\033[1;32m╚═══════════════════════════════════════╝\033[0m\n";
    ros::init(argc, argv, "uart_node");
    qt_serial::Qt_Serial_Async qt_serial_transp("/dev/tty" + port, baudrate);
    protocol_master::ProtocolMaster qt_serial_prot_master(qt_serial_transp, &coreApplication);
    UART_Node uartNode(qt_serial_prot_master);

    uartNode.send_init_cmd();
    while(ros::ok()){
      uartNode.UART_process();
      ros::spinOnce();
    }
  } catch (std::exception e){
      std::cerr << "\nExeption: " << e.what() << std::endl;
  }
  return 0;
};