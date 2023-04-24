#include <iostream>
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
#include <boost/thread/thread.hpp>

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
    debugFromBatCamPub           = node.advertise<fingers::From_Finger>("debugFromBatCamTopic", 0);

    fromFingersPub = node.advertise<std_msgs::ByteMultiArray>("fromFingersTopic", 0);
  };

  void nodeFromTopicProcess(){

    while (count < 10000) {
      ROS_INFO("");
      boost::chrono::system_clock::time_point cur_tp = boost::chrono::system_clock::now();
      boost::chrono::duration<double> ex_time = cur_tp - first_tp;
      std::cout << "Execution time: " << ex_time.count() << "\n";
      std::cout << "count = " << count << "\n";
      first_tp = boost::chrono::system_clock::now();
      toEachFinger();
      sendMsgToTopic();
      count++;
      ROS_INFO("count = %d\n", count);
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
  uint8_t dataFromFinger_old[6][DATA_FROM_FINGER_SIZE] = {0};
  uint32_t dataFromFingerSize = 0;
  ros::NodeHandle node;
  ros::Publisher fromFingersPub;
  ros::Subscriber toFingersSub;

  ros::Publisher debugFromBigFingerPub;
  ros::Publisher debugFromIndexFingerPub;
  ros::Publisher debugFromMidFingerPub;
  ros::Publisher debugFromRingFingerPub;
  ros::Publisher debugFromPinkyPub;
  ros::Publisher debugFromModulOtvPub;
  ros::Publisher debugFromBatCamPub;

  volatile int count = 0;

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
    //if(dataToHandMount == dataFromTopic[sizeof(dataFromTopic) - sizeof(uint8_t)]) return;
    memset(dataFromHandMount, 0, sizeof(dataFromHandMount));
    dataFromHandMountSize = 0;
    dataToHandMount = dataFromTopic[sizeof(dataFromTopic) - sizeof(uint8_t)];
    std::cout << "\ndataToHandMount = ";
    printf("%u\n", dataToHandMount);

    #if CODE_PART == RAW_UDP_DATA

    if(m_protocol.sendCmdReadWrite(hand_mount_addr, 0x30, &dataToHandMount, sizeof(uint8_t), 
                                            dataFromHandMount, &dataFromHandMountSize)){
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
      std::cout << "\033\n[1;34mSIZE OF RECVD DATA FROM TOPIC toFingersTopic = \033[0m" << recvdMsg->data.size() << std::endl;
      std::cout << "recvd_count_topic = " << recvd_count_topic << std::endl;
      memset(dataFromTopic, 0, sizeof(dataFromTopic));
      memset(dataToTopic, 0, sizeof(dataToTopic));
      resvdFromAllDev = 0;
      for (int i = 0; i < recvdMsg->data.size(); i++){
        dataFromTopic[i] = recvdMsg->data[i];
        printf("[%u]", dataFromTopic[i]);
      }
  }

  // void debug_topic_receive(const fingers::To_Fingers_HandMount::ConstPtr& recvdMsg) {
  //   for (size_t i = 0; i < 6; i++)
  //   {
  //     dataFromTopic[i*5] = (recvdMsg->angle[i])&&0xFF;
  //     dataFromTopic[i*5+1] = (recvdMsg->angle[i])>>8;
  //     dataFromTopic[i*5+2] = (recvdMsg->force[i])&&0xFF;
  //     dataFromTopic[i*5+3] = (recvdMsg->force[i])>>8;
  //   }
  //   dataFromTopic[30] = recvdMsg->hand_mount;
  // }

  void toEachFinger(){

    for (int i = 0; i < fingersAddrs.size(); i++){
      memset(dataToFinger, 0, sizeof(dataToFinger));
      memset(dataFromFinger, 0, sizeof(dataFromFinger));
      dataFromFingerSize = 0;
      memcpy(dataToFinger, dataFromTopic + i * sizeof(dataToFinger), sizeof(dataToFinger));
      std::cout << "\ndataToFinger ";
      printf("%u = ", fingersAddrs[i]);
      for (int i = 0; i < sizeof(dataToFinger); i++){
        printf("[%u]", dataToFinger[i]);
      }

      #if CODE_PART == RAW_UDP_DATA

      //
      if (m_protocol.sendCmdReadWrite(fingersAddrs[i], 0x30, dataToFinger, sizeof(dataToFinger), 
                                                    dataFromFinger, &dataFromFingerSize)) {
      	resvdFromAllDev |= fingers_OK[i]; //ответ пришел
      	std::cout << "\033\n[1;32mOk\033\n[0m";
      	recvd_count_rs++;
        rcvd_cnt_f[fingersAddrs[i]- 0x11]++;
      	printf("\nrecvd_count_rs = %u\n", recvd_count_rs);
      	std::cout << "FAIL CNT OF " << fingersAddrs[i] << "device = " << fail_cnt_f[fingersAddrs[i] - 0x11] << std::endl;
        std::cout << "RCVD CNT OF " << fingersAddrs[i] << "device = " << rcvd_cnt_f[fingersAddrs[i] - 0x11] << std::endl;
        memset(dataFromFinger_old[i], 0, dataFromFingerSize);
        memcpy(dataFromFinger_old[i], dataFromFinger, sizeof(dataFromFinger));
      } else {
      	resvdFromAllDev &= ~fingers_OK[i]; //ответ НЕ пришел
      	std::cout << "\033\n[1;31mNO DATA FROM DEVICE\033\n[0m";
        std::cout << "FAIL CNT OF " << fingersAddrs[i] << "device = " << fail_cnt_f[fingersAddrs[i] - 0x11] << std::endl;
        std::cout << "RCVD CNT OF " << fingersAddrs[i] << "device = " << rcvd_cnt_f[fingersAddrs[i] - 0x11] << std::endl;
        fail_cnt_f[fingersAddrs[i] - 0x11]++;
      	//memset(dataFromHandMount, 0, dataFromHandMountSize);
      	//dataFromHandMountSize = 0;
        memset(dataFromFinger, 0, dataFromFingerSize);
        memcpy(dataFromFinger, dataFromFinger_old[i], sizeof(dataFromFinger_old[i]));
      	fail_cnt++;
      	printf("\nfail_cnt = %u\n", fail_cnt);
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
      	resvdFromAllDev &= ~fingers_OK[i]; //ответ НЕ пришел
      	std::cout << "\033\n[1;31mNO DATA FROM HAND_MOUNT\033\n[0m";
      	memset(dataFromHandMount, 0, dataFromHandMountSize);
      	dataFromHandMountSize = 0;
      	fail_cnt++;
      	printf("\nfail_cnt = %u\n", fail_cnt);
      }
      //

      #else 

      #error "INCORRECT CODE_PART"

      #endif
      // std::cout << "\ndata FROM Finger \n";
      // for (int i = 0; i < sizeof(dataFromFinger) - 4; i++)
      //   printf("[%u]", dataFromFinger[i+3]);
      // std::cout << "SIZE OF DATA FROM FINGER: " << dataFromFingerSize << std::endl;
      memcpy(dataToTopic + i * (sizeof(dataFromFinger) - 4 * sizeof(uint8_t)), dataFromFinger + 3 * sizeof(uint8_t), sizeof(dataFromFinger) - 4 * sizeof(uint8_t));
      // std::cout << "\ndata TO TOPIC Finger \n";
      // for (int i = 0; i < sizeof(dataToTopic); i++)
      //   printf("[%u]", dataToTopic[i]);
      // std::cout << "\n!!POSITION!! = " << i * sizeof(dataFromFinger) << std::endl;
    }
    dataToTopic[sizeof(dataToTopic) - sizeof(uint8_t)] = resvdFromAllDev;
  }

  void sendMsgToDebugTopic(const fingers::From_Finger msgsArrToDebugFingers[]){
    if (debugBigFinger)   debugFromBigFingerPub.publish(msgsArrToDebugFingers[0]);
    if (debugIndexFinger) debugFromIndexFingerPub.publish(msgsArrToDebugFingers[1]);
    if (debugMidFinger)   debugFromMidFingerPub.publish(msgsArrToDebugFingers[2]);
    if (debugRingFinger)  debugFromRingFingerPub.publish(msgsArrToDebugFingers[3]);
    if (debugPinky)       debugFromPinkyPub.publish(msgsArrToDebugFingers[4]);
    if (debugModulOtv)    debugFromModulOtvPub.publish(msgsArrToDebugFingers[5]);
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
    std::cout << "\n\033[1;34mSEND MSG TO TOPIC fromFingersTopic: \n\033[0m";
    for (int i = 0; i < sizeof(dataToTopic); i++){
      sendMsgFromFingersTopic.data.push_back(dataToTopic[i]);
      printf("[%u]", dataToTopic[i]);
    }
    std::cout << "\nresvdFromAllDev = " << resvdFromAllDev;
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
  int baudrate = 256000;

  try{

    ros::param::get("/_devPortForFingers",  devPort);
    ros::param::get("/_baudrateForFingers", baudrate);
    ros::param::get("/_debugBigFinger",     debugBigFinger);
    ros::param::get("/_debugIndexFinger",   debugIndexFinger);
    ros::param::get("/_debugMidFinger",     debugMidFinger);
    ros::param::get("/_debugRingFinger",    debugRingFinger);
    ros::param::get("/_debugPinky",         debugPinky);
    ros::param::get("/_debugModulOtv",      debugModulOtv);
    ros::param::get("/_debugBatCam",        debugBatCam);

    printf("devPort           = %s\n", devPort);
    printf("baudrate          = %d\n", baudrate);
    printf("debugBigFinger    = %d\n", debugBigFinger);
    printf("debugIndexFinger  = %d\n", debugIndexFinger);
    printf("debugMidFinger    = %d\n", debugMidFinger);
    printf("debugRingFinger   = %d\n", debugRingFinger);
    printf("debugPinky        = %d\n", debugPinky);
    printf("debugModulOtv     = %d\n", debugModulOtv);
    printf("debugBatCam       = %d\n", debugBatCam);

    boost::this_thread::sleep(boost::posix_time::seconds(60));

    std::cout << "\n\033[1;32m╔═══════════════════════════════════════╗\033[0m"
              << "\n\033[1;32m║       MTOPIC_RECIEVER is running!     ║\033[0m" 
              << "\n\033[1;32m║Baud rate: " << baudrate << ", Port: /dev/tty" << devPort << "\t║\033[0m"
              << "\n\033[1;32m╚═══════════════════════════════════════╝\033[0m\n";
    ros::init(argc, argv, "master_topic_receiver");
    
    qt_serial::Qt_Serial_Async qt_RS_transp("/dev/tty" + devPort, baudrate);
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