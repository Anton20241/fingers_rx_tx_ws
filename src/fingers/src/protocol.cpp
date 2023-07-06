/**
 *  @file       protocol.cpp
 *  @brief      Протокол
 */

#include "protocol.hpp"
#include "umba_crc_table.h"
#include <assert.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <cstring>
#include <QCoreApplication>
#include <boost/chrono.hpp>


namespace protocol
{
  typedef union
  {
      struct
      {
          uint8_t id:4;
          uint8_t cmd:3; //amount of bits
          uint8_t err:1;
      };
  } cmd_field_t;

  /* Индекс полей */
  static const uint8_t indexAddr          = 0;
  static const uint8_t indexLen           = 1;
  static const uint8_t indexCmd           = 2;
  static const uint8_t indexData          = 3;

  /* Номера команд */
  static const uint8_t cmdNop             = 0;
  static const uint8_t cmdRead            = 1;
  static const uint8_t cmdWrite           = 2;
  static const uint8_t cmdReadWrite       = 3;

  /* Длина пакета */
  static const uint8_t packLenMin         = 4; //????
  static const uint8_t packLenMax         = 16; //????

  /* Ошибки */
  static const uint8_t errReadWrite       = 1;
  static const uint8_t errCmd             = 2;
  static const uint8_t errLen             = 3;

  static const uint8_t startRoReg         = 0; //????

  static const uint32_t proto_max_buff    = 32; //????

  static inline uint8_t getAddr(uint8_t* ptrBuff)
  {
      return ptrBuff[indexAddr];
  }

  static inline uint8_t getLen(uint8_t* ptrBuff)
  {
      return ptrBuff[indexLen];
  }

  static inline uint8_t getCrc8(uint8_t* ptrBuff, uint32_t len)
  {
      return ptrBuff[len - sizeof(uint8_t)];
  }

  static inline uint8_t getCmd(uint8_t* ptrBuff)
  {
      return (uint8_t)((cmd_field_t*)(&ptrBuff[indexCmd]))->cmd;
  }

  Protocol::Protocol( i_transport::ITransport& transport,
                      tabl_reg::TablReg& tabl,
                      uint8_t addr) :
      m_transport(transport),
      m_tabl(tabl),
      m_addr(addr)
  {

  }

  void Protocol::process()
  {
      uint8_t     buff[proto_max_buff] = {0};
      uint32_t    len = proto_max_buff;

      if (m_transport.getData(buff, &len)) {
          parser(buff, len);
      }
  }


  void Protocol::parser(uint8_t* ptrBuff, uint32_t len)
  {
      uint8_t m_len;

      /* Игнорируем пакет с чужим адресом */
      if (getAddr(ptrBuff) != m_addr) {
          return;
      }
      /* Если длина пакета не валидная, не отвечаем */
      m_len = getLen(ptrBuff);
      if (m_len < packLenMin) {
          return;
      }
      if (m_len > packLenMax) {
          return;
      }
      if (m_len > len) {
          return;
      }
      /* Если контрольная сумма не совпадает, приняли муссор, не отвечаем */
      if (umba_crc8_table(ptrBuff, m_len - 1) != getCrc8(ptrBuff, m_len)) {
          return;
      }
      switch (getCmd(ptrBuff))
      {
          case cmdNop:
              if (m_len != packLenMin) {
                  sendError(errLen, ptrBuff);
              }
              /* Отправляем зеркало */
              assert(m_transport.sendData(ptrBuff, packLenMin)); //len -> packLenMin
              break;
          case cmdRead:
              if (m_len != packLenMin) {
                  sendError(errLen, ptrBuff);
              }
              /* Считываем и отправляем регистры RO */
              sendData(ptrBuff);
              break;
          case cmdWrite:
              if (m_len == packLenMin) {
                  sendError(errLen, ptrBuff);
              }
              /* Пишем в регист WO */
              if (!writeData(ptrBuff, len)) {
                  sendError(errReadWrite, ptrBuff);
                  break;
              }
              sendSuccess(ptrBuff);
              break;
          case cmdReadWrite:
              if (m_len == packLenMin) {
                  sendError(errLen, ptrBuff);
              }
              /* Пишем в регист WO */
              if (!writeData(ptrBuff, len)) {
                  sendError(errReadWrite, ptrBuff);
                  break;
              }
              /* Считываем и отправляем регистры RO */
              sendData(ptrBuff);
              break;
          default:
              sendError(errCmd, ptrBuff);
              break;
      }
  }

  void Protocol::sendError(uint8_t err, uint8_t* ptrBuff)
  {
      uint8_t len = indexCmd;

      ((cmd_field_t*)(&ptrBuff[len++]))->err = 1;
      ptrBuff[len++] = err;
      ptrBuff[indexLen] = len + sizeof(uint8_t);
      ptrBuff[len++] = umba_crc8_table(ptrBuff, len);

      assert(m_transport.sendData(ptrBuff, len));
  }

  void Protocol::sendData(uint8_t* ptrBuff)
  {
      uint16_t len = m_tabl.getSizeRegRo();

      if (!m_tabl.getRegRaw(&ptrBuff[indexData], startRoReg, len)) {
          sendError(errReadWrite, ptrBuff);
          return;
      }
      len += indexData;
      ptrBuff[indexLen] = len + sizeof(uint8_t);
      ptrBuff[len++] = umba_crc8_table(ptrBuff, len);
      assert(m_transport.sendData(ptrBuff, len));
  }

  bool Protocol::writeData(uint8_t* ptrBuff, uint32_t len)
  {
      uint16_t dataLen = len - indexData - sizeof(uint8_t);

      if (!m_tabl.setRegRaw(&ptrBuff[indexData],
                          m_tabl.getSizeRegRo(),
                          dataLen)) {
          return false;
      }
      m_tabl.setEvent(tabl_reg::UPDATE_WO_REG);
      return true;
  }

  void Protocol::sendSuccess(uint8_t* ptrBuff)
  {
      uint8_t len = indexCmd + sizeof(uint8_t);

      ptrBuff[indexLen] = len + sizeof(uint8_t);
      ptrBuff[len++] = umba_crc8_table(ptrBuff, len);

      assert(m_transport.sendData(ptrBuff, len));
  }
  }

  namespace protocol_master
  {
  typedef union
  {
      struct
      {
          uint8_t id:4;
          uint8_t cmd:3; //amount of bits
          uint8_t err:1;
      };
  } cmd_field_t;

  boost::chrono::system_clock::time_point first_tp = boost::chrono::system_clock::now();

  /* Индекс полей */
  static const uint8_t indexAddr          = 0;
  static const uint8_t indexLen           = 1;
  static const uint8_t indexCmd           = 2;
  static const uint8_t indexData          = 3;

  /* Номера команд */
  static const uint8_t cmdNop             = 0;
  static const uint8_t cmdRead            = 1;
  static const uint8_t cmdWrite           = 2;
  static const uint8_t cmdReadWrite       = 3;

  /* Длина пакета */
  static const uint8_t packLenMin         = 4; //????
  static const uint8_t packLenMax         = 16; //????

  /* Ошибки */
  static const uint8_t errReadWrite       = 1;
  static const uint8_t errCmd             = 2;
  static const uint8_t errLen             = 3;

  static const uint8_t startRoReg         = 0; //????

  static const uint32_t proto_min_buff    = 4;    //????
  static const uint32_t proto_max_buff    = 120;  //????

  static inline uint8_t getAddr(uint8_t* ptrBuff)
  {
      return ptrBuff[indexAddr];
  }

  static inline uint8_t getLen(uint8_t* ptrBuff)
  {
      return ptrBuff[indexLen];
  }

  static inline uint8_t getCrc8(uint8_t* ptrBuff, uint32_t len)
  {
      return ptrBuff[len - sizeof(uint8_t)];
  }

  static inline uint8_t getCmd(uint8_t* ptrBuff)
  {
      return ptrBuff[indexCmd];
  }

  bool ProtocolMaster::parser(uint8_t* ptrBuff, uint32_t len_, uint8_t addressTo)
  {
    /* Если адрес не валидный, ошибка */
    if (getAddr(ptrBuff) != addressTo) {
      printf("\ngetAddr(ptrBuff) = %u\n", getAddr(ptrBuff));
      printf("addressTo = %u\n", addressTo);
      return false;
    }
    /* Если длина пакета не валидная, ошибка */
    if (getLen(ptrBuff) != len_) {
      printf("\ngetLen(ptrBuff) = %u\n", getLen(ptrBuff));
      printf("len_ = %u\n", len_);
      return false;
    }
    /* Если контрольная сумма не совпадает, приняли муссор, ошибка */
    if (umba_crc8_table(ptrBuff, len_ - sizeof(uint8_t)) != getCrc8(ptrBuff, len_)) {
      printf("\numba_crc8_table(ptrBuff, len_ - sizeof(uint8_t)) = %u\n", umba_crc8_table(ptrBuff, len_ - sizeof(uint8_t)));
      printf("getCrc8(ptrBuff, len_) = %u\n", getCrc8(ptrBuff, len_));
      return false;
    }
    return true;
  }
      
  ProtocolMaster::ProtocolMaster(i_transport::ITransport& transport, QCoreApplication* _coreApplication)
  :m_transport(transport) {
      m_coreApplication = _coreApplication;
  }

  uint8_t     buff[proto_max_buff]      = {0};
  uint8_t     recvdBuff[proto_max_buff] = {0};
  uint32_t    len                       = proto_min_buff;
  std::vector<uint8_t> uartCollectPkg;

  bool ProtocolMaster::sendCmdNOP(uint8_t addressTo){
      std::memset(buff, 0, sizeof(buff));
      std::memset(recvdBuff, 0, sizeof(recvdBuff));
      buff[0] = addressTo; //33; //addressTo;
      buff[1] = packLenMin; //4; //packLenMin;
      buff[2] = 0x0;
      buff[3] = umba_crc8_table(buff, 3); //247; //umba_crc8_table(buff, 3);
      /* Отправляем CmdNOP */
      assert(m_transport.sendData(buff, len));
      /* Ждем зеркало */
      std::this_thread::sleep_for(std::chrono::microseconds(1000));
      if (!m_transport.getData(recvdBuff, &len)) return false;
      if (buff[0] != getAddr(recvdBuff)) return false;
      if (buff[1] != getLen(recvdBuff)) return false;
      if (buff[2] != getCmd(recvdBuff)) return false;
      if (buff[3] != getCrc8(recvdBuff, getLen(recvdBuff))) return false;
      return true;
  }

  bool ProtocolMaster::parserRS(uint32_t recvdBuffSize, uint8_t dataFromFinger[][13], uint8_t dataSettingFromFinger[][12],
                      uint8_t* dataFromHandMount, uint8_t* resvdFromAllDev){

    uint8_t fingers_OK[7] = {1, 2, 4, 8, 16, 32, 64}; // ок, если ответ пришел
    bool getRSResponse = false;

    if (recvdBuffSize >= 13){
      //собираем пакет для пальцев в режиме NORM WORK
      for (uint32_t i = 0; i <= recvdBuffSize - 13; i++){
        if (recvdBuff[i] >= 0x11 && recvdBuff[i] <= 0x16 && recvdBuff[i + 1] == 13) {
          uint8_t tempArr[13] = {0};
          memcpy(tempArr, recvdBuff + i, sizeof(tempArr));
          if (parser(tempArr, sizeof(tempArr), tempArr[0])){
            memcpy(&dataFromFinger[tempArr[0] - 0x11][0], tempArr, sizeof(tempArr));
            *resvdFromAllDev |= fingers_OK[tempArr[0] - 0x11]; //ответ пришел
            getRSResponse = true;
          } else {
            memset(&dataFromFinger[tempArr[0] - 0x11][0], 0, sizeof(tempArr));
            *resvdFromAllDev &= ~fingers_OK[tempArr[0] - 0x11]; //ответ НЕ пришел
            getRSResponse = false;

          }
        }
      }
    }

    if (recvdBuffSize >= 12){
      //собираем пакет для пальцев в режиме SETTING
      for (uint32_t i = 0; i <= recvdBuffSize - 12; i++){
        if (recvdBuff[i] >= 0x11 && recvdBuff[i] <= 0x16 && recvdBuff[i + 1] == 12) {
          uint8_t tempArr[12] = {0};
          memcpy(tempArr, recvdBuff + i, sizeof(tempArr));
          if (parser(tempArr, sizeof(tempArr), tempArr[0])){
            memcpy(&dataSettingFromFinger[tempArr[0] - 0x11][0], tempArr, sizeof(tempArr));
            *resvdFromAllDev |= fingers_OK[tempArr[0] - 0x11]; //ответ пришел
            getRSResponse = true;
          } else {
            memset(&dataSettingFromFinger[tempArr[0] - 0x11][0], 0, sizeof(tempArr));
            *resvdFromAllDev &= ~fingers_OK[tempArr[0] - 0x11]; //ответ НЕ пришел
          }
        }
      }
    }

    if (recvdBuffSize >= 5){
      //собираем пакет для hand_mount
      for (uint32_t i = 0; i <= recvdBuffSize - 5; i++){
        if (recvdBuff[i] == 0x31 && recvdBuff[i + 1] == 0x05) {
          uint8_t tempArr[5] = {0};
          memcpy(tempArr, recvdBuff + i, sizeof(tempArr));
          if (parser(tempArr, sizeof(tempArr), tempArr[0])){
            memcpy(dataFromHandMount, tempArr, sizeof(tempArr));
            *resvdFromAllDev |= fingers_OK[6]; //ответ пришел
            getRSResponse = true;
          } else {
            memset(dataFromHandMount, 0, sizeof(tempArr));
            *resvdFromAllDev &= ~fingers_OK[6]; //ответ НЕ пришел
          }
        }
      }
    }

    return getRSResponse;
  }

  bool ProtocolMaster::RSRead(uint8_t dataFromFinger[][13], uint8_t* dataFromHandMount, 
                              uint8_t dataSettingFromFinger[][12], uint8_t* resvdFromAllDev){

    std::memset(buff,             0, sizeof(buff));
    std::memset(recvdBuff,        0, sizeof(recvdBuff));
    uint32_t not_bytes_received = 0;

    while (1){
      m_coreApplication->processEvents();
      m_transport.handleReadyRead();

      //get bytes
      std::memset(recvdBuff, 0, sizeof(recvdBuff));
      uint32_t recvdBuffSize = 0;
      bool get_bytes = m_transport.getData(recvdBuff, &recvdBuffSize);

      if (get_bytes){
        not_bytes_received = 0;
      } else {
        not_bytes_received++;
        if (not_bytes_received > 1){
          std::cout << "\nRS not_bytes_received > 1\n";
          return false;
        }
        continue;
      }       

      if (parserRS(recvdBuffSize, dataFromFinger, dataSettingFromFinger, dataFromHandMount, resvdFromAllDev)) {
        return true;
      } else{
        return false;
      }
    }
  }

  bool ProtocolMaster::parserUART(uint8_t* dataFrom, uint32_t* dataFromSize){
    
    if (uartCollectPkg.empty()) return false;
    uint32_t uartCollectPkgSize = uartCollectPkg.size();

    if (uartCollectPkgSize >= 9){
      //собираем пакет для норм состояния
      for (uint32_t i = 0; i <= uartCollectPkgSize - 9; i++){
        if (uartCollectPkg[i] == 0x00 && uartCollectPkg[i + 1] == 0x09 && uartCollectPkg[i + 2] == 0x00) {
          uint8_t tempArr[9] = {0};
          for (uint32_t j = 0; j < sizeof(tempArr); j++){
            tempArr[j] = uartCollectPkg[j];
          }
          if (parser(tempArr, sizeof(tempArr), tempArr[0])){
            memcpy(dataFrom, tempArr, sizeof(tempArr));
            uartCollectPkg.erase(uartCollectPkg.begin() + i, uartCollectPkg.begin() + i + sizeof(tempArr));
            *dataFromSize = sizeof(tempArr);
            return true;
          } else {
            memset(dataFrom, 0, sizeof(tempArr));
            *dataFromSize = 0;
          }
        }
      }
    }

    if (uartCollectPkgSize >= 5){
      uartCollectPkgSize = uartCollectPkg.size();
      //собираем пакет для норм shutDown
      for (uint32_t i = 0; i <= uartCollectPkgSize - 5; i++){
        if (uartCollectPkg[i] == 0x00 && uartCollectPkg[i + 1] == 0x05 && uartCollectPkg[i + 2] == 0x70) {
          uint8_t tempArr[5] = {0};
          for (uint32_t j = 0; j < sizeof(tempArr); j++){
            tempArr[j] = uartCollectPkg[j];
          }
          if (parser(tempArr, sizeof(tempArr), tempArr[0])){
            memcpy(dataFrom, tempArr, sizeof(tempArr));
            uartCollectPkg.erase(uartCollectPkg.begin() + i, uartCollectPkg.begin() + i + sizeof(tempArr));
            *dataFromSize = sizeof(tempArr);
            return true;
          } else {
            memset(dataFrom, 0, sizeof(tempArr));
            *dataFromSize = 0;
          }
        }
      }
    }

    return false;
  }

  bool ProtocolMaster::sendCmdReadUART(uint8_t addressTo, uint8_t* dataFrom, uint32_t* dataFromSize, 
          bool& getResponse, bool wait_response, uint8_t cam_status){
      
    std::memset(buff,             0, sizeof(buff));
    std::memset(recvdBuff,        0, sizeof(recvdBuff));
    std::memset(dataFrom,         0, 9);
    *dataFromSize               = 0;
    uint32_t not_bytes_received = 0;

    while (1){
      m_coreApplication->processEvents();
      m_transport.handleReadyRead();

      //get bytes
      std::memset(recvdBuff, 0, sizeof(recvdBuff));
      uint32_t recvdBuffSize = 0;
      bool get_bytes = m_transport.getData(recvdBuff, &recvdBuffSize);

      if (get_bytes){
        not_bytes_received = 0;
        for (uint32_t i = 0; i < recvdBuffSize; i++){
          uartCollectPkg.push_back(recvdBuff[i]);
        }

      } else if (!uartCollectPkg.empty()){
        not_bytes_received = 0;

      } else if (!get_bytes && !wait_response){
        getResponse = !wait_response;
        return false;

      } else {
        not_bytes_received++;
        if (not_bytes_received > 10){
          getResponse = false;
          std::memset(dataFrom, 0, 9);
          *dataFromSize = 0;
          std::cout << "UART not_bytes_received > 10\n";
          return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));  //sum 20 ms
        continue;
      }
      
      if (parserUART(dataFrom, dataFromSize)) {
        return true;
      } else return false;
    }
  }

  bool ProtocolMaster::sendCmdRead(uint8_t addressTo, uint8_t cmd){
    std::memset(buff, 0, sizeof(buff));
    std::memset(recvdBuff, 0, sizeof(recvdBuff));
    buff[0] = addressTo;
    buff[1] = packLenMin;
    buff[2] = cmd;
    buff[3] = umba_crc8_table(buff, 3);
    /* Отправляем CmdRead */
    if (m_transport.sendData(buff, (uint32_t)buff[1])) return true;
    else return false;
  }

  bool ProtocolMaster::sendCmdWriteRS(uint8_t addressTo, const uint8_t* dataTo, uint32_t dataToSize){ // dataTo включает cmd
    std::memset(buff, 0, sizeof(buff));
    std::memset(recvdBuff, 0, sizeof(recvdBuff));
    buff[0] = addressTo;                                                              // add
    buff[1] = packLenMin + (uint8_t)dataToSize - sizeof(uint8_t);                     // len 4 + 6 - 1 = 9
    memcpy(buff + 2, dataTo, dataToSize);                                             // cmd + data
    buff[packLenMin + (uint8_t)dataToSize - 2 * sizeof(uint8_t)] = umba_crc8_table(buff,packLenMin + (uint8_t)dataToSize - 2 * sizeof(uint8_t));
    /* Отправляем CmdWrite */
    if (m_transport.sendData(buff, (uint32_t)buff[1])) return true;
    else return false;
  }

  bool ProtocolMaster::sendCmdWriteUART(uint8_t addressTo, uint8_t cmd, const uint8_t* dataTo, uint32_t dataToSize){
    std::memset(buff, 0, sizeof(buff));
    std::memset(recvdBuff, 0, sizeof(recvdBuff));
    buff[0] = addressTo;
    buff[1] = packLenMin + (uint8_t)dataToSize;
    buff[2] = cmd;
    memcpy(buff + 3, dataTo, dataToSize);
    buff[packLenMin + (uint8_t)dataToSize - sizeof(uint8_t)] = umba_crc8_table(buff,packLenMin + (uint8_t)dataToSize - sizeof(uint8_t));
    /* Отправляем CmdWrite */
    if (m_transport.sendData(buff, (uint32_t)buff[1])) return true;
    else return false;
  }

} //namespace protocol_master


