/*
  Класс Qt_Serial_Async предназначен для реализации функционала обмена данными между программой и устройством по Serial порту.
  Класс Qt_Serial_Async включает следующие методы:
  * sendData()        - отправка данных по Serial порту
  * getData()         - обработка данных, полученных по Serial порту
  * transportReset()  - переподключение к устройству по Serial порту (НЕ РЕАЛИЗОВАНО и ПОКА НЕ ИСПОЛЬЗУЕТСЯ)
  * handleReadyRead() - обработчик сообщений от Serial порта 
  * handleError()     - обработчик ошибок    от Serial порта 
*/

#pragma once

#include "i_transport.hpp"
#include <QByteArray>
#include <QSerialPort>
#include <QTextStream>
#include <mutex>
#include <boost/chrono.hpp>
#include <QCoreApplication>

namespace qt_serial{

  class Qt_Serial_Async : public i_transport::ITransport, public QObject
  {
  public:
    Qt_Serial_Async(std::string port_, std::string boudrate);
    bool sendData(const uint8_t* ptrData, uint32_t len);
    bool getData(uint8_t* ptrData, uint32_t* lenInOut);
    bool transportReset();
    void handleReadyRead();
    void handleError(QSerialPort::SerialPortError error);
    ~Qt_Serial_Async();

  private:
    QSerialPort m_serialPort;
    QByteArray m_readData;
    QTextStream m_standardOutput;
    QCoreApplication* m_coreApplication;
    bool sendDataProcess = false;
    
    std::vector<uint8_t> m_copyRecvdData;
    uint32_t m_sendCount = 0;
    uint32_t m_recvdCount = 0;
    uint32_t timeFailCount = 0;
    std::mutex my_mytex;
    uint32_t bytesGet = 0;
    boost::chrono::system_clock::time_point from_send_to_get_tp = boost::chrono::system_clock::now();
  };
}

