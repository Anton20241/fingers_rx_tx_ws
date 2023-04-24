#pragma once

#include "i_transport.hpp"
#include <QByteArray>
#include <QSerialPort>
#include <QTextStream>
#include <mutex>

namespace qt_serial{

  class Qt_Serial_Async : public i_transport::ITransport, public QObject
  {
  public:
    Qt_Serial_Async(std::string port_, int boudrate);
    bool sendData(const uint8_t* ptrData, uint32_t len);
    bool getData(uint8_t* ptrData, uint32_t* lenInOut);
    bool transportReset();
    ~Qt_Serial_Async();

  private slots:
    void handleReadyRead();
    void handleError(QSerialPort::SerialPortError error);

  private:
    QSerialPort m_serialPort;
    QByteArray m_readData;
    QTextStream m_standardOutput;
    
    std::vector<uint8_t> m_copyRecvdData;
    uint32_t m_sendCount = 0;
    uint32_t m_recvdCount = 0;
    std::mutex my_mytex;
    uint32_t bytesGet = 0;
  };
}

