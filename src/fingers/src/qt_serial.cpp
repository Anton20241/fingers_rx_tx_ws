#include "qt_serial.hpp"


#include <QCoreApplication>
#include <QDebug>
#include <iostream>
#include <thread>

using namespace std;

namespace qt_serial{

  inline std::vector<uint8_t> to_vector(const QByteArray& arr) {
    return std::vector<uint8_t> (arr.begin(),arr.end());
  }

  inline void add_to_vector(const QByteArray& arr, std::vector<uint8_t> &vec) {
    std::vector<uint8_t> add_vec = to_vector(arr);
    vec.insert(vec.end(),add_vec.begin(),add_vec.end());
  }

  void Qt_Serial_Async::handleReadyRead(){
    
    if (sendDataProcess) return;
    my_mytex.lock();
    m_readData.clear();
    m_readData.append(m_serialPort.readAll());
    m_serialPort.clear(QSerialPort::AllDirections);  
    m_recvdCount++;

    if(send_error) {
      m_copyRecvdData.clear();
      send_error = false;
    }

    add_to_vector(m_readData, m_copyRecvdData);
    
    // printf("\n[RECEIVED]:\n");
    // for (size_t i = 0; i < m_copyRecvdData.size(); i++)
    // {
    //   printf("[%u]", m_copyRecvdData[i]);
    // }
    // std::cout << std::endl;
    // cout << "bytes_transferred: "<< m_readData.size() << endl;

    my_mytex.unlock();
  }

  void Qt_Serial_Async::handleError(QSerialPort::SerialPortError serialPortError)
  {
    if (serialPortError == QSerialPort::ReadError) {
      m_standardOutput << QObject::tr("An I/O error occurred while reading "
                                      "the data from port %1, error: %2")
              .arg(m_serialPort.portName())
              .arg(m_serialPort.errorString())
                      << endl;
      QCoreApplication::exit(1);
    }
  }

  Qt_Serial_Async::Qt_Serial_Async(std::string port_, std::string boudrate):
          m_standardOutput(stdout)
  {
    const QString serialPortName = QString::fromStdString(port_);
    m_serialPort.setPortName(serialPortName);

    if(!m_serialPort.setBaudRate(std::stoi(boudrate)))
      qDebug() << m_serialPort.errorString();

    if(!m_serialPort.setDataBits(QSerialPort::Data8))
      qDebug()<<m_serialPort.errorString();

    if(!m_serialPort.setParity(QSerialPort::NoParity))
      qDebug()<<m_serialPort.errorString();

    if(!m_serialPort.setStopBits(QSerialPort::OneStop))
      qDebug()<<m_serialPort.errorString();

    if(!m_serialPort.setFlowControl(QSerialPort::NoFlowControl))
      qDebug()<<m_serialPort.errorString();

    bool failStart = true;
    while (!m_serialPort.open(QIODevice::ReadWrite)) {
      if (failStart) {
        failStart = false;
        m_standardOutput << QObject::tr("Failed to open port %1, error: %2")
                .arg(serialPortName)
                .arg(m_serialPort.errorString())
                      << endl;
      }
    }
    printf("[SUCCESS PORT OPEN]\n");

    m_readData.clear();
    m_readData.append(m_serialPort.readAll());
    m_readData.clear();
    m_serialPort.clear(QSerialPort::AllDirections);

    connect(&m_serialPort, &QSerialPort::readyRead, this, &Qt_Serial_Async::handleReadyRead);
    connect(&m_serialPort, &QSerialPort::errorOccurred, this, &Qt_Serial_Async::handleError);
  }

  bool Qt_Serial_Async::sendData(const uint8_t* ptrData, uint32_t len)
  {
    sendDataProcess = true;
    QByteArray m_sendData;
    
    for (size_t i = 0; i < len; i++){
      m_sendData.push_back(ptrData[i]);
    }

    uint64_t sendBytes = 0;
    
    while (!sendBytes){
      sendBytes = m_serialPort.write(m_sendData);
      m_coreApplication->processEvents();
    }

    from_send_to_get_tp = boost::chrono::system_clock::now();
    if(sendBytes > 0){
      m_sendCount++;
      printf("\n[SEND]:\n");
      for (size_t i = 0; i < sendBytes; i++) {
        printf("[%u]", (uint8_t)m_sendData.at(i));
      }
      std::cout << std::endl;
      cout << "sendBytes: "<< sendBytes << endl;
      sendDataProcess = false;
      return true;
    } else {
      // std::cout << "error.what()\n";
      sendDataProcess = false;
      return false;
    }  
    
  }

  bool Qt_Serial_Async::getData(uint8_t* ptrData, uint32_t* lenInOut)
  {
    my_mytex.lock();

    if (m_copyRecvdData.size() != bytesGet || m_copyRecvdData.empty()){
      // std::cout << "m_copyRecvdData.size() = " << m_copyRecvdData.size() << "\n";
      // std::cout << "bytesGet = " << bytesGet << "\n";
      bytesGet = m_copyRecvdData.size();
      my_mytex.unlock();
      return false;
    }

    if (m_copyRecvdData.size() > 100) {
      m_copyRecvdData.clear();
      //std::cout << "\n!!!SIZE > 100!!!" << std::endl;
      my_mytex.unlock();
      return false;
    }

    std::cout << "m_copyRecvdData:";
    for (int i = 0; i < m_copyRecvdData.size(); i++){
      if (i % 13 == 0) std::cout << std::endl;
      ptrData[i] = m_copyRecvdData[i];
      printf("[%u]", m_copyRecvdData[i]);
    }    
    std::cout << std::endl;        

    *lenInOut = m_copyRecvdData.size();

    bytesGet = 0;
    m_copyRecvdData.clear();
    my_mytex.unlock();

    return true;
  }

  bool Qt_Serial_Async::transportReset() {return true;}

  Qt_Serial_Async::~Qt_Serial_Async() {
    m_serialPort.close();
  };
}