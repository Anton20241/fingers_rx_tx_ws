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

  // void show_array(QByteArray& arr) {
  //   QByteArray arr_to_show = arr.toHex('|');
  //   for (size_t i = 0; i < arr_to_show.size(); i++)
  //   {
  //     std::cout << arr_to_show.at(i);
  //   }
  //   std::cout << " arr size = "<< arr.size() << std::endl; 
  // }

  void Qt_Serial_Async::handleReadyRead(){  

    my_mytex.lock();
    m_readData.clear();
    m_readData.append(m_serialPort.read(32));    
    m_recvdCount++;

    if(send_error) {
      m_copyRecvdData.clear();
      send_error = false;
    }
    //cout << " m_readData.size(): = "<<  m_readData.size() << endl;
    add_to_vector(m_readData, m_copyRecvdData);
    
    printf("\n[RECEIVED]:\n");
    for (size_t i = 0; i < m_copyRecvdData.size(); i++)
    {
      printf("[%u]", m_copyRecvdData[i]);
    }
    std::cout << std::endl;
    cout << "bytes_transferred: "<< m_readData.size() << endl;

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

  Qt_Serial_Async::Qt_Serial_Async(std::string port_, int boudrate):
          m_standardOutput(stdout)
  {
    const QString serialPortName = QString::fromStdString(port_);
    m_serialPort.setPortName(serialPortName);

    if(!m_serialPort.setBaudRate(boudrate))
      qDebug() << m_serialPort.errorString();

    if(!m_serialPort.setDataBits(QSerialPort::Data8))
      qDebug()<<m_serialPort.errorString();

    if(!m_serialPort.setParity(QSerialPort::NoParity))
      qDebug()<<m_serialPort.errorString();

    if(!m_serialPort.setStopBits(QSerialPort::OneStop))
      qDebug()<<m_serialPort.errorString();

    if(!m_serialPort.setFlowControl(QSerialPort::NoFlowControl))
      qDebug()<<m_serialPort.errorString();

    if (!m_serialPort.open(QIODevice::ReadWrite)) {
      m_standardOutput << QObject::tr("Failed to open port %1, error: %2")
              .arg(serialPortName)
              .arg(m_serialPort.errorString())
                    << endl;
      exit(-1);
    }

    connect(&m_serialPort, &QSerialPort::readyRead, this, &Qt_Serial_Async::handleReadyRead);
    connect(&m_serialPort, &QSerialPort::errorOccurred, this, &Qt_Serial_Async::handleError);
  }

  bool Qt_Serial_Async::sendData(const uint8_t* ptrData, uint32_t len)
  {
    QByteArray m_sendData;
    
    for (size_t i = 0; i < len; i++){
      m_sendData.push_back(ptrData[i]);
    }

    uint64_t sendBytes = m_serialPort.write(m_sendData);
    if(sendBytes > 0){
      m_sendCount++;
      printf("\n[SEND]:\n");
      for (size_t i = 0; i < sendBytes; i++)
      {
          printf("[%u]", m_sendData.at(i));
      }
      std::cout << std::endl;
      cout << "sendBytes: "<< sendBytes << endl;
      return true;
    } else {
      std::cout << "error.what()\n";
      return false;
    }  
  }

  bool Qt_Serial_Async::getData(uint8_t* ptrData, uint32_t* lenInOut)
  {
    my_mytex.lock();
    if (m_copyRecvdData.size() > 32) {
      m_copyRecvdData.clear();
      std::cout << "\n!!!SIZE > 32!!!" << std::endl;
      my_mytex.unlock();
      return false;
    }

    // std::cout << "bytesGet = " << bytesGet << std::endl;
    // std::cout << "m_copyRecvdData.size() = " << m_copyRecvdData.size() << std::endl;

    if (m_copyRecvdData.size() != bytesGet || m_copyRecvdData.empty()){
      //std::cout << "empty or not BytesGet\n";
      bytesGet = m_copyRecvdData.size();
      my_mytex.unlock();
      return false;
    }

    if (m_copyRecvdData.size() < 2 || m_copyRecvdData[1] > 13 || m_copyRecvdData[1] == 0){
      std::cout << "non-valid m_copyRecvdData length\n";
      m_copyRecvdData.clear();
      my_mytex.unlock();
      return true;
    }
    
    uint32_t packageLen           = m_copyRecvdData[1];
    uint32_t m_copyRecvdData_size = m_copyRecvdData.size();

    if (packageLen > m_copyRecvdData_size) {
      my_mytex.unlock();
      return false;
    }     
       
    std::cout << "m_copyRecvdDataSZ: = " << packageLen << std::endl;
    std::cout << "m_copyRecvdData:\n";
    for (int i = 0; i < packageLen; i++){
      ptrData[i] = m_copyRecvdData[i];
      printf("[%u]", m_copyRecvdData[i]);
    }
    std::cout << std::endl;        

    *lenInOut = packageLen;
    std::cout << "*lenInOut = " << *lenInOut << std::endl;

    auto position = m_copyRecvdData.begin();
    auto begin = m_copyRecvdData.begin() + packageLen;
    auto end = begin + m_copyRecvdData.size() - packageLen;
    // std::cout << "\ninsert data: ";
    // for (auto iter = begin; iter != end; iter++) {
    //     printf("[%u]",*iter);
    // }
    
    m_copyRecvdData.insert(position, begin, end);
    

    //std::cout << "\npackageLen = " << packageLen << std::endl;
    //std::cout << "m_copyRecvdData.size() = " <<  m_copyRecvdData.size() << std::endl;

    m_copyRecvdData.erase(m_copyRecvdData.begin(), m_copyRecvdData.begin() + m_copyRecvdData_size);

    //std::cout << "after erase() m_copyRecvdData.size() = " << m_copyRecvdData.size() << std::endl;

    std::cout << "m_copyRecvdData:\n";
    for (int i = 0; i < m_copyRecvdData.size(); i++){
      printf("[%u]", m_copyRecvdData[i]);
    }
    std::cout << std::endl;    

    bytesGet = 0;

    my_mytex.unlock();

    return true;
  }

  bool Qt_Serial_Async::transportReset() {return true;}

  Qt_Serial_Async::~Qt_Serial_Async() {
    m_serialPort.close();
  };
}