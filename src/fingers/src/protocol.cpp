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

    bool ProtocolMaster::parser(uint8_t* ptrBuff, uint32_t len)
    {
        /* Если адрес не валидный, ошибка */
        if (getAddr(ptrBuff) != 0x00) {
            return false;
        }
        /* Если длина пакета не валидная, ошибка */
        if (getLen(ptrBuff) != len) {
            return false;
        }
        if (getLen(ptrBuff) != 5 && getLen(ptrBuff) != 8) {
            return false;
        }
        /* Если команды не соответствуют пакетам, ошибка*/
        if (getLen(ptrBuff) == 8 && getCmd(ptrBuff) != 0x00) {
            return false;
        }
        if (getLen(ptrBuff) == 5 && getCmd(ptrBuff) != 0x70) {
            return false;
        }
        /* Если контрольная сумма не совпадает, приняли муссор, ошибка */
        if (umba_crc8_table(ptrBuff, len - sizeof(uint8_t)) != getCrc8(ptrBuff, len)) {
            return false;
        }
        return true;
    }
    
    ProtocolMaster::ProtocolMaster(i_transport::ITransport& transport)
    :m_transport(transport) {}

    uint8_t     buff[proto_max_buff] = {0};
    uint8_t     recvdBuff[proto_max_buff] = {0};
    uint32_t    len = packLenMin;

    bool ProtocolMaster::sendCmdNOP(uint8_t addressTo){
        std::memset(buff, 0, sizeof(buff));
        std::memset(recvdBuff, 0, sizeof(recvdBuff));
        len = packLenMin;
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

    bool ProtocolMaster::sendCmdRead(uint8_t addressTo, uint8_t* dataFrom, uint32_t* dataFromSize){
        std::memset(buff, 0, sizeof(buff));
        std::memset(recvdBuff, 0, sizeof(recvdBuff));
        std::memset(dataFrom, 0, *dataFromSize);
        len = packLenMin;
        buff[0] = addressTo;
        buff[1] = packLenMin;
        buff[2] = 0x1;
        buff[3] = umba_crc8_table(buff, 3);
        /* Отправляем CmdRead */
        assert(m_transport.sendData(buff, len));
        /* Ждем DATA */
        std::this_thread::sleep_for(std::chrono::microseconds(300));
        if (!m_transport.getData(dataFrom, dataFromSize)) return false;
        return true;
    }

    bool ProtocolMaster::sendCmdReadUART(uint8_t addressTo, uint8_t* dataFrom, 
            uint32_t* dataFromSize, bool& getResponse, bool wait_response, uint8_t cam_status){
        std::memset(buff, 0, sizeof(buff));

        
        bool byteIsGetBefore = false;
        uint32_t not_response_on_request = 0;
        uint32_t not_bytes_received = 0;

        while (1){
            bool pkgIsReadyForParsing = false;

            //get bytes
            std::memset(recvdBuff, 0, sizeof(recvdBuff));
            uint32_t recvdBuffSize = 0;
            m_transport.getData(recvdBuff, &recvdBuffSize);

            if (!byteIsGetBefore && (recvdBuffSize == 0) && !wait_response){
                getResponse = !wait_response;
                return false;

            } else if ((!byteIsGetBefore && (recvdBuffSize == 0) && wait_response) ||
                            (byteIsGetBefore && (recvdBuffSize == 0))){
                not_bytes_received++;
                if (not_bytes_received > 6){
                    getResponse = false;
                    return false;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
                
            } else if (recvdBuffSize != 0){
                not_bytes_received = 0;
                byteIsGetBefore = true;
                collectPkg(recvdBuff, recvdBuffSize, dataFrom, *dataFromSize, pkgIsReadyForParsing);
            }

            if (!pkgIsReadyForParsing){
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }

            if (parser(dataFrom, *dataFromSize)) return true;

            not_response_on_request++;
            if (not_response_on_request > 5){
                getResponse = false;
                return false;
            }
            sendCmdWrite(0x01, 0x10, &cam_status, sizeof(uint8_t));
            
            memset(dataFrom, 0, *dataFromSize);
            *dataFromSize = 0;
            wait_response = true;
            byteIsGetBefore = false;
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    void ProtocolMaster::collectPkg(uint8_t* resvdData, uint32_t resvdBytes, 
            uint8_t* dataUart, uint32_t& dataUartSize, bool pkgIsReadyForParsing){

        memcpy(dataUart + dataUartSize, resvdData, resvdBytes);
        dataUartSize += resvdBytes;
        if (dataUartSize >= 8) pkgIsReadyForParsing = true;
        if (dataUartSize >= 5 && dataUart[1] == 5) pkgIsReadyForParsing = true;
    }

    // bool ProtocolMaster::parserOk(uint8_t* dataUart, uint32_t& dataUartSize){
    //     if ((dataUartSize != 8) && (dataUartSize != 5))                                                     return false;
    //     if (dataUart[0] != 0)                                                                               return false;
    //     if ((dataUartSize == 8 && dataUart[1] != 8) || (dataUartSize == 5 && dataUart[1] != 5))             return false;
    //     if ((dataUartSize == 8 && dataUart[2] != 0) || (dataUartSize == 5 && dataUart[2] != 0x70))          return false;
    //     uint8_t crc_8 = umba_crc8_table(dataUart, dataUartSize - sizeof(uint8_t));
    //     if (dataUart[dataUartSize - sizeof(uint8_t)] != crc_8)                                              return false;
    //     return true;
    // }

    // void ProtocolMaster::updateDataUart(uint8_t* dataUart, uint32_t& dataUartSize){
    //     if ((dataUartSize <= 2) && (dataUart[0] == 0))                                                          return;
    //     if ((dataUartSize < 8 && dataUart[1] == 8) || (dataUartSize < 5 && dataUart[1] == 5))                   return;
    //     if ((dataUartSize < 8 && dataUart[2] == 0) || (dataUartSize < 5 && dataUart[2] == 0x70))                return;
    //     std::cout << "\n\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!NON-Valid MESSAGE:\n";
    //     for (int i = 0; i < dataUartSize; i++){
    //         printf("[%u]", dataUart[i]);
    //     }
    //     std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
    //     memset(dataUart, 0, dataUartSize);
    //     dataUartSize = 0;
    // }
    
    bool ProtocolMaster::sendCmdWrite(uint8_t addressTo, uint8_t cmd, const uint8_t* dataTo, uint32_t dataToSize){
        std::memset(buff, 0, sizeof(buff));
        std::memset(recvdBuff, 0, sizeof(recvdBuff));
        buff[0] = addressTo;
        buff[1] = len + dataToSize;
        buff[2] = cmd;
        memcpy(buff + 3, dataTo, dataToSize);
        buff[len + dataToSize - sizeof(uint8_t)] = umba_crc8_table(buff,len + dataToSize - sizeof(uint8_t));
        /* Отправляем CmdWrite */
        assert(m_transport.sendData(buff, buff[1]));
        return true;
    }

    bool ProtocolMaster::sendCmdReadWrite(uint8_t addressTo, uint8_t cmd, const uint8_t* dataTo, uint32_t dataToSize, 
                                        uint8_t* dataFrom, uint32_t* dataFromSize){
        std::memset(buff, 0, sizeof(buff));
        std::memset(recvdBuff, 0, sizeof(recvdBuff));
        std::memset(dataFrom, 0, *dataFromSize);
        buff[0] = addressTo;
        buff[1] = len + dataToSize;
        buff[2] = cmd;
        memcpy(buff + 3, dataTo, dataToSize);
        buff[len + dataToSize - sizeof(uint8_t)] = umba_crc8_table(buff,len + dataToSize - sizeof(uint8_t));
        /* Отправляем CmdReadWrite */
        assert(m_transport.sendData(buff, buff[1]));
        /* Ждем DATA */
        std::this_thread::sleep_for(std::chrono::microseconds(300));
        if (!m_transport.getData(dataFrom, dataFromSize)) return false;
        return true;
    }

    bool ProtocolMaster::sendSomeCmd(const uint8_t* dataTo, uint32_t dataToSize, uint8_t* dataFrom, uint32_t* dataFromSize){
        std::memset(buff, 0, sizeof(buff));
        std::memset(recvdBuff, 0, sizeof(recvdBuff));
        std::memset(dataFrom, 0, *dataFromSize);
        memcpy(buff, dataTo, dataToSize);
        buff[dataToSize] = umba_crc8_table(buff, dataToSize);
        /* Отправляем SomeCmd */
        assert(m_transport.sendData(buff, dataToSize + sizeof(uint8_t)));
        /* Ждем DATA */
        std::this_thread::sleep_for(std::chrono::microseconds(300));
        if (getCmd(buff) == cmdWrite) return true;
        if (!m_transport.getData(dataFrom, dataFromSize)) return false;
        if (getCmd(buff) == cmdNop){
            if (buff[0] != getAddr(dataFrom)) return false;
            if (buff[1] != getLen(dataFrom)) return false;
            if (buff[2] != getCmd(dataFrom)) return false;
            if (buff[3] != getCrc8(dataFrom, getLen(dataFrom))) return false;
        }
        return true;
    }

} //namespace protocol_master
