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
    
    ProtocolMaster::ProtocolMaster(i_transport::ITransport& transport)
    :m_transport(transport) {}

    uint8_t     buff[proto_max_buff] = {0};
    uint8_t     recvdBuff[proto_max_buff] = {0};
    uint32_t    len = packLenMin;

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

    bool ProtocolMaster::sendCmdRead(uint8_t addressTo){
        std::memset(buff, 0, sizeof(buff));
        std::memset(recvdBuff, 0, sizeof(recvdBuff));
        buff[0] = addressTo;
        buff[1] = packLenMin;
        buff[2] = 0x1;
        buff[3] = umba_crc8_table(buff, 3);
        /* Отправляем CmdRead */
        assert(m_transport.sendData(buff, len));
        /* Ждем DATA */
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
        if (!m_transport.getData(recvdBuff, &len)) return false;
        if (buff[0] != recvdBuff[0]) return false;
        return true;
    }

    bool ProtocolMaster::sendCmdWrite(uint8_t addressTo, const uint8_t* data, uint32_t dataSize){
        std::memset(buff, 0, sizeof(buff));
        std::memset(recvdBuff, 0, sizeof(recvdBuff));
        buff[0] = addressTo;
        buff[1] = len + dataSize;
        buff[2] = 0x2;
        uint8_t m_len = 3;
        for (; m_len < dataSize; m_len++){
            buff[m_len] = data[m_len - 3];
        }
        buff[m_len] = umba_crc8_table(buff, buff[1] - 1);
        /* Отправляем CmdWrite */
        assert(m_transport.sendData(buff, len));
        return true;
    }

    bool ProtocolMaster::sendCmdReadWrite(uint8_t addressTo, const uint8_t* data, uint32_t dataSize){
        std::memset(buff, 0, sizeof(buff));
        std::memset(recvdBuff, 0, sizeof(recvdBuff));
        buff[0] = addressTo;
        buff[1] = len + dataSize;
        buff[2] = 0x3;
        uint8_t m_len = 3;
        for (; m_len < dataSize; m_len++){
            buff[m_len] = data[m_len - 3];
        }
        buff[m_len] = umba_crc8_table(buff, buff[1] - 1);
        /* Отправляем CmdReadWrite */
        assert(m_transport.sendData(buff, m_len));
        /* Ждем DATA */
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
        if (!m_transport.getData(recvdBuff, &len)) return false;
        if (buff[0] != recvdBuff[0]) return false;
        return true;
    }

    bool ProtocolMaster::sendSomeCmd(const uint8_t* data, uint32_t dataSize, uint8_t* bufToTopic, uint32_t bufToTopicSize){
        std::memset(buff, 0, sizeof(buff));
        std::memset(recvdBuff, 0, sizeof(recvdBuff));
        memcpy(buff, data, dataSize);
        buff[dataSize] = umba_crc8_table(buff, dataSize);
        /* Отправляем SomeCmd */
        assert(m_transport.sendData(buff, dataSize + sizeof(uint8_t)));
        /* Ждем DATA */
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
        if (getCmd(buff) == cmdWrite) return true;
        if (!m_transport.getData(recvdBuff, &len)) return false;
        if (getCmd(buff) == cmdNop){
            if (buff[0] != getAddr(recvdBuff)) return false;
            if (buff[1] != getLen(recvdBuff)) return false;
            if (buff[2] != getCmd(recvdBuff)) return false;
            if (buff[3] != getCrc8(recvdBuff, getLen(recvdBuff))) return false;
        }
        else if (buff[0] != getAddr(recvdBuff)) return false;
        bufToTopicSize = getLen(recvdBuff);
        memcpy(bufToTopic, recvdBuff, bufToTopicSize);
        
        return true;
    }
}