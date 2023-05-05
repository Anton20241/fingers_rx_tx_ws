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

    static const uint32_t proto_max_buff    = 100; //????

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

    bool ProtocolMaster::parser(uint8_t* ptrBuff, uint32_t len, uint8_t addressTo)
    {
        /* Если адрес не валидный, ошибка */
        if (getAddr(ptrBuff) != addressTo) {
            printf("\ngetAddr(ptrBuff) = %u\n", getAddr(ptrBuff));
            printf("addressTo = %u\n", addressTo);
            std::cout << "\ngetAddr(ptrBuff)\n";
            return false;
        }
        /* Если длина пакета не валидная, ошибка */
        if (getLen(ptrBuff) != len) {
            std::cout << "\ngetLen(ptrBuff) != len\n";
            return false;
        }
        if (getLen(ptrBuff) != 4 && getLen(ptrBuff) != 5 && getLen(ptrBuff) != 6  && getLen(ptrBuff) != 8 && getLen(ptrBuff) != 9 && getLen(ptrBuff) != 13) {
            std::cout << "\ngetLen(ptrBuff) != 5\n";
            return false;
        }
        /* Если команды не соответствуют пакетам, ошибка*/
        if (getLen(ptrBuff) == 8 && getCmd(ptrBuff) != 0x00) {
            std::cout << "\ngetLen(ptrBuff) == 8\n";
            return false;
        }
        if (getLen(ptrBuff) == 5 && !(getCmd(ptrBuff) == 0x70 || getCmd(ptrBuff) == 0x30)) {
            std::cout << "\ngetLen(ptrBuff) == 5\n";
            return false;
        }
        /* Если контрольная сумма не совпадает, приняли муссор, ошибка */
        if (umba_crc8_table(ptrBuff, len - sizeof(uint8_t)) != getCrc8(ptrBuff, len)) {
            std::cout << "\numba_crc8_table(ptrBuff, len \n";
            return false;
        }
        return true;
    }
        
    ProtocolMaster::ProtocolMaster(i_transport::ITransport& transport, QCoreApplication* _coreApplication)
    :m_transport(transport) {
        m_coreApplication = _coreApplication;
    }

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

    bool ProtocolMaster::parserRS(uint8_t* collectPckg, uint32_t collectPckgSize, uint8_t dataFrom[][13], uint8_t* resvdFromAllDev){
        uint8_t fingers_OK[7] = {1, 2, 4, 8, 16, 32, 64}; ////ок, если ответ пришел

        for (size_t i = 0; i <= collectPckgSize - 13; i++){
            if (collectPckg[i] >= 0x11 && collectPckg[i] <= 0x16 && collectPckg[i + 1] == 0x0D) {
                uint8_t tempArr[13] = {0};
                memcpy(tempArr, collectPckg + i, sizeof(tempArr));
                if (parser(tempArr, 13, tempArr[0])){
                    memcpy(&dataFrom[tempArr[0] - 0x11][0], tempArr, 13);
                    *resvdFromAllDev |= fingers_OK[tempArr[0] - 0x11]; //ответ пришел
                } else {
                    memset(&dataFrom[tempArr[0] - 0x11][0], 0, 13);
                    *resvdFromAllDev &= ~fingers_OK[tempArr[0] - 0x11]; //ответ НЕ пришел
                }
            }
        }
        if (*resvdFromAllDev != 0) return true;
        return false;
    }

    bool ProtocolMaster::RSRead(uint8_t dataFrom[][13], uint8_t* resvdFromAllDev){
        std::memset(buff, 0, sizeof(buff));
        std::memset(recvdBuff, 0, sizeof(recvdBuff));
        std::memset(dataFrom, 0, 13 * 6);
        *resvdFromAllDev = 0;

        uint32_t not_response_on_request        = 0;
        uint32_t not_bytes_received             = 0;
        uint8_t  collectPckg[proto_max_buff]    = {0};
        uint32_t collectPckgSize                = 0;

        while (1){
            m_coreApplication->processEvents();
            bool pkgIsReadyToParse = false;

            //get bytes
            std::memset(recvdBuff, 0, sizeof(recvdBuff));
            uint32_t recvdBuffSize = 0;
            bool get_bytes = m_transport.getData(recvdBuff, &recvdBuffSize);

            if (get_bytes){
                not_bytes_received = 0;
                // boost::chrono::system_clock::time_point cur_tp = boost::chrono::system_clock::now();
                // boost::chrono::duration<double> ex_time = cur_tp - first_tp;
                // std::cout << "Execution time: " << ex_time.count() << std::endl;
            } else {
                //std::cout << "else\n";
                not_bytes_received++;
                if (not_bytes_received > 190){
                    // std::cout << "not_bytes_received > 190\n";
                    return false;
                }
                std::this_thread::sleep_for(std::chrono::microseconds(5)); //sum 950us
                continue;
            }
            collectPkg(recvdBuff, recvdBuffSize, collectPckg, &collectPckgSize, pkgIsReadyToParse);
            
            if (parserRS(collectPckg, sizeof(collectPckg), dataFrom, resvdFromAllDev)) return true;
            // std::cout << "!parserRS()\n";
            clear(collectPckg, &collectPckgSize);
            return false;
        }
    }

    bool ProtocolMaster::sendCmdReadUART(uint8_t addressTo, uint8_t* dataFrom, 
            uint32_t* dataFromSize, bool& getResponse, bool wait_response, uint8_t cam_status){
        
        std::memset(buff, 0, sizeof(buff));
        std::memset(recvdBuff, 0, sizeof(recvdBuff));
        clear(dataFrom, dataFromSize);
        bool byteIsGetBefore = false;
        uint32_t not_response_on_request = 0;
        uint32_t not_bytes_received = 0;

        while (1){
            m_coreApplication->processEvents();
            bool pkgIsReadyToParse = false;

            //get bytes
            std::memset(recvdBuff, 0, sizeof(recvdBuff));
            uint32_t recvdBuffSize = 0;
            bool get_bytes = m_transport.getData(recvdBuff, &recvdBuffSize);

            if (get_bytes){
                //std::cout << "get_bytes\n";
                not_bytes_received = 0;
                byteIsGetBefore = true;
            
            } else if (!byteIsGetBefore && !get_bytes && !wait_response){
                //std::cout << "!byteIsGetBefore && !get_bytes && !wait_response\n";
                getResponse = !wait_response;
                return false;

            } else {
                //std::cout << "else\n";
                not_bytes_received++;
                if (not_bytes_received > 5){
                    //std::cout << "not_bytes_received > 10\n";
                    getResponse = false;
                    clear(dataFrom, dataFromSize);
                    return false;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                continue;
            }

            collectPkg(recvdBuff, recvdBuffSize, dataFrom, dataFromSize, pkgIsReadyToParse);
            if (!pkgIsReadyToParse){
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                continue;
            }

            if (parser(dataFrom, *dataFromSize, 0x00)) return true;
            std::cout << "[PARSER FAIL]\n";
            
            not_response_on_request++;
            if (not_response_on_request > 5){
                getResponse = false;
                return false;
            }
            sendCmdWrite(0x01, 0x10, &cam_status, sizeof(uint8_t));
            std::this_thread::sleep_for(std::chrono::milliseconds(5));

            clear(dataFrom, dataFromSize);
            wait_response = true;
            byteIsGetBefore = false;
            
        }
    }

    void ProtocolMaster::collectPkg(uint8_t* resvdData, uint32_t resvdBytes, 
            uint8_t* dataUart, uint32_t* dataUartSize, bool& pkgIsReadyToParse){
        
        if (resvdBytes == 0) return;

        memcpy(dataUart + (*dataUartSize), resvdData, resvdBytes);
        *dataUartSize += resvdBytes;
        // std::cout << "Data uart size = " << *dataUartSize << "\n[collectPkg]:\n";
        // for (size_t i = 0; i < *dataUartSize; i++){
        //     printf("[%u]", dataUart[i]);
        // }
        // std::cout << std::endl;
        
        if (*dataUartSize >= 4  && dataUart[1] == 4)  pkgIsReadyToParse = true;
        if (*dataUartSize >= 5  && dataUart[1] == 5)  pkgIsReadyToParse = true;
        if (*dataUartSize >= 6  && dataUart[1] == 6)  pkgIsReadyToParse = true;
        if (*dataUartSize >= 8  && dataUart[1] == 8)  pkgIsReadyToParse = true;
        if (*dataUartSize >= 9  && dataUart[1] == 9)  pkgIsReadyToParse = true;

    }
    
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

    bool ProtocolMaster::sendCmdWriteComplete(const uint8_t* dataTo, uint32_t dataToSize){
        std::memset(buff, 0, sizeof(buff));
        std::memset(recvdBuff, 0, sizeof(recvdBuff));
        memcpy(buff, dataTo, dataToSize);
        /* Отправляем CmdWrite */
        assert(m_transport.sendData(buff, buff[1]));
        return true;
    }
    

    bool ProtocolMaster::sendCmdReadWrite(uint8_t addressTo, uint8_t cmd, const uint8_t* dataTo, uint32_t dataToSize, 
                                        uint8_t* dataFrom, uint32_t* dataFromSize){

        std::memset(buff, 0, sizeof(buff));
        std::memset(recvdBuff, 0, sizeof(recvdBuff));
        clear(dataFrom, dataFromSize);

        buff[0] = addressTo;
        buff[1] = len + dataToSize;
        //std::cout << "\nLEN + DtSz = " << len + dataToSize << std::endl;
        buff[2] = cmd;
        memcpy(buff + 3, dataTo, dataToSize);
        buff[len + dataToSize - sizeof(uint8_t)] = umba_crc8_table(buff,len + dataToSize - sizeof(uint8_t));
        printf("\nCRC8 = %d\n", buff[len + dataToSize - sizeof(uint8_t)]);

        /* Отправляем CmdReadWrite */
        assert(m_transport.sendData(buff, buff[1]));

        first_tp = boost::chrono::system_clock::now();

        /* Ждем DATA */
        std::this_thread::sleep_for(std::chrono::microseconds(430));        //microseconds(500)

        uint32_t not_response_on_request = 0;
        uint32_t not_bytes_received = 0;

        while (1){
            m_coreApplication->processEvents();
            bool pkgIsReadyToParse = false;

            //get bytes
            std::memset(recvdBuff, 0, sizeof(recvdBuff));
            uint32_t recvdBuffSize = 0;

            bool get_bytes = m_transport.getData(recvdBuff, &recvdBuffSize);

            // if (recvdBuffSize != 0)
            //     std::cout << "recvdBuffSize = " << recvdBuffSize << std::endl;

            if (get_bytes){
                std::cout << "get_bytes\n";
                //std::cout << "recvdBuffSize = " << recvdBuffSize << std::endl;
                //std::cout << "not_bytes_received = " << not_bytes_received << std::endl;
                not_bytes_received = 0;
                boost::chrono::system_clock::time_point cur_tp = boost::chrono::system_clock::now();
                boost::chrono::duration<double> ex_time = cur_tp - first_tp;
                std::cout << "Execution time: " << ex_time.count() << std::endl;
            } else {
                //std::cout << "else\n";
                not_bytes_received++;
                if (not_bytes_received > 190){
                    //std::cout << "not_bytes_received > 5\n";
                    clear(dataFrom, dataFromSize);
                    return false;
                }
                std::this_thread::sleep_for(std::chrono::microseconds(5)); //50us
                continue;
            } 
            //std::cout << "\nDATA UART SZ CP = " << *dataFromSize << std::endl;
            collectPkg(recvdBuff, recvdBuffSize, dataFrom, dataFromSize, pkgIsReadyToParse);

            if (!pkgIsReadyToParse){
                //std::cout << "!pkgIsReadyToParse" << std::endl;
                std::this_thread::sleep_for(std::chrono::microseconds(100)); //50us
                continue;
            }

            if (parser(dataFrom, *dataFromSize, addressTo)) return true;
            std::cout << "[PARSER RS FAIL]\n";
            clear(dataFrom, dataFromSize);
            //std::cout << "\nEND*dataFromSize = " << *dataFromSize << std::endl;
            return false;
        }
    }

    bool ProtocolMaster::sendSomeCmd(const uint8_t* dataTo, uint32_t dataToSize, uint8_t* dataFrom, uint32_t* dataFromSize){

        std::memset(buff, 0, sizeof(buff));
        std::memset(recvdBuff, 0, sizeof(recvdBuff));
        clear(dataFrom, dataFromSize);

        memcpy(buff, dataTo, dataToSize);
        /* Отправляем SomeCmd */
        assert(m_transport.sendData(buff, dataToSize));

        first_tp = boost::chrono::system_clock::now();

        if (getCmd(buff) == cmdWrite) return true;
        
        /* Ждем DATA */
        //std::this_thread::sleep_for(std::chrono::microseconds(5000));
        uint32_t not_response_on_request = 0;
        uint32_t not_bytes_received = 0;

        while (1){
            m_coreApplication->processEvents();
            bool pkgIsReadyToParse = false;

            //get bytes
            std::memset(recvdBuff, 0, sizeof(recvdBuff));
            uint32_t recvdBuffSize = 0;
            bool get_bytes = m_transport.getData(recvdBuff, &recvdBuffSize);

            if (get_bytes){
                //std::cout << "get_bytes\n";
                not_bytes_received = 0;

            } else {
                //std::cout << "else\n";
                not_bytes_received++;
                if (not_bytes_received > 50){
                    //std::cout << "not_bytes_received > 50\n";
                    clear(dataFrom, dataFromSize);
                    return false;
                }
                std::this_thread::sleep_for(std::chrono::microseconds(100));
                continue;
            } 
            std::cout << "\n*dataFromSize = " << *dataFromSize << std::endl;
            collectPkg(recvdBuff, recvdBuffSize, dataFrom, dataFromSize, pkgIsReadyToParse);

            if (!pkgIsReadyToParse){
                std::this_thread::sleep_for(std::chrono::microseconds(100));
                continue;
            }

            if (parser(dataFrom, *dataFromSize, dataTo[0])) {
                // boost::chrono::system_clock::time_point cur_tp = boost::chrono::system_clock::now();
                // boost::chrono::duration<double> ex_time = cur_tp - first_tp;
                // std::cout << "Execution time: " << ex_time.count() << std::endl;
                return true;
            }
            
            std::cout << "[PARSER RS FAIL]\n";
            clear(dataFrom, dataFromSize);
            std::cout << "\nEND*dataFromSize = " << *dataFromSize << std::endl;
            return false;
        }
    }

    void ProtocolMaster::clear(uint8_t* dataFrom, uint32_t* dataFromSize) {
        std::memset(dataFrom, 0, *dataFromSize);
        *dataFromSize = 0;
    }

} //namespace protocol_master