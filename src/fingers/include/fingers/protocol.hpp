/**
 *  @file       protocol.hpp
 *  @brief
 */
 /**
  * Описание протокола
  * Синхронизация пакетов по синхропаузе >= (T_byte * 2) - время паузы не менее
  * времени передачи 2-х байт.
  * [pack_master][sync pause][pack_slave][sync pause][pack_master]......
  *
  * Формат пакета [ADDR][LEN][CMD][DATA][CRC8]
  *
  *     [ADDR](1 Byte) - Адрес устройства
  *         0 - общевещательный адрес (резерв)
  *         1 - 247 - адрес устройства
  *         248 - 255 - резерв
  *
  *     [LEN](1 Byte) - Длина пакета
  *         Длина пакета включая [ADDR] - [CRC8]
  *
  *     [CMD](1 Byte) - Команда
  *         Bit 0 - 3 - ID пакета (задается при необходимости мастером,
  *         устройство при ответе копирует ID пакета)
  *         Bit 4 - 6 - номер команды
  *             0x0 - NOP, используется для проверки наличия устройства в сети
  *             0x1 - чтение данных
  *             0x2 - запись данных
  *             0x3 - чтение/запись данных
  *         Bit 7 - флаг ошибки команды (используется при ответе)
  *
  *     [DATA](0 - 12 Bytes) - Поле данных команды
  *         CMD 0x0 - поле данных отсутствует
  *         CMD 0x1
  *         Master
  *             Поле данных отсутствует
  *         Slave
  *             [READ_DATA](1 - 12 Bytes) - считанные данные
  *         CMD 0x2
  *         Master
  *             [WRITE_DATA](1 - 12 Bytes) - данные для записи
  *         Slave
  *             Поле данных отсутствует
  *         CMD 0x3
  *         Master
  *             [WRITE_DATA](1 - 12 Bytes) - данные для записи
  *         Slave
  *             [READ_DATA](1 - 12 Bytes) - считанные данные
  *
  *     [CRC8](1 Byte) - контрольная сумма
  *
  *     Ошибки пакетов
  *     [ADDR][LEN][CMD][ERR][CRC8]
  *     В поле команды устанавливается бит ошибки (bit 7)
  *     Поле [DATA] заменяется кодом ошибки
  *         0x01 - не возможно считать/записать данные
  *         0x02 - команда не поддерживается
  *         0x03 - неверная длина пакета
  *
  *     Пример команды на чтение/записть:
  *     Master  [ADDR][LEN] [CMD 3 ID 1][   DATA  ][CRC8]
  *             [0x02][0x06][   0x31   ][0x22,0x33][CRC8]
  *     Slave   [ADDR][LEN] [CMD 3 ID 1][   DATA  ][CRC8]
  *             [0x02][0x06][   0x31   ][0x55,0x66][CRC8]
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#pragma once

/* Includes ------------------------------------------------------------------*/
//#include "main.h"
#include "i_transport.hpp"
#include "tabl_reg.hpp"

namespace protocol
{
/* Exported defines ----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
    class Protocol
    {
        public:
            Protocol(   i_transport::ITransport& transport,
                        tabl_reg::TablReg& tabl,
                        uint8_t addr);
            ~Protocol() {};

            void process();

        private: 
            i_transport::ITransport&    m_transport;
            tabl_reg::TablReg&          m_tabl;
            uint8_t                     m_addr;

            void parser(uint8_t* ptrBuff, uint32_t len);
            void sendError(uint8_t err, uint8_t* ptrBuff);
            void sendData(uint8_t* ptrBuff);
            bool writeData(uint8_t* ptrBuff, uint32_t len);
            void sendSuccess(uint8_t* ptrBuff);
    };
}

namespace protocol_master
{
    class ProtocolMaster
    {
        public:
            ProtocolMaster(i_transport::ITransport& transport);
            ~ProtocolMaster() {};

            bool sendCmdNOP(uint8_t addressTo);
            bool sendCmdRead(uint8_t addressTo, uint8_t* dataFrom, uint32_t* dataFromSize);
            bool sendCmdWrite(uint8_t addressTo, uint8_t cmd, const uint8_t* dataTo, uint32_t dataToSize);
            bool sendCmdReadWrite(uint8_t addressTo, uint8_t cmd, const uint8_t* dataTo, uint32_t dataToSize, 
                                        uint8_t* dataFrom, uint32_t* dataFromSize);
            bool sendSomeCmd(const uint8_t* dataTo, uint32_t dataToSize, uint8_t* dataFrom, uint32_t* dataFromSize);

            //UART
            bool sendCmdReadUART(uint8_t addressTo, uint8_t* dataFrom, uint32_t* dataFromSize, 
                                        bool& getResponse, bool wait_response, uint8_t cam_status);
            bool sendCmdReadWriteUART(uint8_t addressTo, uint8_t cmd, const uint8_t* dataTo, uint32_t dataToSize, 
                                        uint8_t* dataFrom, uint32_t* dataFromSize);
            i_transport::ITransport&    m_transport;
        private:
            //i_transport::ITransport&    m_transport;
            void collectPkg(uint8_t* resvdData, uint32_t resvdBytes, uint8_t* dataUart, uint32_t& dataUartSize, bool& pkgIsReady);
            bool parserOk(uint8_t* dataUart, uint32_t& dataUartSize);
            void updateDataUart(uint8_t* dataUart, uint32_t& dataUartSize);
            bool parser(uint8_t* ptrBuff, uint32_t len);

    };
}


