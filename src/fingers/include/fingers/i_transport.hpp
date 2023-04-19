/**
 *  @file       i_transport.hpp
 *  @brief      Базовый класс для транспортного уровня
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#pragma once

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
namespace i_transport
{
/* Exported defines ----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported classes ----------------------------------------------------------*/
    class ITransport
    {
    public:
        ITransport() {}
        virtual ~ITransport() {}

        virtual bool transportReset() = 0;

        virtual bool sendData(const uint8_t* ptrData, uint32_t len) = 0;
        virtual bool getData(uint8_t* ptrData, uint32_t* lenInOut) = 0;
        bool send_error = false;
    };
/* Exported functions --------------------------------------------------------*/
}
