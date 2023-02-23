/**
 *  @file       tabl_reg.hpp
 *  @brief      Таблица регистров
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#pragma once

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

namespace tabl_reg
{
/* Exported defines ----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
    typedef struct
    {
        uint8_t*        prtTabl;
        uint32_t        tablSize;
        uint16_t        RegRoMaxSize;
    } tablRegConfig_t;

    typedef enum
    {
        NONE = 0,
        UPDATE_WO_REG,
        UPDATE_RO_REG
    } evt_t;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported classes ----------------------------------------------------------*/
    class TablReg
    {
    public:
        TablReg(const tablRegConfig_t& config);

        bool setRegRaw(const uint8_t* ptrData, uint16_t regOffset, uint8_t len);
        bool getRegRaw(uint8_t* ptrData, uint16_t regOffset, uint8_t len);

        void setEvent(evt_t event);
        bool getEvent(evt_t event);
    
        uint16_t getSizeRegRo();
    private:
        const tablRegConfig_t&  m_config;
        evt_t                   m_evtR;
        evt_t                   m_evtW;
    };
/* Exported functions --------------------------------------------------------*/
}
