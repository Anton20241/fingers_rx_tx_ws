/**
 *  @file       tabl_reg.cpp
 *  @brief      Таблица регистров
 */

#include <cstring>
#include "tabl_reg.hpp"

namespace tabl_reg
{
    TablReg::TablReg(const tablRegConfig_t& config) :
        m_config(config),
        m_evtR(NONE),
        m_evtW(NONE)
    {
    }

    bool TablReg::setRegRaw(const uint8_t* ptrData,
                            uint16_t regOffset,
                            uint8_t len)
    {
        if (ptrData == NULL) return false;
        if (regOffset + len > m_config.tablSize) return false;
        if (len == 0) return false;

        std::memcpy(&m_config.prtTabl[regOffset], ptrData, len);
        return true;
    }

    bool TablReg::getRegRaw(uint8_t* ptrData, uint16_t regOffset, uint8_t len)
    {
        if (ptrData == NULL) return false;
        if (regOffset + len > m_config.tablSize) return false;
        if (len == 0) return false;

        std::memcpy(ptrData, &m_config.prtTabl[regOffset], len);
        return true;
    }

    void TablReg::setEvent(evt_t event)
    {
        switch (event)
        {
            case NONE: return;
            case UPDATE_WO_REG: m_evtW = UPDATE_WO_REG; break;
            case UPDATE_RO_REG: m_evtR = UPDATE_RO_REG; break;
        }
    }

    bool TablReg::getEvent(evt_t event)
    {
        evt_t* evt;

        if (event == NONE) return false;
        if (event == UPDATE_WO_REG) evt = &m_evtW;
        if (event == UPDATE_RO_REG) evt = &m_evtR;

        if (*evt != NONE) {
            *evt = NONE;
            return true;
        }
        return false;
    }

    uint16_t TablReg::getSizeRegRo()
    {
        return m_config.RegRoMaxSize;
    }
}
