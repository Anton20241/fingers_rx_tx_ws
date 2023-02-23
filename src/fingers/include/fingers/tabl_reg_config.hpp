/**
 *  @file       tabl_reg_config.hpp
 */
/**
 * Tabl reg
 * |-----------------|
 * | Reg RO          |
 * |                 |
 * |-----------------|
 * | Reg RW          |
 * |                 |
 * |-----------------|
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#pragma once

/* Includes ------------------------------------------------------------------*/
#include "tabl_reg.hpp"

namespace tabl_reg_cfg
{
/* Exported defines ----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
#ifdef BOARD_HINGE
    static const uint16_t RO_REG_SIZE   = 2;
    static const uint16_t WO_REG_SIZE   = 0;
    static const uint16_t TABL_REG_SIZE = (RO_REG_SIZE + WO_REG_SIZE);

    static const uint16_t RO_REG_ANGLE  = 0;    // Угол 16 bit
#endif

//#ifdef BOARD_FINGER_SET
    static const uint16_t RO_REG_SIZE   = 6;
    static const uint16_t WO_REG_SIZE   = 2;
    static const uint16_t TABL_REG_SIZE = (RO_REG_SIZE + WO_REG_SIZE);

    static const uint16_t RO_REG_FORCE  = 0; //сила 16 bit

//#endif

#ifdef BOARD_FINGER_EXEC
    static const uint16_t RO_REG_SIZE   = 4;
    static const uint16_t WO_REG_SIZE   = 0;
    static const uint16_t TABL_REG_SIZE = (RO_REG_SIZE + WO_REG_SIZE);

    static const uint16_t RO_REG_ANGLE  = 0;
#endif
/* Exported constants --------------------------------------------------------*/
    static uint8_t tablReg[TABL_REG_SIZE] = {0,};

    static const tabl_reg::tablRegConfig_t tablRegConfig =
    {
        .prtTabl        = tablReg,
        .tablSize       = TABL_REG_SIZE,
        .RegRoMaxSize   = RO_REG_SIZE
    };

/* Exported macro ------------------------------------------------------------*/
/* Exported classes ----------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
}
