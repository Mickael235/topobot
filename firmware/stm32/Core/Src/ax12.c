/**
 * ax12.c
 * Driver AX12 Dynamixel sur UART5 (PC12=TX, PD2=RX) — UART dediee.
 *
 * Protocole Dynamixel v1 :
 *   Instruction : FF FF [ID] [LEN] [INSTR] [PARAMS...] [CHK]
 *   Statut      : FF FF [ID] [LEN] [ERR]  [PARAMS...] [CHK]
 *   LEN = nb_params + 2  (instruction + checksum)
 *   CHK = ~(ID + LEN + INSTR + PARAMS) & 0xFF
 */

#include "ax12.h"
#include "robot_config.h"
#include "main.h"

extern UART_HandleTypeDef AX12_UART;

#define AX12_INSTR_WRITE    0x03U
#define AX12_REG_TORQUE_EN  0x18U
#define AX12_REG_GOAL_POS   0x1EU
#define AX12_REG_MOVING_SPD 0x20U   /* Moving Speed (RAM, 2 octets) */

/* ------------------------------------------------------------------ */
/*  Utilitaires paquet                                                  */
/* ------------------------------------------------------------------ */

static uint8_t pkt_checksum(const uint8_t *pkt, uint8_t total_len)
{
    uint8_t sum = 0;
    for (uint8_t i = 2; i < total_len - 1; i++) {
        sum = (uint8_t)(sum + pkt[i]);
    }
    return (uint8_t)(~sum);
}

static bool ax12_write_byte(uint8_t id, uint8_t reg, uint8_t value)
{
    uint8_t pkt[8];
    pkt[0] = 0xFF;
    pkt[1] = 0xFF;
    pkt[2] = id;
    pkt[3] = 4U;
    pkt[4] = AX12_INSTR_WRITE;
    pkt[5] = reg;
    pkt[6] = value;
    pkt[7] = pkt_checksum(pkt, 8U);

    return HAL_UART_Transmit(&AX12_UART, pkt, 8U, AX12_TIMEOUT_MS) == HAL_OK;
}

/* ------------------------------------------------------------------ */
/*  API publique                                                        */
/* ------------------------------------------------------------------ */

void AX12_Init(void)
{
    /* CubeMX initialise UART5 a 115200 — on reconfigure a AX12_BAUDRATE */
    AX12_UART.Init.BaudRate = AX12_BAUDRATE;
    AX12_UART.Init.Mode     = UART_MODE_TX_RX;
    HAL_UART_Init(&AX12_UART);

    HAL_Delay(500);
    ax12_write_byte(AX12_ID_PRISM, AX12_REG_TORQUE_EN, 1U);
    HAL_Delay(10);
}

bool AX12_SetGoalPosition(uint8_t id, uint16_t position)
{
    /*
     * Trame WRITE_DATA Goal Position :
     *   FF FF [ID] 05 03 1E [POS_L] [POS_H] [CHK]
     *   LEN = 5 = 1(INSTR) + 3(PARAMS) + 1(CHK)
     */
    uint8_t pkt[9];
    pkt[0] = 0xFF;
    pkt[1] = 0xFF;
    pkt[2] = id;
    pkt[3] = 5U;
    pkt[4] = AX12_INSTR_WRITE;
    pkt[5] = AX12_REG_GOAL_POS;
    pkt[6] = (uint8_t)(position & 0xFFU);
    pkt[7] = (uint8_t)((position >> 8U) & 0x03U);
    pkt[8] = pkt_checksum(pkt, 9U);

    return HAL_UART_Transmit(&AX12_UART, pkt, 9U, AX12_TIMEOUT_MS) == HAL_OK;
}

bool AX12_SetMovingSpeed(uint8_t id, uint16_t speed)
{
    /*
     * Trame WRITE_DATA Moving Speed :
     *   FF FF [ID] 05 03 20 [SPD_L] [SPD_H] [CHK]
     *   speed : 0 = vitesse max, 1..1023 = lent -> rapide
     */
    uint8_t pkt[9];
    pkt[0] = 0xFF;
    pkt[1] = 0xFF;
    pkt[2] = id;
    pkt[3] = 5U;
    pkt[4] = AX12_INSTR_WRITE;
    pkt[5] = AX12_REG_MOVING_SPD;
    pkt[6] = (uint8_t)(speed & 0xFFU);
    pkt[7] = (uint8_t)((speed >> 8U) & 0x03U);
    pkt[8] = pkt_checksum(pkt, 9U);

    return HAL_UART_Transmit(&AX12_UART, pkt, 9U, AX12_TIMEOUT_MS) == HAL_OK;
}

bool AX12_Deploy(void)
{
    /* Mode normal : vitesse max par defaut (annule un eventuel reglage lent precedent) */
    AX12_SetMovingSpeed(AX12_ID_PRISM, 0U);
    return AX12_SetGoalPosition(AX12_ID_PRISM, AX12_POS_DEPLOY);
}

bool AX12_DeployCustom(uint16_t position, uint16_t speed)
{
    /* Regle la vitesse AVANT la consigne de position pour que le mouvement
     * vers la cible se fasse a la vitesse demandee. */
    if (!AX12_SetMovingSpeed(AX12_ID_PRISM, speed))
        return false;
    return AX12_SetGoalPosition(AX12_ID_PRISM, position);
}

bool AX12_Retract(void)
{
    return AX12_SetGoalPosition(AX12_ID_PRISM, AX12_POS_RETRACT);
}
