/**
 * ax12.h
 * Driver AX12 Dynamixel sur UART5 (PC12=TX, PD2=RX) — UART dediee.
 */

#ifndef AX12_H
#define AX12_H

#include <stdint.h>
#include <stdbool.h>

void AX12_Init(void);

bool AX12_Deploy(void);

bool AX12_Retract(void);

bool AX12_SetGoalPosition(uint8_t id, uint16_t position);

/* Regle la vitesse de deplacement du servo (0 = vitesse max, 1..1023 = lent->rapide) */
bool AX12_SetMovingSpeed(uint8_t id, uint16_t speed);

/* Deploie le prisme a une position et une vitesse choisies (mode parametrable) */
bool AX12_DeployCustom(uint16_t position, uint16_t speed);

#endif /* AX12_H */
