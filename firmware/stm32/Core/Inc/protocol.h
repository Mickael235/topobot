/**
 * protocol.h
 * Gestion des communications UART avec la Raspberry Pi.
 */

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

/* Définition du protocole */
#define PROTO_START         0xAA
#define PROTO_MAX_FRAME     64

/* Commandes (Raspberry -> STM32) */
#define CMD_MOVE_TO         0x01
#define CMD_STOP            0x02
#define CMD_RESET_ODOM      0x03
#define CMD_GET_STATUS      0x04
#define CMD_SET_GRID        0x05
#define CMD_SET_GAINS       0x06
#define CMD_SET_VELOCITY    0x07   /* payload: vx + vy + wz (3 floats) */

/* Commandes AX12 / prisme topographique */
#define CMD_DEPLOY_PRISM    0x10   /* Déploie le prisme. Payload optionnel: position(u16 LE)+vitesse(u16 LE) */
#define CMD_RETRACT_PRISM   0x11   /* Rentre le prisme */

/* Réponses (STM32 -> Raspberry) */
#define RESP_ACK            0xA1
#define RESP_DONE           0xA2
#define RESP_STATUS         0xA3
#define RESP_ERROR          0xA4

/* Codes d'erreur */
#define ERR_CRC             0x01
#define ERR_UNKNOWN_CMD     0x02
#define ERR_BUSY            0x03
#define ERR_OUT_BOUNDS      0x04
#define ERR_ARU             0x05
#define ERR_AX12            0x06   /* Echec communication AX12 */

/* Initialise la réception UART en mode interruption (1 octet) */
void Protocol_Init(void);

/* Relance la réception UART (reset state machine + IT) */
void Protocol_RestartRx(void);

/* Fonction appelée par le Callback HAL à chaque octet reçu */
void Protocol_UART_RxCallback(void);

/* Machine d'état à appeler dans le while(1) du main pour traiter les trames complètes */
void Protocol_Process(void);

/* --- Fonctions d'envoi vers la Raspberry Pi --- */
bool Protocol_SendAck(uint8_t cmd_echo);
bool Protocol_SendDone(float x, float y, float theta);
bool Protocol_SendStatus(void);
bool Protocol_SendError(uint8_t code);

#endif /* PROTOCOL_H */