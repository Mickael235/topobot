/**
 * protocol.c - Protocole UART STM32 ↔ Raspberry Pi
 *
 * Réception en mode interrupt octet par octet (simple et robuste).
 * La trame est reconstruite dans un buffer circulaire.
 */

#include "protocol.h"
#include "robot_config.h"
#include "mecanum.h"
#include "odometry.h"
#include "ax12.h"
#include <string.h>
#include <math.h>
#include "robot_control.h"
extern TIM_HandleTypeDef ENC_FL_TIM;
extern TIM_HandleTypeDef ENC_FR_TIM;
extern TIM_HandleTypeDef ENC_RL_TIM;
extern TIM_HandleTypeDef ENC_RR_TIM;

/* Consigne de vitesse partagée avec main.c (mode test minimal) */
extern volatile float g_vx;
extern volatile float g_vy;
extern volatile float g_wz;

extern UART_HandleTypeDef RASP_UART;

/* ------------------------------------------------------------------ */
/*  State machine de réception                                          */
/* ------------------------------------------------------------------ */

typedef enum {
    RX_WAIT_START = 0,
    RX_WAIT_CMD,
    RX_WAIT_LEN,
    RX_DATA,
    RX_WAIT_CRC
} RxState_t;

static RxState_t rx_state = RX_WAIT_START;
static uint8_t   rx_byte;       /* Reçu via HAL interrupt (1 octet à la fois) */
static uint8_t   rx_cmd;
static uint8_t   rx_len;
static uint8_t   rx_cnt;
static uint8_t   rx_buf[PROTO_MAX_FRAME];
static uint8_t   rx_crc_acc;

/* Flag : trame complète prête */
static volatile uint8_t frame_ready = 0;
static uint8_t  frame_cmd;
static uint8_t  frame_len;
static uint8_t  frame_data[PROTO_MAX_FRAME];

/* ------------------------------------------------------------------ */
/*  Helpers                                                             */
/* ------------------------------------------------------------------ */

static uint8_t calc_crc(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    uint8_t crc = cmd;
    crc ^= len;
    for (uint8_t i = 0; i < len; i++) crc ^= data[i];
    return crc;
}

static void float_to_bytes(float f, uint8_t *buf)
{
    memcpy(buf, &f, 4);
}

static float bytes_to_float(const uint8_t *buf)
{
    float f;
    memcpy(&f, buf, 4);
    return f;
}

static bool send_frame(uint8_t resp, const uint8_t *data, uint8_t len)
{
    uint8_t frame[PROTO_MAX_FRAME + 4];
    frame[0] = PROTO_START;
    frame[1] = resp;
    frame[2] = len;
    if (len > 0 && data != NULL)
        memcpy(&frame[3], data, len);
    frame[3 + len] = calc_crc(resp, data, len);
    uint16_t total = (uint16_t)(4 + len);

    /* Mode full-duplex (test ST-Link) : TE et RE restent activés en permanence,
     * pas besoin de basculer comme en half-duplex. */
    HAL_StatusTypeDef st = HAL_UART_Transmit(&RASP_UART, frame, total, 20);
    /* Relancer la réception IT octet par octet (au cas où elle aurait été interrompue) */
    HAL_UART_Receive_IT(&RASP_UART, &rx_byte, 1);

    return (st == HAL_OK);
}

/* ------------------------------------------------------------------ */
/*  Initialisation                                                      */
/* ------------------------------------------------------------------ */

void Protocol_Init(void)
{
    rx_state    = RX_WAIT_START;
    frame_ready = 0;
    /* Lance la réception du premier octet */
    HAL_UART_Receive_IT(&RASP_UART, &rx_byte, 1);
}

void Protocol_RestartRx(void)
{
    rx_state = RX_WAIT_START;
    HAL_UART_Receive_IT(&RASP_UART, &rx_byte, 1);
}

/* ------------------------------------------------------------------ */
/*  Callback interrupt HAL (appeler depuis HAL_UART_RxCpltCallback)    */
/* ------------------------------------------------------------------ */

void Protocol_UART_RxCallback(void)
{
    uint8_t b = rx_byte;

    switch (rx_state) {
    case RX_WAIT_START:
        if (b == PROTO_START) rx_state = RX_WAIT_CMD;
        break;

    case RX_WAIT_CMD:
        rx_cmd    = b;
        rx_crc_acc = b;
        rx_state  = RX_WAIT_LEN;
        break;

    case RX_WAIT_LEN:
        rx_len    = b;
        rx_crc_acc ^= b;
        rx_cnt    = 0;
        if (rx_len == 0) rx_state = RX_WAIT_CRC;
        else if (rx_len < PROTO_MAX_FRAME) rx_state = RX_DATA;
        else if (rx_len < PROTO_MAX_FRAME && rx_len <= 30) rx_state = RX_DATA;
        else rx_state = RX_WAIT_START;   /* Trame invalide */
        break;

    case RX_DATA:
        rx_buf[rx_cnt++] = b;
        rx_crc_acc ^= b;
        if (rx_cnt >= rx_len) rx_state = RX_WAIT_CRC;
        break;

    case RX_WAIT_CRC:
        if (b == rx_crc_acc) {
            /* CRC OK → copier dans le buffer de traitement */
            if (!frame_ready) {
                frame_cmd = rx_cmd;
                frame_len = rx_len;
                memcpy(frame_data, rx_buf, rx_len);
                frame_ready = 1;
            }
        }
        /* Dans tous les cas, on repart en attente */
        rx_state = RX_WAIT_START;
        break;
    }

    /* Relancer la réception du prochain octet */
    HAL_UART_Receive_IT(&RASP_UART, &rx_byte, 1);
}

/* ------------------------------------------------------------------ */
/*  Traitement des trames (appelé dans la boucle principale)           */
/* ------------------------------------------------------------------ */

void Protocol_Process(void)
{
    if (!frame_ready) return;

    uint8_t cmd = frame_cmd;
    const uint8_t *d = frame_data;
    frame_ready = 0;    /* Acquitter */

    switch (cmd) {

    /* ── CMD_STOP : annule tout mouvement asservi, remet la consigne à zéro,
     * stoppe les moteurs et repasse la state machine en IDLE. ── */
    case CMD_STOP:
        g_vx = 0.0f;
        g_vy = 0.0f;
        g_wz = 0.0f;
        Mecanum_Stop();
        RobotControl_Reset();   /* sort de MOVING/DONE/ERROR vers IDLE */
        Protocol_SendAck(cmd);
        break;

    /* ── CMD_RESET_ODOM : remet x, y, θ et compteurs encodeurs à zéro,
     * et reset la state machine (évite un MOVE_TO résiduel sur la nouvelle origine). ── */
    case CMD_RESET_ODOM:
        Odom_Reset();
        RobotControl_Reset();
        Protocol_SendAck(cmd);
        break;

    /* ── CMD_SET_VELOCITY : fixe la nouvelle consigne (vx, vy, wz) ──
     * La main loop la ré-applique à chaque tick TIM6 (100 Hz) tant qu'aucun
     * STOP ou nouveau SET_VELOCITY n'est reçu. */
    case CMD_SET_VELOCITY:
        if (frame_len >= 12) {
            g_vx = bytes_to_float(d + 0);
            g_vy = bytes_to_float(d + 4);
            g_wz = bytes_to_float(d + 8);
            Protocol_SendAck(cmd);
        } else {
            Protocol_SendError(ERR_CRC);
        }
        break;

    /* ── CMD_GET_STATUS : consigne + compteurs encodeurs + pose odométrie ── */
    case CMD_GET_STATUS:
    {
        typedef struct __attribute__((packed)) {
            /* Consigne courante (envoyée par le Pi) */
            float   vx_cmd;
            float   vy_cmd;
            float   wz_cmd;
            /* Compteurs encodeurs bruts (FL=TIM8, FR=TIM3, RL=TIM4, RR=TIM2) */
            int32_t e1;
            int32_t e2;
            int32_t e3;
            int32_t e4;
            /* Pose odométrie repère monde */
            float   x;
            float   y;
            float   theta;
            /* Vitesses mesurées par odométrie repère robot */
            float   vx_meas;
            float   vy_meas;
            float   wz_meas;
            /* État machine asservissement (0=IDLE, 1=MOVING, 2=SETTLING,
             *                              3=DONE, 4=ESTOP, 5=ERROR) */
            uint8_t state;
        } StatusPayload_t;

        StatusPayload_t st;

        st.vx_cmd = g_vx;
        st.vy_cmd = g_vy;
        st.wz_cmd = g_wz;

        st.e1 = __HAL_TIM_GET_COUNTER(&ENC_FL_TIM);   /* FL (TIM4, hors service) */
        st.e2 = __HAL_TIM_GET_COUNTER(&ENC_FR_TIM);   /* FR (TIM2, hors service) */
        st.e3 = __HAL_TIM_GET_COUNTER(&ENC_RL_TIM);   /* RL (TIM8, fonctionne) */
        st.e4 = __HAL_TIM_GET_COUNTER(&ENC_RR_TIM);   /* RR (TIM3, fonctionne) */

        const OdomState_t *o = Odom_GetState();
        st.x       = o->x;
        st.y       = o->y;
        st.theta   = o->theta;
        st.vx_meas = o->vx;
        st.vy_meas = o->vy;
        st.wz_meas = o->wz;

        st.state   = (uint8_t)RobotControl_GetState();

        send_frame(RESP_STATUS, (uint8_t *)&st, sizeof(st));
        break;
    }

    case CMD_MOVE_TO:
           if (frame_len >= 12) {
               float target_x = bytes_to_float(d + 0);
               float target_y = bytes_to_float(d + 4);
               float target_th = bytes_to_float(d + 8);

               RobotControl_MoveTo(target_x, target_y, target_th);
               Protocol_SendAck(cmd);
           } else {
               Protocol_SendError(ERR_CRC);
           }
           break;

    case CMD_DEPLOY_PRISM:
    {
        /* Charge optionnelle : [position(u16 LE)] [vitesse(u16 LE)] = 4 octets.
         * - 4 octets  : deploiement parametrable (position + vitesse choisies)
         * - sinon     : deploiement par defaut (AX12_POS_DEPLOY, vitesse max). */
        bool ok;
        if (frame_len >= 4) {
            uint16_t pos   = (uint16_t)(d[0] | (d[1] << 8));
            uint16_t speed = (uint16_t)(d[2] | (d[3] << 8));
            ok = AX12_DeployCustom(pos, speed);
        } else {
            ok = AX12_Deploy();
        }
        if (ok)
            Protocol_SendAck(cmd);
        else
            Protocol_SendError(ERR_AX12);
        break;
    }

    case CMD_RETRACT_PRISM:
        if (AX12_Retract())
            Protocol_SendAck(cmd);
        else
            Protocol_SendError(ERR_AX12);
        break;

    default:
        Protocol_SendError(ERR_UNKNOWN_CMD);
        break;
    }

}

/* ------------------------------------------------------------------ */
/*  Fonctions d'envoi                                                   */
/* ------------------------------------------------------------------ */

bool Protocol_SendAck(uint8_t cmd_echo)
{
    return send_frame(RESP_ACK, &cmd_echo, 1);
}

bool Protocol_SendDone(float x, float y, float theta)
{
    uint8_t buf[12];
    float_to_bytes(x,     buf + 0);
    float_to_bytes(y,     buf + 4);
    float_to_bytes(theta, buf + 8);
    return send_frame(RESP_DONE, buf, 12);
}

bool Protocol_SendStatus(void)
{
    /* Mode test minimal : on renvoie juste la consigne courante (12 octets).
     * Pas d'odométrie, pas d'état machine — juste de quoi vérifier que le STM32
     * a bien reçu la dernière commande. */
    uint8_t buf[12];
    float_to_bytes(g_vx, buf + 0);
    float_to_bytes(g_vy, buf + 4);
    float_to_bytes(g_wz, buf + 8);
    return send_frame(RESP_STATUS, buf, 12);
}

bool Protocol_SendError(uint8_t code)
{
    return send_frame(RESP_ERROR, &code, 1);
}
