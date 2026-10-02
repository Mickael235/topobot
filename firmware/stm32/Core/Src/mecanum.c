/**
 * mecanum.c
 *
 * Protocole Sabertooth Packetized Serial (mode indépendant) :
 *   Trame : [Address] [Command] [Value] [Checksum]
 *   Checksum = (Address + Command + Value) & 0x7F
 *
 *   Cmd 0 : moteur A avant   (0-127)
 *   Cmd 1 : moteur A arrière (0-127)
 *   Cmd 4 : moteur B avant   (0-127)
 *   Cmd 5 : moteur B arrière (0-127)
 *
 * Les deux Sabertooth partagent le même fil S1 sur PC10 (UART4_TX).
 * Distinction par adresse : SABRE1_ADDR=128 (FL+RL), SABRE2_ADDR=129 (FR+RR).
 *
 * Câblage observé sur le robot (validé par le script Python direct
 * Raspberry → Sabertooth) :
 *   - Les moteurs FL et RR sont câblés en polarité INVERSÉE
 *   - FR et RL sont en polarité directe
 *   => les constantes POL_xx ci-dessous remettent le signe correct
 *      avant l'envoi à la Sabertooth.
 *
 * Cinématique simplifiée SANS rotation (translation pure) :
 *   wFL = (vx - vy) / r
 *   wFR = (vx + vy) / r
 *   wRL = (vx + vy) / r
 *   wRR = (vx - vy) / r
 *
 * Pour AVANT/ARRIERE (vx pur)  : les 4 roues tournent au même rythme
 * Pour GAUCHE/DROITE (vy pur)  : FL/RR opposes a FR/RL (meme magnitude)
 */

#include "mecanum.h"
#include "robot_config.h"
#include <math.h>
#include <string.h>

extern UART_HandleTypeDef SABRE1_UART;

/* Polarité de câblage : +1 si le moteur tourne dans le sens attendu pour
 * la commande "avant" Sabertooth, -1 si le moteur est câblé inversé. */
#define POL_FL   (-1)
#define POL_FR   (+1)
#define POL_RL   (+1)
#define POL_RR   (-1)

/* ------------------------------------------------------------------ */
/*  Primitives                                                          */
/* ------------------------------------------------------------------ */

/** Sérialise un paquet Sabertooth à l'adresse dest[0..3] */
static void pack_packet(uint8_t *dest,
                        uint8_t addr, uint8_t cmd, uint8_t val)
{
    val   &= 0x7F;   /* max 127 */
    dest[0] = addr;
    dest[1] = cmd;
    dest[2] = val;
    dest[3] = (uint8_t)((addr + cmd + val) & 0x7F);
}

/** Convertit une vitesse normalisée [-1,+1] en (cmd, val) Sabertooth.
 *  @param motor  0 = moteur A (cmd 0/1), 1 = moteur B (cmd 4/5)
 */
static void norm_to_cmd_val(float norm, uint8_t motor,
                            uint8_t *out_cmd, uint8_t *out_val)
{
    /* Zone morte minimale (Sabertooth ignore <5% en général) */
    if (fabsf(norm) < MIN_MOTOR_NORM) norm = 0.0f;

    /* Saturation */
    if (norm >  1.0f) norm =  1.0f;
    if (norm < -1.0f) norm = -1.0f;

    *out_val = (uint8_t)(fabsf(norm) * 127.0f);

    if (norm >= 0.0f) {
        /* Avant */
        *out_cmd = (motor == 0) ? 0u : 4u;
    } else {
        /* Arrière */
        *out_cmd = (motor == 0) ? 1u : 5u;
    }
}

/** Envoi bloquant d'un paquet Sabertooth */
static void sabre_send_blocking(uint8_t addr, uint8_t cmd, uint8_t val)
{
    uint8_t buf[4];
    pack_packet(buf, addr, cmd, val);
    HAL_UART_Transmit(&SABRE1_UART, buf, 4, 10);
}

/* ------------------------------------------------------------------ */
/*  Interface publique                                                  */
/* ------------------------------------------------------------------ */

void Mecanum_Init(void)
{
    Mecanum_Stop();
}

void Mecanum_SetVelocity(float vx, float vy)
{
    /* ---------- Saturation des consignes globales ---------- */
    if (vx >  MAX_VX) vx =  MAX_VX;
    if (vx < -MAX_VX) vx = -MAX_VX;
    if (vy >  MAX_VY) vy =  MAX_VY;
    if (vy < -MAX_VY) vy = -MAX_VY;

    float r = WHEEL_RADIUS_M;

    /* ---------- Cinématique inverse mecanum (translation pure) ---------- */
    float w_FL = (vx - vy) / r;
    float w_FR = (vx + vy) / r;
    float w_RL = (vx + vy) / r;
    float w_RR = (vx - vy) / r;

    /* ---------- Normalisation si l'une des roues dépasse max ---------- */
    float w_max_cmd = MAX_VX / r;
    float biggest   = fabsf(w_FL);
    if (fabsf(w_FR) > biggest) biggest = fabsf(w_FR);
    if (fabsf(w_RL) > biggest) biggest = fabsf(w_RL);
    if (fabsf(w_RR) > biggest) biggest = fabsf(w_RR);

    if (biggest > w_max_cmd && biggest > 1e-6f) {
        float scale = w_max_cmd / biggest;
        w_FL *= scale;
        w_FR *= scale;
        w_RL *= scale;
        w_RR *= scale;
    }

    /* ---------- Normalisation [-1, +1] + polarité câblage ---------- */
    float n_FL = (float)POL_FL * (w_FL / w_max_cmd);
    float n_FR = (float)POL_FR * (w_FR / w_max_cmd);
    float n_RL = (float)POL_RL * (w_RL / w_max_cmd);
    float n_RR = (float)POL_RR * (w_RR / w_max_cmd);

    /* ---------- Envoi bloquant aux deux Sabertooth (même bus UART4) ---------- */
    uint8_t cmd, val;
    uint8_t buf[4];

    norm_to_cmd_val(n_FL, 0, &cmd, &val);          /* SAB1 motor A = FL */
    pack_packet(buf, SABRE1_ADDR, cmd, val);
    HAL_UART_Transmit(&SABRE1_UART, buf, 4, 10);

    norm_to_cmd_val(n_FR, 0, &cmd, &val);          /* SAB2 motor A = FR */
    pack_packet(buf, SABRE2_ADDR, cmd, val);
    HAL_UART_Transmit(&SABRE2_UART, buf, 4, 10);


    norm_to_cmd_val(n_RL, 1, &cmd, &val);          /* SAB1 motor B = RL */
    pack_packet(buf, SABRE1_ADDR, cmd, val);
    HAL_UART_Transmit(&SABRE1_UART, buf, 4, 10);


    norm_to_cmd_val(n_RR, 1, &cmd, &val);          /* SAB2 motor B = RR */
    pack_packet(buf, SABRE2_ADDR, cmd, val);
    HAL_UART_Transmit(&SABRE2_UART, buf, 4, 10);
}

void Mecanum_Stop(void)
{
    /* 4 paquets de 0 (un par moteur), bloquant pour fiabilité */
    sabre_send_blocking(SABRE1_ADDR, 0, 0);  /* SAB1 motor A (FL) */
    sabre_send_blocking(SABRE1_ADDR, 4, 0);  /* SAB1 motor B (RL) */
    sabre_send_blocking(SABRE2_ADDR, 0, 0);  /* SAB2 motor A (FR) */
    sabre_send_blocking(SABRE2_ADDR, 4, 0);  /* SAB2 motor B (RR) */
}
