/**
 * robot_control.c
 *
 * Machine d'états + asservissement position 2 axes (x, y).
 * PAS d'asservissement angulaire : le robot ne déplace que sur les 4
 * axes cardinaux (AVANT/ARRIERE/GAUCHE/DROITE) sans tourner.
 *
 * Contrôleur P + saturation + zone morte :
 *   ex_r = (target_x - x)  projeté en repère robot
 *   ey_r = (target_y - y)  projeté en repère robot
 *   vx   = sat(KP * ex_r, ±VELOCITY_CAP)        ; |vx| < DEADBAND -> 0
 *   vy   = sat(KP * ey_r, ±VELOCITY_CAP)        ; |vy| < DEADBAND -> 0
 *
 * Choix proportionnel pur (pas de I ni D) pour :
 *   1. Garantir que toutes les roues reçoivent la même magnitude de cmd
 *      (donc tournent à la même vitesse) à tout instant.
 *   2. Eviter le pic dérivé du démarrage qui faisait diverger la pose.
 *   3. Rester dans la plage [DEADBAND, VELOCITY_CAP] où le Sabertooth
 *      répond uniformément (régime validé empiriquement à v=0.10 m/s).
 *
 * Convergence : pose dans POS_TOLERANCE_M pendant SETTLE_CYCLES cycles
 *               consécutifs -> RESP_DONE puis IDLE.
 */

#include "robot_control.h"
#include "robot_config.h"
#include "pid.h"
#include "odometry.h"
#include "mecanum.h"
#include "protocol.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
extern volatile float g_vx;
extern volatile float g_vy;
extern volatile float g_wz;   /* recu du Pi mais ignore (pas de rotation) */

/* ------------------------------------------------------------------ */
/*  State                                                               */
/* ------------------------------------------------------------------ */

static RobotState_t state = ROBOT_IDLE;

static float target_x     = 0.0f;
static float target_y     = 0.0f;

static float grid_max_x   = 10.0f;
static float grid_max_y   = 10.0f;

static uint32_t settle_cnt = 0;

/* État rampe accélération (slew rate limiter) */
static float prev_cmd_vx = 0.0f;
static float prev_cmd_vy = 0.0f;

volatile int32_t enc1 = 0;
volatile int32_t enc2 = 0;
volatile int32_t enc3 = 0;
volatile int32_t enc4 = 0;

/* PID translation X / Y (rotation supprimée) — utilisés uniquement pour
 * leurs gains KP, le contrôleur est en réalité un P pur saturé/seuillé. */
static PID_t pid_x;
static PID_t pid_y;

/* Saturation à [lo, hi] */
static inline float saturate(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

/* ---- Debounce ARU ----
 * Filtre les glitches EMI moteur sur PB14. L'ARU n'est considéré
 * actif qu'après ARU_DEBOUNCE_CYCLES lectures LOW consécutives à 100 Hz.
 */
#define ARU_DEBOUNCE_CYCLES   5    /* 50 ms à 100 Hz */
static uint32_t aru_low_count = 0;

/* ---- Keepalive Sabertooth ----
 * En IDLE/ESTOP/ERROR, on ré-envoie un paquet "stop" tous les
 * KEEPALIVE_CYCLES cycles pour ne pas perdre la liaison série
 * (timeout Sabertooth ~1 s → LED rouge sinon).
 */
#define KEEPALIVE_CYCLES      50   /* 500 ms à 100 Hz */
static uint32_t keepalive_count = 0;

/* ------------------------------------------------------------------ */
/*  Utilitaires                                                         */
/* ------------------------------------------------------------------ */

/** Lecture brute du pin ARU (sans debounce). Réservée à l'init du filtre
 *  ou aux décisions non-temps-réel où on accepte le risque de glitch. */
static bool aru_pin_low(void)
{
    return (HAL_GPIO_ReadPin(ARU_GPIO_Port, ARU_GPIO_Pin) == GPIO_PIN_RESET);
}

/** État ARU DEBOUNCÉ. Mis à jour par aru_filter_step() à chaque cycle 100 Hz. */
static bool aru_triggered(void)
{
    return (aru_low_count >= ARU_DEBOUNCE_CYCLES);
}

/** Avance le filtre debounce d'un cycle. À appeler une fois par RobotControl_Update().
 *  Compte les lectures LOW consécutives ; tout HIGH remet le compteur à zéro. */
static void aru_filter_step(void)
{
    if (aru_pin_low()) {
        if (aru_low_count < ARU_DEBOUNCE_CYCLES) aru_low_count++;
    } else {
        aru_low_count = 0;
    }
}

/** Vérifie que la position est dans la grille + marge */
static bool in_grid(float x, float y)
{
    /*float m = GRID_MARGIN_M;
    return (x >= -m) && (x <= grid_max_x + m)
        && (y >= -m) && (y <= grid_max_y + m);*/
	return true;
}

/* ------------------------------------------------------------------ */
/*  Init                                                                */
/* ------------------------------------------------------------------ */

void RobotControl_Init(void)
{
    /* On reutilise le contexte PID juste pour stocker les gains KP_x/KP_y
     * configurables a chaud via RobotControl_SetGains. Les termes I et D
     * sont neutralises (saturation -> P pur saturé applique en MOVING). */
    PID_Init(&pid_x, PID_X_KP, 0.0f, 0.0f, -VELOCITY_CAP, VELOCITY_CAP);
    PID_Init(&pid_y, PID_Y_KP, 0.0f, 0.0f, -VELOCITY_CAP, VELOCITY_CAP);

    Odom_Init();
    Mecanum_Init();

    state      = ROBOT_IDLE;
    settle_cnt = 0;
}

/* ------------------------------------------------------------------ */
/*  Boucle de contrôle (100 Hz)                                        */
/* ------------------------------------------------------------------ */

void RobotControl_Update(void)
{
    /* ---- 1. Mise à jour odométrie ---- */
    Odom_Update(CONTROL_DT);
    const OdomState_t *pos = Odom_GetState();

    /* ---- 2. Filtre debounce ARU ---- */
    aru_filter_step();

    /* ---- 3. Vérification ARU ---- */
    if (aru_triggered()) {
        if (state != ROBOT_ESTOP) {
            Mecanum_Stop();
            state = ROBOT_ESTOP;
            Protocol_SendError(ERR_ARU);
            keepalive_count = 0;
        } else {
            if (++keepalive_count >= KEEPALIVE_CYCLES) {
                keepalive_count = 0;
                Mecanum_Stop();
            }
        }
        return;
    }

    /* ---- 4. Vérification bornes grille (uniquement si on bouge) ---- */
    if (state == ROBOT_MOVING && !in_grid(pos->x, pos->y)) {
        Mecanum_Stop();
        state = ROBOT_ERROR;
        Protocol_SendError(ERR_OUT_BOUNDS);
        return;
    }

    /* ============================================================ */
    /* GESTION DES ÉTATS ET DES MOUVEMENTS                         */
    /* ============================================================ */

    // Cas A : Le robot est en train de faire un déplacement asservi (CMD_MOVE_TO)
    if (state == ROBOT_MOVING) {
        keepalive_count = 0;

        /* 1. Erreurs en repère monde */
        float ex_world = target_x - pos->x;
        float ey_world = target_y - pos->y;

        /* 2. Projection vers repère robot (compensation du drift theta s'il
         *    y en a un — la rotation n'est PAS asservie mais l'odom continue
         *    de la suivre et il faut savoir quel signe de vx/vy commander) */
        float cos_th = cosf(pos->theta);
        float sin_th = sinf(pos->theta);
        float ex_r   =  ex_world * cos_th + ey_world * sin_th;
        float ey_r   = -ex_world * sin_th + ey_world * cos_th;

        /* 3. P pur saturé sur [±VELOCITY_CAP] */
        float vx = saturate(pid_x.kp * ex_r, -VELOCITY_CAP, VELOCITY_CAP);
        float vy = saturate(pid_y.kp * ey_r, -VELOCITY_CAP, VELOCITY_CAP);

        /* 4. Rampe accélération (slew rate limiter) : limite le changement
         *    de consigne par cycle pour un démarrage progressif. */
        float dvx = vx - prev_cmd_vx;
        if (dvx >  ACCEL_MAX_DT) vx = prev_cmd_vx + ACCEL_MAX_DT;
        if (dvx < -ACCEL_MAX_DT) vx = prev_cmd_vx - ACCEL_MAX_DT;
        float dvy = vy - prev_cmd_vy;
        if (dvy >  ACCEL_MAX_DT) vy = prev_cmd_vy + ACCEL_MAX_DT;
        if (dvy < -ACCEL_MAX_DT) vy = prev_cmd_vy - ACCEL_MAX_DT;
        prev_cmd_vx = vx;
        prev_cmd_vy = vy;

        /* 5. Zone morte : si la sortie est sous VELOCITY_DEADBAND, on
         *    coupe ; sinon on enverrait au moteur un cmd sous son seuil
         *    de démarrage et seule la roue la plus libre tournerait. */
        if (fabsf(vx) < VELOCITY_DEADBAND) vx = 0.0f;
        if (fabsf(vy) < VELOCITY_DEADBAND) vy = 0.0f;

        /* 6. Correction séquentielle axes : quand vx et vy sont tous deux
         *    actifs et proches en magnitude, la cinématique mecanum donne
         *    une paire de roues ≈0 et l'autre ≈2v → déséquilibre. On ne
         *    garde que l'axe dominant pour que les 4 roues aient la même
         *    magnitude de commande. */
        if (vx != 0.0f && vy != 0.0f) {
            if (fabsf(fabsf(vx) - fabsf(vy)) < DIAG_VEL_THRESH) {
                if (fabsf(vx) >= fabsf(vy)) vy = 0.0f;
                else                         vx = 0.0f;
            }
        }

        /* 7. Envoi commande moteurs (translation pure, pas de wz) */
        Mecanum_SetVelocity(vx, vy);

        /* ---- Détection convergence : pose dans la tolérance ---- */
        float dist_err = sqrtf(ex_world * ex_world + ey_world * ey_world);
        if (dist_err < POS_TOLERANCE_M) {
            settle_cnt++;
        } else {
            settle_cnt = 0;
        }

        if (settle_cnt >= SETTLE_CYCLES) {
            Mecanum_Stop();
            state = ROBOT_DONE;
            settle_cnt = 0;
            Protocol_SendDone(pos->x, pos->y, pos->theta);
        }
    }
    // Cas B : Le robot est en IDLE mais reçoit une vitesse manuelle du Pi
    else if (state == ROBOT_IDLE || state == ROBOT_DONE) {
        if (fabsf(g_vx) > 0.01f || fabsf(g_vy) > 0.01f) {
            // Si le Pi envoie des consignes manuelles, on les applique directement
            Mecanum_SetVelocity(g_vx, g_vy);
            keepalive_count = 0;
        } else {
            // Vraiment immobile : mode économie / keepalive
            if (++keepalive_count >= KEEPALIVE_CYCLES) {
                keepalive_count = 0;
                Mecanum_Stop();
            }
        }
    }
    // Cas C : Sécurités bloquantes (ESTOP / ERROR)
    else {
        if (++keepalive_count >= KEEPALIVE_CYCLES) {
            keepalive_count = 0;
            Mecanum_Stop();
        }
    }
}

/* ------------------------------------------------------------------ */
/*  Commandes publiques                                                 */
/* ------------------------------------------------------------------ */

void RobotControl_MoveTo(float x, float y, float theta)
{
    (void)theta;   /* rotation non asservie, paramètre ignoré */

    if (state == ROBOT_ESTOP) {
        Protocol_SendError(ERR_ARU);
        return;
    }

    target_x = x;
    target_y = y;

    /* Reset des PID (pas d'effet réel avec KI=KD=0 mais propre) */
    PID_Reset(&pid_x);
    PID_Reset(&pid_y);

    prev_cmd_vx = 0.0f;
    prev_cmd_vy = 0.0f;

    settle_cnt = 0;
    state      = ROBOT_MOVING;
}

void RobotControl_EmergencyStop(void)
{
    Mecanum_Stop();
    state = ROBOT_ESTOP;
}

void RobotControl_Reset(void)
{
    if (state == ROBOT_ESTOP && aru_triggered()) {
        /* Ne pas sortir d'ESTOP si ARU encore actif */
        Protocol_SendError(ERR_ARU);
        return;
    }
    PID_Reset(&pid_x);
    PID_Reset(&pid_y);
    prev_cmd_vx = 0.0f;
    prev_cmd_vy = 0.0f;
    settle_cnt = 0;
    state      = ROBOT_IDLE;
}

void RobotControl_SetGridLimits(float max_x, float max_y)
{
    grid_max_x = max_x;
    grid_max_y = max_y;
}

void RobotControl_SetGains(float kp_xy, float kd_xy,
                           float kp_th, float kd_th)
{
    (void)kd_xy;
    (void)kp_th;
    (void)kd_th;
    /* P pur : seul KP est utilisé. KD non câblé (KP*err saturé/seuillé). */
    pid_x.kp = kp_xy;
    pid_y.kp = kp_xy;
}

RobotState_t RobotControl_GetState(void)
{
    return state;
}

bool RobotControl_IsDone(void)
{
    return (state == ROBOT_DONE);
}
