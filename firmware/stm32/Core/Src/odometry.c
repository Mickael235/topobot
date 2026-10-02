/**
 * odometry.c - Odométrie mecanum
 *
 * Convention d'indice encodeur (vérifié par test_motor_mapping.py 2026-05-22) :
 *   [0] = Front Left  (FL)  → TIM4  (PB6/PB7, CodG)   — HORS SERVICE
 *   [1] = Front Right (FR)  → TIM2  (PA0/PA1, CodD)   — HORS SERVICE
 *   [2] = Rear  Left  (RL)  → TIM8  (PC6/PC7, CodG2)  — fonctionne
 *   [3] = Rear  Right (RR)  → TIM3  (PA6/PA7, CodD2)  — fonctionne
 *
 * Mode ODOM_REAR_ONLY=1 : seuls les 2 encodeurs arrière sont utilisés.
 *   dm_RL ∝ (vx + vy),  dm_RR ∝ (vx - vy)
 *   → dx  = (dm_RL + dm_RR) / 2
 *   → dy  = (dm_RL - dm_RR) / 2
 *   → dθ  = (-dm_RL + dm_RR) / (2*(lx+ly))
 */

#include "odometry.h"
#include "robot_config.h"
#include <string.h>
#include <math.h>

/* Handles timers - définis dans main.c par CubeMX */
extern TIM_HandleTypeDef ENC_FL_TIM;
extern TIM_HandleTypeDef ENC_FR_TIM;
extern TIM_HandleTypeDef ENC_RL_TIM;
extern TIM_HandleTypeDef ENC_RR_TIM;

/* ---- State ---- */
static OdomState_t odom;
static int32_t     enc_prev[4];   /* Valeurs CNT précédentes */

/* Lit le compteur d'un timer en gérant les overflow 16 bits */
static inline int32_t enc_delta(TIM_HandleTypeDef *htim, int32_t *prev)
{
    int32_t cnt = (int32_t)(uint16_t)__HAL_TIM_GET_COUNTER(htim);
    int32_t delta = cnt - *prev;

    /* Correction d'overflow : le CNT est 16 bits (0…65535) */
    if (delta >  32767) delta -= 65536;
    if (delta < -32768) delta += 65536;

    *prev = cnt;
    return delta;
}

void Odom_Init(void)
{
    memset(&odom, 0, sizeof(odom));
    memset(enc_prev, 0, sizeof(enc_prev));

    /* Démarrer les timers encodeurs (CubeMX les initialise, on les lance) */
    HAL_TIM_Encoder_Start(&ENC_FL_TIM, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&ENC_FR_TIM, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&ENC_RL_TIM, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&ENC_RR_TIM, TIM_CHANNEL_ALL);

    /* Remettre les compteurs à zéro */
    __HAL_TIM_SET_COUNTER(&ENC_FL_TIM, 0);
    __HAL_TIM_SET_COUNTER(&ENC_FR_TIM, 0);
    __HAL_TIM_SET_COUNTER(&ENC_RL_TIM, 0);
    __HAL_TIM_SET_COUNTER(&ENC_RR_TIM, 0);
}

void Odom_Update(float dt)
{
    /* 1. Lire les deltas encodeurs [ticks] */
    int32_t dtick[4];
    dtick[0] = enc_delta(&ENC_FL_TIM, &enc_prev[0]);
    dtick[1] = enc_delta(&ENC_FR_TIM, &enc_prev[1]);
    dtick[2] = enc_delta(&ENC_RL_TIM, &enc_prev[2]);
    dtick[3] = enc_delta(&ENC_RR_TIM, &enc_prev[3]);

    /* 2. Appliquer polarité encodeurs (compense A/B swappé côté gauche) */
    static const int32_t enc_pol[4] = { ENC_POL_FL, ENC_POL_FR, ENC_POL_RL, ENC_POL_RR };
    float dm[4];
    float tpm = TICKS_PER_METER;
    for (int i = 0; i < 4; i++) {
        dm[i] = (float)(dtick[i] * enc_pol[i]) / tpm;
    }

    /* 3. Cinématique directe : déplacement en repère robot */
    float lxy = LXY;
#if ODOM_REAR_ONLY
    float dx_r   = (dm[2] + dm[3]) * 0.5f;
    float dy_r   = (dm[2] - dm[3]) * 0.5f;
    /* Theta forcé à 0 : avec seulement 2 encodeurs, le calcul de dtheta
     * génère un drift fantôme massif pendant les strafes (un encodeur
     * tombe à zéro → le firmware "voit" une rotation qui n'existe pas).
     * Puisque la rotation n'est ni asservie ni commandée, on l'ignore. */
    float dtheta = 0.0f;
#else
    float dx_r   = ( dm[0] + dm[1] + dm[2] + dm[3]) * 0.25f;
    float dy_r   = (-dm[0] + dm[1] + dm[2] - dm[3]) * 0.25f;
    float dtheta = (-dm[0] + dm[1] - dm[2] + dm[3]) / (4.0f * lxy);
#endif

    /* 4. Vitesses robot frame (pour télémétrie) */
    if (dt > 1e-6f) {
        odom.vx = dx_r / dt;
        odom.vy = dy_r / dt;
        odom.wz = dtheta / dt;
    }

    /* 5. Intégration en repère monde (RK2 – milieu d'intervalle) */
    float th_mid = odom.theta + dtheta * 0.5f;
    float cos_m  = cosf(th_mid);
    float sin_m  = sinf(th_mid);

    odom.x     += dx_r * cos_m - dy_r * sin_m;
    odom.y     += dx_r * sin_m + dy_r * cos_m;
    odom.theta += dtheta;

    /* 6. Normalisation angle [-π, π] */
    while (odom.theta >  (float)M_PI) odom.theta -= 2.0f * (float)M_PI;
    while (odom.theta < -(float)M_PI) odom.theta += 2.0f * (float)M_PI;
}

void Odom_Reset(void)
{
    memset(&odom, 0, sizeof(odom));
    memset(enc_prev, 0, sizeof(enc_prev));

    __HAL_TIM_SET_COUNTER(&ENC_FL_TIM, 0);
    __HAL_TIM_SET_COUNTER(&ENC_FR_TIM, 0);
    __HAL_TIM_SET_COUNTER(&ENC_RL_TIM, 0);
    __HAL_TIM_SET_COUNTER(&ENC_RR_TIM, 0);
}

const OdomState_t* Odom_GetState(void)
{
    return &odom;
}

void Odom_SetPose(float x, float y, float theta)
{
    odom.x     = x;
    odom.y     = y;
    odom.theta = theta;
    odom.vx = odom.vy = odom.wz = 0.0f;
}
