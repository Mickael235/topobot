/**
 * pid.c - Régulateur PID avec anti-windup et filtre dérivée
 */
#include "pid.h"
#include <math.h>
#include <string.h>

void PID_Init(PID_t *pid, float kp, float ki, float kd,
              float out_min, float out_max)
{
    memset(pid, 0, sizeof(PID_t));
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->output_min = out_min;
    pid->output_max = out_max;
    /* Anti-windup : limite l'intégrale à la moitié de la plage de sortie */
    pid->integral_max = (out_max - out_min) * 0.5f;
    /* Filtre dérivée : alpha=0.1 (léger filtrage du bruit encodeur) */
    pid->deriv_alpha = 0.1f;
}

float PID_Compute(PID_t *pid, float error, float dt)
{
    if (dt <= 0.0f) return 0.0f;

    /* ---- Terme Proportionnel ---- */
    float P = pid->kp * error;

    /* ---- Terme Intégral avec anti-windup ---- */
    pid->integral += error * dt;
    if (pid->integral >  pid->integral_max) pid->integral =  pid->integral_max;
    if (pid->integral < -pid->integral_max) pid->integral = -pid->integral_max;
    float I = pid->ki * pid->integral;

    /* ---- Terme Dérivé filtré ---- */
    float raw_deriv = (error - pid->prev_error) / dt;
    /* Filtre passe-bas premier ordre : alpha=0 → pas de filtre, 1 → bloqué */
    pid->filtered_deriv = pid->deriv_alpha * pid->filtered_deriv
                        + (1.0f - pid->deriv_alpha) * raw_deriv;
    float D = pid->kd * pid->filtered_deriv;
    pid->prev_error = error;

    /* ---- Sortie saturée ---- */
    float out = P + I + D;
    if (out >  pid->output_max) out =  pid->output_max;
    if (out <  pid->output_min) out =  pid->output_min;

    return out;
}

void PID_Reset(PID_t *pid)
{
    pid->integral      = 0.0f;
    pid->prev_error    = 0.0f;
    pid->filtered_deriv = 0.0f;
}

void PID_SetGains(PID_t *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}
