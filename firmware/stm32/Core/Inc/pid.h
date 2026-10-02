/**
 * pid.h
 * Régulateur PID générique avec anti-windup et filtrage dérivé.
 */

#ifndef PID_H
#define PID_H

/* Structure de contexte pour un régulateur PID */
typedef struct {
    /* Gains */
    float kp;
    float ki;
    float kd;
    
    /* Limites de sortie (ex: -1.0 à 1.0) */
    float output_min;
    float output_max;
    
    /* Mémoire pour l'anti-windup et la dérivée */
    float integral;
    float integral_max;
    float prev_error;
    
    /* Filtre sur l'action dérivée pour lisser le bruit */
    float filtered_deriv;
    float deriv_alpha;
} PID_t;

/* Initialise un PID avec ses gains et ses limites de saturation */
void PID_Init(PID_t *pid, float kp, float ki, float kd, float out_min, float out_max);

/* Calcule la nouvelle sortie du PID. dt = temps en secondes depuis le dernier appel */
float PID_Compute(PID_t *pid, float error, float dt);

/* Réinitialise la mémoire intégrale et dérivée (à faire avant un nouveau mouvement) */
void PID_Reset(PID_t *pid);

/* Modifie les gains à la volée */
void PID_SetGains(PID_t *pid, float kp, float ki, float kd);

#endif /* PID_H */