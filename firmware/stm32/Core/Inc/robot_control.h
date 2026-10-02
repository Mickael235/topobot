/**
 * robot_control.h
 * Superviseur de haut niveau pour les déplacements du robot.
 */

#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

/* États de la machine d'état du robot */
typedef enum {
    ROBOT_IDLE = 0,     /* En attente de commande */
    ROBOT_MOVING,       /* En cours de déplacement vers une cible */
    ROBOT_SETTLING,     /* Sur la cible, en attente de stabilisation (optionnel) */
    ROBOT_DONE,         /* Cible atteinte et stabilisée */
    ROBOT_ESTOP,        /* Arrêt d'urgence (ARU matériel ou logiciel) */
    ROBOT_ERROR         /* Erreur (hors limites, etc.) */
} RobotState_t;

/* Initialisation globale (lance aussi PID, Odom, Mecanum) */
void RobotControl_Init(void);

/* Fonction principale à appeler à fréquence fixe (ex: 100Hz via TIM6) */
void RobotControl_Update(void);

/* Donne une nouvelle consigne absolue (x, y en mètres, theta en radians) */
void RobotControl_MoveTo(float x, float y, float theta);

/* Arrête les moteurs immédiatement et passe en ESTOP */
void RobotControl_EmergencyStop(void);

/* Sort de l'état d'erreur/ESTOP (si l'ARU physique est relâché) et repasse en IDLE */
void RobotControl_Reset(void);

/* Modifie les limites de la grille de sécurité (en mètres) */
void RobotControl_SetGridLimits(float max_x, float max_y);

/* Met à jour les gains PID à la volée */
void RobotControl_SetGains(float kp_xy, float kd_xy, float kp_th, float kd_th);

/* Récupère l'état actuel pour la télémétrie */
RobotState_t RobotControl_GetState(void);

/* Vérifie si le robot a terminé son mouvement */
bool RobotControl_IsDone(void);

#endif /* ROBOT_CONTROL_H */