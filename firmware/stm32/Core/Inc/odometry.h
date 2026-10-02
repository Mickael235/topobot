/**
 * odometry.h
 * Calcul de la position (X, Y, Theta) à partir des 4 encodeurs.
 */

#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <stdint.h>

/* Structure contenant la pose et les vitesses du robot */
typedef struct {
    float x;        /* Position X globale (m) */
    float y;        /* Position Y globale (m) */
    float theta;    /* Angle global (rad, de -PI à PI) */
    
    float vx;       /* Vitesse locale sur l'axe X du robot (m/s) */
    float vy;       /* Vitesse locale sur l'axe Y du robot (m/s) */
    float wz;       /* Vitesse de rotation (rad/s) */
} OdomState_t;

/* Initialise l'odométrie et démarre les timers encodeurs */
void Odom_Init(void);

/* Met à jour la position. À appeler à fréquence fixe. dt = temps en secondes */
void Odom_Update(float dt);

/* Remet X, Y et Theta à zéro */
void Odom_Reset(void);

/* Force une position spécifique (utile pour la calibration initiale) */
void Odom_SetPose(float x, float y, float theta);

/* Récupère un pointeur en lecture seule vers l'état actuel */
const OdomState_t* Odom_GetState(void);

#endif /* ODOMETRY_H */