/**
 * mecanum.h
 * Cinématique inverse et contrôle bas niveau des Sabertooth.
 *
 * Robot mecanum 4 roues SANS rotation : seules les translations vx/vy
 * sont prises en compte (la rotation theta n'est PAS asservie, le robot
 * se déplace en ligne droite sur les 4 axes cardinaux).
 */

#ifndef MECANUM_H
#define MECANUM_H

/* Initialise la communication avec les contrôleurs moteurs */
void Mecanum_Init(void);

/* Applique des vitesses de translation au robot.
 * vx : Translation avant/arrière (m/s, + = avant)
 * vy : Translation latérale     (m/s, + = gauche selon convention firmware)
 */
void Mecanum_SetVelocity(float vx, float vy);

/* Force les 4 moteurs à l'arrêt complet (vitesse = 0) */
void Mecanum_Stop(void);

#endif /* MECANUM_H */
