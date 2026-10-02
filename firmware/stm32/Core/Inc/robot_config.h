/**
 * robot_config.h
 * Configuration matérielle du robot - STM32F446RETx
 * À adapter selon votre câblage exact.
 *
 * Encodeurs (vérifié par test_motor_mapping.py 2026-05-22) :
 *   ENC FR : TIM2  (PA0=CH1, PA1=CH2) -> CodDA/CodDB    — HORS SERVICE
 *   ENC RR : TIM3  (PA6=CH1, PA7=CH2) -> CodDA2/CodDB2  — fonctionne
 *   ENC FL : TIM4  (PB6=CH1, PB7=CH2) -> CodGB/CodGA    — HORS SERVICE
 *   ENC RL : TIM8  (PC6=CH1, PC7=CH2) -> CodG2B/CodGA2  — fonctionne
 *
 *   Sabertooth 1 (FL+RL) : UART4   PC10=TX (S1 commun aux 2 Sabertooth)
 *   Sabertooth 2 (FR+RR) : UART4   PC10=TX (même fil, distingué par adresse 129)
 *   AX12 Dynamixel       : UART5   PC12=TX, PD2=RX
 *
 *   Raspberry Pi UART   : USART2  (PA2, full-duplex via ST-Link)
 *   AX12 Dynamixel      : UART5   (PC12=TX, PD2=RX)
 *
 *   ARU signal          : PB14
 */

#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include "main.h"

/* ===================== HANDLES TIMERS ENCODEURS ===================== */
/* Mapping vérifié par test_motor_mapping.py (2026-05-22) :
 *   Les encodeurs suffixés "2" (CodG2, CodD2) sont à l'ARRIÈRE.
 *   Les encodeurs sans suffixe (CodG, CodD) sont à l'AVANT.
 *
 *   TIM2  (PA0/PA1) = CodDA/CodDB   → FR (Front Right)  — HORS SERVICE
 *   TIM3  (PA6/PA7) = CodDA2/CodDB2 → RR (Rear Right)   — fonctionne
 *   TIM4  (PB6/PB7) = CodGB/CodGA   → FL (Front Left)   — HORS SERVICE
 *   TIM8  (PC6/PC7) = CodG2B/CodGA2 → RL (Rear Left)    — fonctionne
 *
 * NOTE : seuls TIM8 (RL) et TIM3 (RR) produisent des ticks.
 *        TIM4 (FL) et TIM2 (FR) sont en panne ou déconnectés.
 *        L'odométrie utilise uniquement les encodeurs arrière (ODOM_REAR_ONLY).
 */
#define ENC_FL_TIM      htim4   /* Front Left  = CodG  (Gauche)   — HORS SERVICE */
#define ENC_FR_TIM      htim2   /* Front Right = CodD  (Droite)   — HORS SERVICE */
#define ENC_RL_TIM      htim8   /* Rear Left   = CodG2 (Gauche 2) — fonctionne */
#define ENC_RR_TIM      htim3   /* Rear Right  = CodD2 (Droite 2) — fonctionne */

/* Polarité comptage encodeurs : +1 si ticks positifs = roue avance,
 * -1 si le compteur est inversé (signaux A/B permutés sur les canaux timer).
 * Droite (TIM2, TIM3) : CodDA sur CH1, CodDB sur CH2 → A/B normal  → +1
 * Gauche (TIM4, TIM8) : CodGB sur CH1, CodGA sur CH2 → A/B swappé  → -1
 * Vérifié par test_motor_mapping.py (Test C) : RL et RR cohérents en "avant". */
#define ENC_POL_FL      (-1)   /* TIM4 Gauche  : A/B swappé → inverser (HORS SERVICE) */
#define ENC_POL_FR      (-1)   /* TIM2 Droite  : compteur décroît en avant → inverser (HORS SERVICE) */
#define ENC_POL_RL      (-1)   /* TIM8 Gauche2 : A/B swappé → inverser */
#define ENC_POL_RR      (+1)   /* TIM3 Droite2 : A/B normal */

/* Période ARR des timers encodeurs (CubeMX → TIMx→Counter Period) */
#define ENC_TIMER_ARR   65535

/* ===================== HANDLES UART ===================== */
/* Bus Sabertooth Packetized Serial : S1 des deux Sabertooth en commun
 * sur PC10 (UART4_TX). Les deux contrôleurs sont distingués par leur
 * adresse (128 / 129) sur le même fil. */
#define SABRE1_UART         huart4   /* Sabertooth 1 (addr 128) : FL + RL */
#define SABRE2_UART         huart4   /* Sabertooth 2 (addr 129) : FR + RR — MÊME bus */
#define RASP_UART           huart2   /* Liaison série Raspberry Pi */
#define RASP_UART_BAUDRATE  115200U  /* Baud rate Raspberry Pi */
#define AX12_UART           huart5   /* UART dédiée AX12 Dynamixel (PC12=TX, PD2=RX) */

/* Adresses Sabertooth (mode Packetized Serial, configurées sur les DIP) */
#define SABRE1_ADDR     128
#define SABRE2_ADDR     129

/* ===================== AX12 DYNAMIXEL - UART5 DÉDIÉ =====================
 * L'AX12 utilise UART5 (PC12=TX, PD2=RX) — UART dédiée, pas de conflit
 * avec la Raspberry Pi (USART2).
 * ============================================================================= */
#define AX12_BAUDRATE       1000000U /* Baud rate AX12 (defaut usine = 1 Mbps) */
#define AX12_ID_PRISM       2        /* ID Dynamixel du servo porte-prisme */
#define AX12_POS_DEPLOY     700U     /* Position déployée  (0=0°, 1023=300°) */
#define AX12_POS_RETRACT    200U     /* Position rentrée */
#define AX12_TIMEOUT_MS     100U     /* Timeout réponse servo (ms) */

/* ===================== GPIO ARU ===================== */
#define ARU_GPIO_Port   GPIOB
#define ARU_GPIO_Pin    GPIO_PIN_14

/* ===================== ODOMÉTRIE : MODE ENCODEURS ===================== */
/* Seuls les encodeurs arrière (RL=TIM8, RR=TIM3) fonctionnent.
 * Mettre à 0 quand les encodeurs avant (FL=TIM4, FR=TIM2) seront réparés. */
#define ODOM_REAR_ONLY      1

/* ===================== DIMENSIONS ROBOT ===================== */
/* Rayon de roue (m) — À MESURER PRÉCISÉMENT : diamètre d'une roue / 2 */
#define WHEEL_RADIUS_M      0.076f    /* 76 mm (3 pouces) — vérifier ! */

/* Demi-empattement X (avant-arrière depuis centre, m) */
#define WHEEL_BASE_LX       0.110f

/* Demi-empattement Y (gauche-droite depuis centre, m) */
#define WHEEL_BASE_LY       0.18725f

/* (lx + ly) utilisé dans les cinématiques */
#define LXY                 (WHEEL_BASE_LX + WHEEL_BASE_LY)

/* ===================== ENCODEURS ===================== */
/* Impulsions par tour (PPR × 4 en mode quadrature ×4) */
#define ENCODER_PPR         12      /* lignes physiques de l'encodeur */
#define ENCODER_COUNTS      (ENCODER_PPR * 4)

/* Rapport de réduction boîte */
#define GEAR_RATIO          51.0f

/* Multiplicateur empirique pour aligner l'odométrie sur la mesure physique.
 * Remis à 1.0 après correction du mapping encodeurs (2026-05-22).
 * L'ancienne valeur 4.64 compensait un mapping erroné (1 seul encodeur lisible).
 * À recalibrer : pousser le robot sur 1 m mesuré au mètre et ajuster pour
 * que l'odom affiche ~1.00 m. Si odom < physique → réduire. Si odom > → augmenter. */
#define TICKS_PER_METER_CALIB   1.0f

/* Ticks encodeur par mètre de déplacement de roue (calibré) */
#define TICKS_PER_METER     (TICKS_PER_METER_CALIB * \
                             (float)(ENCODER_COUNTS * GEAR_RATIO) / \
                             (2.0f * 3.14159265f * WHEEL_RADIUS_M))

/* ===================== CONTROLEUR POSITION (P pur saturé) =====================
 * Contrôleur proportionnel saturé avec zone morte :
 *   vx = sat(KP * (target_x - x), ±VELOCITY_CAP) ; |vx| < DEADBAND -> 0
 *
 * Pas de I, pas de D :
 *   - Pas d'intégrale -> pas de windup, pas de dérive a l'arrêt.
 *   - Pas de dérivée  -> pas de pic au démarrage qui saturait MAX_VX.
 *
 * KP est dimensionné pour que la sortie soit dans [DEADBAND, VELOCITY_CAP]
 * sur la plage d'erreur utile :
 *   error 0.10 m  -> vx = 0.10 m/s (saturé)
 *   error 0.05 m  -> vx = 0.05 m/s
 *   error 0.015 m -> vx = 0.015 m/s (a la limite de la zone morte)
 *   error < 0.015 -> vx = 0  (la zone morte coupe avant POS_TOLERANCE_M)
 */
#define PID_X_KP    3.0f
#define PID_X_KI    0.0f      /* non utilisé (P pur) */
#define PID_X_KD    0.0f      /* non utilisé (P pur) */

#define PID_Y_KP    3.0f
#define PID_Y_KI    0.0f
#define PID_Y_KD    0.0f

/* Gains theta laissés à 0 — la rotation n'est pas asservie. Les defines
 * restent pour ne pas casser robot_config.h en cas de réactivation. */
#define PID_TH_KP   0.0f
#define PID_TH_KI   0.0f
#define PID_TH_KD   0.0f

/* ===================== LIMITES VITESSES ASSERVISSEMENT =====================
 * VELOCITY_CAP = 0.10 m/s : seuil empiriquement validé sur les tests
 *   open-loop (SET_VELOCITY) — à cette vitesse, les 4 roues répondent
 *   uniformément. Au-dessus on observait des roues qui ne tournaient pas.
 * VELOCITY_DEADBAND = 0.015 m/s : en dessous, on force à 0 pour ne pas
 *   envoyer un cmd qui tomberait sous le seuil de démarrage moteur
 *   (sinon une seule roue répond, le robot derive en biais).
 */
#define VELOCITY_CAP        0.10f    /* m/s — plafond sortie controleur */
#define VELOCITY_DEADBAND   0.015f   /* m/s — sortie sous ce seuil -> 0 */

/* ===================== RAMPE ACCÉLÉRATION =====================
 * Limite le changement de consigne par cycle pour démarrer en douceur.
 * 0.40 m/s² → 250 ms pour atteindre VELOCITY_CAP (0.10 m/s). */
#define ACCEL_MAX           0.40f    /* m/s² — accélération max */
#define ACCEL_MAX_DT        (ACCEL_MAX / (float)CONTROL_FREQ_HZ)  /* m/s par cycle */

/* Seuil correction séquentielle axes : quand |vx| et |vy| sont tous deux
 * non nuls et proches en magnitude (diff < seuil), on ne garde que le plus
 * grand pour éviter l'annulation diagonale mecanum (une paire de roues ≈0
 * tandis que l'autre tourne → déséquilibre). */
#define DIAG_VEL_THRESH     0.050f   /* m/s */

/* MAX_VX/VY/WZ utilisés par Mecanum_SetVelocity pour la normalisation
 * Sabertooth (cmd 0.40 m/s -> 100% Sabertooth duty). Ne PAS modifier
 * sans recalibrer. */
#define MAX_VX          0.40f    /* m/s translation avant-arrière (norm Sabertooth) */
#define MAX_VY          0.40f    /* m/s translation latérale */
#define MAX_WZ          1.20f    /* rad/s rotation (inutilisé, mecanum sans wz) */

/* Vitesse minimale (zone morte moteur Sabertooth ≈ 2-5% selon moteur).
 * Baissée à 0.02 pour permettre des commandes plus fines en fin de
 * convergence. Si certaines roues ne démarrent pas a cette valeur,
 * remonter a 0.03 ou 0.04. */
#define MIN_MOTOR_NORM  0.02f

/* ===================== SEUILS ARRIVÉE =====================
 * POS_TOLERANCE_M doit être > DEADBAND/KP (zone d'arrêt effectif).
 * Avec KP=3.0 et DEADBAND=0.015 : arrêt à |err| < 0.005 m par axe.
 * dist_stop = sqrt(2)*0.005 = 0.007 m. Tolerance 0.015 donne 2x marge. */
#define POS_TOLERANCE_M     0.015f   /* 1.5 cm */
#define ANGLE_TOLERANCE_RAD 0.017f   /* ~1° (non utilisé, theta non asservi) */

/* Nombre de cycles consécutifs dans la tolérance avant validation */
#define SETTLE_CYCLES       20       /* 20 × 10 ms = 200 ms de stabilité */

/* ===================== BOUCLE DE CONTRÔLE ===================== */
/* Période appelée depuis TIM6 (ou SysTick) à 100 Hz */
#define CONTROL_FREQ_HZ     100
#define CONTROL_DT          (1.0f / CONTROL_FREQ_HZ)

/* ===================== PROTECTION GRILLE ===================== */
/* Marge de sécurité autour de la grille (m) - stoppe si dépassé */
#define GRID_MARGIN_M       0.05f    /* 5 cm */

#endif /* ROBOT_CONFIG_H */
