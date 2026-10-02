# Brochage STM32 F446RE

Tableau complet des broches utilisées, extrait de la configuration CubeMX
(`ProjetRobot.ioc`). Les **étiquettes** (« Label ») sont celles définies dans
le `.ioc` et reprises dans le code.

> Pour modifier le brochage, ouvrez `ProjetRobot.ioc` dans STM32CubeIDE
> (double‑clic) plutôt que d'éditer le code à la main : CubeMX régénère
> ensuite les fonctions d'init dans `main.c`.

## Moteurs (signaux PWM + direction)

> Note : les moteurs sont en réalité pilotés par les **Sabertooth via UART4**
> (protocole série), pas directement par ces PWM. Ces broches PWM/DIR
> proviennent d'une première approche « pont en H direct » et restent câblées
> dans le `.ioc`. Le firmware actuel n'utilise PAS ces PWM pour la marche
> (voir `mecanum.c`).

| Broche | Label | Fonction | Périphérique |
|--------|-------|----------|--------------|
| PA8  | Moteur_1_PWM | PWM moteur 1 | TIM1_CH1 |
| PA9  | Moteur_2_PWM | PWM moteur 2 | TIM1_CH2 |
| PA10 | Moteur_3_PWM | PWM moteur 3 | TIM1_CH3 |
| PA11 | Moteur_4_PWM | PWM moteur 4 | TIM1_CH4 |
| PB1  | Moteur_1_DIR | Direction moteur 1 | GPIO sortie |
| PB2  | Moteur_2_DIR | Direction moteur 2 | GPIO sortie |
| PB3  | Moteur_3_DIR | Direction moteur 3 | GPIO sortie |
| PB4  | Moteur_4_DIR | Direction moteur 4 | GPIO sortie |

## Encodeurs (mode quadrature)

| Broches | Labels | Roue | Timer | État |
|---------|--------|------|-------|------|
| PA0 / PA1 | CodDA / CodDB   | FR (avant droite)   | TIM2 (CH1/CH2) | 🟡 OK, dépend de la carte de commande |
| PA6 / PA7 | CodDA2 / CodDB2 | RR (arrière droite) | TIM3 (CH1/CH2) | 🟡 OK, dépend de la carte de commande |
| PB6 / PB7 | CodGB / CodGA   | FL (avant gauche)   | TIM4 (CH1/CH2) | 🟡 OK, dépend de la carte de commande |
| PC6 / PC7 | CodG2B / CodGA2 | RL (arrière gauche) | TIM8 (CH1/CH2) | 🟡 OK, dépend de la carte de commande |

> **Les 4 encodeurs fonctionnent.** Mais selon l'état de la **carte de
> commande**, le comptage peut être intermittent → **vérifier les 4 roues avec
> `test_encoders.py` avant chaque test** (voir
> [Matériel §2.4](README.md#24-état-du-matériel-fin-dannée)).

> **Attention au mapping** : il a été vérifié expérimentalement par
> `test_motor_mapping.py` le 2026‑05‑22. Les encodeurs suffixés « 2 » sont à
> l'**arrière**. Côté gauche, les voies A/B sont permutées sur les canaux du
> timer → d'où les polarités `ENC_POL_*` négatives dans `robot_config.h`.

## Liaisons série (UART/USART)

| Broche | Label | Fonction | Périphérique | Débit |
|--------|-------|----------|--------------|-------|
| PA2 | data_AX12 *(nom historique)* | TX vers Raspberry Pi | USART2_TX | 115200 |
| PA3 | — | RX depuis Raspberry Pi | USART2_RX | 115200 |
| PC10 | Sabertooth1_TX | TX vers bus Sabertooth | UART4_TX | 9600 |
| PC11 | Sabertooth1_RX | RX bus Sabertooth (inutilisé) | UART4_RX | 9600 |
| PC12 | Sabertooth2_TX *(nom historique)* | TX vers servo AX12 | UART5_TX | 1 000 000 |
| PD2  | Sabertooth2_RX *(nom historique)* | RX AX12 | UART5_RX | 1 000 000 |

> ⚠️ **Pièges de nommage hérités.** Certains labels du `.ioc` ne correspondent
> plus à leur usage réel :
> - `data_AX12` (PA2) est en fait l'**USART2 vers la Raspberry Pi**, pas l'AX12.
> - `Sabertooth2_TX/RX` (PC12/PD2) sont en fait l'**UART5 vers le servo AX12**.
>
> Le code (`robot_config.h`) est la **source de vérité**, pas les labels du
> `.ioc`. Les deux Sabertooth partagent **un seul fil** sur PC10 (UART4) et
> sont distingués par leur adresse (128 / 129).

## GPIO divers

| Broche | Label | Fonction |
|--------|-------|----------|
| PA5  | Power_enable_stm | Active le relais d'alimentation puissance (sortie) |
| PA4  | Battery_level | Mesure tension batterie (ADC1_IN4) |
| PB14 | ARU_signal | Entrée bouton arrêt d'urgence (pull‑up, actif à l'état bas) |

## Horloge et debug

| Broche | Fonction |
|--------|----------|
| PH0 / PH1 | Oscillateur (OSC_IN / OSC_OUT) |
| PA13 / PA14 | SWD debug (SWDIO / SWCLK) |

## Timers — récapitulatif des rôles

| Timer | Rôle |
|-------|------|
| TIM1 | PWM moteurs (hérité, non utilisé pour la marche actuelle) |
| TIM2 | Encodeur FR 🟡 (dépend de la carte de commande) |
| TIM3 | Encodeur RR 🟡 (dépend de la carte de commande) |
| TIM4 | Encodeur FL 🟡 (dépend de la carte de commande) |
| TIM6 | **Base de temps 100 Hz** de la boucle de contrôle (configuré à la main dans `main.c`, PAS dans le `.ioc`) |
| TIM8 | Encodeur RL 🟡 (dépend de la carte de commande) |

> **À retenir** : TIM6 n'est **pas** généré par CubeMX. Il est initialisé
> manuellement dans `main.c` (`MX_TIM6_Init`). Si vous régénérez le projet
> depuis le `.ioc`, vérifiez que ce code n'a pas disparu, sinon la boucle de
> contrôle 100 Hz ne tourne plus.
