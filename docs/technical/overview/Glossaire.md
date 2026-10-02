# Glossaire

Tous les sigles, termes et abréviations rencontrés dans le projet et le code.

| Terme | Signification |
|-------|---------------|
| **ARU** | Arrêt d'Urgence — bouton physique (broche PB14) qui coupe les moteurs |
| **AX12 / Dynamixel** | Servomoteur intelligent (Robotis) commandant le bras du prisme |
| **Backend** | Le serveur Python FastAPI tournant sur la Raspberry Pi |
| **Boustrophédon** | Parcours en serpentin (aller‑retour ligne par ligne) ; cf. labour |
| **CRC** | Code de contrôle d'intégrité d'une trame (ici un simple XOR) |
| **Encodeur en quadrature** | Capteur qui compte la rotation d'une roue (2 voies A/B) |
| **FastAPI** | Framework web Python utilisé pour le backend |
| **Firmware** | Le programme embarqué dans le STM32 (dossier `Core/`) |
| **FL / FR / RL / RR** | Front‑Left / Front‑Right / Rear‑Left / Rear‑Right (les 4 roues) |
| **Frontend** | L'interface web (React) affichée sur la tablette/PC |
| **gon (grade)** | Unité d'angle de la station Topcon (400 gon = 360°) |
| **HAL** | Hardware Abstraction Layer — bibliothèque STMicroelectronics |
| **Hz** | Ici : **angle Horizontal** mesuré par la station (à ne pas confondre avec les hertz) |
| **Mecanum** | Roues à galets inclinés permettant le déplacement omnidirectionnel |
| **MS1AX** | Modèle de station totale robotisée Topcon utilisée |
| **Odométrie** | Estimation de la position (x, y, θ) à partir des encodeurs |
| **Pose** | Position + orientation du robot : (x, y, θ) |
| **PPR** | Pulses Per Revolution — impulsions par tour d'un encodeur |
| **Prisme** | Réflecteur visé par la station pour mesurer angle et distance |
| **Quadrature ×4** | Comptage exploitant les 4 fronts des voies A et B (×4 résolution) |
| **Sabertooth** | Contrôleur de puissance pilotant 2 moteurs DC (mode « Packetized Serial ») |
| **Slew rate / rampe** | Limitation de la variation de consigne par cycle (démarrage doux) |
| **STM32 F446RE** | Microcontrôleur ARM Cortex‑M4 utilisé (carte Nucleo) |
| **Strafe** | Déplacement latéral pur (selon y) sans rotation |
| **Tracking** | Mode où la station suit en continu le prisme en mouvement |
| **TopoBot** | Nom du projet / de l'application Raspberry Pi |
| **Topcon** | Fabricant de la station topographique |
| **UART / USART** | Liaisons série utilisées (Pi↔STM32, STM32↔Sabertooth, STM32↔AX12) |
| **V** | **angle Vertical** mesuré par la station |
| **Waypoint** | Point de passage de la grille (coordonnée x, y à atteindre) |
| **XBee** | Module radio/série reliant la Raspberry Pi à la station Topcon |

## Repères et conventions

- **Repère monde** : repère fixe lié au point de départ. Origine (0,0) = position
  du robot au lancement de la mission. X = avant, Y = gauche.
- **Repère robot** : repère mobile lié au châssis. Le firmware projette les
  erreurs de position du repère monde vers le repère robot pour décider quelles
  vitesses vx/vy commander.
- **Unités** : mètres (m), mètres par seconde (m/s), radians (rad) côté firmware.
  La station Topcon travaille en **gon** (angles) et **mètres** (distances).
