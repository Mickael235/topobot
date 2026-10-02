# 1 — Vue d'ensemble

## 1.1 Objectif du projet

Concevoir un **robot topographe autonome**. Le robot remplace l'opérateur qui,
classiquement, déplace à la main un prisme (réflecteur) sur le terrain pendant
qu'une station totale relève sa position.

Déroulé visé :

1. L'utilisateur définit une **zone rectangulaire** (largeur × longueur) et un
   **pas de mesure** depuis une interface web.
2. Le robot se déplace automatiquement sur une **grille en serpentin**
   (boustrophédon) couvrant la zone.
3. À chaque point de la grille, il **déploie un prisme** monté sur un servo.
4. La **station Topcon MS1AX** vise le prisme (mode tracking puis mesure
   précise) et relève l'angle horizontal (Hz), l'angle vertical (V) et la
   distance.
5. Les mesures brutes sont **enregistrées** (base SQLite). L'altitude Z sera
   recalculée plus tard par trigonométrie.
6. En fin de parcours, le robot revient à son point de départ.

## 1.2 Architecture matérielle et logicielle

Trois sous‑systèmes, chacun avec son rôle :

### Tablette / PC — *interface utilisateur*
Application web (React) servie par la Raspberry Pi. Permet de configurer la
mission, de la lancer, de suivre la progression et de déclencher l'arrêt
d'urgence. Voir [Interface Web](../04_Raspberry_Pi/Interface_Web.md).

### Raspberry Pi — *chef d'orchestre*
Serveur Python **FastAPI** (« TopoBot Manager »). Il :
- génère les points de la grille (waypoints),
- pilote le STM32 via une liaison série binaire (USART2, 115200 bauds),
- pilote la station Topcon via XBee (liaison série),
- stocke les mesures et les expose à l'interface.

Voir [Raspberry Pi / TopoBot](../04_Raspberry_Pi/README.md).

### STM32 F446RE — *temps réel*
Microcontrôleur qui gère le **bas niveau temps réel** :
- cinématique mecanum + envoi des consignes aux **2 contrôleurs Sabertooth**,
- **odométrie** par les encodeurs des roues,
- asservissement de position (déplacement vers une cible x, y),
- pilotage du **servo AX12** (prisme),
- **arrêt d'urgence** matériel (bouton ARU).

Voir [Firmware STM32](../03_Firmware_STM32/README.md).

## 1.3 Le robot : base mecanum 4 roues

Les **roues mecanum** (à galets inclinés à 45°) permettent au robot de se
déplacer dans toutes les directions sans tourner sur lui‑même : avant/arrière,
latéral (« strafe »), diagonale.

```
        AVANT
   FL ┌───────┐ FR        FL = Front Left   (avant gauche)
      │       │           FR = Front Right  (avant droite)
      │  ↑ x  │           RL = Rear  Left   (arrière gauche)
      │  →y   │           RR = Rear  Right  (arrière droite)
   RL └───────┘ RR
       ARRIÈRE
```

> **Décision de conception importante :** dans la version actuelle, **la
> rotation du robot n'est PAS asservie**. Le robot ne se déplace que sur les
> axes cardinaux (avant/arrière/gauche/droite) en translation pure. C'est un
> choix volontaire pour simplifier et fiabiliser l'asservissement (voir
> [Firmware](../03_Firmware_STM32/README.md) et la
> [passation](../08_Passation/README.md)).

Chaque roue est entraînée par un moteur à courant continu piloté par un
contrôleur **Sabertooth** (2 contrôleurs, 2 moteurs chacun) et équipée d'un
**encodeur en quadrature** pour l'odométrie.

## 1.4 Déroulé détaillé d'une mission (côté logiciel)

Le cœur de la mission est la fonction `websocket_mission` du backend
(`topobot/topobot/backend/main.py`). Pour chaque point de la grille :

1. **Déplacement** `CMD_MOVE_TO(x, y)` vers le point (sauf le tout premier
   point (0,0) où le robot est déjà). Pendant le trajet, la station suit le
   prisme en **mode tracking**.
2. **Arrêt du tracking** pour préparer une mesure précise.
3. **Déploiement du prisme** `CMD_DEPLOY_PRISM` (servo AX12 vers le bas).
4. **Mesure précise** Topcon → Hz, V, distance.
5. **Repli du prisme** `CMD_RETRACT_PRISM`.
6. **Sauvegarde** en base : heure, X, Y, Hz, V, distance.
7. **Reprise du tracking** et passage au point suivant.

À la fin : arrêt du tracking, retour à l'origine (0,0), notification
« mission terminée » à l'interface.

## 1.5 Pour aller plus loin

- Comprendre le **matériel et le câblage** → [chapitre 2](../02_Materiel/README.md)
- Comprendre le **firmware** → [chapitre 3](../03_Firmware_STM32/README.md)
- Comprendre la **Raspberry Pi** → [chapitre 4](../04_Raspberry_Pi/README.md)
- Le **rapport final** (`Rapport_final.pdf`, à la racine) donne le contexte,
  les choix et les résultats détaillés de l'année.
