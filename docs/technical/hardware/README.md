# 2 — Matériel

Ce chapitre décrit les composants du robot et comment ils sont reliés.
Voir aussi :
- [Brochage STM32](Brochage_STM32.md) — tableau complet des broches
- [Câblage](Cablage.md) — détail des bus et des liaisons

## 2.1 Liste des composants principaux

| Composant | Rôle | Interface vers le STM32 |
|-----------|------|--------------------------|
| **Carte STM32 Nucleo F446RE** | Calculateur temps réel | — |
| **Raspberry Pi** | Superviseur (serveur, station, web) | USB → USART2 (via ST‑Link) |
| **2× contrôleurs Sabertooth** | Pilotage des 4 moteurs DC | UART4 (PC10), bus partagé |
| **4× moteurs DC + réducteur** | Entraînement des roues mecanum | via Sabertooth |
| **4× encodeurs en quadrature** | Odométrie | TIM2, TIM3, TIM4, TIM8 |
| **Servo AX12 Dynamixel** | Déploiement du prisme | UART5 (PC12) |
| **Bouton d'arrêt d'urgence (ARU)** | Sécurité | GPIO PB14 |
| **Relais d'alimentation puissance** | Coupe l'alim moteurs | GPIO PA5 (Power_enable) |
| **Module XBee** | Liaison vers la station Topcon | USB sur la Raspberry Pi (FTDI) |
| **Station Topcon MS1AX** | Mesures topographiques | via XBee |
| **Prisme topographique** | Réflecteur visé par la station | monté sur le servo AX12 |
| **Capteur niveau batterie** | Mesure tension (ADC) | PA4 (ADC1_IN4) |

## 2.2 Schéma des liaisons

```
                         ┌───────────────────────────┐
                         │      STM32 F446RE          │
                         │                            │
  Raspberry Pi ──USB────►│ USART2 (PA2/PA3) 115200    │
  (ST-Link série)        │                            │
                         │ UART4  (PC10)  ────────────┼──► Sabertooth #1 (addr 128) ─► moteurs FL, RL
                         │   bus série partagé 9600   ┼──► Sabertooth #2 (addr 129) ─► moteurs FR, RR
                         │                            │
                         │ UART5  (PC12)  1 Mbps ─────┼──► Servo AX12 (prisme)
                         │                            │
                         │ TIM2 (PA0/PA1) ◄───────────┼─── Encodeur FR  🔴 hors service
                         │ TIM3 (PA6/PA7) ◄───────────┼─── Encodeur RR  🟢
                         │ TIM4 (PB6/PB7) ◄───────────┼─── Encodeur FL  🔴 hors service
                         │ TIM8 (PC6/PC7) ◄───────────┼─── Encodeur RL  🟢
                         │                            │
                         │ PB14 ◄─────────────────────┼─── Bouton ARU
                         │ PA5  ────────────────────► relais alim puissance
                         │ PA4  ◄── tension batterie  │
                         └────────────────────────────┘

  Raspberry Pi ──USB(FTDI)──► XBee ))) ((( XBee ──► Station Topcon MS1AX
```

## 2.3 Alimentation

- Une **batterie** alimente la partie puissance (moteurs via Sabertooth).
- Le STM32 active la puissance via le **relais sur PA5** (`Power_enable_stm`).
  Au démarrage le firmware met PA5 à l'état haut (cf. `main.c`, juste après
  l'init) pour autoriser le courant vers les Sabertooth.
- La **tension batterie** est lisible sur l'ADC (PA4 / `Battery_level`). Cette
  mesure n'est pas encore exploitée dans le logiciel — piste d'amélioration
  (cf. [passation](../08_Passation/README.md)).

> ⚠️ Couper la puissance (relais / batterie) avant toute intervention sur le
> câblage. Le bouton ARU coupe la **commande** mais vérifiez toujours
> physiquement.

## 2.4 État du matériel (fin d'année)

| Élément | État | Remarque |
|---------|------|----------|
| Moteurs + Sabertooth | 🟢 | Les 4 roues répondent (voir polarités dans `mecanum.c`) |
| **Les 4 encodeurs** (TIM2/3/4/8) | 🟡 | **Fonctionnent**, mais comptage intermittent à cause de la carte de commande (voir ci‑dessous) |
| Carte de commande (interface encodeurs) | 🟡 | **Problèmes intermittents** : certains tics ne remontent pas tant que la carte/les connecteurs ne sont pas bien en place |
| Servo AX12 | 🟢 | Déploie / replie le prisme |
| Bouton ARU | 🟢 | Avec filtrage anti‑parasites moteurs (debounce) |
| Station Topcon + XBee | 🟡 | Fonctionne mais sensible à la config série / au timing |
| Mesure batterie | 🔴 | Câblée mais non exploitée par le logiciel |

> ⚠️ **Les 4 encodeurs sont fonctionnels.** Le problème vient de la **carte de
> commande** : selon son état (connecteurs, contacts), certains encodeurs
> peuvent **ne pas compter** ponctuellement, ce qui **fausse l'odométrie**.
>
> **PROCÉDURE OBLIGATOIRE avant chaque test** : lancer `test_encoders.py`,
> **tourner chaque roue à la main** une par une, et vérifier que les tics de
> chacune des 4 roues augmentent bien. Si une roue ne compte pas → vérifier /
> repositionner la carte de commande **avant** de continuer, sinon tous les
> déplacements asservis seront faux.
>
> Note : le code firmware actuel (`robot_config.h`, mode `ODOM_REAR_ONLY=1` et
> commentaires « HORS SERVICE ») reflète un **ancien diagnostic** où seuls les
> encodeurs arrière étaient lus. Comme les 4 fonctionnent, ce mode peut être
> repassé à `0` **une fois le comptage des 4 roues confirmé** (voir
> [firmware](../03_Firmware_STM32/Modules.md) et
> [passation](../08_Passation/README.md)).

## 2.5 Cartes électroniques (KiCad)

Les cartes d'interface/alimentation conçues cette année seront documentées dans
le dossier [`07_KiCad/`](../07_KiCad/README.md) : déposez‑y les fichiers
sources KiCad (schémas, routage, BOM, gerbers).
