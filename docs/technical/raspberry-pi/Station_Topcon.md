# Station topographique Topcon MS1AX

La station totale **Topcon MS1AX** mesure, pour chaque visée du prisme :
- **Hz** : angle horizontal (en **gon**),
- **V** : angle vertical (en **gon**),
- **distance** station → prisme (en **mètres**).

Elle communique en **série à 9600 bauds** via une liaison radio **XBee**
(branchée en USB sur la Pi via un adaptateur FTDI → `/dev/ttyUSB0`).

## Fichier de référence : `ms1ax.py`

`ms1ax.py` (à la racine) est le **pilote de référence** de la station, écrit et
validé en début de projet. Utilisation autonome :

```bash
python ms1ax.py <PORT> <mode>
#   PORT : COM2, /dev/ttyUSB0, ...
#   mode : std   -> mode standard de mesure
#          trk   -> mode tracking (poursuite)
#          mes   -> lance une mesure en mode standard
#          suivi -> suivi + mesure continue (stoppé par la touche "q")
```

C'est la **source de vérité** du protocole Topcon. Le backend (`main.py`) en
reprend la logique. En cas de doute sur une commande, référez‑vous à `ms1ax.py`.

## Commandes série de la station (extrait)

| Commande | Effet |
|----------|-------|
| `*PON` | Réveil / mise sous tension. |
| `*RM1` | Mode mesure. |
| `*/PA 1,1,,` | Active la poursuite (tracking). |
| `*/PA 1,0,,` | Désactive la poursuite (mesure précise). |
| `*/PH 1` / `*/PH 0` | Active / désactive un sous‑mode de poursuite. |
| `Xe` / `Xa` | Bascule des modes (entrée/sortie). |
| `*ST2` | Déclenche / configure la sortie de mesure. |
| `*ST0` | Arrête le flux de tracking. |
| `*GLON` / `*GLOFF` | Laser / pointeur ON / OFF. |

> Les séquences exactes (et les temporisations `time.sleep` entre commandes)
> sont **critiques** et ont été calées empiriquement. Voir les fonctions
> `_topcon_*` dans `backend/main.py` et `ms1ax.py`.

## Mode tracking vs mesure précise (dans une mission)

La mission alterne deux modes :

1. **Tracking** (`_topcon_init_tracking`, `_topcon_resume_tracking`) : la
   station **suit le prisme en continu** pendant que le robot se déplace. Cela
   garde la visée verrouillée et accélère la mesure suivante.
2. **Mesure précise** (`_topcon_precise_measure`) : on **arrête le tracking**
   (`_topcon_stop_tracking`), on bascule en mode standard, on allume le laser,
   on déclenche `*ST2`, on lit la trame CSV, on éteint le laser.

## Format de la réponse de mesure

La station renvoie une trame **CSV**. Le parsing extrait :
```
parts = response.split(",")
hz   = float(parts[2])     # angle horizontal (gon)
v    = float(parts[3])     # angle vertical   (gon)
dist = float(parts[4])     # distance (m)
```

Le code de lecture (`_read_topcon_line`) :
- attend une ligne contenant une **virgule** (= trame CSV valide),
- ignore les réponses courtes contenant `E` (station en mouvement / recherche),
- a un **timeout** (par défaut 20 s) car une mesure précise peut être longue.

## Diagnostic de la liaison

Script dédié : **`test_xbee.py`** (voir [catalogue des tests](../05_Tests/README.md)).
Il calque le protocole de `ms1ax.py` et vérifie :
- que le port XBee s'ouvre,
- que la station répond (`\x06` ACK après config),
- qu'une mesure renvoie bien une trame CSV.

```bash
python3 test_xbee.py
```

## Pièges connus

- **Port `/dev/ttyUSB0`** : peut changer de numéro selon l'ordre de branchement
  USB. Vérifiez avec `ls /dev/serial/by-id/`.
- **Temporisations** : si vous réduisez les `time.sleep`, la station peut ne pas
  avoir fini de basculer de mode et ne pas répondre. Ne les raccourcissez pas
  sans tester.
- **Visée** : la station doit « voir » le prisme. En tracking, si elle perd le
  prisme (obstacle, vitesse trop élevée), elle renvoie des erreurs courtes.
- **Z non mesuré** : la station ne donne pas l'altitude directement ; Hz/V/dist
  permettent de la recalculer par trigonométrie (à implémenter).

Voir aussi la [calibration & dépannage](../06_Calibration_Depannage.md) et la
[passation](../08_Passation/README.md).
