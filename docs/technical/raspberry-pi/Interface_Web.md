# Interface Web (Frontend)

Application **React** (un seul composant `App.jsx`), construite avec **Vite** et
stylée avec **Tailwind CSS**. Servie en statique par le backend (le build est
dans `frontend/dist/`).

Code : `topobot/topobot/frontend/src/App.jsx`

## Écrans

L'interface est une machine à 3 vues (`currentView`) :

1. **Connexion** (`connection`) — bouton « Connecter » (vérification de liaison).
2. **Configuration** (`config`) — saisie de :
   - largeur X (m),
   - longueur Y (m),
   - pas de mesure (m),
   puis « Démarrer la mission ».
3. **Mission** (`mission`) — suivi en temps réel :
   - position courante du robot,
   - barre de progression,
   - journal (logs) en direct,
   - points mesurés (X, Y, Hz, V, distance),
   - indicateur de **tracking** actif,
   - gros **bouton rouge d'arrêt d'urgence**.

## Communication avec le backend

- **REST** : `POST /start-mission` (envoie la config), `POST /stop` (arrêt
  d'urgence).
- **WebSocket** `/ws` : ouvert au lancement de la mission. Le backend pousse des
  messages typés que le frontend interprète :

| `type` | Contenu | Effet UI |
|--------|---------|----------|
| `log` | message texte | ajouté au journal |
| `tracking` | `active: bool` | met à jour l'indicateur tracking |
| `position` | x, y, hz, v, dist, time, progress | ajoute un point + progression |
| `aborted` | — | mission interrompue |
| `done` | — | mission terminée |
| `error` | message | affiche l'erreur |

- **Abort** : le bouton d'arrêt envoie d'abord `{action: "abort"}` sur le
  WebSocket (best effort), puis appelle `POST /stop` (chemin fiable). Double
  sécurité.

## Construire / servir

```bash
cd topobot/topobot/frontend
npm install
npm run build          # → frontend/dist/  (servi par le backend FastAPI)
```

Pour développer l'interface en direct (hot reload), sans reconstruire à chaque
fois :

```bash
npm run dev            # serveur de dev Vite (port 5173 par défaut)
```

> En mode `npm run dev`, pensez à configurer le proxy Vite (`vite.config.js`)
> ou les URLs pour qu'elles pointent vers le backend (port 8000), sinon les
> appels REST/WebSocket échouent.

## Pistes d'amélioration (frontend)

- Affichage **graphique de la grille** et du trajet du robot (vue de dessus).
- Export des résultats (CSV) directement depuis l'interface.
- Affichage de la **tension batterie** (déjà mesurée côté STM32, à remonter).
- Indicateur d'état de connexion série (STM32 / Topcon) en temps réel.

Voir aussi la [passation](../08_Passation/README.md).
