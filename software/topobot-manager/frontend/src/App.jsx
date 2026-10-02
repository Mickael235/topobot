import React, { useState, useEffect, useRef, useCallback } from 'react';

function App() {
  const [currentView, setCurrentView] = useState('connection');
  const [isConnecting, setIsConnecting] = useState(false);

  const [gridX, setGridX] = useState(10);
  const [gridY, setGridY] = useState(10);
  const [step, setStep] = useState(1);
  const [isStarting, setIsStarting] = useState(false);

  const [robotPos, setRobotPos] = useState({ x: 0, y: 0 });
  const [progress, setProgress] = useState(0);
  const [logs, setLogs] = useState([]);
  const [measuredPoints, setMeasuredPoints] = useState([]);
  const [missionDone, setMissionDone] = useState(false);
  const [trackingActive, setTrackingActive] = useState(false);

  const logsEndRef = useRef(null);
  const wsRef = useRef(null);

  const handleConnect = () => {
    setIsConnecting(true);
    setTimeout(() => {
      setIsConnecting(false);
      setCurrentView('config');
    }, 1500);
  };

  const handleStartMission = async () => {
    setIsStarting(true);
    setLogs([]);
    setMeasuredPoints([]);
    setRobotPos({ x: 0, y: 0 });
    setProgress(0);
    setMissionDone(false);
    setTrackingActive(false);

    try {
      const response = await fetch('/start-mission', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ largeur_x: gridX, longueur_y: gridY, pas_mesure: step }),
      });
      const data = await response.json();
      if (data.status === 'success') {
        setCurrentView('mission');
      }
    } catch (error) {
      alert("Impossible de joindre le robot !");
    } finally {
      setIsStarting(false);
    }
  };

  const handleEmergencyStop = useCallback(async () => {
    // 1) Signal d'abort via WebSocket (best effort, ne doit jamais bloquer le POST)
    try {
      if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) {
        wsRef.current.send(JSON.stringify({ action: 'abort' }));
      }
    } catch (e) {
      console.error("Erreur abort WebSocket:", e);
    }
    // 2) Arret d'urgence cote backend (chemin fiable, independant du WebSocket)
    try {
      await fetch('/stop', { method: 'POST' });
    } catch (e) {
      console.error("Erreur arret urgence:", e);
    }
  }, []);

  const handleExportCSV = () => {
    // Pas de Z : seulement les donnees brutes de la station (Hz, V, distance).
    // Le Z (altitude) est calcule plus tard par formules trigonometriques.
    let csv = "data:text/csv;charset=utf-8,ID,HEURE,X,Y,HZ_GON,V_GON,DIST_M\n";
    (measuredPoints || []).forEach(pt => {
      csv += `${pt.id},${pt.time},${(Number(pt.x)||0).toFixed(3)},${(Number(pt.y)||0).toFixed(3)},${(Number(pt.hz)||0).toFixed(4)},${(Number(pt.v)||0).toFixed(4)},${(Number(pt.dist)||0).toFixed(3)}\n`;
    });
    const link = document.createElement("a");
    link.setAttribute("href", encodeURI(csv));
    link.setAttribute("download", "TopoBot_Points.csv");
    document.body.appendChild(link);
    link.click();
    document.body.removeChild(link);
  };

  useEffect(() => {
    if (currentView === 'mission') {
      const protocol = window.location.protocol === 'https:' ? 'wss://' : 'ws://';
      const ws = new WebSocket(`${protocol}${window.location.host}/ws`);
      wsRef.current = ws;

      ws.onmessage = (event) => {
        const data = JSON.parse(event.data);
        const time = new Date().toLocaleTimeString('fr-FR', { hour12: false });

        if (data.type === 'position') {
          setRobotPos({ x: data.x, y: data.y });
          setProgress(data.progress);
          setMeasuredPoints(prev => [...prev, {
            id: `PT_${prev.length}`, time: data.time || time,
            x: data.x, y: data.y,
            hz: data.hz || 0, v: data.v || 0, dist: data.dist || 0,
          }]);
        } else if (data.type === 'tracking') {
          setTrackingActive(data.active);
        } else if (data.type === 'log') {
          setLogs(prev => [...prev, `[${time}] ${data.message}`]);
        } else if (data.type === 'done') {
          setMissionDone(true);
          setLogs(prev => [...prev, `[${time}] Mission terminee`]);
        } else if (data.type === 'aborted') {
          setMissionDone(true);
          setLogs(prev => [...prev, `[${time}] ARRET D'URGENCE`]);
        } else if (data.type === 'error') {
          setMissionDone(true);
          setLogs(prev => [...prev, `[${time}] ERREUR : ${data.message}`]);
        }
      };

      ws.onclose = () => { wsRef.current = null; };
      return () => { ws.close(); wsRef.current = null; };
    }
  }, [currentView]);

  useEffect(() => {
    logsEndRef.current?.scrollIntoView({ behavior: "smooth" });
  }, [logs]);

  /* ================================================================== */

  const renderConnectionView = () => (
    <main className="flex flex-col items-center justify-center mt-24 px-4">
      <svg className="w-20 h-20 text-blue-500 mb-6" fill="currentColor" viewBox="0 0 24 24"><path d="M12 21c-1.1 0-2-.9-2-2s.9-2 2-2 2 .9 2 2-.9 2-2 2zm-4.6-5.6c2.5-2.5 6.6-2.5 9.2 0l1.4-1.4c-3.3-3.3-8.6-3.3-11.9 0l1.3 1.4zm-4.3-4.2c4.9-4.9 12.8-4.9 17.7 0l1.4-1.4c-5.7-5.7-14.9-5.7-20.5 0l1.4 1.4z"/></svg>
      <h2 className="text-3xl font-extrabold mb-4">Connexion au Robot</h2>
      <p className="text-center text-gray-700 mb-10 max-w-md">Verifiez que le robot est sous tension et la liaison serie connectee.</p>
      <button onClick={handleConnect} disabled={isConnecting} className={`text-white font-bold py-4 px-8 rounded-xl w-full max-w-md text-lg shadow-md ${isConnecting ? 'bg-blue-400 cursor-not-allowed' : 'bg-blue-600 hover:bg-blue-700'}`}>
        {isConnecting ? 'Connexion...' : 'Connexion'}
      </button>
    </main>
  );

  const renderConfigView = () => (
    <main className="max-w-3xl mx-auto mt-10 px-4">
      <h2 className="text-3xl font-bold mb-8">Configuration de la mission</h2>
      <div className="bg-gray-50 rounded-2xl p-8 mb-8">
        <div className="mb-6">
          <label className="block text-lg mb-2">Largeur X (m)</label>
          <input type="number" step="any" min="0.1" value={gridX} onChange={e => { const v = parseFloat(e.target.value); if (!isNaN(v)) setGridX(v); }} className="bg-white border w-full p-3 rounded-lg text-lg focus:ring-2 focus:ring-blue-500 focus:outline-none" />
        </div>
        <div className="mb-6">
          <label className="block text-lg mb-2">Longueur Y (m)</label>
          <input type="number" step="any" min="0.1" value={gridY} onChange={e => { const v = parseFloat(e.target.value); if (!isNaN(v)) setGridY(v); }} className="bg-white border w-full p-3 rounded-lg text-lg focus:ring-2 focus:ring-blue-500 focus:outline-none" />
        </div>
        <div className="mb-6">
          <label className="block text-lg mb-2">Pas de mesure (m)</label>
          <input type="number" step="any" min="0.01" value={step} onChange={e => { const v = parseFloat(e.target.value); if (!isNaN(v)) setStep(v); }} className="bg-white border w-full p-3 rounded-lg text-lg focus:ring-2 focus:ring-blue-500 focus:outline-none" />
        </div>

        <div className="bg-blue-100 rounded-xl p-6 text-blue-700 font-medium mb-6">
          <p className="mb-3 font-bold">Workflow mission :</p>
          <ol className="list-decimal list-inside space-y-1">
            <li>Initialisation station en mode tracking</li>
            <li>Deplacement vers le point (station suit le prisme)</li>
            <li>Arret tracking + abaissement prisme AX12</li>
            <li>Mesure precise (Hz, V, Distance)</li>
            <li>Remontee prisme + reprise tracking</li>
            <li>Sauvegarde en base de donnees</li>
            <li>Point suivant</li>
          </ol>
        </div>

        <div className="bg-gray-100 rounded-xl p-4 text-gray-600 text-sm mb-6">
          Grille : {gridX} x {gridY} m — Pas : {step} m — ~{Math.ceil((gridX/step+1) * (gridY/step+1))} points
        </div>

        <button onClick={handleStartMission} disabled={isStarting} className="bg-blue-500 hover:bg-blue-600 disabled:bg-blue-400 text-white font-bold py-4 rounded-xl w-full text-xl shadow-md">
          {isStarting ? 'Envoi...' : 'Lancer la mission'}
        </button>
      </div>
    </main>
  );

  const renderMissionView = () => (
    <main className="max-w-6xl mx-auto mt-6 px-4">
      {/* Barre du haut */}
      <div className="bg-gray-100 rounded-2xl py-4 px-8 flex justify-between items-center mb-8 shadow-sm">
        <div>
          <h2 className="text-xl font-bold">{missionDone ? 'Mission terminee' : 'Mission en cours...'}</h2>
          <p className="text-gray-500 text-sm">X: {robotPos.x} / Y: {robotPos.y}</p>
          <span className={`inline-block mt-1 text-xs font-bold px-2 py-0.5 rounded ${trackingActive ? 'bg-green-200 text-green-700' : 'bg-yellow-200 text-yellow-700'}`}>
            {trackingActive ? 'TRACKING ACTIF' : 'TRACKING INACTIF'}
          </span>
        </div>
        <div className="flex-1 mx-8">
          <div className="flex justify-between text-sm mb-1"><span>Progression</span><span className="font-bold">{progress}%</span></div>
          <div className="w-full bg-gray-300 rounded-full h-2.5"><div className="bg-blue-600 h-2.5 rounded-full transition-all" style={{ width: `${progress}%` }}></div></div>
        </div>
        <div className="flex space-x-3">
          <button onClick={handleEmergencyStop} className="bg-red-600 hover:bg-red-700 text-white font-bold py-3 px-6 rounded-xl shadow-lg">ARRET D'URGENCE</button>
          {missionDone && <button onClick={() => setCurrentView('config')} className="bg-white border text-gray-800 font-bold py-3 px-4 rounded-xl">Nouvelle mission</button>}
        </div>
      </div>

      <div className="grid grid-cols-1 lg:grid-cols-3 gap-8">
        {/* Carte */}
        <div className="lg:col-span-2 bg-slate-800 rounded-2xl min-h-[400px] p-6 relative overflow-hidden border-4 border-slate-700">
          <div className="absolute inset-0 opacity-20" style={{ backgroundImage: 'radial-gradient(circle, #ffffff 1px, transparent 1px)', backgroundSize: '20px 20px' }}></div>
          {measuredPoints.map((pt, i) => (
            <div key={i} className="absolute w-3 h-3 bg-green-500 rounded-full" style={{ left: `${(pt.x / gridX) * 80 + 10}%`, bottom: `${(pt.y / gridY) * 80 + 10}%`, transform: 'translate(-50%, 50%)' }}></div>
          ))}
          <div className="absolute text-white bg-orange-500 p-2 rounded-lg transition-all duration-1000 z-10 shadow-lg" style={{ left: `${(robotPos.x / gridX) * 80 + 10}%`, bottom: `${(robotPos.y / gridY) * 80 + 10}%`, transform: 'translate(-50%, 50%)' }}>
            <svg className="w-6 h-6" fill="none" stroke="currentColor" viewBox="0 0 24 24"><path strokeLinecap="round" strokeLinejoin="round" strokeWidth="2" d="M4 8V4m0 0h4M4 4l5 5m11-1V4m0 0h-4m4 0l-5 5M4 16v4m0 0h4m-4 0l5-5m11 5l-5-5m5 5v-4m0 4h-4"></path></svg>
          </div>
          <p className="absolute bottom-2 right-4 text-xs text-slate-400">{gridX}m (X)</p>
          <p className="absolute top-4 left-2 text-xs text-slate-400">{gridY}m (Y)</p>
          <p className="absolute bottom-2 left-2 text-xs text-slate-400">0m</p>
        </div>

        {/* Journal */}
        <div className="flex flex-col space-y-4">
          <div className="bg-slate-50 border rounded-2xl p-4 flex flex-col h-[400px]">
            <div className="flex justify-between items-center mb-3">
              <h3 className="font-bold text-gray-700">Journal</h3>
              <span className={`text-xs font-bold px-2 py-1 rounded ${missionDone ? 'bg-gray-200 text-gray-500' : 'bg-green-200 text-green-700'}`}>
                {missionDone ? 'FIN' : 'LIVE'}
              </span>
            </div>
            <div className="bg-white border rounded-xl flex-1 p-3 overflow-y-auto font-mono text-xs text-slate-600 shadow-inner">
              {logs.map((l, i) => <div key={i} className="mb-1">{l}</div>)}
              <div ref={logsEndRef} />
            </div>
          </div>

          <div className="grid grid-cols-5 gap-2">
            <div className="bg-indigo-500 text-white rounded-xl p-3 text-center"><p className="text-xs opacity-80">X</p><p className="text-lg font-bold">{robotPos.x}</p></div>
            <div className="bg-indigo-500 text-white rounded-xl p-3 text-center"><p className="text-xs opacity-80">Y</p><p className="text-lg font-bold">{robotPos.y}</p></div>
            <div className="bg-emerald-600 text-white rounded-xl p-3 text-center"><p className="text-xs opacity-80">Dist</p><p className="text-lg font-bold">{measuredPoints.length > 0 ? measuredPoints[measuredPoints.length - 1].dist.toFixed(3) : '-'}</p></div>
            <div className="bg-amber-600 text-white rounded-xl p-3 text-center"><p className="text-xs opacity-80">Hz</p><p className="text-lg font-bold">{measuredPoints.length > 0 ? measuredPoints[measuredPoints.length - 1].hz.toFixed(2) : '-'}</p></div>
            <div className="bg-amber-600 text-white rounded-xl p-3 text-center"><p className="text-xs opacity-80">V</p><p className="text-lg font-bold">{measuredPoints.length > 0 ? measuredPoints[measuredPoints.length - 1].v.toFixed(2) : '-'}</p></div>
          </div>

          {missionDone && measuredPoints.length > 0 && (
            <button onClick={handleExportCSV} className="bg-emerald-600 hover:bg-emerald-700 text-white font-bold py-3 rounded-xl w-full shadow-md">
              Exporter CSV ({measuredPoints.length} points)
            </button>
          )}
        </div>
      </div>
    </main>
  );

  /* ================================================================== */

  return (
    <div className="min-h-screen bg-white font-sans text-gray-800">
      <header className="bg-slate-900 text-white p-4 flex items-center justify-between">
        <div className="flex items-center">
          <div className="bg-orange-500 p-2 rounded-lg mr-3">
            <svg className="w-6 h-6 text-white" fill="none" stroke="currentColor" viewBox="0 0 24 24"><path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M4 8V4m0 0h4M4 4l5 5m11-1V4m0 0h-4m4 0l-5 5M4 16v4m0 0h4m-4 0l5-5m11 5l-5-5m5 5v-4m0 4h-4" /></svg>
          </div>
          <div>
            <h1 className="text-xl font-bold">TopoBot <span className="text-orange-500">Manager</span></h1>
            <p className="text-xs text-gray-400">ENSIM x ESGT</p>
          </div>
        </div>
        {currentView !== 'connection' && (
          <div className="flex items-center space-x-3">
            <button onClick={handleEmergencyStop} className="bg-red-600 hover:bg-red-700 text-white font-bold py-2 px-4 rounded-lg text-sm">STOP</button>
            <button onClick={() => setCurrentView('connection')} className="text-gray-400 hover:text-white px-3 py-1 bg-slate-800 rounded text-sm">Deconnexion</button>
          </div>
        )}
      </header>

      {currentView === 'connection' && renderConnectionView()}
      {currentView === 'config' && renderConfigView()}
      {currentView === 'mission' && renderMissionView()}
    </div>
  );
}

export default App;
