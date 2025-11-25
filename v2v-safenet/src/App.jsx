import React, { useState, useEffect, useMemo } from 'react';
import VehicleMap from './components/VehicleMap';
import CollisionAlert from './components/CollisionAlert';
import VehicleInfoPanel from './components/VehicleInfoPanel';
import { useVehicleTracking, useVehiclesData } from './hooks/useVehicleTracking';
import { analyzeAllCollisionRisks, findNearestVehicle } from './utils/collisionDetection';
import './App.css';

function App() {
  // Generate or retrieve vehicle ID
  const [vehicleId] = useState(() => {
    let id = localStorage.getItem('vehicleId');
    if (!id) {
      id = `vehicle_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
      localStorage.setItem('vehicleId', id);
    }
    return id;
  });

  // Track current vehicle
  const { position, speed, heading, error, isTracking } = useVehicleTracking(vehicleId);

  // Get all vehicles data
  const { vehicles, loading } = useVehiclesData();

  // Filter out current vehicle from vehicles list
  const otherVehicles = useMemo(() => {
    const filtered = { ...vehicles };
    delete filtered[vehicleId];
    return filtered;
  }, [vehicles, vehicleId]);

  // Analyze collision risks
  const collisionRisks = useMemo(() => {
    if (!position) return [];

    const currentVehicle = {
      id: vehicleId,
      lat: position.lat,
      lng: position.lng,
      speed,
      heading
    };

    return analyzeAllCollisionRisks(currentVehicle, otherVehicles);
  }, [position, speed, heading, otherVehicles, vehicleId]);

  // Find nearest vehicle
  const nearestVehicle = useMemo(() => {
    if (!position) return null;

    const currentVehicle = {
      id: vehicleId,
      lat: position.lat,
      lng: position.lng,
      speed,
      heading
    };

    return findNearestVehicle(currentVehicle, otherVehicles);
  }, [position, speed, heading, otherVehicles, vehicleId]);

  // Determine overall risk level
  const overallRiskLevel = useMemo(() => {
    if (collisionRisks.length === 0) return 'LOW';
    return collisionRisks[0].riskLevel;
  }, [collisionRisks]);

  return (
    <div className="app">
      {/* Header */}
      <header className="app-header">
        <div className="header-content">
          <h1>🚗 V2V SafeNet</h1>
          <p className="subtitle">Real-Time Collision Warning System</p>
        </div>
        <div className="vehicle-id">
          <span className="id-label">Vehicle ID:</span>
          <span className="id-value">{vehicleId.substring(0, 12)}...</span>
        </div>
      </header>

      {/* Main Content */}
      <div className="app-content">
        {/* Left Panel - Info */}
        <aside className="info-sidebar">
          <VehicleInfoPanel
            currentSpeed={speed}
            nearestVehicle={nearestVehicle}
            riskLevel={overallRiskLevel}
            vehicleCount={Object.keys(otherVehicles).length}
            isTracking={isTracking}
          />

          {/* Instructions */}
          <div className="instructions-panel">
            <h3>📱 How to Test</h3>
            <ol>
              <li>Open this app in another browser tab or device</li>
              <li>Grant location permissions on both</li>
              <li>Move devices closer together</li>
              <li>Watch collision alerts activate!</li>
            </ol>
            <div className="legend">
              <h4>Map Legend</h4>
              <div className="legend-item">
                <span className="marker green"></span> You
              </div>
              <div className="legend-item">
                <span className="marker blue"></span> Normal
              </div>
              <div className="legend-item">
                <span className="marker orange"></span> Caution
              </div>
              <div className="legend-item">
                <span className="marker red"></span> Danger
              </div>
            </div>
          </div>
        </aside>

        {/* Map Container */}
        <main className="map-container">
          {error && (
            <div className="error-banner">
              <span className="error-icon">⚠️</span>
              {error}
            </div>
          )}

          {!isTracking && !error && (
            <div className="loading-banner">
              <div className="spinner"></div>
              <span>Waiting for GPS signal...</span>
            </div>
          )}

          <VehicleMap
            currentVehicle={{ position, speed, heading }}
            vehicles={otherVehicles}
            collisionRisks={collisionRisks}
          />

          {/* Collision Alerts Overlay */}
          <CollisionAlert risks={collisionRisks} />
        </main>
      </div>

      {/* Footer */}
      <footer className="app-footer">
        <div className="footer-stats">
          <span>🌐 Connected Vehicles: {Object.keys(vehicles).length}</span>
          <span>📡 Status: {isTracking ? 'Active' : 'Inactive'}</span>
          <span>⚡ Risk Level: <strong style={{ color: overallRiskLevel === 'HIGH' ? '#FF4444' : overallRiskLevel === 'MEDIUM' ? '#FFA500' : '#4CAF50' }}>{overallRiskLevel}</strong></span>
        </div>
      </footer>
    </div>
  );
}

export default App;
