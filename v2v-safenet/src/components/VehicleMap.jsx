import React, { useMemo } from 'react';
import { MapContainer, TileLayer, Marker, Popup, useMap } from 'react-leaflet';
import L from 'leaflet';
import 'leaflet/dist/leaflet.css';

// Fix for default marker icons in React-Leaflet
delete L.Icon.Default.prototype._getIconUrl;
L.Icon.Default.mergeOptions({
    iconRetinaUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.7.1/images/marker-icon-2x.png',
    iconUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.7.1/images/marker-icon.png',
    shadowUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.7.1/images/marker-shadow.png',
});

// Custom vehicle marker icons
const createVehicleIcon = (color, rotation = 0) => {
    return L.divIcon({
        className: 'custom-vehicle-marker',
        html: `
      <div style="transform: rotate(${rotation}deg); width: 30px; height: 30px; display: flex; align-items: center; justify-content: center;">
        <svg width="30" height="30" viewBox="0 0 24 24" fill="${color}" xmlns="http://www.w3.org/2000/svg">
          <path d="M12 2L4 8V14L12 20L20 14V8L12 2Z" stroke="white" stroke-width="2"/>
          <circle cx="12" cy="12" r="4" fill="white"/>
        </svg>
      </div>
    `,
        iconSize: [30, 30],
        iconAnchor: [15, 15],
    });
};

// Component to recenter map on user's position
function RecenterMap({ position }) {
    const map = useMap();

    React.useEffect(() => {
        if (position) {
            map.setView([position.lat, position.lng], map.getZoom());
        }
    }, [position, map]);

    return null;
}

/**
 * VehicleMap component - displays all vehicles on a map
 */
export default function VehicleMap({ currentVehicle, vehicles, collisionRisks }) {
    const defaultCenter = [12.9716, 77.5946]; // Bangalore coordinates as default
    const mapCenter = currentVehicle?.position
        ? [currentVehicle.position.lat, currentVehicle.position.lng]
        : defaultCenter;

    // Get color based on risk level
    const getVehicleColor = (vehicleId) => {
        const risk = collisionRisks.find(r => r.vehicleId === vehicleId);
        if (!risk) return '#4A90E2'; // Blue - normal

        switch (risk.riskLevel) {
            case 'HIGH':
                return '#FF4444'; // Red - danger
            case 'MEDIUM':
                return '#FFA500'; // Orange - caution
            default:
                return '#4A90E2'; // Blue - normal
        }
    };

    return (
        <div style={{ height: '100%', width: '100%', borderRadius: '12px', overflow: 'hidden' }}>
            <MapContainer
                center={mapCenter}
                zoom={15}
                style={{ height: '100%', width: '100%' }}
                zoomControl={true}
            >
                <TileLayer
                    attribution='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
                    url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
                />

                {currentVehicle?.position && (
                    <RecenterMap position={currentVehicle.position} />
                )}

                {/* Current vehicle marker (green) */}
                {currentVehicle?.position && (
                    <Marker
                        position={[currentVehicle.position.lat, currentVehicle.position.lng]}
                        icon={createVehicleIcon('#00FF00', currentVehicle.heading)}
                    >
                        <Popup>
                            <div style={{ fontFamily: 'Inter, sans-serif' }}>
                                <strong>You</strong><br />
                                Speed: {currentVehicle.speed.toFixed(1)} km/h<br />
                                Heading: {currentVehicle.heading.toFixed(0)}°
                            </div>
                        </Popup>
                    </Marker>
                )}

                {/* Other vehicles */}
                {Object.entries(vehicles).map(([id, vehicle]) => {
                    if (!vehicle.lat || !vehicle.lng) return null;

                    const color = getVehicleColor(id);
                    const risk = collisionRisks.find(r => r.vehicleId === id);

                    return (
                        <Marker
                            key={id}
                            position={[vehicle.lat, vehicle.lng]}
                            icon={createVehicleIcon(color, vehicle.heading || 0)}
                        >
                            <Popup>
                                <div style={{ fontFamily: 'Inter, sans-serif' }}>
                                    <strong>Vehicle {id.substring(0, 6)}</strong><br />
                                    Speed: {vehicle.speed?.toFixed(1) || 0} km/h<br />
                                    {risk && (
                                        <>
                                            Distance: {risk.distance}m<br />
                                            Risk: <span style={{ color }}>{risk.riskLevel}</span><br />
                                            {risk.ttc && `TTC: ${risk.ttc}s`}
                                        </>
                                    )}
                                    {vehicle.braking && (
                                        <div style={{ color: '#FF4444', fontWeight: 'bold' }}>
                                            ⚠️ BRAKING
                                        </div>
                                    )}
                                </div>
                            </Popup>
                        </Marker>
                    );
                })}
            </MapContainer>
        </div>
    );
}
