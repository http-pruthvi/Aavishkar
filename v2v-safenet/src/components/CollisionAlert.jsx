import React, { useEffect, useState } from 'react';
import './CollisionAlert.css';

/**
 * CollisionAlert component - displays collision warnings
 */
export default function CollisionAlert({ risks }) {
    const [showAlert, setShowAlert] = useState(false);
    const [alertSound] = useState(new Audio('data:audio/wav;base64,UklGRnoGAABXQVZFZm10IBAAAAABAAEAQB8AAEAfAAABAAgAZGF0YQoGAACBhYqFbF1fdJivrJBhNjVgodDbq2EcBj+a2/LDciUFLIHO8tiJNwgZaLvt559NEAxQp+PwtmMcBjiR1/LMeSwFJHfH8N2QQAoUXrTp66hVFApGn+DyvmwhBTGH0fPTgjMGHm7A7+OZURE='));

    const highestRisk = risks.find(r => r.riskLevel === 'HIGH');
    const mediumRisk = risks.find(r => r.riskLevel === 'MEDIUM');
    const brakeAlert = risks.find(r => r.vehicle?.braking);

    useEffect(() => {
        if (highestRisk || brakeAlert) {
            setShowAlert(true);

            // Play alert sound (muted by default, user can enable)
            // alertSound.play().catch(() => {});
        } else {
            setShowAlert(false);
        }
    }, [highestRisk, brakeAlert]);

    if (!showAlert && !mediumRisk) return null;

    return (
        <div className={`collision-alert ${highestRisk ? 'high-risk' : mediumRisk ? 'medium-risk' : ''}`}>
            {/* Brake Alert Banner */}
            {brakeAlert && (
                <div className="brake-alert-banner">
                    <span className="alert-icon">⚠️</span>
                    <span className="alert-text">VEHICLE AHEAD BRAKING!</span>
                    <span className="alert-distance">{brakeAlert.distance}m</span>
                </div>
            )}

            {/* Collision Warning */}
            {highestRisk && (
                <div className="collision-warning">
                    <div className="warning-header">
                        <span className="warning-icon">🚨</span>
                        <h3>COLLISION WARNING</h3>
                    </div>
                    <div className="warning-details">
                        <div className="detail-item">
                            <span className="label">Distance:</span>
                            <span className="value danger">{highestRisk.distance}m</span>
                        </div>
                        <div className="detail-item">
                            <span className="label">Time to Collision:</span>
                            <span className="value danger">{highestRisk.ttc}s</span>
                        </div>
                        <div className="detail-item">
                            <span className="label">Relative Speed:</span>
                            <span className="value">{highestRisk.relativeSpeed} km/h</span>
                        </div>
                    </div>
                    <div className="warning-action">
                        <strong>SLOW DOWN IMMEDIATELY</strong>
                    </div>
                </div>
            )}

            {/* Medium Risk Warning */}
            {mediumRisk && !highestRisk && (
                <div className="caution-warning">
                    <span className="caution-icon">⚡</span>
                    <span className="caution-text">
                        Vehicle ahead - {mediumRisk.distance}m
                        {mediumRisk.ttc && ` - ${mediumRisk.ttc}s`}
                    </span>
                </div>
            )}
        </div>
    );
}
