import { calculateDistance } from './geoUtils';

/**
 * Calculate Time-to-Collision (TTC) between two vehicles
 * @param {Object} vehicle1 - First vehicle data
 * @param {Object} vehicle2 - Second vehicle data
 * @returns {Object} Collision analysis result
 */
export function analyzeCollisionRisk(vehicle1, vehicle2) {
    // Calculate distance between vehicles
    const distance = calculateDistance(
        vehicle1.lat,
        vehicle1.lng,
        vehicle2.lat,
        vehicle2.lng
    );

    // Calculate relative velocity (simplified approach)
    // In a real system, you'd use velocity vectors
    const relativeSpeed = Math.abs(vehicle1.speed - vehicle2.speed);

    // Calculate Time-to-Collision (TTC)
    let ttc = Infinity;
    if (relativeSpeed > 0) {
        ttc = distance / ((relativeSpeed * 1000) / 3600); // Convert km/h to m/s
    }

    // Determine risk level
    let riskLevel = 'LOW';
    if (ttc < 3) {
        riskLevel = 'HIGH';
    } else if (ttc < 5) {
        riskLevel = 'MEDIUM';
    }

    // Check if vehicles are on collision course
    // Simplified: if they're close and moving towards each other
    const isCollisionCourse = distance < 100 && relativeSpeed > 5;

    return {
        distance: Math.round(distance),
        ttc: ttc === Infinity ? null : ttc.toFixed(1),
        riskLevel,
        isCollisionCourse,
        relativeSpeed: relativeSpeed.toFixed(1)
    };
}

/**
 * Detect sudden braking
 * @param {number} currentSpeed - Current speed in km/h
 * @param {number} previousSpeed - Previous speed in km/h
 * @param {number} timeDiff - Time difference in milliseconds
 * @returns {boolean} True if sudden braking detected
 */
export function detectSuddenBraking(currentSpeed, previousSpeed, timeDiff) {
    const speedDrop = previousSpeed - currentSpeed;
    const timeInSeconds = timeDiff / 1000;

    // Detect if speed dropped more than 20 km/h in 1 second
    if (timeInSeconds > 0) {
        const deceleration = speedDrop / timeInSeconds;
        return deceleration > 20;
    }

    return false;
}

/**
 * Find nearest vehicle from a list
 * @param {Object} currentVehicle - Current vehicle data
 * @param {Object} vehicles - Object containing all vehicles
 * @returns {Object|null} Nearest vehicle with distance
 */
export function findNearestVehicle(currentVehicle, vehicles) {
    let nearest = null;
    let minDistance = Infinity;

    Object.entries(vehicles).forEach(([id, vehicle]) => {
        if (id === currentVehicle.id) return; // Skip self

        const distance = calculateDistance(
            currentVehicle.lat,
            currentVehicle.lng,
            vehicle.lat,
            vehicle.lng
        );

        if (distance < minDistance) {
            minDistance = distance;
            nearest = { ...vehicle, id, distance };
        }
    });

    return nearest;
}

/**
 * Analyze all nearby vehicles for collision risks
 * @param {Object} currentVehicle - Current vehicle data
 * @param {Object} vehicles - Object containing all vehicles
 * @returns {Array} Array of collision risks sorted by severity
 */
export function analyzeAllCollisionRisks(currentVehicle, vehicles) {
    const risks = [];

    Object.entries(vehicles).forEach(([id, vehicle]) => {
        if (id === currentVehicle.id) return; // Skip self

        const analysis = analyzeCollisionRisk(currentVehicle, vehicle);

        // Only include vehicles within 500m
        if (analysis.distance < 500) {
            risks.push({
                vehicleId: id,
                vehicle,
                ...analysis
            });
        }
    });

    // Sort by risk level (HIGH > MEDIUM > LOW) and then by distance
    risks.sort((a, b) => {
        const riskOrder = { HIGH: 3, MEDIUM: 2, LOW: 1 };
        if (riskOrder[a.riskLevel] !== riskOrder[b.riskLevel]) {
            return riskOrder[b.riskLevel] - riskOrder[a.riskLevel];
        }
        return a.distance - b.distance;
    });

    return risks;
}
