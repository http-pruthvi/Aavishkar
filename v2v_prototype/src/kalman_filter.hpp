#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

#include <vector>
#include <cmath>

class KalmanFilter {
public:
    // State: [x, y, vx, vy]
    std::vector<double> state;
    
    KalmanFilter() {
        state = {0.0, 0.0, 0.0, 0.0};
    }

    void predict(double dt) {
        // x = x + vx * dt
        state[0] += state[2] * dt;
        // y = y + vy * dt
        state[1] += state[3] * dt;
        // vx, vy constant velocity model
    }

    void update(double meas_x, double meas_y, double meas_vx, double meas_vy) {
        // Simple filter gain
        double alpha = 0.6; // Position weight
        double beta = 0.4;  // Velocity weight

        state[0] = state[0] * (1 - alpha) + meas_x * alpha;
        state[1] = state[1] * (1 - alpha) + meas_y * alpha;
        state[2] = state[2] * (1 - beta) + meas_vx * beta;
        state[3] = state[3] * (1 - beta) + meas_vy * beta;
    }
};

#endif
