#ifndef TTC_HPP
#define TTC_HPP

#include <cmath>
#include <limits>

struct State {
    double x, y, vx, vy;
};

inline double calculate_ttc(const State& ego, const State& other) {
    double rx = other.x - ego.x;
    double ry = other.y - ego.y;
    double rvx = other.vx - ego.vx;
    double rvy = other.vy - ego.vy;

    double dot_r_v = rx * rvx + ry * rvy;
    double v_sq = rvx * rvx + rvy * rvy;
    
    if (v_sq < 1e-6) return std::numeric_limits<double>::infinity();
    
    // Time to closest approach
    double t = -dot_r_v / v_sq;
    
    if (t < 0) return std::numeric_limits<double>::infinity(); // Moving away or already passed
    
    // Distance at closest approach
    double dist_sq = (rx + rvx*t)*(rx + rvx*t) + (ry + rvy*t)*(ry + rvy*t);
    
    // Threshold for collision (e.g., 2 meters radius)
    if (dist_sq < 4.0) {
        return t;
    }
    
    return std::numeric_limits<double>::infinity();
}

#endif
