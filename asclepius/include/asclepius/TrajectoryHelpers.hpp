#ifndef TRAJECTORY_HELPERS_HPP
#define TRAJECTORY_HELPERS_HPP

#include <iostream>
#include <cmath>
#include <utility>  

//
// Constant Helpers
//
// Compute constant position and zero velocity.
//
std::pair<double, double> hold(double p0) {
    // Position is constant, velocity is zero.
    double p = p0;
    double v = 0.0;
    return std::make_pair(p, v);
}

//
// Linear Helpers
//
// Linearly interpolate between an initial and final position over time T.
//
std::pair<double, double> interpolate(double t, double T, double p0, double pf) {
    // Compute the current (p, v).
    double p = p0 + (pf - p0) / T * t;
    double v = (pf - p0) / T;
    return std::make_pair(p, v);
}

//
// Cubic Spline Helpers
//
// Compute a cubic spline that moves from (p0, v0) to (pf, vf) over time T.
//
std::pair<double, double> goto_cubic(double t, double T, double p0, double pf) {
    // Compute the current (p, v) using cubic spline formula.
    double p = p0 + (pf - p0) * (3 * std::pow(t / T, 2) - 2 * std::pow(t / T, 3));
    double v = (pf - p0) / T * (6 * (t / T) - 6 * std::pow(t / T, 2));
    return std::make_pair(p, v);
}

std::pair<double, double> spline(double t, double T, double p0, double pf, double v0, double vf) {
    // Compute the parameters for the cubic spline.
    double a = p0;
    double b = v0;
    double c = 3 * (pf - p0) / std::pow(T, 2) - vf / T - 2 * v0 / T;
    double d = -2 * (pf - p0) / std::pow(T, 3) + vf / std::pow(T, 2) + v0 / std::pow(T, 2);

    // Compute the current (p, v).
    double p = a + b * t + c * std::pow(t, 2) + d * std::pow(t, 3);
    double v = b + 2 * c * t + 3 * d * std::pow(t, 2);
    return std::make_pair(p, v);
}

//
// Quintic Spline Helpers
//
// Compute a quintic spline that moves from (p0, v0, a0) to (pf, vf, af) over time T.
//
std::pair<double, double> goto_quintic(double t, double T, double p0, double pf) {
    // Compute the current (p, v) using quintic spline formula.
    double p = p0 + (pf - p0) * (10 * std::pow(t / T, 3) - 15 * std::pow(t / T, 4) + 6 * std::pow(t / T, 5));
    double v = (pf - p0) / T * (30 * std::pow(t / T, 2) - 60 * std::pow(t / T, 3) + 30 * std::pow(t / T, 4));
    return std::make_pair(p, v);
}

std::pair<double, double> spline5(double t, double T, double p0, double pf, double v0, double vf, double a0, double af) {
    // Compute the parameters for the quintic spline.
    double a = p0;
    double b = v0;
    double c = a0;
    double d = 10 * (pf - p0) / std::pow(T, 3) - 6 * v0 / std::pow(T, 2) - 3 * a0 / T - 4 * vf / std::pow(T, 2) + 0.5 * af / T;
    double e = -15 * (pf - p0) / std::pow(T, 4) + 8 * v0 / std::pow(T, 3) + 3 * a0 / std::pow(T, 2) + 7 * vf / std::pow(T, 3) - af / std::pow(T, 2);
    double f = 6 * (pf - p0) / std::pow(T, 5) - 3 * v0 / std::pow(T, 4) - a0 / std::pow(T, 3) - 3 * vf / std::pow(T, 4) + 0.5 * af / std::pow(T, 3);

    // Compute the current (p, v).
    double p = a + b * t + c * std::pow(t, 2) + d * std::pow(t, 3) + e * std::pow(t, 4) + f * std::pow(t, 5);
    double v = b + 2 * c * t + 3 * d * std::pow(t, 2) + 4 * e * std::pow(t, 3) + 5 * f * std::pow(t, 4);
    return std::make_pair(p, v);
}

// Test the functions
int main() {
    // Test cubic interpolation
    double t = 1.0;
    double T = 2.0;
    double p0 = 0.0;
    double pf = 10.0;

    std::pair<double, double> result = goto_cubic(t, T, p0, pf);
    std::cout << "Cubic Position: " << result.first << ", Velocity: " << result.second << std::endl;

    // Test quintic interpolation
    result = goto_quintic(t, T, p0, pf);
    std::cout << "Quintic Position: " << result.first << ", Velocity: " << result.second << std::endl;

    return 0;
}
#endif // TRAJECTORY_HELPERS_HPP