#ifndef PPCA_SRC_HPP
#define PPCA_SRC_HPP

#include <cmath>
#include <vector>
#include "math.h"
#include "monitor.h"

class Controller {

public:
    Controller(const Vec &_pos_tar, double _v_max, double _r, int _id, Monitor *_monitor) {
        pos_tar = _pos_tar;
        v_max = _v_max;
        r = _r;
        id = _id;
        monitor = _monitor;
    }

    void set_pos_cur(const Vec &_pos_cur) {
        pos_cur = _pos_cur;
    }

    void set_v_cur(const Vec &_v_cur) {
        v_cur = _v_cur;
    }

private:
    int id;
    Vec pos_tar;
    Vec pos_cur;
    Vec v_cur;
    double v_max, r;
    Monitor *monitor;

    // Helper: clamp vector length to max_len
    Vec clamp_len(const Vec &v, double max_len) const {
        double n = std::sqrt(v.x * v.x + v.y * v.y);
        if (n <= 1e-12) return Vec(0, 0);
        if (n <= max_len) return v;
        double s = max_len / n;
        return v * s;
    }

    bool will_collide(const Vec &pos_i, const Vec &v_i, int j) const {
        Vec pos_j = monitor->get_pos_cur(j);
        Vec v_j = monitor->get_v_cur(j);
        double r_i = r;
        double r_j = monitor->get_r(j);
        Vec dp = pos_i - pos_j;
        Vec dv = v_i - v_j;
        double proj = dp.dot(dv);
        if (proj >= 0) return false;
        double dvn = std::sqrt(dv.x * dv.x + dv.y * dv.y);
        if (dvn <= 1e-12) return false;
        proj /= -dvn; // distance along dv direction to closest approach
        double min_dis_sqr;
        double delta_r = r_i + r_j;
        if (proj < dvn * TIME_INTERVAL) {
            // closest approach within interval
            min_dis_sqr = dp.norm_sqr() - proj * proj;
        } else {
            // check end of interval
            Vec endp = dp + dv * TIME_INTERVAL;
            min_dis_sqr = endp.norm_sqr();
        }
        return min_dis_sqr <= delta_r * delta_r - EPSILON;
    }

public:

    Vec get_v_next() {
        // If already very close to target, stop.
        Vec to_tar = pos_tar - pos_cur;
        double dist = to_tar.norm();
        if (dist <= EPSILON) {
            return Vec(0, 0);
        }

        // Desired velocity towards target with proportional slowdown near target.
        double desired_speed = v_max;
        // Smooth approach: limit speed so we don't overshoot in one interval by too much
        double max_step_speed = dist / TIME_INTERVAL; // to arrive in one step
        desired_speed = std::min(desired_speed, max_step_speed);
        // Also keep some margin
        desired_speed = std::max(0.0, desired_speed);
        Vec v_des = to_tar.normalize() * desired_speed;

        // Basic right-of-way: lower ID robots have priority. If our proposed velocity would
        // collide with any lower-ID robot, try to adjust.
        int n = monitor->get_robot_number();

        // Try several fallback strategies: slow, stop, sidestep.
        auto safe_with = [&](const Vec &v_try) -> bool {
            for (int j = 0; j < n; ++j) {
                if (j == id) continue;
                // Give way to lower IDs, be conservative with higher IDs too
                if (will_collide(pos_cur, v_try, j)) return false;
            }
            return true;
        };

        // First try desired velocity clamped to v_max
        Vec v_try = clamp_len(v_des, v_max);
        if (safe_with(v_try)) return v_try;

        // Try reducing speed in factors
        const double factors[] = {0.7, 0.5, 0.3, 0.2, 0.1};
        for (double f : factors) {
            v_try = v_try * f;
            if (safe_with(v_try)) return v_try;
        }

        // Try a small sidestep perpendicular to target direction
        Vec dir = to_tar.normalize();
        Vec perp(dir.y, -dir.x);
        double side_speed = std::min(v_max * 0.3, desired_speed * 0.3);
        Vec sidesteps[4] = {perp * side_speed, perp * (-side_speed), (perp * 0.5) * side_speed, (perp * -0.5) * side_speed};
        for (auto &s : sidesteps) {
            v_try = s;
            if (safe_with(v_try)) return v_try;
        }

        // Fallback: full stop
        return Vec(0, 0);
    }
};


#endif //PPCA_SRC_HPP

