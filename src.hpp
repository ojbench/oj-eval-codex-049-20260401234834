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

        int n = monitor->get_robot_number();

        // Desired velocity towards target with proportional slowdown near target.
        double desired_speed = std::min(v_max, dist / TIME_INTERVAL);
        desired_speed = std::max(0.0, desired_speed);
        Vec dir = to_tar.normalize();
        Vec v_des = dir * desired_speed;

        // Compute repulsion from nearby robots (potential field)
        Vec repel(0, 0);
        for (int j = 0; j < n; ++j) {
            if (j == id) continue;
            Vec pj = monitor->get_pos_cur(j);
            double rj = monitor->get_r(j);
            Vec dp = pos_cur - pj;
            double d = dp.norm();
            double safe = r + rj;
            double infl = safe * 3.0; // influence radius
            if (d < 1e-6) {
                // overlapping or nearly same pos: push strongly along perpendicular to break symmetry
                Vec p(dir.y, -dir.x);
                repel += p * (v_max * 0.8);
                continue;
            }
            if (d < infl) {
                double k = 0.6; // repulsion strength
                double strength;
                if (d <= safe) {
                    strength = v_max; // strong push when too close
                } else {
                    strength = k * (1.0 / (d - safe + 1e-6) - 1.0 / (infl - safe + 1e-6));
                    strength = std::max(0.0, std::min(strength, v_max));
                }
                repel += dp.normalize() * strength;
            }
        }

        Vec v_try = clamp_len(v_des + repel, v_max);

        auto safe_with = [&](const Vec &vtest) -> bool {
            for (int j = 0; j < n; ++j) {
                if (j == id) continue;
                if (will_collide(pos_cur, vtest, j)) return false;
            }
            return true;
        };

        // React to last-round warnings: if we were involved in collisions, yield or sidestep
        if (monitor->get_warning()) {
            auto collided = monitor->get_collision(id);
            if (!collided.empty() || monitor->get_speeding(id)) {
                // Try yielding: reduce speed significantly
                Vec v_yield = clamp_len(v_des, v_max * 0.2);
                if (safe_with(v_yield)) return v_yield;
                // Sidestep based on id parity to break symmetry
                Vec perp(dir.y, -dir.x);
                double side_speed = std::min(v_max * 0.3, desired_speed * 0.3);
                Vec options[4] = {perp * side_speed, perp * (-side_speed), (perp * 0.5) * side_speed, (perp * -0.5) * side_speed};
                for (auto &opt : options) {
                    if (safe_with(opt)) return opt;
                }
                return Vec(0, 0);
            }
        }

        if (safe_with(v_try)) return v_try;

        // Try reducing speed in factors
        const double factors[] = {0.8, 0.6, 0.5, 0.3, 0.2, 0.1};
        Vec base = clamp_len(v_des, v_max);
        for (double f : factors) {
            Vec cand = base * f;
            if (safe_with(cand)) return cand;
        }

        // Try small sidestep perpendicular to target direction
        Vec perp(dir.y, -dir.x);
        double side_speed = std::min(v_max * 0.3, desired_speed * 0.3);
        Vec sidesteps[4] = {perp * side_speed, perp * (-side_speed), (perp * 0.5) * side_speed, (perp * -0.5) * side_speed};
        for (auto &s : sidesteps) {
            if (safe_with(s)) return s;
        }

        // Fallback: stop
        return Vec(0, 0);
    }
};


#endif //PPCA_SRC_HPP
