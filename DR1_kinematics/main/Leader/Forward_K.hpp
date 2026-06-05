// Author: Seth Campbell
// Forward_K.hpp
// Computes the end-effector position (XYZ in metres) from raw servo tick readings
// by walking the joint chain and multiplying each joint's transform in sequence.

#pragma once
#include "Frame.hpp"
#include <array>

// Per-joint calibration offsets in radians — converts raw ticks to true angles.
// TODO: Replace with values from servo_scanner.py once calibrated.
static constexpr std::array<double, 6> JOINT_OFFSETS = {
    2.7857,  // J1 — shoulder pan    (servo id 0)
   -1.8898,  // J2 — shoulder lift   (servo id 1)
   -2.4344,  // J3 — elbow flex      (servo id 2)
   -2.1384,  // J4 — wrist flex      (servo id 3)
    2.7029,  // J5 — wrist roll      (servo id 4)
    2.6722   // J6 — jaw (fixed, not used)
};

class ForwardK {
public:
    Frame&        frame;
    const double* offsets;
    double        t_result[4][4]; // Accumulated transform after the last compute() call

    ForwardK(Frame& f, const double* cal_offsets)
        : frame(f), offsets(cal_offsets) {}

    // Full FK cycle: update joint angles from ticks, walk the chain, return XYZ in metres.
    std::array<double, 3> compute(uint16_t* ticks) {
        frame.update(ticks, offsets);
        compute_transform();
        return get_position();
    }

    // Returns the end-effector XYZ from the most recent compute() call.
    std::array<double, 3> get_position() const {
        return { t_result[0][3], t_result[1][3], t_result[2][3] };
    }

private:
    // Resets to identity and walks the joint chain, accumulating transforms into t_result.
    void compute_transform() {
        double T[4][4] = {};
        T[0][0] = T[1][1] = T[2][2] = T[3][3] = 1.0;
        traverse(frame.head, T);
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                t_result[i][j] = T[i][j];
    }

    // Recursively applies each joint's transform (fixed offset * live rotation) onto T.
    void traverse(Joint* node, double T[4][4]) {
        if (node == nullptr) return;

        double result[4][4];

        if (node->is_fixed) {
            Frame::multiply(T, node->proj, result);
        } else {
            double rot[4][4];
            Frame::build_rot_z(node->c, node->s, rot);

            double joint_transform[4][4];
            Frame::multiply(node->proj, rot, joint_transform);
            Frame::multiply(T, joint_transform, result);
        }

        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                T[i][j] = result[i][j];

        traverse(node->next, T);
    }
};