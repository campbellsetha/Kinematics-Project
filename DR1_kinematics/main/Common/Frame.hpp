// Author: Seth Campbell
// Frame.hpp
// Defines the Joint and Frame types that represent the robot arm's kinematic chain.
// Joints are linked in order from base to tip; Frame walks the chain to update angles.

#pragma once
#include <cstdint>
#include <cmath>

struct Joint {
    double  proj[4][4]; // Fixed transform encoding this joint's position and orientation in the arm
    double  c;          // cos(angle) — refreshed each cycle from live servo ticks
    double  s;          // sin(angle) — refreshed each cycle from live servo ticks
    uint8_t id;         // Which servo drives this joint (index into ticks/offsets arrays)
    bool    is_fixed;   // True if this joint has no servo (e.g. the passive jaw)
    char    axis;       // Rotation axis — always 'z' for every joint on this robot
    Joint*  next;       // Next joint toward the tip of the arm (nullptr at end of chain)

    // Converts raw servo ticks to an angle in radians and stores cos/sin for FK use.
    void update(uint16_t ticks, double calibration_offset) {
        if (is_fixed) return;
        double theta = ((ticks - 2048) / 2048.0) * M_PI + calibration_offset;
        c = std::cos(theta);
        s = std::sin(theta);
    }
};

class Frame {
public:
    Joint* head; // First joint in the chain (shoulder pan)

    // Walks the entire joint chain and updates every servo's angle from live tick readings.
    void update(uint16_t* ticks, const double* offsets) {
        Joint* cur = head;
        while (cur != nullptr) {
            if (!cur->is_fixed)
                cur->update(ticks[cur->id], offsets[cur->id]);
            cur = cur->next;
        }
    }

    // Standard 4x4 matrix multiply — result can safely overlap with A or B.
    static void multiply(
        const double A[4][4],
        const double B[4][4],
        double result[4][4])
    {
        double tmp[4][4] = {};
        for (int i = 0; i < 4; i++)
            for (int k = 0; k < 4; k++)
                for (int j = 0; j < 4; j++)
                    tmp[i][k] += A[i][j] * B[j][k];
        for (int i = 0; i < 4; i++)
            for (int k = 0; k < 4; k++)
                result[i][k] = tmp[i][k];
    }

    // Builds a rotation matrix for spinning around the Z axis using pre-computed cos/sin.
    static void build_rot_z(double c, double s, double out[4][4]) {
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                out[i][j] = 0.0;
        out[0][0] =  c;  out[0][1] = -s;
        out[1][0] =  s;  out[1][1] =  c;
        out[2][2] =  1.0;
        out[3][3] =  1.0;
    }
};

// Links the static joint chain and sets frame.head — call once at startup.
void init_chain(Frame& frame);