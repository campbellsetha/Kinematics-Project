#pragma once
#include <cstdint>
#include <cmath>
#include <array>
 
// Joint node
struct Joint {
 
    double proj[4][4];
 
    double c;
    double s;
 
    uint8_t id;
 
    bool is_fixed;
 
    Joint* next;
 
    void update(uint16_t ticks, double calibration_offset) {
        if (is_fixed) return;
        double theta = ((ticks / 4096.0) * 2.0 * M_PI) + calibration_offset;
        c = std::cos(theta);
        s = std::sin(theta);
    }
};

class Frame {
    public:
    // Head of the kinematic chain
    Joint* head;
 
    
    void update(uint16_t* ticks, double* offsets) {
        Joint* current = head;
        while (current != nullptr) {
            if (!current->is_fixed)
                current->update(ticks[current->id], offsets[current->id]);
            current = current->next;
        }
    }


    static void multiply(
        const double A[4][4],
        const double B[4][4],
        double result[4][4])
    {
        double temp[4][4] = {};
        for (int i = 0; i < 4; i++)
            for (int k = 0; k < 4; k++)
                for (int j = 0; j < 4; j++)
                    temp[i][k] += A[i][j] * B[j][k];
 
        for (int i = 0; i < 4; i++)
            for (int k = 0; k < 4; k++)
                result[i][k] = temp[i][k];
    }

    // Build a ROT_Z from cached cos/sin values
    static void buildRotZ(double c, double s, double out[4][4]) {
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                out[i][j] = 0.0;
 
        out[0][0] =  c;
        out[0][1] = -s;
        out[1][0] =  s;
        out[1][1] =  c;
        out[2][2] =  1.0;
        out[3][3] =  1.0;
    }

// ------------------------------------------------------------
// initChain — declared here, defined in Frame.cpp
// Links static joint nodes and sets frame.head
// ------------------------------------------------------------
void initChain(Frame& frame);
