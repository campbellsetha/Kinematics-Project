#pragma once
#include "Frame.hpp"
#include <array>
 
// ------------------------------------------------------------
// ForwardK — owns the full FK pipeline
// Traverses the Frame chain and computes end effector position
// ------------------------------------------------------------
class ForwardK {
public:
 
    Frame& frame;
    double* offsets;
 
    // Last computed full transform — ForwardK owns this result
    double T_result[4][4];
 
    ForwardK(Frame& f, double* cal_offsets)
        : frame(f), offsets(cal_offsets) {}
 
    // ----------------------------------------------------------
    // compute — full FK pipeline for one control cycle
    // 1. Push new tick values into joint model via Frame::update
    // 2. Run the recursive chain traversal
    // 3. Return x, y, z in metres
    // ----------------------------------------------------------
    std::array<double, 3> compute(uint16_t* ticks) {
        frame.update(ticks, offsets);
        computeTransform();
        return getPosition();
    }
 
    // ----------------------------------------------------------
    // getPosition — extract x, y, z from last column of T_result
    // ----------------------------------------------------------
    std::array<double, 3> getPosition() const {
        return {
            T_result[0][3],
            T_result[1][3],
            T_result[2][3]
        };
    }
 
private:
 
    // ----------------------------------------------------------
    // computeTransform — seeds identity accumulator and kicks
    // off recursive traversal, stores result in T_result
    // ----------------------------------------------------------
    void computeTransform() {
        double T[4][4] = {};
        T[0][0] = T[1][1] = T[2][2] = T[3][3] = 1.0;
 
        traverse(frame.head, T);
 
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                T_result[i][j] = T[i][j];
    }
 
    // ----------------------------------------------------------
    // traverse — recursive chain walk
    // Base case: node == nullptr
    // ----------------------------------------------------------
    void traverse(Joint* node, double T[4][4]) {
        if (node == nullptr) return;
 
        double joint_transform[4][4];
        double result[4][4];
 
        if (node->is_fixed) {
            Frame::multiply(T, node->proj, result);
        } else {
            double rot[4][4];
            Frame::buildRotZ(node->c, node->s, rot);
 
            // T_joint = PROJ @ ROT_Z
            Frame::multiply(node->proj, rot, joint_transform);
 
            // Accumulate into running transform
            Frame::multiply(T, joint_transform, result);
        }
 
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                T[i][j] = result[i][j];
 
        traverse(node->next, T);
    }
};
