// Author: Seth Campbell
// All values sourced from so101_new_calib.urdf via TheRobotStudio/SO-ARM100
//
// Convention: T_joint(θ) = T_origin × Rz(θ)
// T_origin is built from URDF <origin xyz rpy> as:
//   R_origin = Rz(yaw) @ Ry(pitch) @ Rx(roll)
//   T_origin = [ R_origin | xyz ]
//              [  0  0  0 |  1  ]

#include "Frame.hpp"

// ── J6 — moving jaw (fixed, no servo) ────────────────────────
// URDF joint name="6"  parent="gripper"  child="moving_jaw_so101_v1"
// xyz="0.0202  0.0188  -0.0234"
// rpy="1.5708  ~0  ~0"   →  Rx(π/2)
// R_origin = [[1, 0,  0],
//             [0, 0, -1],
//             [0, 1,  0]]
static Joint J6 = {
    .proj = {
        { 1,  0,  0,  0.0202},
        { 0,  0, -1,  0.0188},
        { 0,  1,  0, -0.0234},
        { 0,  0,  0,  1.0   }
    },
    .c=1.0, .s=0.0, .id=0xFF, .is_fixed=true, .axis='z', .next=nullptr
};

// ── J5 — wrist roll ──────────────────────────────────────────
// URDF joint name="5"  parent="wrist"  child="gripper"
// xyz="~0  -0.0611  0.0181"
// rpy="1.5708  0.0486795  3.14159"
// R_origin = [[-0.9988154, -0.0486603,  0.0000028],
//             [ 0.0000027,  0.0000038,  1.0000000],
//             [-0.0486603,  0.9988154, -0.0000037]]
// axis='z'
static Joint J5 = {
    .proj = {
        {-0.9988154, -0.0486603,  0.0000028,  0.0   },
        { 0.0000027,  0.0000038,  1.0000000, -0.0611},
        {-0.0486603,  0.9988154, -0.0000037,  0.0181},
        { 0.0,        0.0,        0.0,         1.0   }
    },
    .c=1.0, .s=0.0, .id=4, .is_fixed=false, .axis='z', .next=&J6
};

// ── J4 — wrist flex ──────────────────────────────────────────
// URDF joint name="4"  parent="lower_arm"  child="wrist"
// xyz="-0.1349  0.0052  ~0"
// rpy="~0  ~0  -1.5708"   →  Rz(-π/2)
// R_origin = [[ 0,  1,  0],
//             [-1,  0,  0],
//             [ 0,  0,  1]]
// axis='z'
static Joint J4 = {
    .proj = {
        { 0,  1,  0, -0.1349},
        {-1,  0,  0,  0.0052},
        { 0,  0,  1,  0.0   },
        { 0,  0,  0,  1.0   }
    },
    .c=1.0, .s=0.0, .id=3, .is_fixed=false, .axis='z', .next=&J5
};

// ── J3 — elbow flex ──────────────────────────────────────────
// URDF joint name="3"  parent="upper_arm"  child="lower_arm"
// xyz="-0.11257  -0.028  ~0"
// rpy="~0  ~0  1.5708"   →  Rz(+π/2)
// R_origin = [[ 0, -1,  0],
//             [ 1,  0,  0],
//             [ 0,  0,  1]]
// axis='z'
static Joint J3 = {
    .proj = {
        { 0, -1,  0, -0.11257},
        { 1,  0,  0, -0.028  },
        { 0,  0,  1,  0.0    },
        { 0,  0,  0,  1.0    }
    },
    .c=1.0, .s=0.0, .id=2, .is_fixed=false, .axis='z', .next=&J4
};

// ── J2 — shoulder lift ───────────────────────────────────────
// URDF joint name="2"  parent="shoulder"  child="upper_arm"
// xyz="-0.0303992  -0.0182778  -0.0542"
// rpy="-1.5708  -1.5708  0"   →  Rz(0) @ Ry(-π/2) @ Rx(-π/2)
// R_origin = [[ 0,  1,  0],
//             [ 0,  0,  1],
//             [ 1,  0,  0]]
// axis='z'
static Joint J2 = {
    .proj = {
        { 0,  1,  0, -0.0303992},
        { 0,  0,  1, -0.0182778},
        { 1,  0,  0, -0.0542   },
        { 0,  0,  0,  1.0      }
    },
    .c=1.0, .s=0.0, .id=1, .is_fixed=false, .axis='z', .next=&J3
};

// ── J1 — shoulder pan ────────────────────────────────────────
// URDF joint name="1"  parent="base"  child="shoulder"
// xyz="-0.124202  -0.168068  0.0948817"
// rpy="3.14159  ~0  -3.14159"   →  Rz(-π) @ Ry(0) @ Rx(π)
// R_origin = [[-1,  0,  0],
//             [ 0,  1,  0],
//             [ 0,  0, -1]]
// axis='z'
static Joint J1 = {
    .proj = {
        {-1,  0,  0, -0.124202 },
        { 0,  1,  0, -0.168068 },
        { 0,  0, -1,  0.0948817},
        { 0,  0,  0,  1.0      }
    },
    .c=1.0, .s=0.0, .id=0, .is_fixed=false, .axis='z', .next=&J2
};

void init_chain(Frame& frame) {
    frame.head = &J1;
}