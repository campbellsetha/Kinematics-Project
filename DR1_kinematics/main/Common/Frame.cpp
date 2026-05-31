#include "Frame.hpp"
#include <cmath>

// End Effector
static Joint J6 = {
    .proj = {
        { 0,  0,  1, -0.0079},
        { 0,  1,  0,  0.0   },
        {-1,  0,  0, -0.0981},
        { 0,  0,  0,  1.0   }
    },
    .c        = 1.0,   
    .s        = 0.0,   
    .id       = 0xFF,  
    .is_fixed = true,
    .next     = nullptr
};

// Wrist roll
static Joint J5 = {
    .proj = {
        {-1,  0,  0,  0.0   },
        { 0,  0, -1, -0.0611},
        { 0, -1,  0,  0.0181},
        { 0,  0,  0,  1.0   }
    },
    .c        = 1.0,
    .s        = 0.0,
    .id       = 4,
    .is_fixed = false,
    .next     = &J6
};

// Wrist flex
static Joint J4 = {
    .proj = {
        { 0,  1,  0, -0.1349},
        {-1,  0,  0,  0.0052},
        { 0,  0,  1,  0.0   },
        { 0,  0,  0,  1.0   }
    },
    .c        = 1.0,
    .s        = 0.0,
    .id       = 3,
    .is_fixed = false,
    .next     = &J5
}; 

// Elbow flex
static Joint J3 = {
    .proj = {
        { 0, -1,  0, -0.1126},
        { 1,  0,  0, -0.0280},
        { 0,  0,  1,  0.0   },
        { 0,  0,  0,  1.0   }
    },
    .c        = 1.0,
    .s        = 0.0,
    .id       = 2,
    .is_fixed = false,
    .next     = &J4
};

// Sholder lift
static Joint J2 = {
    .proj = {
        { 0,  0, -1, -0.0304},
        { 1,  0,  0, -0.0183},
        { 0, -1,  0, -0.0542},
        { 0,  0,  0,  1.0   }
    },
    .c        = 1.0,
    .s        = 0.0,
    .id       = 1,
    .is_fixed = false,
    .next     = &J3
};

// Shoulder rotation
static Joint J1 = {
    .proj = {
        {-1,  0,  0,  0.0388},
        { 0,  1,  0,  0.0   },
        { 0,  0, -1,  0.0624},
        { 0,  0,  0,  1.0   }
    },
    .c        = 1.0,   // cos(0) at init
    .s        = 0.0,   // sin(0) at init
    .id       = 0,
    .is_fixed = false,
    .next     = &J2
};



static double jointOffsets[5] = {
    0.0,   
    0.0,   
    0.0,   
    0.0,   
    0.0    
};
 
void init(Frame& frame) { 
    frame.head = &J1;
}


    

    







