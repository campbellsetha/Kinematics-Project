#include <cmath>

double PROJ_J2_ONTO_J1;
double PROJ_J3_ONTO_J2;
double PROJ_J4_ONTO_J3;
double PROJ_J5_ONTO_J4;
double PROJ_J6_ONTO_J5;

double ROT_X;
double ROT_Y;
double ROT_Z;

double ALPHA;
double BETA;
double GAMMA;


main(){
    setup();
    home();
}

void setup() {

    ROT_X = {
        {std::cos(ALPHA), -std::sin(ALPHA), 0},
        {std::sin(ALPHA), std::cos(ALPHA), 0},
        {0, 0, 1}
    };

    ROT_Y = {
        {std::cos(BETA), 0, std::sin(BETA)},
        {0, 1, 0},
        {-std::sin(BETA), 0, std::cos(BETA)}
    };

    ROT_Z = {
        {1, 0, 0},
        {0, std::cos(GAMMA), -std::sin(GAMMA)},
        {0, std::sin(GAMMA), std::cos(GAMMA)}
    };
    // Need to clarify x axis.
    PROJ_J2_ONTO_J1 = {
        {-1, 0, 0},
        {0, 0, -1},
        {0, -1, 0}
    };

    PROJ_J3_ONTO_J2 = {
        {-1, 0, 0},
        {0, -1, 0},
        {0, 0, 1}
    };

    PROJ_J4_ONTO_J3 = {
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1}
    };

    // Need clarification on x and y axis.
    PROJ_J5_ONTO_J4 = {
        {},
        {},
        {}
    };

    // End effector (gripper) actuation.
    PROJ_J6_ONTO_J5 = {
        {},
        {},
        {}
    };
}

void setJointAngle(int jointID, double theAngle){

}

double setPosition() {
    return 0.0;
}