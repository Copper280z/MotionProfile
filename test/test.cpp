#include "MotionProfile.h"
#include <stdio.h>
#include <vector>

float vmax(std::vector<float> vec) {

    float max_val = 0;
    for (auto val : vec) {
        max_val = fmaxf(val, max_val);
    }
    return max_val;
}

int main(int argc, char** argv){
    (void) argc;
    (void) argv;

    MoveParams params;
    params.start_time = 0;
    params.acceleration = 20;
    params.distance = 10;
    params.velocity = 8;
    params.jerk = 100;

    float Ts = 1.0/20000.0;
    float max_time = 100;
    float t=0;

    MotionProfile profile(params, Ts);

    std::vector<float> jerk;
    std::vector<float> accel;
    std::vector<float> vel;
    std::vector<float> pos;
    
    while (t<max_time) {
        auto step = profile.get(t);
        jerk.push_back(step.jerk);
        accel.push_back(step.acceleration);
        vel.push_back(step.velocity);
        pos.push_back(step.distance);
        printf("%.4f, %.3f, %.3f, %.3f, %.3f\n", t, step.distance, step.velocity, step.acceleration, step.jerk);
        t+=Ts;
        if (fabsf(step.distance - params.distance) < 1e-8) break;
    }

    float max_jerk = vmax(jerk);
    float max_accel = vmax(accel);
    float max_vel = vmax(vel);
    float end_pos = *pos.end();

    // printf("set:max jerk: %.3f:%.3f, accel: %.3f:%.3f, vel: %.3f,%.3f\n", params.jerk, max_jerk, params.acceleration, max_accel, params.velocity, max_vel);
}