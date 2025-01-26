#pragma once

#include <inttypes.h>

// more profiles to investigate
// https://www.20sim.com/webhelp/toolboxes_mechatronics_toolbox_motion_profile_wizard_motionprofiles.php

struct MoveParams{
    uint32_t start_time = 0;
    float distance = 0;
    float velocity = 0; 
    float acceleration = 0;
    float jerk = 0;
};

class MotionProfile {
    public:
        MotionProfile(MoveParams move, float Ts);

        /*
        Calculates the profile paramters for the given time, relative to the start time
        */
        MoveParams get(float time);

    private:  
        float Ts = 1.0f/20000.0f;      
        float t1=0;
        float t2=0;
        float t3=0;
        float t_total=0;
        float jerk_limit = 0;
        float target_position=0;

        float accel = 0;
        float vel = 0;
        float pos = 0;

        float REF_t[8] = {0};
        float REF_a[8] = {0};
        float REF_v[8] = {0};
        float REF_p[8] = {0};

        MoveParams limits;
};