#include "MotionProfile.h"
#include "Math.h"
#include "stdio.h"

// #define PRINT printf
#define PRINT if(false) printf
#define PRINT_JERK(x) PRINT("jerk: %.3f at %d\n", x, __LINE__)
#define PRINT_ACCEL(x) PRINT("accel: %.3f at %d\n", x, __LINE__)
#define PRINT_VEL(x) PRINT("vel: %.3f at %d\n", x, __LINE__)

static inline float pow3(float val){
    return val*val*val;
}
static inline float pow2(float val){
    return val*val;
}

static inline float trunc_ts(float val, float Ts){
    return int(val/Ts)*Ts;
}

MotionProfile::MotionProfile(MoveParams move, float _Ts) {
    Ts = _Ts;
    if (move.distance > 0) {
        direction = 1;
    } else {
        direction = -1;
    }

    jerk_limit = move.jerk;
    float t1_jerk_limit = move.jerk;
    PRINT_JERK(t1_jerk_limit);
    float absMove = abs(move.distance);

    t1 = cbrtf(absMove/(2.0f*t1_jerk_limit));
    t1 = trunc_ts(t1, Ts); // truncate to a controller update, possibly unnecessary?
    PRINT("t1: %.3f\n", t1);
    t1_jerk_limit = 0.5f*absMove/pow3(t1);
    PRINT_JERK(t1_jerk_limit);

    // check to see if velocity limit is violated with the 
    // given jerk limit
    float v_test = t1_jerk_limit*pow2(t1);
    if (v_test > move.velocity) {
        PRINT("velocity limit violated with given jerk limit\n");
        t1 = sqrtf(move.velocity/move.jerk);
        t1 = trunc_ts(t1, Ts);
        t1_jerk_limit = move.velocity/pow2(t1);
    }
    PRINT_JERK(t1_jerk_limit);

    // check to see if acceleration limit is violated with the 
    // jerk limit 
    // not sure if velocity or accel is more stringent?
    float a_test = t1_jerk_limit*t1;
    if (a_test > move.acceleration) {
        PRINT("acceleration limit violated with given jerk limit\n");
        PRINT("\tlimit:val %.3f:%.3f\n", move.acceleration, a_test);
        t1 = move.acceleration / move.jerk;
        t1 = trunc_ts(t1, Ts);
        t1_jerk_limit = move.acceleration/t1;
        PRINT("\tNew accel: %.3f\n", t1_jerk_limit*t1);
        PRINT("\tNew t1: %.3f\n", t1);
        PRINT_JERK(t1_jerk_limit);
    }
    PRINT_JERK(t1_jerk_limit);

    jerk_limit = t1_jerk_limit;
    float t2_jerk_limit = t1_jerk_limit;
    PRINT_JERK(t2_jerk_limit);
    // t1 is fixed, so we can precalculate pow2(t1)
    const float t1_2 = pow2(t1);
    const float t1_3 = t1_2 * t1;
    t2 = sqrtf(t1_2 / 4.0f + absMove/jerk_limit/t1) - 3.0f*t1/2.0f;
    t2 = trunc_ts(t2, Ts);
    PRINT("t2: %.3f\n", t2);

    t2_jerk_limit = absMove/(2.0f*t1_3 + 3.0f*t1_2 * t2 + t1*pow2(t2));
    PRINT_JERK(t2_jerk_limit);

    float t2_v_test = t2_jerk_limit * t1_2 + t2_jerk_limit * t1 * t2;
    if (t2_v_test > move.velocity) {
        PRINT("velocity limit violated with given accel limit");
        t2 = move.velocity/(jerk_limit*t1) - t1;
        t2 = trunc_ts(t2, Ts);
        t2_jerk_limit = move.velocity/(t1_2 + t1*t2);
    }
    jerk_limit = t2_jerk_limit;
    PRINT_JERK(jerk_limit);

    // t2 now fixed
    const float t2_2 = pow2(t2);
    t3 = (absMove - 2.0f*jerk_limit*t1_3 - 3.0f*jerk_limit*t1_2*t2 - jerk_limit*t1*t2_2)/move.velocity;
    t3 = trunc_ts(t3, Ts);
    jerk_limit = absMove/(2.0f*t1_3 + 3.0f*t1_2*t2 + t1*t2_2 + t1_2*t3 + t1*t2*t3);

    const float j = jerk_limit;
    PRINT_JERK(jerk_limit);

    REF_t[0] = move.start_time;
    REF_t[1] = REF_t[0] + t1; // jerk ramps acceleration up from 0
    REF_t[2] = REF_t[1] + t2; // constant accel
    REF_t[3] = REF_t[2] + t1; // jerk ramps accel down to 0
    REF_t[4] = REF_t[3] + t3; // constant velocity
    REF_t[5] = REF_t[4] + t1;
    REF_t[6] = REF_t[5] + t2; // constant accel
    REF_t[7] = REF_t[6] + t1;

//	Compute reference values at switching times
    REF_p[0] = 0.0f;
    
    REF_a[1] = j*(REF_t[1]-REF_t[0]);
    REF_v[1] = 0.5f*j*(REF_t[1]-REF_t[0])*(REF_t[1]-REF_t[0]);
    REF_p[1] = REF_p[0]+j*(REF_t[1]-REF_t[0])*(REF_t[1]-REF_t[0])*(REF_t[1]-REF_t[0])/6.0f;
    
    REF_a[2]=REF_a[1];
    REF_v[2]=REF_v[1]+REF_a[1]*(REF_t[2]-REF_t[1]);
    REF_p[2]=REF_p[1]+REF_v[1]*(REF_t[2]-REF_t[1])+0.5f*REF_a[1]*(REF_t[2]-REF_t[1])*(REF_t[2]-REF_t[1]);
    
    REF_a[3]=REF_a[2]-j*(REF_t[3]-REF_t[2]);
    REF_v[3]=REF_v[2]+REF_a[2]*(REF_t[3]-REF_t[2])-0.5f*j*(REF_t[3]-REF_t[2])*(REF_t[3]-REF_t[2]);
    REF_p[3]=REF_p[2]+REF_v[2]*(REF_t[3]-REF_t[2])+0.5f*REF_a[2]*(REF_t[3]-REF_t[2])*(REF_t[3]-REF_t[2])
    -j*(REF_t[3]-REF_t[2])*(REF_t[3]-REF_t[2])*(REF_t[3]-REF_t[2])/6.0f;
    
    REF_a[4]=0.0f;
    REF_v[4]=REF_v[3];
    REF_p[4]=REF_p[3]+REF_v[3]*(REF_t[4]-REF_t[3]);
    
    REF_a[5]=-j*(REF_t[5]-REF_t[4]);
    REF_v[5]=REF_v[4]-0.5f*j*(REF_t[5]-REF_t[4])*(REF_t[5]-REF_t[4]);
    REF_p[5]=REF_p[4]+REF_v[4]*(REF_t[5]-REF_t[4])-j*(REF_t[5]-REF_t[4])*(REF_t[5]-REF_t[4])*(REF_t[5]-REF_t[4])/6.0f;
    
    REF_a[6]=REF_a[5];
    REF_v[6]=REF_v[5]+REF_a[5]*(REF_t[6]-REF_t[5]);
    REF_p[6]=REF_p[5]+REF_v[5]*(REF_t[6]-REF_t[5])+0.5f*REF_a[5]*(REF_t[6]-REF_t[5])*(REF_t[6]-REF_t[5]);
    
    REF_a[7]=REF_a[6]+j*(REF_t[7]-REF_t[6]);
    REF_v[7]=REF_v[6]+REF_a[6]*(REF_t[7]-REF_t[6])+0.5f*j*(REF_t[7]-REF_t[6])*(REF_t[7]-REF_t[6]);
	REF_p[7] = absMove;
}

MoveParams MotionProfile::get(float t) {
    float jref=0;
    float accel=0;
    float vel=0;

    if (t <= REF_t[0]) {
        jref = 0;
        accel = 0;
        vel = 0;
    }
    else if (t <= REF_t[1]) {
        jref = jerk_limit;
        accel = jerk_limit*(t-REF_t[0]);
        vel = 0.5f*jerk_limit*(t-REF_t[0])*(t-REF_t[0]);
    }
    else if (t <= REF_t[2]) {
        jref = 0;
        accel = REF_a[1];
        vel = REF_v[1]+REF_a[1]*(t-REF_t[1]);
    }
    else if (t <= REF_t[3]) {
        jref = -jerk_limit;
        accel = REF_a[2]-jerk_limit*(t-REF_t[2]);
        vel = REF_v[2]+REF_a[2]*(t-REF_t[2])-0.5*jerk_limit*(t-REF_t[2])*(t-REF_t[2]);
    }
    else if (t <= REF_t[4]) {
        jref = 0;
        accel = 0;
        vel = REF_v[3];
    }
    else if (t <= REF_t[5]) {
        jref = -jerk_limit;
        accel = -jerk_limit*(t-REF_t[4]);
        vel = REF_v[4]-0.5f*jerk_limit*(t-REF_t[4])*(t-REF_t[4]);
    }
    else if (t <= REF_t[6]) {
        jref = 0;
        accel = REF_a[5];
        vel = REF_v[5]+REF_a[5]*(t-REF_t[5]);
    }
    else if (t <= REF_t[7]) {
        jref = jerk_limit;
        accel = REF_a[6]+jerk_limit*(t-REF_t[6]);
        vel = REF_v[6]+REF_a[6]*(t-REF_t[6])+0.5*jerk_limit*(t-REF_t[6])*(t-REF_t[6]);
    }
    else {
        jref = 0.0f;
        accel = 0.0f;
        vel = 0.0f;
    }

    // t -= rdelay*Ts; 

    if (t <= REF_t[0]) {
        pos = REF_p[0];
    }
    else if (t <= REF_t[1]) {
        pos = REF_p[0]+jerk_limit*(t-REF_t[0])*(t-REF_t[0])*(t-REF_t[0])/6.0;
    }
    else if (t <= REF_t[2]) {
        pos = REF_p[1]+REF_v[1]*(t-REF_t[1])+0.5f*REF_a[1]*(t-REF_t[1])*(t-REF_t[1]);
    }
    else if (t <= REF_t[3]) {
        pos = REF_p[2]+REF_v[2]*(t-REF_t[2])+0.5f*REF_a[2]*(t-REF_t[2])*(t-REF_t[2])
        -jerk_limit*(t-REF_t[2])*(t-REF_t[2])*(t-REF_t[2])/6.0f;
    }
    else if (t <= REF_t[4]) {
        pos = REF_p[3]+REF_v[3]*(t-REF_t[3]);
    }
    else if (t <= REF_t[5]) {
        pos = REF_p[4]+REF_v[4]*(t-REF_t[4])-jerk_limit*(t-REF_t[4])*(t-REF_t[4])*(t-REF_t[4])/6.0f;
    }
    else if (t <= REF_t[6]) {
        pos = REF_p[5]+REF_v[5]*(t-REF_t[5])+0.5f*REF_a[5]*(t-REF_t[5])*(t-REF_t[5]);
    }
    else if (t <= REF_t[7]) {
        pos = REF_p[6]+REF_v[6]*(t-REF_t[6])+0.5f*REF_a[6]*(t-REF_t[6])*(t-REF_t[6])
        +jerk_limit*(t-REF_t[6])*(t-REF_t[6])*(t-REF_t[6])/6.0f;
    }
    else {
        pos = REF_p[7];
    }
	
    MoveParams ret = MoveParams();
    ret.distance = direction*pos;
    ret.velocity = direction*vel;
    ret.acceleration = direction*accel;
    ret.jerk = direction*jref;
	return ret;
}