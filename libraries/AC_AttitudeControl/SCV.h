#pragma once
#include <AP_Math/AP_Math.h>

class SR_V{
public:
    bool detect_attack(int r);
    // bool detect_attack_vector(int r);
    
    void save_srv_float(float value);
    // void save_srv_vector(Vector3f value);

    float recover_scv_1(float value, int n);

    float recover_scv_2(float value, int n);

    void set_recovery(void){ recovery = true;};

    void reset_recovery(void){ recovery = false;};

    bool get_recovery(void){ return recovery;};

    float monitor_SCV(float x, int n);

    float monitor_actuator(float x, int n);

    bool roll_back(float n);

    void cal_recovery_1();

    void cal_recovery_2();

    bool check_actuator();

    enum RULE{
        _pwm_max = 0,
        _pwm_min = 1,
        _spin_max = 2,
        _spin_min = 3,
        compensation_gain =4,
        gyro_x = 5,
        gyro_y = 6,
        gyro_z = 7,
        throttle_thrust = 8,
        euler_roll_angle = 9,
        euler_pitch_angle = 10,
        euler_yaw_angle = 11,
        euler_roll_angle2 = 12,
        euler_pitch_angle2 = 13,
        euler_yaw_rate2 = 14,
        roll_p = 15,
        roll_i = 16,
        roll_d = 17,
        roll_ff = 18,
        pitch_p = 19,
        pitch_i = 20,
        pitch_d = 21,
        pitch_ff = 22,
        yaw_p = 23,
        yaw_i = 24,
        yaw_d = 25,
        yaw_ff = 26,
        _actuator1 = 27,
        _actuator2 = 28,
        _actuator3 = 29,
        _actuator4 = 30
    };


    SR_V(){
        loc = -1;
        count = 0;
        r_count = 0;
        recovery = false;
        rec_type = 0;
        save_rec = false;
        recovery_value = 0;
        first = true;
    }
    
protected:
    float _srv[400];
    int loc;
    int count;
    int r_count;
    bool recovery;
    float recovery_value;
    bool save_rec;
    int feature;
    int rec_type;
    bool first;
    float initial;

};
