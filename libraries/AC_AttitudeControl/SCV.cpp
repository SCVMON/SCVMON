#include "SCV.h"
#include <AP_Math/AP_Math.h>

extern int t_a1;
long r_check = 0;
bool total_recovery = false;
//float var[40];
float act[4];
float act_p[400] = {0};

int modulo(int a, int b) {
    int m = a % b;
    if (m < 0) {
        m = (b < 0) ? m - b : m + b;
    }
    return m;
}

bool SR_V::detect_attack(int rule_number){
    float avg = 100;
    float max_val = 100;
    float val1 = 0;
    float sum = 0;
    int wd = 5;
    int tmp, tmp1, tmp2;

    switch(rule_number){
        case _pwm_max:{
            rec_type = 1;
            break;
        }
        case _pwm_min:{
            rec_type = 1;
            break;
        }
        case _spin_max:{
            rec_type = 1;
            break;
        }
        case _spin_min:{
            rec_type = 1;
            break;
        }
        case compensation_gain:{
            rec_type = 1;
            break;
        }
        case gyro_x:{
            rec_type = 2;
            avg = 0.05;
            max_val = 0.02;
            break;
        }
        case gyro_y:{
            rec_type = 2;
            avg = 0.05; 
            max_val = 0.02;
            break;
        }
        case gyro_z:{
            rec_type = 2;
            avg = 0.05;
            wd = 20;
            max_val = 0.02; 
            break;
        }
        case throttle_thrust:{
            rec_type = 2;
            avg = 0.2;
            wd = 7;
            max_val = 0.1;
            break;
        }
        case euler_roll_angle:{
            rec_type = 2;
            avg = 0.1;
            max_val = 0.05;
            break;
        }
        case euler_pitch_angle:{
            rec_type = 2;
            avg = 0.1;
            max_val = 0.05;
            break;
        }
        case euler_yaw_angle:{
            rec_type = 2;
            avg = 0.25;
            max_val = 0.15;
            break;
        }
        case euler_roll_angle2:{
            rec_type = 2;
            avg = 0.1;
            max_val = 0.05;
            break;
        }
        case euler_pitch_angle2:{
            rec_type = 2;
            avg = 0.1;
            max_val = 0.05;
            break;
        }
        case euler_yaw_rate2:{
            rec_type = 2;
            avg = 0.25;
            max_val = 0.15;
            break;
        }
        case roll_p:{
            rec_type = 1;
            break;
        }
        case roll_i:{
            rec_type = 1;
            break;
        }
        case roll_d:{
            rec_type = 1;
            break;
        }
        case roll_ff:{
            rec_type = 1;
            break;
        }
        case pitch_p:{
            rec_type = 1;
            break;
        }
        case pitch_i:{
            rec_type = 1;
            break;
        }
        case pitch_d:{
            rec_type = 1;
            break;
        }
        case pitch_ff:{
            rec_type = 1;
            break;
        }
        case yaw_p:{
            rec_type = 1;
            break;
        }
        case yaw_i:{
            rec_type = 1;
            break;
        }
        case yaw_d:{
            rec_type = 1;
            break;
        }
        case yaw_ff:{
            rec_type = 1;
            break;
        }
        case _actuator1:{
            rec_type = 2;
            avg = 0.1;
            max_val = 0.05;
            break;
        }
        case _actuator2:{
            rec_type = 2;
            avg = 0.1;
            max_val = 0.05;
            break;
        }
        case _actuator3:{
            rec_type = 2;
            avg = 0.1;
            max_val = 0.05;
            break;
        }
        case _actuator4:{
            rec_type = 2;
            avg = 0.1;
            max_val = 0.05;
            break;
        }
        default:{
            avg = 1;
            max_val = 5;
            break;
        }
    }
    if(rec_type==1){
        tmp = modulo(loc-1 , 400);
        val1 = fabsf(_srv[loc] - _srv[tmp]);
        if(val1 > 0){
            return true;
        }    
    }
    else if(rec_type==2){
        tmp = modulo(loc-1 , 400);
        if(rule_number == 12){
            val1 = fabsf(fabsf(_srv[loc]) - fabsf(_srv[loc]));
        }
        else {
            val1 = fabsf(_srv[loc] - _srv[tmp]);
        }
        if(val1 > max_val){
            return true; 
        }
        sum = 0;
        for(int i=0; i< wd; i++){
            tmp1 = modulo(loc-i , 400);
            tmp2 = modulo(loc-i-1 , 400);
            if(rule_number == 12){
                val1 = fabsf(fabsf(_srv[tmp1]) - fabsf(_srv[tmp2]));
            }
            else {
                val1 = fabsf(_srv[tmp1]-_srv[tmp2]);
            }
            sum = sum + val1;
        }
        if(sum > avg){
            return true;
        }
    }
    return false;
}


void SR_V::save_srv_float(float value){

    if(first == true){
        for(int i =0; i< 400; i++){
            _srv[i] = value;
        }
        first = false;
        initial = value;
    }
    loc = (loc + 1) % 400;
    _srv[loc] = value;
}

bool SR_V::roll_back(float value){

    cal_recovery_2();

    if(fabsf((recovery_value-value)/recovery_value) <0.1f || fabsf(recovery_value-value) < 0.01f ){
        r_count++;
    }
    else{
        r_count = 0;
    }
    if(r_count>5){
        r_count = 0;
        return true;
    }
    return false;

}

float SR_V::recover_scv_1(float value, int n){

    float tmp;
    if(save_rec == false){
        cal_recovery_1();
        // n-400 step
        save_rec = true;
    }
    tmp = fabsf(value- recovery_value); 
    if(tmp <= 0){
        reset_recovery();
        r_check = r_check ^ (1<<n);
        save_rec = false;
        return value;
    }

    return recovery_value;
}

float SR_V::recover_scv_2(float value, int n){

    if(save_rec == false){
        cal_recovery_2();
        // n-400 step
        save_rec = true;
    }    

    // f = recovery end limit

    // stop recovery 
    if (fabsf(fabsf(value - recovery_value) / recovery_value ) < 0.1f ){
        r_count++;
    }
    else if (fabsf(value - recovery_value) < 0.01f && fabsf(recovery_value) > 0.1f){
        r_count++;
    }
    else {
        r_count = 0;
    }
    if (r_count > 20){
        r_count = 0;
        reset_recovery();
        r_check = r_check ^ (1<<n);
        save_rec = false;
        return value;
    }
    return recovery_value;
}

bool SR_V::check_actuator(){
    float avg, sum, sd = 0;
    int tmp1, tmp2;

    avg = (act[0] + act[1] + act[2] + act[3]) /4;

    for(int i=0; i<4; i++){
        sd = sd + fabsf(act[i]-avg);
    }
    act_p[loc] = sd;

    sum = 0;
    for(int i=0; i< 10; i++){
        tmp1 = modulo(loc-i , 400);
        tmp2 = modulo(loc-i-1 , 400);
        sum = sum + fabsf(act_p[tmp1]-act_p[tmp2]);
    }

    if(sum > 0.1){
        return false;
    }
    return true;
}

float SR_V::monitor_actuator(float x, int n){

    save_srv_float(x);
    act[n-27] = x;

    // Identifies if it is necessary to enter recovery mode

    if((r_check >> 27 != 0) && (r_check % 134217728!= 0)){
        total_recovery = true;

        //  actuator value changed + SCV changed
    }
    else{
        total_recovery = false;
    }

    if(get_recovery() == false && detect_attack(n) == true){
        r_check = r_check | (1<<n);
        set_recovery();
    }

    if (n==30){
        if(check_actuator()== false){
            r_check = r_check | (1<<n);
            set_recovery();
        }
    }
    return x;
}  
float SR_V::monitor_SCV(float x, int n){

    save_srv_float(x);

    if((r_check >> 27 != 0) && (r_check % 134217728!= 0)){
        total_recovery = true;
        //  actuator value changed + SCV changed
    }
    else{
        total_recovery = false;
    }
    if(get_recovery() == false && detect_attack(n) == true ){
        r_check = r_check | (1<<n);
        set_recovery();
    }
    if (get_recovery() == true && total_recovery == false){
        if(rec_type ==2){
            if(roll_back(x) == true){
                r_check = r_check ^ (1<<n);
                reset_recovery();
            }
        }
    }
    else if (get_recovery() == true && total_recovery == true){
        if(rec_type == 1){
            x = recover_scv_1(x, n);
        }
        else if(rec_type ==2){
            x = recover_scv_2(x, n);
        }
    }
    return x;
}

void SR_V::cal_recovery_1(){
    recovery_value = initial;

}

void SR_V::cal_recovery_2(){
    float r_value =0;   
    int i, tmp;

    for (i=1; i<=20; i++){
        tmp = (loc+i+300)%400;
        r_value = r_value + _srv[tmp];
    }
    recovery_value= r_value / 20;
}
