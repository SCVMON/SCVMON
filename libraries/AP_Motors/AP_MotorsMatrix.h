/// @file	AP_MotorsMatrix.h
/// @brief	Motor control class for Matrixcopters
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <RC_Channel/RC_Channel.h>     // RC Channel Library
#include "AP_MotorsMulticopter.h"
#include "AC_AttitudeControl/SCV.h"

#define AP_MOTORS_MATRIX_YAW_FACTOR_CW   -1
#define AP_MOTORS_MATRIX_YAW_FACTOR_CCW   1

/// @class      AP_MotorsMatrix
class AP_MotorsMatrix : public AP_MotorsMulticopter {
public:

    /// Constructor
    AP_MotorsMatrix(uint16_t loop_rate, uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT) :
        AP_MotorsMulticopter(loop_rate, speed_hz)
    {};

    // init
    void                init(motor_frame_class frame_class, motor_frame_type frame_type) override;

    // set frame class (i.e. quad, hexa, heli) and type (i.e. x, plus)
    void                set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type) override;

    // set update rate to motors - a value in hertz
    // you must have setup_motors before calling this
    void                set_update_rate(uint16_t speed_hz) override;

    // output_test_seq - spin a motor at the pwm value specified
    //  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
    //  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
    virtual void        output_test_seq(uint8_t motor_seq, int16_t pwm) override;

    // output_test_num - spin a motor connected to the specified output channel
    //  (should only be performed during testing)
    //  If a motor output channel is remapped, the mapped channel is used.
    //  Returns true if motor output is set, false otherwise
    //  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
    bool                output_test_num(uint8_t motor, int16_t pwm);

    // output_to_motors - sends minimum values out to the motors
    void                output_to_motors() override;

    // get_motor_mask - returns a bitmask of which outputs are being used for motors (1 means being used)
    //  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
    uint16_t            get_motor_mask() override;

    // return number of motor that has failed.  Should only be called if get_thrust_boost() returns true
    uint8_t             get_lost_motor() const override { return _motor_lost_index; }

    // return the roll factor of any motor, this is used for tilt rotors and tail sitters
    // using copter motors for forward flight
    float               get_roll_factor(uint8_t i) override { return _roll_factor[i]; }

protected:
    // output - sends commands to the motors
    void                output_armed_stabilizing() override;

    // check for failed motor
    void                check_for_failed_motor(float throttle_thrust_best);

    // add_motor using raw roll, pitch, throttle and yaw factors
    void                add_motor_raw(int8_t motor_num, float roll_fac, float pitch_fac, float yaw_fac, uint8_t testing_order);

    // add_motor using just position and yaw_factor (or prop direction)
    void                add_motor(int8_t motor_num, float angle_degrees, float yaw_factor, uint8_t testing_order);

    // add_motor using separate roll and pitch factors (for asymmetrical frames) and prop direction
    void                add_motor(int8_t motor_num, float roll_factor_in_degrees, float pitch_factor_in_degrees, float yaw_factor, uint8_t testing_order);

    // remove_motor
    void                remove_motor(int8_t motor_num);

    // configures the motors for the defined frame_class and frame_type
    virtual void        setup_motors(motor_frame_class frame_class, motor_frame_type frame_type);

    // normalizes the roll, pitch and yaw factors so maximum magnitude is 0.5
    void                normalise_rpy_factors();

    // call vehicle supplied thrust compensation if set
    void                thrust_compensation(void) override;

    float               _roll_factor[AP_MOTORS_MAX_NUM_MOTORS]; // each motors contribution to roll
    float               _pitch_factor[AP_MOTORS_MAX_NUM_MOTORS]; // each motors contribution to pitch
    float               _yaw_factor[AP_MOTORS_MAX_NUM_MOTORS];  // each motors contribution to yaw (normally 1 or -1)
    float               _thrust_rpyt_out[AP_MOTORS_MAX_NUM_MOTORS]; // combined roll, pitch, yaw and throttle outputs to motors in 0~1 range
    uint8_t             _test_order[AP_MOTORS_MAX_NUM_MOTORS];  // order of the motors in the test sequence
    motor_frame_class   _last_frame_class; // most recently requested frame class (i.e. quad, hexa, octa, etc)
    motor_frame_type    _last_frame_type; // most recently requested frame type (i.e. plus, x, v, etc)

    // motor failure handling
    float               _thrust_rpyt_out_filt[AP_MOTORS_MAX_NUM_MOTORS];    // filtered thrust outputs with 1 second time constant
    uint8_t             _motor_lost_index;  // index number of the lost motor


    SR_V                _scv_compensation_gain;

    SR_V                _scv_roll_thrust;
    SR_V                _scv_pitch_thrust;
    SR_V                _scv_yaw_thrust;
    SR_V                _scv_rpy_scale;
    SR_V                _scv_throttle_thrust_max;
    SR_V                _scv_yaw_allowed;

    SR_V                _scv_throttle_thrust_best_rpy;
    SR_V                _scv_throttle_thrust;

    SR_V                _scv_actuator_1;
    SR_V                _scv_actuator_2;
    SR_V                _scv_actuator_3;
    SR_V                _scv_actuator_4;


/*
    SR_V    xx1;
    SR_V    xx2;
    SR_V    xx3;
    SR_V    xx4;
    SR_V    xx5;
    SR_V    xx6;
    SR_V    xx7;
    SR_V    xx8;
    SR_V    xx9;
    SR_V    xx10;
    SR_V    xx11;
    SR_V    xx12;
    SR_V    xx13;
    SR_V    xx14;
    SR_V    xx15;

    
    SR_V    xx16;
    SR_V    xx17;
    SR_V    xx18;
    SR_V    xx19;
    SR_V    xx20;
    SR_V    xx21;
    SR_V    xx22;
    SR_V    xx23;
    SR_V    xx24;
    SR_V    xx25;
    SR_V    xx26;
    SR_V    xx27;
    SR_V    xx28;
    SR_V    xx29;
    SR_V    xx30;
    SR_V    xx31;
    SR_V    xx32;
    SR_V    xx33;
    SR_V    xx34;
    SR_V    xx35;
    SR_V    xx36;
    SR_V    xx37;
    SR_V    xx38;
    SR_V    xx39;
    SR_V    xx40;
    
    SR_V    xx41;
    SR_V    xx42;
    SR_V    xx43;
    SR_V    xx44;
    SR_V    xx45;
    SR_V    xx46;
    SR_V    xx47;
    SR_V    xx48;
    SR_V    xx49;
    SR_V    xx50;
    SR_V    xx51;
    SR_V    xx52;
    SR_V    xx53;
    SR_V    xx54;
    SR_V    xx55;
    
    SR_V    xx56;
    SR_V    xx57;
    SR_V    xx58;
    SR_V    xx59;
    SR_V    xx60;
    SR_V    xx61;
    SR_V    xx62;
    SR_V    xx63;
    SR_V    xx64;
    SR_V    xx65;
    SR_V    xx66;
    SR_V    xx67;
    SR_V    xx68;
    SR_V    xx69;
    SR_V    xx70;
    SR_V    xx71;
    SR_V    xx72;
    SR_V    xx73;
    SR_V    xx74;
    SR_V    xx75;
    SR_V    xx76;
    SR_V    xx77;
    SR_V    xx78;
    SR_V    xx79;
    SR_V    xx80;
    
    SR_V    xx81;
    SR_V    xx82;
    SR_V    xx83;
    SR_V    xx84;
    SR_V    xx85;
    SR_V    xx86;
    SR_V    xx87;
    SR_V    xx88;
    SR_V    xx89;
    SR_V    xx90;
    SR_V    xx91;
    SR_V    xx92;
    SR_V    xx93;
    SR_V    xx94;
    SR_V    xx95;
    
    SR_V    xx96;
    SR_V    xx97;
    SR_V    xx98;
    SR_V    xx99;
    SR_V    xx100;
    SR_V    xx101;
    SR_V    xx102;
    SR_V    xx103;
    SR_V    xx104;
    SR_V    xx105;
    SR_V    xx106;
    SR_V    xx107;
    SR_V    xx108;
    SR_V    xx109;
    SR_V    xx110;
    SR_V    xx111;
    SR_V    xx112;
    SR_V    xx113;
    SR_V    xx114;
    SR_V    xx115;
    SR_V    xx116;
    SR_V    xx117;
    SR_V    xx118;
    SR_V    xx119;
    SR_V    xx120;
    
    SR_V    xx121;
    SR_V    xx122;
    SR_V    xx123;
    SR_V    xx124;
    SR_V    xx125;
    SR_V    xx126;
    SR_V    xx127;
    SR_V    xx128;
    SR_V    xx129;
    SR_V    xx130;
    SR_V    xx131;
    SR_V    xx132;
    SR_V    xx133;
    SR_V    xx134;
    SR_V    xx135;
    
    SR_V    xx136;
    SR_V    xx137;
    SR_V    xx138;
    SR_V    xx139;
    SR_V    xx140;
    SR_V    xx141;
    SR_V    xx142;
    SR_V    xx143;
    SR_V    xx144;
    SR_V    xx145;
    SR_V    xx146;
    SR_V    xx147;
    SR_V    xx148;
    SR_V    xx149;
    SR_V    xx150;
    SR_V    xx151;
    SR_V    xx152;
    SR_V    xx153;
    SR_V    xx154;
    SR_V    xx155;
    SR_V    xx156;
    SR_V    xx157;
    SR_V    xx158;
    SR_V    xx159;
    SR_V    xx160;

    
    SR_V    xx161;
    SR_V    xx162;
    SR_V    xx163;
    SR_V    xx164;
    SR_V    xx165;
    SR_V    xx166;
    SR_V    xx167;
    SR_V    xx168;
    SR_V    xx169;
    SR_V    xx170;
    SR_V    xx171;
    SR_V    xx172;
    SR_V    xx173;
    SR_V    xx174;
    SR_V    xx175;
    
    SR_V    xx176;
    SR_V    xx177;
    SR_V    xx178;
    SR_V    xx179;
    SR_V    xx180;
    SR_V    xx181;
    SR_V    xx182;
    SR_V    xx183;
    SR_V    xx184;
    SR_V    xx185;
    SR_V    xx186;
    SR_V    xx187;
    SR_V    xx188;
    SR_V    xx189;
    SR_V    xx190;
    SR_V    xx191;
    SR_V    xx192;
    SR_V    xx193;
    SR_V    xx194;
    SR_V    xx195;
    SR_V    xx196;
    SR_V    xx197;
    SR_V    xx198;
    SR_V    xx199;
    SR_V    xx200;
    
    SR_V    xx201;
    SR_V    xx202;
    SR_V    xx203;
    SR_V    xx204;
    SR_V    xx205;
    SR_V    xx206;
    SR_V    xx207;
    SR_V    xx208;
    SR_V    xx209;
    SR_V    xx210;
    SR_V    xx211;
    SR_V    xx212;
    SR_V    xx213;
    SR_V    xx214;
    SR_V    xx215;
    SR_V    xx216;
    SR_V    xx217;
    SR_V    xx218;
    SR_V    xx219;
    SR_V    xx220;
    SR_V    xx221;
    SR_V    xx222;
    SR_V    xx223;
    SR_V    xx224;
    SR_V    xx225;
    SR_V    xx226;
    SR_V    xx227;
    SR_V    xx228;
    SR_V    xx239;
    SR_V    xx240;
*/

};
