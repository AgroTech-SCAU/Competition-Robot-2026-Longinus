/* Service integration regression: real odom + fusion/Kalman, mocked device boundaries. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "odom.h"
#include "chassis.h"
#include "imu.h"

static ImuSample sample;
static ChassisObservation wheel;
static SteerWheelControl command;
static bool output_enabled;
static bool imu_failure;
static bool sample_failure;
static bool calibrated;
static bool stationary;
static unsigned calibration_count;
static unsigned calibration_target;
static unsigned calibration_resets;
static uint32_t now_us;
static float tilt_roll, tilt_pitch;
static ImuAngle imu_angle;

static void near(float actual,float expected,float tolerance){
    if(fabsf(actual-expected)>tolerance){
        fprintf(stderr,"actual=%g expected=%g tolerance=%g\n",actual,expected,tolerance);
        assert(0);
    }
}
static void near_double(double actual,double expected,double tolerance){
    if(!isfinite(actual) || fabs(actual-expected)>tolerance){
        fprintf(stderr,"actual=%.12g expected=%.12g tolerance=%g\n",actual,expected,tolerance);
        assert(0);
    }
}
static ImuStatus mock_update(void){
    if(imu_failure)return IMU_STATUS_ERROR;
    if(!calibrated && stationary && ++calibration_count>=calibration_target)calibrated=true;
    return IMU_STATUS_OK;
}
static ImuStatus mock_get_sample(ImuSample *out){
    if(sample_failure)return IMU_STATUS_NOT_READY;
    *out=sample;return IMU_STATUS_OK;
}
static ImuGyro mock_gyro(void){return sample.gyro;}
static ImuGyro mock_bias(void){return (ImuGyro){0};}
static ImuAngle mock_angle(void){return imu_angle;}
static ImuStatus mock_stationary(bool value){
    stationary=value;
    if(!value && !calibrated){
        if(calibration_count>0)++calibration_resets;
        calibration_count=0;
    }
    return IMU_STATUS_OK;
}
static bool mock_calibrated(void){return calibrated;}
static const ImuInterface mock_imu={
    .update=mock_update,.get_sample=mock_get_sample,.get_gyro=mock_gyro,
    .get_gyro_corrected=mock_gyro,.get_gyro_bias=mock_bias,.get_angle=mock_angle,
    .set_stationary=mock_stationary,.is_calibrated=mock_calibrated
};
static const SteerWheelControl* mock_control(void){return &command;}
const struct ChassisInterface chassis_interface={.get_control=mock_control};
bool chassis_get_observation(ChassisObservation *out){*out=wheel;return wheel.valid;}
bool chassis_is_output_enabled(void){return output_enabled;}
void chassis_update_imu_tilt(float roll,float pitch){tilt_roll=roll;tilt_pitch=pitch;}
static uint32_t clock_us(void *context){(void)context;return now_us;}

static void reset_case(bool already_calibrated){
    memset(&sample,0,sizeof(sample));memset(&wheel,0,sizeof(wheel));memset(&command,0,sizeof(command));
    memset(&imu_angle,0,sizeof(imu_angle));
    now_us=100000;
    sample.acc.z=9.80665f;sample.temperature=25;
    sample.gyro_timestamp_us=now_us;sample.acc_timestamp_us=now_us-10;sample.temp_timestamp_us=now_us;
    sample.flags=IMU_SAMPLE_GYRO_NEW|IMU_SAMPLE_GYRO_VALID|IMU_SAMPLE_ACC_NEW|IMU_SAMPLE_ACC_VALID|IMU_SAMPLE_TEMP_VALID;
    wheel.valid=true;wheel.sample_sequence=1;wheel.sample_time_ms=now_us/1000;
    output_enabled=false;imu_failure=false;sample_failure=false;calibrated=already_calibrated;
    stationary=false;calibration_count=calibration_resets=0;calibration_target=4;
    tilt_roll=tilt_pitch=NAN;
    imu_set_instance(&mock_imu);
    OdomConfig c=odom_default_config();c.now_us=clock_us;
    assert(odom_init(&c)==ODOM_OK);
}
static void advance(uint32_t delta_us,bool new_wheel){
    now_us+=delta_us;
    sample.gyro_timestamp_us=now_us;sample.acc_timestamp_us=now_us-10;sample.temp_timestamp_us=now_us;
    if(new_wheel){++wheel.sample_sequence;wheel.sample_time_ms=now_us/1000;}
}
static void move_forward(void){
    wheel.velocity.vx=wheel.raw_velocity.vx=1.0f;command.vx=1.0f;
    for(unsigned i=0;i<12;++i){advance(4000,true);assert(odom_process()==ODOM_OK);}
    assert(odom_is_ready());assert(odom_get_state()->velocity.x>0.9f);
}

static void test_boot_calibration_gate(void){
    reset_case(false);
    for(unsigned i=0;i<calibration_target-1;++i){
        advance(4000,true);assert(odom_process()==ODOM_NOT_READY);
        assert(calibration_count==i+1);assert(stationary);assert(!odom_is_ready());
    }
    advance(4000,true);assert(odom_process()==ODOM_OK);
    assert(calibrated && calibration_resets==0 && odom_is_ready());

    /* Missing motor feedback while output is disabled must not deadlock startup IMU calibration. */
    reset_case(false);wheel.valid=false;
    for(unsigned i=0;i<calibration_target;++i){advance(4000,false);assert(odom_process()==ODOM_NOT_READY);}
    assert(calibrated && calibration_resets==0);assert(!odom_is_ready());

    /* With output enabled, missing wheel feedback cannot prove standstill. */
    reset_case(false);wheel.valid=false;output_enabled=true;
    for(unsigned i=0;i<8;++i){advance(4000,false);assert(odom_process()==ODOM_NOT_READY);}
    assert(!calibrated && calibration_count==0);
    reset_case(false);command.vx=0.1f;
    for(unsigned i=0;i<8;++i){advance(4000,true);assert(odom_process()==ODOM_NOT_READY);}
    assert(!calibrated && calibration_count==0);
}
static void test_stationary_feedback_and_motion_exit(void){
    reset_case(true);
    for(unsigned i=0;i<160;++i){advance(4000,true);assert(odom_process()==ODOM_OK);}
    assert(odom_get_state()->static_window && stationary);
    command.vx=0.1f;
    advance(4000,true);assert(odom_process()==ODOM_OK);
    assert(!odom_get_state()->static_window && !stationary);
}
static void test_imu_failure_invalidates_ready(void){
    reset_case(true);move_forward();
    const Vector3 position=odom_get_state()->odom;
    imu_failure=true;advance(4000,true);
    assert(odom_process()==ODOM_NOT_READY);assert(!odom_is_ready());
    assert(!odom_get_state()->imu_ready && !odom_get_state()->fusion_ready && !stationary);
    assert(isnan(tilt_roll) && isnan(tilt_pitch));
    Vector3 v;
    assert(odom_get_velocity(&v)==ODOM_NOT_READY);assert(odom_get_odom(&v)==ODOM_NOT_READY);
    near(odom_get_state()->odom.x,position.x,1e-8f);
    imu_failure=false;sample_failure=true;advance(4000,true);
    assert(odom_process()==ODOM_NOT_READY);assert(!odom_is_ready());
    sample_failure=false;advance(4000,true);assert(odom_process()==ODOM_OK);
}
static void test_duplicate_gyro_and_long_gap(void){
    reset_case(true);move_forward();
    Vector3 previous=odom_get_state()->odom;
    uint32_t discontinuities=odom_get_state()->discontinuity_count;
    /* Even if a buggy device repeats NEW, the timestamp identity forbids re-integration. */
    for(unsigned i=0;i<4;++i){
        assert(odom_process()==ODOM_NOT_READY);assert(!odom_get_state()->timing_valid);
        near(odom_get_state()->odom.x,previous.x,1e-8f);
    }
    assert(odom_get_state()->discontinuity_count>discontinuities);
    advance(4000,true);assert(odom_process()==ODOM_OK);
    previous=odom_get_state()->odom;
    advance(80000,true);assert(odom_process()==ODOM_NOT_READY);
    assert(!odom_get_state()->timing_valid);assert(odom_get_state()->process_dt_s>0.05f);
    near(odom_get_state()->odom.x,previous.x,1e-8f);
    advance(4000,true);assert(odom_process()==ODOM_OK);
    near(odom_get_state()->process_dt_s,0.004f,1e-7f);
}
static void test_wheel_sequence_fused_once(void){
    reset_case(true);command.vx=1;wheel.velocity.vx=wheel.raw_velocity.vx=1;
    assert(odom_process()==ODOM_OK);
    const float first=odom_get_state()->velocity.x;
    assert(first>0.0f && first<0.999f);
    for(unsigned i=0;i<8;++i){
        advance(4000,false);assert(odom_process()==ODOM_OK);
        near(odom_get_state()->velocity.x,first,1e-6f);
    }
    advance(4000,true);assert(odom_process()==ODOM_OK);
    assert(odom_get_state()->velocity.x>first+0.001f);
}
static void test_missing_wheel_stops_unbounded_prediction(void){
    reset_case(true);move_forward();
    const Vector3 last_valid=odom_get_state()->odom;
    wheel.valid=false;wheel.age_ms=1000;
    for(unsigned i=0;i<500;++i){
        advance(4000,false);assert(odom_process()==ODOM_NOT_READY);
        assert(!odom_is_ready() && !odom_get_state()->wheel_valid && !stationary);
    }
    /* Missing velocity is not zero measured velocity, nor permission to extrapolate forever. */
    near(odom_get_state()->odom.x,last_valid.x,0.01f);
    near(odom_get_state()->odom.y,last_valid.y,0.01f);
    wheel.valid=true;wheel.velocity.vx=0;wheel.raw_velocity.vx=0;command.vx=0;
    advance(4000,true);assert(odom_process()==ODOM_OK);
    near(odom_get_state()->odom.x,last_valid.x,0.01f);
}
static void assert_yaw_integrals(double wheel_rad,double imu_rad,double time_s,unsigned intervals){
    const Odom *state=odom_get_state();
    near_double(state->wheel_yaw_integral_rad,wheel_rad,1e-6);
    near_double(state->imu_yaw_integral_rad,imu_rad,1e-6);
    near_double(state->yaw_integrated_time_s,time_s,1e-6);
    assert(state->yaw_integrated_samples==intervals);
}
static void test_yaw_integrals_signed_constant_speed(void){
    for(int sign=-1;sign<=1;sign+=2){
        reset_case(true);
        command.wz=(float)sign*2.0f;
        wheel.velocity.wz=sample.gyro.z=command.wz;
        assert(odom_process()==ODOM_OK);
        assert_yaw_integrals(0,0,0,0); /* First endpoint has no elapsed interval. */
        for(unsigned i=0;i<1000;++i){
            /* A valid held wheel measurement is still the current rate input. */
            advance(4000,false);assert(odom_process()==ODOM_OK);
        }
        assert_yaw_integrals(sign*8.0,sign*8.0,4.0,1000); /* No angle wrapping. */
        OdomConfig c=odom_default_config();c.now_us=clock_us;
        assert(odom_init(&c)==ODOM_OK);
        assert_yaw_integrals(0,0,0,0);
        assert(odom_process()==ODOM_OK);
        assert_yaw_integrals(0,0,0,0);
    }
}
static void test_yaw_integrals_projection_scales_and_actual_dt(void){
    reset_case(true);
    OdomConfig c=odom_default_config();c.now_us=clock_us;
    c.wheel_yaw_rate_scale=1.5f;c.imu_yaw_rate_scale=0.75f;
    assert(odom_init(&c)==ODOM_OK);
    command.wz=1;wheel.velocity.wz=0.4f;
    sample.gyro.y=0.2f;sample.gyro.z=0.5f;
    imu_angle.roll=0.3f;imu_angle.pitch=0.4f;
    assert(odom_process()==ODOM_OK);
    for(unsigned i=0;i<100;++i){
        advance(i%2==0?2000:6000,true);assert(odom_process()==ODOM_OK);
    }
    const double projected=(0.2*sin(0.3)+0.375*cos(0.3))/cos(0.4);
    assert_yaw_integrals(0.6*0.4,projected*0.4,0.4,100);
}
static void test_yaw_integrals_capture_short_pulse(void){
    reset_case(true);
    assert(odom_process()==ODOM_OK);
    for(unsigned i=1;i<=50;++i){
        const bool pulse=i>=10 && i<15;
        command.wz=wheel.velocity.wz=pulse?0.8f:0.0f;
        sample.gyro.z=pulse?0.4f:0.0f;
        advance(4000,true);assert(odom_process()==ODOM_OK);
    }
    /* A 5 Hz reader sees zero at both 0 and 200 ms; all five pulse points are
       nevertheless included in the service's triangular edge/plateau area. */
    assert_yaw_integrals(0.8*0.020,0.4*0.020,0.2,50);
}
static void test_yaw_integrals_never_bridge_invalid_endpoints(void){
    enum { DUPLICATE,GAP,WHEEL_INVALID,ANGLE_INVALID,PITCH_SINGULAR,
           CALIBRATING,GYRO_INVALID,UPDATE_FAILURE,SAMPLE_FAILURE,WHEEL_NAN,CASE_COUNT };
    for(unsigned fault=DUPLICATE;fault<CASE_COUNT;++fault){
        reset_case(true);
        command.wz=wheel.velocity.wz=1;sample.gyro.z=2;
        assert(odom_process()==ODOM_OK);
        advance(4000,true);assert(odom_process()==ODOM_OK);
        assert_yaw_integrals(0.004,0.008,0.004,1);
        if(fault!=DUPLICATE)advance(fault==GAP?80000:4000,true);
        switch(fault){
            case WHEEL_INVALID:wheel.valid=false;break;
            case ANGLE_INVALID:imu_angle.roll=NAN;break;
            case PITCH_SINGULAR:imu_angle.pitch=acosf(0.1f);break;
            case CALIBRATING:calibrated=false;break;
            case GYRO_INVALID:sample.flags&=~IMU_SAMPLE_GYRO_VALID;break;
            case UPDATE_FAILURE:imu_failure=true;break;
            case SAMPLE_FAILURE:sample_failure=true;break;
            case WHEEL_NAN:wheel.velocity.wz=NAN;break;
            default:break;
        }
        /* Existing fusion can still use wheel observations near the Euler
           singularity; only these two-source diagnostics lose that endpoint. */
        assert(odom_process()==(fault==PITCH_SINGULAR?ODOM_OK:ODOM_NOT_READY));
        assert_yaw_integrals(0.004,0.008,0.004,1);
        wheel.valid=true;wheel.velocity.wz=1;imu_angle=(ImuAngle){0};
        calibrated=true;sample.flags|=IMU_SAMPLE_GYRO_VALID;
        imu_failure=sample_failure=false;
        advance(4000,true);assert(odom_process()==ODOM_OK);
        assert_yaw_integrals(0.004,0.008,0.004,1); /* Recovery only seeds. */
        advance(4000,true);assert(odom_process()==ODOM_OK);
        assert_yaw_integrals(0.008,0.016,0.008,2);
    }
}
static void test_yaw_integrals_retain_stationary_residual(void){
    reset_case(true);wheel.velocity.wz=0.0005f;sample.gyro.z=0.002f;
    for(unsigned i=0;i<160;++i){advance(4000,true);assert(odom_process()==ODOM_OK);}
    assert(odom_get_state()->static_window);
    const Odom before=*odom_get_state();
    for(unsigned i=0;i<250;++i){advance(4000,true);assert(odom_process()==ODOM_OK);}
    const Odom *after=odom_get_state();
    assert(after->static_window);
    near(after->angle.z,before.angle.z,1e-8f);
    near(after->velocity.z,0,1e-8f);
    near_double(after->wheel_yaw_integral_rad-before.wheel_yaw_integral_rad,0.0005,1e-8);
    near_double(after->imu_yaw_integral_rad-before.imu_yaw_integral_rad,0.002,1e-8);
    near_double(after->yaw_integrated_time_s-before.yaw_integrated_time_s,1,1e-6);
    assert(after->yaw_integrated_samples-before.yaw_integrated_samples==250);
}
int main(void){
    test_boot_calibration_gate();
    test_stationary_feedback_and_motion_exit();
    test_imu_failure_invalidates_ready();
    test_duplicate_gyro_and_long_gap();
    test_wheel_sequence_fused_once();
    test_missing_wheel_stops_unbounded_prediction();
    test_yaw_integrals_signed_constant_speed();
    test_yaw_integrals_projection_scales_and_actual_dt();
    test_yaw_integrals_capture_short_pulse();
    test_yaw_integrals_never_bridge_invalid_endpoints();
    test_yaw_integrals_retain_stationary_residual();
    puts("Odometry service regression tests passed");
    return 0;
}
