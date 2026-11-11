#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "imu/imu.h"
#include "imu/imu_attitude.h"
#include "imu/bmi088.h"

static void near(float actual, float expected, float tolerance) {
    if(fabsf(actual - expected) > tolerance) {
        fprintf(stderr, "actual=%g expected=%g tolerance=%g\n", actual, expected, tolerance);
        assert(0);
    }
}

static ImuAttitudeConfig config(void) {
    ImuAttitudeConfig c = {0};
    c.mode = IMU_ATTITUDE_MAHONY_6AXIS;
    c.gyro_calib_samples = 16;
    c.acc_norm = 9.80665f;
    c.acc_norm_tolerance = 0.5f;
    c.max_acc_age_us = 20000;
    c.gyro_calib_var_threshold = 1e-4f;
    c.mahony_kp = 2.0f;
    c.gyro_x_temp_coeff = 1e-5f;
    c.gyro_y_temp_coeff = -2e-5f;
    c.gyro_z_temp_coeff = 3.8e-5f;
    c.gyro_z_bias_offset = 0.9f; /* Legacy intercept must not be subtracted again. */
    c.zru_gyro_threshold = 0.01f;
    c.zru_bias_gain = 0.2f;
    return c;
}

static ImuSample frame(ImuGyro gyro, float temperature, uint32_t timestamp) {
    ImuSample s = {0};
    s.gyro = gyro;
    s.acc.z = 9.80665f;
    s.temperature = temperature;
    s.gyro_timestamp_us = timestamp;
    s.acc_timestamp_us = timestamp - 10;
    s.temp_timestamp_us = timestamp;
    s.flags = IMU_SAMPLE_GYRO_NEW | IMU_SAMPLE_GYRO_VALID |
        IMU_SAMPLE_ACC_NEW | IMU_SAMPLE_ACC_VALID | IMU_SAMPLE_TEMP_VALID;
    return s;
}

static void calibrate(ImuAttitude *a, ImuGyro bias, float temperature, uint32_t *time) {
    imu_attitude_set_stationary(a, true);
    for(unsigned i=0; i<a->config.gyro_calib_samples; ++i) {
        *time += 4000;
        ImuSample s=frame(bias, temperature, *time);
        imu_attitude_update(a, &s);
    }
    assert(a->calibrated);
    imu_attitude_set_stationary(a, false);
}

static void test_temperature_and_boot_bias(void) {
    const ImuAttitudeConfig c = config();
    for(unsigned boot=0; boot<2; ++boot) {
        ImuAttitude a;
        uint32_t time=0;
        const ImuGyro bias={0.001f, -0.002f, boot ? 0.004f : 0.002f};
        assert(imu_attitude_init(&a,&c)==IMU_ATTITUDE_STATUS_OK);
        calibrate(&a,bias,25.0f,&time);
        ImuSample s=frame((ImuGyro){bias.x+10*c.gyro_x_temp_coeff,
            bias.y+10*c.gyro_y_temp_coeff,bias.z+10*c.gyro_z_temp_coeff},35.0f,time+4000);
        assert(imu_attitude_update(&a,&s)==IMU_ATTITUDE_STATUS_OK);
        near(a.gyro_filtered.x,0,1e-7f);
        near(a.gyro_filtered.y,0,1e-7f);
        near(a.gyro_filtered.z,0,1e-7f);
        near(a.gyro_bias_effective.z,bias.z+10*c.gyro_z_temp_coeff,1e-7f);
        near(a.gyro_temp_ref,25,1e-6f);
        /* A temporary missing temperature read cannot abruptly remove known compensation. */
        s.flags &= (uint8_t)~IMU_SAMPLE_TEMP_VALID;
        s.gyro_timestamp_us+=4000;s.acc_timestamp_us+=4000;
        imu_attitude_update(&a,&s);
        near(a.gyro_filtered.z,0,1e-7f);
    }
}

static void test_calibration_gate_and_motion(void) {
    ImuAttitude a;
    ImuAttitudeConfig c=config();
    uint32_t time=0;
    imu_attitude_init(&a,&c);
    for(unsigned i=0;i<20;++i) {
        time+=4000;ImuSample s=frame((ImuGyro){0,0,0.002f},25,time);
        imu_attitude_update(&a,&s);
    }
    assert(!a.calibrated && a.calib_count==0);
    imu_attitude_set_stationary(&a,true);
    for(unsigned i=0;i<8;++i) {
        time+=4000;ImuSample s=frame((ImuGyro){0,0,0.002f},25,time);
        imu_attitude_update(&a,&s);
    }
    assert(a.calib_count==8);
    imu_attitude_set_stationary(&a,false);
    assert(a.calib_count==0);
    calibrate(&a,(ImuGyro){0,0,0.002f},25,&time);
    /* Ten seconds of actual rotation must remain rotation, even below residual threshold. */
    for(unsigned i=0;i<2500;++i) {
        time+=4000;ImuSample s=frame((ImuGyro){0,0,0.006f},25,time);
        imu_attitude_update(&a,&s);
    }
    near(a.gyro_bias_residual.z,0,1e-8f);
    near(a.gyro_filtered.z,0.004f,1e-7f);
    imu_attitude_set_stationary(&a,true);
    for(unsigned i=0;i<2500;++i) {
        time+=4000;ImuSample s=frame((ImuGyro){0,0,0.006f},25,time);
        imu_attitude_update(&a,&s);
    }
    assert(a.zru_active);
    near(a.gyro_filtered.z,0.004f*expf(-2.0f),2e-6f);
    const float residual=a.gyro_bias_residual.z;
    imu_attitude_set_stationary(&a,false);
    time+=4000;ImuSample s=frame((ImuGyro){0,0,0.010f},25,time);
    imu_attitude_update(&a,&s);
    assert(!a.zru_active);
    near(a.gyro_bias_residual.z,residual,1e-8f);
}

static void test_corrected_gyro_is_measurement(void) {
    ImuAttitude a;
    ImuAttitudeConfig c=config();
    uint32_t time=0;
    imu_attitude_init(&a,&c);
    calibrate(&a,(ImuGyro){0,0,0},25,&time);
    ImuSample s=frame((ImuGyro){0,0,0},25,time+4000);
    s.acc.y=9.80665f*sinf(0.1f);s.acc.z=9.80665f*cosf(0.1f);
    imu_attitude_update(&a,&s);
    assert(a.angle.roll>0.0f); /* Mahony correction is actually active. */
    near(a.gyro_filtered.x,0,1e-8f);
    near(a.gyro_filtered.y,0,1e-8f);
    near(a.gyro_filtered.z,0,1e-8f);
}

/* Minimal SPI register model exercises complete-frame publication and error propagation. */
static uint8_t regs[2][256];
static int selected=-1;
static unsigned phase,reg,byte_count;
static bool reading;
static int fail_after=-1;
static uint32_t now_us=1000;
static void acc_low(void){selected=0;phase=0;}
static void gyro_low(void){selected=1;phase=0;}
static void cs_high(void){selected=-1;}
static bool spi_byte(uint8_t tx,uint8_t *rx){
    ++byte_count;
    if(fail_after==0)return false;
    if(fail_after>0)--fail_after;
    assert(selected>=0);
    *rx=0;
    if(phase++==0){reading=(tx&0x80)!=0;reg=tx&0x7f;return true;}
    if(reading){
        if(selected==0 && phase==2)return true;
        *rx=regs[selected][reg++];
    }else regs[selected][reg++]=tx;
    return true;
}
static uint32_t micros(void){return now_us++;}
static uint32_t millis(void){return now_us/1000;}
static void delay_ms(uint32_t ms){now_us+=ms*1000;}
static void delay_us(uint16_t us){now_us+=us;}
static void raw16(unsigned sensor,unsigned address,int16_t value){
    regs[sensor][address]=(uint8_t)value;regs[sensor][address+1]=(uint8_t)(value>>8);
}
static void test_spi_failure(void){
    memset(regs,0,sizeof(regs));regs[0][0]=0x1e;regs[1][0]=0x0f;
    raw16(0,0x16,10927);
    Bmi088PortOps ops={.accel_cs_low=acc_low,.accel_cs_high=cs_high,
        .gyro_cs_low=gyro_low,.gyro_cs_high=cs_high,.read_write_byte=spi_byte,
        .now_ms=millis,.now_us=micros,.delay_ms=delay_ms,.delay_us=delay_us};
    Bmi088Config c;
    bmi088_make_config(&c,&ops,0,0);
    imu_set_instance(&bmi088_blocking_instance);
    assert(imu_init(&c)==IMU_STATUS_OK);
    assert(imu_update()==IMU_STATUS_OK);
    ImuSample before;
    assert(imu_get_sample(&before)==IMU_STATUS_OK);
    assert((before.flags&(IMU_SAMPLE_GYRO_VALID|IMU_SAMPLE_ACC_VALID))==(IMU_SAMPLE_GYRO_VALID|IMU_SAMPLE_ACC_VALID));
    fail_after=3;byte_count=0;
    assert(imu_update()==IMU_STATUS_ERROR);
    assert(byte_count==4); /* Stop executing SPI transfers immediately after one error. */
    ImuSample failed;
    assert(imu_get_sample(&failed)==IMU_STATUS_NOT_READY);
    Bmi088AttitudeDebug debug;
    bmi088_get_attitude_debug(&debug);
    assert(debug.gyro_timestamp_us==before.gyro_timestamp_us);
    assert((debug.sample_flags&(IMU_SAMPLE_ACC_VALID|IMU_SAMPLE_GYRO_VALID))==0);
    assert(!debug.stationary && !debug.zru_active);
    fail_after=-1;now_us+=4000;
    assert(imu_update()==IMU_STATUS_OK);
    assert(imu_get_sample(&failed)==IMU_STATUS_OK);
    assert(failed.gyro_timestamp_us>before.gyro_timestamp_us);
    /* Failed initialization cannot report a usable sensor. */
    fail_after=0;
    assert(imu_init(&c)==IMU_STATUS_ERROR);
}

int main(void){
    test_temperature_and_boot_bias();
    test_calibration_gate_and_motion();
    test_corrected_gyro_is_measurement();
    test_spi_failure();
    puts("IMU regression tests passed");
    return 0;
}
