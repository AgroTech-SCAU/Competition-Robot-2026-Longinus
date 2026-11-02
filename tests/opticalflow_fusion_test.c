/* Host integration tests: compile with the real parser, service, filter and matrix.c. */
#include "opticalflow.h"
#include "opticalflow/upixels_flow.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(condition) do { if(!(condition)) { printf("FAIL line %d: %s\n", __LINE__, #condition); exit(1); } } while(0)
#define CLOSE(a,b,e) CHECK(fabsf((a)-(b)) <= (e))

static uint32_t now_ms;
static uint8_t* receive_target;
static bool receive_ok = true;
static unsigned abort_count;
static unsigned cases;

static bool receive(void* context, uint8_t* data, uint16_t length) {
    (void)context;
    CHECK(length == 1u);
    receive_target = data;
    return receive_ok;
}
static bool abort_receive(void* context) { (void)context; ++abort_count; return true; }
static uint32_t clock_ms(void* context) { (void)context; return now_ms; }
static uint32_t enter(void* context) { (void)context; return 0u; }
static void leave(void* context, uint32_t state) { (void)context; (void)state; }
static const OpticalflowPortOps ops = {receive, abort_receive, clock_ms, enter, leave};

static OpticalflowConfig init_service(void) {
    OpticalflowConfig config = opticalflow_default_config();
    config.ops = &ops;
    now_ms = 10u;
    receive_ok = true;
    abort_count = 0u;
    CHECK(opticalflow_init(&config));
    return config;
}
static void feed(const uint8_t* data, unsigned length) {
    for(unsigned i = 0u; i < length; ++i) {
        *receive_target = data[i];
        opticalflow_on_rx_complete();
    }
}
static void put16(uint8_t* p, uint16_t value) { p[0] = (uint8_t)value; p[1] = (uint8_t)(value >> 8); }
static void put32(uint8_t* p, uint32_t value) { for(unsigned i=0;i<4;++i) p[i]=(uint8_t)(value>>(8*i)); }
static void put_float(uint8_t* p, float value) { memcpy(p, &value, 4u); }
static void upixels(uint8_t* frame, int16_t x, int16_t y, uint16_t height, uint8_t quality, uint16_t dt) {
    memset(frame, 0, 14u);
    frame[0] = 0xfe; frame[1] = 0x0a;
    put16(frame+2, (uint16_t)x); put16(frame+4, (uint16_t)y);
    put16(frame+6, dt); put16(frame+8, height); frame[10] = quality;
    for(unsigned i=2;i<12;++i) frame[12] ^= frame[i];
    frame[13] = 0x55;
}
static void send_upixels(int16_t x, int16_t y, uint16_t height, uint8_t quality, uint16_t dt) {
    uint8_t frame[14]; upixels(frame,x,y,height,quality,dt); feed(frame,14); opticalflow_process();
}
static uint16_t crc_byte(uint16_t crc, uint8_t data) {
    uint8_t t=(uint8_t)(data^(uint8_t)crc); t^=(uint8_t)(t<<4);
    return (uint16_t)((crc>>8)^((uint16_t)t<<8)^((uint16_t)t<<3)^(t>>4));
}
static void finish_mavlink(uint8_t* frame, unsigned payload, uint8_t extra) {
    uint16_t crc=0xffff;
    for(unsigned i=1;i<payload+6;++i) crc=crc_byte(crc,frame[i]);
    put16(frame+payload+6,crc_byte(crc,extra));
}
static void send_px4_flow(float x, float y, float distance) {
    uint8_t frame[52] = {0xfe,0x2c,0,0,0x9e,0x6a};
    put32(frame+6+8,20000u); put_float(frame+6+12,x); put_float(frame+6+16,y);
    put_float(frame+6+36,distance); frame[6+43]=0xf5; finish_mavlink(frame,44,138);
    feed(frame,52); opticalflow_process();
}
static void send_px4_distance(uint16_t cm) {
    uint8_t frame[22] = {0xfe,0x0e,0,0,0x9e,0x84};
    put16(frame+6+4,2); put16(frame+6+6,400); put16(frame+6+8,cm);
    finish_mavlink(frame,14,85); feed(frame,22); opticalflow_process();
}

static void test_parser_service(void) {
    ChassisImuOdomFlow sample;
    uint8_t frame[14];
    init_service();
    send_upixels(200,-100,500,0xf5,20000);
    sample=opticalflow_take_sample(0,0);
    CHECK(sample.valid && sample.updated); CLOSE(sample.vx,.5f,1e-6f); CLOSE(sample.vy,-.25f,1e-6f);
    sample=opticalflow_take_sample(0,0); CHECK(sample.valid && !sample.updated); ++cases;

    init_service(); (void)opticalflow_take_sample(.1f,.05f);
    send_upixels(200,-100,500,0xf5,20000); sample=opticalflow_take_sample(.2f,.15f);
    CLOSE(sample.vx,cosf(.1f)*.5f+sinf(.1f)*sinf(.2f)*(-.25f),1e-6f);
    CLOSE(sample.vy,-sinf(.1f)*sinf(.1f)*.5f+
          (cosf(.1f)*cosf(.2f)+sinf(.1f)*cosf(.1f)*sinf(.2f))*(-.25f),1e-6f); ++cases;

    send_upixels(200,0,500,0,20000); CHECK(!opticalflow_take_sample(0,0).valid); ++cases;
    send_upixels(200,0,500,0xf5,0); CHECK(!opticalflow_take_sample(0,0).valid); ++cases;
    send_upixels(3000,0,500,0xf5,20000); CHECK(!opticalflow_take_sample(0,0).valid); ++cases;
    send_upixels(200,0,500,0xf5,20000); CHECK(!opticalflow_take_sample(.4f,0).valid); ++cases;

    init_service(); send_upixels(200,0,500,0xf5,20000); now_ms+=101;
    send_upixels(200,0,0,0xf5,20000); CHECK(!opticalflow_take_sample(0,0).valid); ++cases;
    init_service(); send_upixels(200,0,0,0xf5,20000); CHECK(!opticalflow_take_sample(0,0).valid); ++cases;
    init_service(); send_upixels(200,0,500,0xf5,20000); now_ms+=101; opticalflow_process();
    CHECK(!opticalflow_take_sample(0,0).valid); ++cases;

    init_service(); upixels(frame,200,0,500,0xf5,20000); feed(frame,14); now_ms+=101; opticalflow_process();
    CHECK(!opticalflow_take_sample(0,0).valid); ++cases;
    init_service(); upixels(frame,200,0,500,0xf5,20000); frame[12]^=1; feed(frame,14);
    send_upixels(200,0,500,0xf5,20000); CHECK(opticalflow_take_sample(0,0).valid);
    CHECK(opticalflow_get_state()->crc_errors > 0); ++cases;

    init_service(); now_ms=UINT32_MAX-20u; send_upixels(200,0,500,0xf5,20000);
    now_ms+=40; send_upixels(200,0,0,0xf5,20000); CHECK(opticalflow_take_sample(0,0).valid); ++cases;

    OpticalflowConfig config=init_service(); config.sensor_yaw_rad=1.57079632679f; config.flow_sign=-1;
    CHECK(opticalflow_init(&config)); send_upixels(200,0,500,0xf5,20000);
    sample=opticalflow_take_sample(0,0); CLOSE(sample.vx,0,1e-6f); CLOSE(sample.vy,-.5f,1e-6f); ++cases;

    init_service(); send_upixels(200,0,500,0xf5,20000); opticalflow_on_rx_error(); opticalflow_process();
    CHECK(abort_count==1 && !opticalflow_take_sample(0,0).valid);
    send_upixels(200,0,500,0xf5,20000); CHECK(opticalflow_take_sample(0,0).valid); ++cases;
    init_service(); receive_ok=false; *receive_target=0; opticalflow_on_rx_complete(); opticalflow_process();
    CHECK(!opticalflow_take_sample(0,0).valid); receive_ok=true; opticalflow_process();
    send_upixels(200,0,500,0xf5,20000); CHECK(opticalflow_take_sample(0,0).valid); ++cases;
    init_service(); for(unsigned i=0;i<300;i++) { *receive_target=0xfe; opticalflow_on_rx_complete(); }
    opticalflow_process(); CHECK(opticalflow_get_state()->queue_overflows>0);
    CHECK(!opticalflow_take_sample(0,0).valid);
    send_upixels(200,0,500,0xf5,20000); CHECK(opticalflow_take_sample(0,0).valid); ++cases;

    init_service(); send_px4_flow(.02f,0,-1); CHECK(!opticalflow_take_sample(0,0).valid);
    send_px4_distance(50); send_px4_flow(.02f,0,-1); sample=opticalflow_take_sample(0,0);
    CHECK(sample.valid && sample.updated); CLOSE(sample.vx,.5f,1e-6f); ++cases;
    now_ms+=101; send_px4_distance(0xffff); send_px4_flow(.02f,0,-1);
    CHECK(!opticalflow_take_sample(0,0).valid); ++cases;
    init_service(); send_px4_flow(NAN,0,.5f); CHECK(!opticalflow_take_sample(0,0).valid); ++cases;
    init_service(); send_px4_flow(INFINITY,0,.5f); CHECK(!opticalflow_take_sample(0,0).valid); ++cases;

    /* Extended 20-byte frame, including the compatibility length byte. */
    for(unsigned mode=0;mode<2;mode++) {
        uint8_t ext[20]={0}; init_service(); upixels(frame,200,0,500,0xf5,20000);
        memcpy(ext,frame,12); ext[1]=mode ? 0x0a : 0x10; ext[12]=0x31;
        for(unsigned i=2;i<18;i++) ext[18]^=ext[i]; ext[19]=0x55;
        feed(ext,20); opticalflow_process(); CHECK(opticalflow_take_sample(0,0).valid); ++cases;
    }
}

static ChassisImuOdomImu resting_imu(void) {
    ChassisImuOdomImu imu={{0,0,9.80665f},{0,0,0}}; return imu;
}
static void test_fusion(void) {
    ChassisImuOdom fusion;
    ChassisImuOdomImu imu=resting_imu();
    ChassisImuOdomChassis wheel={1,0,0};
    ChassisImuOdomFlow flow={.vx=.5f,.valid=true};
    CHECK(chassis_imu_odom.init(&fusion,NULL)==chassis_imu_odom.OK);
    for(unsigned i=0;i<1000;i++) {
        flow.updated=i%5==0;
        CHECK(chassis_imu_odom.update_with_flow(&fusion,wheel,imu,&flow,.004f)==chassis_imu_odom.OK);
    }
    CHECK(fusion.flow_updates==200); CHECK(fusion.output.velocity.vx < .8f && fusion.output.velocity.vx > .45f);
    printf("wheel=1, flow=0.5: fused vx=%.6f, x(4s)=%.6f\n",fusion.output.velocity.vx,fusion.output.odom.x); ++cases;

    wheel.vx=0; CHECK(chassis_imu_odom.init(&fusion,NULL)==chassis_imu_odom.OK);
    for(unsigned i=0;i<250;i++) {
        flow.updated=i%5==0; CHECK(chassis_imu_odom.update_with_flow(&fusion,wheel,imu,&flow,.004f)==chassis_imu_odom.OK);
    }
    CHECK(fusion.output.odom.x > .1f && fusion.output.velocity.vx > .1f); ++cases;
    flow.vx=5; flow.updated=true;
    CHECK(chassis_imu_odom.update_with_flow(&fusion,wheel,imu,&flow,.004f)==chassis_imu_odom.OK);
    CHECK(fusion.flow_rejects==1); ++cases;

    CHECK(chassis_imu_odom.init(&fusion,NULL)==chassis_imu_odom.OK);
    flow=(ChassisImuOdomFlow){.vx=.01f,.vy=-.005f,.valid=true};
    for(unsigned i=0;i<1000;i++) {
        flow.updated=i%5==0;
        CHECK(chassis_imu_odom.update_with_flow(&fusion,wheel,imu,&flow,.004f)==chassis_imu_odom.OK);
    }
    CLOSE(fusion.output.odom.x,0,1e-6f); CLOSE(fusion.output.odom.y,0,1e-6f); ++cases;

    /* Pure rotation with an offset sensor must not turn into origin translation. */
    CHECK(chassis_imu_odom.init(&fusion,NULL)==chassis_imu_odom.OK);
    wheel.wz=imu.gyro.z=1;
    flow=(ChassisImuOdomFlow){.vx=-.1f,.vy=.2f,.sensor_x_m=.2f,.sensor_y_m=.1f,.valid=true};
    for(unsigned i=0;i<1000;i++) {
        flow.updated=i%5==0; CHECK(chassis_imu_odom.update_with_flow(&fusion,wheel,imu,&flow,.004f)==chassis_imu_odom.OK);
    }
    CLOSE(fusion.output.odom.x,0,.002f); CLOSE(fusion.output.odom.y,0,.002f); CLOSE(fusion.output.velocity.wz,1,.002f); ++cases;

    /* 50,000 cycles with independent 50Hz flow and a 250Hz filter. */
    CHECK(chassis_imu_odom.init(&fusion,NULL)==chassis_imu_odom.OK);
    wheel=(ChassisImuOdomChassis){.5f,.1f,.2f}; imu.gyro.z=.2f;
    flow=(ChassisImuOdomFlow){.vx=.5f,.vy=.1f,.valid=true};
    for(unsigned i=0;i<50000;i++) {
        flow.updated=i%5==0; CHECK(chassis_imu_odom.update_with_flow(&fusion,wheel,imu,&flow,.004f)==chassis_imu_odom.OK);
        for(unsigned j=0;j<9;j++) CHECK(isfinite(fusion.filter.x_data[j]));
        for(unsigned j=0;j<81;j++) CHECK(isfinite(fusion.filter.P_data[j]));
    }
    CHECK(fusion.flow_updates==10000);
    for(unsigned i=0;i<9;i++) {
        CHECK(fusion.filter.P_data[i*9+i]>=0);
        for(unsigned j=0;j<9;j++) CLOSE(fusion.filter.P_data[i*9+j],fusion.filter.P_data[j*9+i],1e-4f);
    }
    ++cases;
}

/* The legacy adapter is compiled separately from the unmodified HEAD source. */
void opticalflow_test_legacy(float* state, float* covariance);
static void test_legacy_equivalence(void) {
    float state[9], covariance[81]; ChassisImuOdom fusion;
    opticalflow_test_legacy(state,covariance);
    CHECK(chassis_imu_odom.init(&fusion,NULL)==chassis_imu_odom.OK);
    for(unsigned i=0;i<2500;i++) {
        ChassisImuOdomChassis wheel={.3f*sinf(i*.01f),.1f*cosf(i*.02f),.2f};
        ChassisImuOdomImu imu=resting_imu(); imu.gyro.z=.21f;
        if(i>=2000) { wheel=(ChassisImuOdomChassis){0}; imu.gyro.z=0; }
        CHECK(chassis_imu_odom.update(&fusion,wheel,imu,.004f)==chassis_imu_odom.OK);
    }
    for(unsigned i=0;i<9;i++) CLOSE(fusion.filter.x_data[i],state[i],1e-6f);
    for(unsigned i=0;i<81;i++) CLOSE(fusion.filter.P_data[i],covariance[i],1e-6f);
    ++cases;
}

int main(void) {
    test_parser_service(); test_fusion(); test_legacy_equivalence();
    printf("PASS: %u integration scenarios; 50,000-cycle run; legacy comparison\n", cases);
    return 0;
}
