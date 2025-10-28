
#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include <math.h>
#include "webint.h"
#include "webint.c"

#define WIFI_SSID "s"
#define WIFI_PASS "d"

volatile float RatePitch, RateRoll, RateYaw;
float RateCalibrationPitch, RateCalibrationRoll, RateCalibrationYaw,AccXCalibration,AccYCalibration,AccZCalibration = 0;

float PAngleRoll=2; float PAnglePitch=2;
float IAngleRoll=0.5; float IAnglePitch=0.5;
float DAngleRoll=0.007; float DAnglePitch=0.07;

float PRateRoll = 0.1;
float IRateRoll = 0.1;
float DRateRoll = 0.001;

float PRatePitch = 0.1;
float IRatePitch = 0.1;
float DRatePitch = 0.01;

float PRateYaw = 4;
float IRateYaw = 3;
float DRateYaw = 0;

float t=0.004;      //time cycle for loops

#define IMU_ADDR 0x34
#define I2C_MASTER_NUM I2C_NUM_0


#define ESC_FREQ_HZ 400
#define ESC_TIMER LEDC_TIMER_0
#define ESC_SPEEDMODE LEDC_LOW_SPEED_MODE

#define MOTOR1_PIN 22
#define MOTOR2_PIN 12
#define MOTOR3_PIN 14
#define MOTOR4_PIN 27

// LEDC channel for each motor in array
static ledc_channel_t channels[4] = {
    LEDC_CHANNEL_0,
    LEDC_CHANNEL_1,
    LEDC_CHANNEL_2,
    LEDC_CHANNEL_3
};

volatile uint32_t current_time;
volatile uint32_t last_channel_1 = 0;
volatile uint32_t last_channel_2 = 0;
volatile uint32_t last_channel_3 = 0;
volatile uint32_t last_channel_4 = 0;
volatile uint32_t last_channel_5 = 0;
volatile uint32_t last_channel_6 = 0;
volatile uint32_t timer_1;
volatile uint32_t timer_2;
volatile uint32_t timer_3;
volatile uint32_t timer_4;
volatile uint32_t timer_5;
volatile uint32_t timer_6;
volatile int ReceiverValue[6];


volatile float PtermRoll;
volatile float ItermRoll;
volatile float DtermRoll;
volatile float PIDOutputRoll;
volatile float PtermPitch;
volatile float ItermPitch;
volatile float DtermPitch;
volatile float PIDOutputPitch;
volatile float PtermYaw;
volatile float ItermYaw;
volatile float DtermYaw;
volatile float PIDOutputYaw;
volatile float KalmanGainPitch;
volatile float KalmanGainRoll;

int ThrottleIdle = 1170;
int ThrottleCutOff = 1000;

volatile float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
volatile float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
volatile float InputRoll, InputThrottle, InputPitch, InputYaw;
volatile float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
volatile float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
volatile float PIDReturn[] = {0, 0, 0};

//Kalman filters for angle mode
volatile float AccX, AccY, AccZ;
volatile float AngleRoll, AnglePitch;
volatile float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2;
volatile float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2;
volatile float Kalman1DOutput[]={0,0};
volatile float DesiredAngleRoll, DesiredAnglePitch;
volatile float ErrorAngleRoll, ErrorAnglePitch;
volatile float PrevErrorAngleRoll, PrevErrorAnglePitch;
volatile float PrevItermAngleRoll, PrevItermAnglePitch;


float complementaryAngleRoll = 0.0f;
float complementaryAnglePitch = 0.0f;

volatile float MotorInput1, MotorInput2, MotorInput3, MotorInput4;

void imu_init(void)
{
    // Data registers
    #define REG_ACCEL_XOUT_H  0x1F
    #define REG_GYRO_XOUT_H   0x25

    // Config/power registers
    #define REG_PWR_MGMT0     0x4E
    #define REG_ACCEL_CONFIG0 0x4C
    #define REG_GYRO_CONFIG0  0x4D

    uint8_t buf[2];
    // Wake up: enable accel+gyro, set ODR to e.g. 1 kHz
    buf[0] = REG_PWR_MGMT0;
    buf[1] = 0x0F;  // ACCEL_LN + GYRO_LN
    i2c_master_write_to_device(I2C_MASTER_NUM, IMU_ADDR, buf, 2, pdMS_TO_TICKS(10));
    
    buf[0] = REG_ACCEL_CONFIG0;
    buf[1] = 0x48;  // check bits in datasheet
    i2c_master_write_to_device(I2C_MASTER_NUM, IMU_ADDR, buf, 2, pdMS_TO_TICKS(10));

    buf[0] = REG_GYRO_CONFIG0;
    buf[1] = 0x48;  // check bits
    i2c_master_write_to_device(I2C_MASTER_NUM, IMU_ADDR, buf, 2, pdMS_TO_TICKS(10));

}
void calibrate_IMU_vals(void){
    // Adjust readings to calibrated values
    RateRoll -= RateCalibrationRoll;
    RatePitch -= RateCalibrationPitch;
    RateYaw -= RateCalibrationYaw;

    AccX -= AccXCalibration ;
    AccY -= AccYCalibration ;
    AccZ -= AccZCalibration;
}
void imu_read(void){
    uint8_t data[6];
    int16_t ax, ay, az, gx, gy, gz;

    // Read accel
    i2c_master_write_read_device(I2C_MASTER_NUM, IMU_ADDR,
                                 (uint8_t[]){REG_ACCEL_XOUT_H}, 1,
                                 data, 6, pdMS_TO_TICKS(10));
    ax = (data[0] << 8) | data[1];
    ay = (data[2] << 8) | data[3];
    az = (data[4] << 8) | data[5];

    // Read gyro
    i2c_master_write_read_device(I2C_MASTER_NUM, IMU_ADDR,
                                 (uint8_t[]){REG_GYRO_XOUT_H}, 1,
                                 data, 6, pdMS_TO_TICKS(10));
    gx = (data[0] << 8) | data[1];
    gy = (data[2] << 8) | data[3];
    gz = (data[4] << 8) | data[5];

    // Convert to physical units
    float accel_scale = 1.0f / 8192.0f;  
    float gyro_scale  = 1.0f / 32.8f; 
    AccX = ax * accel_scale;
    AccY = ay * accel_scale;
    AccZ = az * accel_scale;
    RateRoll  = gx * gyro_scale;
    RatePitch = gy * gyro_scale;
    RateYaw   = gz * gyro_scale;

    calibrate_IMU_vals();

    AngleRoll  = atan2f(AccY, sqrtf(AccX*AccX + AccZ*AccZ)) * 57.29f;
    AnglePitch = -atan2f(AccX, sqrtf(AccY*AccY + AccZ*AccZ)) * 57.29f;

}

void MPU_6050_old(void)
{
    uint8_t data[6];

    // Set DLPF to 44Hz
    uint8_t dlpf[2] = {0x1A, 0x03}; 
    i2c_master_write_to_device(I2C_MASTER_NUM, IMU_ADDR, dlpf, 2, 100 / portTICK_PERIOD_MS);

    uint8_t accel_cfg[2] = {0x1C, 0x10};
    i2c_master_write_to_device(I2C_MASTER_NUM, IMU_ADDR, accel_cfg, 2, 100 / portTICK_PERIOD_MS);

    // Read accelerometer data
    uint8_t reg = 0x1F;
    i2c_master_write_read_device(I2C_MASTER_NUM, IMU_ADDR, &reg, 1, data, 6, 100 / portTICK_PERIOD_MS);
    int16_t AccXLSB = (data[0] << 8) | data[1];
    int16_t AccYLSB = (data[2] << 8) | data[3];
    int16_t AccZLSB = (data[4] << 8) | data[5];

    uint8_t gyro_cfg[2] = {0x1B, 0x08};
    i2c_master_write_to_device(I2C_MASTER_NUM, IMU_ADDR, gyro_cfg, 2, 100 / portTICK_PERIOD_MS);

    // Read gyroscope data 
    reg = 0x43;
    i2c_master_write_read_device(I2C_MASTER_NUM, IMU_ADDR, &reg, 1, data, 6, 100 / portTICK_PERIOD_MS);
    int16_t GyroX = (data[0] << 8) | data[1];
    int16_t GyroY = (data[2] << 8) | data[3];
    int16_t GyroZ = (data[4] << 8) | data[5];

    RateRoll = (float)GyroX / 65.5;
    RatePitch = (float)GyroY / 65.5;
    RateYaw = (float)GyroZ / 65.5;
    AccX = (float)AccXLSB / 4096;
    AccY = (float)AccYLSB / 4096;
    AccZ = (float)AccZLSB / 4096;

    calibrate_IMU_vals();

    AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 57.29;
    AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 57.29;

}

static esp_err_t mpu6050_read(uint8_t reg, uint8_t *data, size_t len) { 
    #define MPU6050_ADDR 0x68 
    return i2c_master_write_read_device(I2C_MASTER_NUM, MPU6050_ADDR, &reg, 1, data, len, 1000 / portTICK_PERIOD_MS); }

void MPU_6050(void){
    #define I2C_MASTER_SCL_IO 25 
    #define I2C_MASTER_SDA_IO 33
    #define I2C_MASTER_NUM I2C_NUM_0 
    #define I2C_MASTER_FREQ_HZ 400000 
    #define MPU6050_ADDR 0x68 
    #define MPU6050_PWR_MGMT_1 0x6B 
    #define MPU6050_ACCEL_XOUT_H 0x3B 
    #define RAD_TO_DEG 57.29577951308232f

    // Wake MPU6050 
    uint8_t data[2] = {MPU6050_PWR_MGMT_1, 0x00}; 
    i2c_master_write_to_device(I2C_MASTER_NUM, MPU6050_ADDR, data, 2, 1000 / portTICK_PERIOD_MS);
    uint8_t buf[14]; int16_t rawAcc[3], rawGyro[3];  
    if (mpu6050_read(MPU6050_ACCEL_XOUT_H, buf, 14) != ESP_OK) return; 
    rawAcc[0] = (buf[0] << 8) | buf[1]; rawAcc[1] = (buf[2] << 8) | buf[3]; rawAcc[2] = (buf[4] << 8) | buf[5]; rawGyro[0] = (buf[8] << 8) | buf[9]; rawGyro[1] = (buf[10] << 8) | buf[11]; rawGyro[2] = (buf[12] << 8) | buf[13]; 
    AccX = rawAcc[0] / 16384.0f; AccY = rawAcc[1] / 16384.0f; AccZ = rawAcc[2] / 16384.0f; RateRoll = rawGyro[0] / 131.0f; RatePitch = rawGyro[1] / 131.0f; RateYaw = rawGyro[2] / 131.0f; 
    calibrate_IMU_vals();
    AngleRoll = atan2f(AccY, AccZ) * RAD_TO_DEG; AnglePitch = atan2f(-AccX, sqrtf(AccY*AccY + AccZ*AccZ)) * RAD_TO_DEG;
}


uint32_t esc_duty_from_us(uint32_t pulse_us)
{
    const uint32_t period_us = 1000000 / ESC_FREQ_HZ; 
    return (pulse_us * ((1 << 16) - 1)) / period_us;
}

void report_speed(void) {
    esp_log_level_set("esc", ESP_LOG_INFO);  
    esp_log_level_set("imu", ESP_LOG_INFO);  
    esp_log_level_set("pic", ESP_LOG_INFO); 
    while(true) {
        ESP_LOGI("esc", "MOT1: %.2f MOT2: %.2f MOT3: %.2f MOT4: %.2f", MotorInput1, MotorInput2, MotorInput3, MotorInput4); 
        ESP_LOGI("imu", "RateRoll: %.2f RatePitch: %.2f RateYaw: %.2f AccX: %.2f AccY: %.2f AccZ: %.2f AngleRoll: %.2f AnglePitch: %.2f", RateRoll, RatePitch, RateYaw, AccX, AccY, AccZ, AngleRoll, AnglePitch); 
        ESP_LOGI("pid", "InputRoll: %.2f InputPitch: %.2f InputYaw: %.2f ", InputRoll, InputPitch, InputYaw); 
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


void calibrate_IMU(void) {
    float sumRateRoll = 0, sumRatePitch = 0, sumRateYaw = 0;
    float sumAccX = 0, sumAccY = 0, sumAccZ = 0;

    for (int i = 0; i < 1000; i++) {
        MPU_6050();  // reads into RateRoll, RatePitch, RateYaw, AccX, AccY, AccZ

        sumRateRoll  += RateRoll;
        sumRatePitch += RatePitch;
        sumRateYaw   += RateYaw;

        vTaskDelay(2 / portTICK_PERIOD_MS);
    }

    RateCalibrationRoll  = sumRateRoll  / 1000;
    RateCalibrationPitch = sumRatePitch / 1000;
    RateCalibrationYaw   = sumRateYaw   / 1000;
}

void wifi_init_softap(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = "esp32",
            .ssid_len = 0,
            .channel = 1,
            .password = "1234",
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_log_level_set("wifi", ESP_LOG_INFO);  
    ESP_LOGI("wifi", "AP started. SSID:%s PW:%s", wifi_config.ap.ssid, wifi_config.ap.password);
}

// Wi-Fi connect to existing network (BROKEN?)
void wifi_init_sta(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_log_level_set("wifi", ESP_LOG_INFO);  
    ESP_LOGI("wifi", "Connecting to %s...", WIFI_SSID);

}

void updateMotors(void) {
        ledc_set_duty(ESC_SPEEDMODE, channels[0], esc_duty_from_us(MotorInput1));
        ledc_set_duty(ESC_SPEEDMODE, channels[1], esc_duty_from_us(MotorInput2));
        ledc_set_duty(ESC_SPEEDMODE, channels[2], esc_duty_from_us(MotorInput3));
        ledc_set_duty(ESC_SPEEDMODE, channels[3], esc_duty_from_us(MotorInput4));
        ledc_update_duty(ESC_SPEEDMODE, channels[0]);
        ledc_update_duty(ESC_SPEEDMODE, channels[1]);
        ledc_update_duty(ESC_SPEEDMODE, channels[2]);
        ledc_update_duty(ESC_SPEEDMODE, channels[3]);
}

// Initial takeoff procedure
void controlled_takeoff(void)
{
    #define MIN_THROTTLE        1050   // ESC idle
    #define TAKEOFF_THROTTLE    1400   // Approx hover power, tune experimentally
    #define MAX_THROTTLE        1800
    #define TAKEOFF_RAMP_TIME   2000   // ms for throttle ramp
    #define LOOP_DELAY_MS       20     // loop interval
    printf("Starting controlled takeoff...\n");

    // Reset PID integrators and desired angles
    ItermRoll = 0;
    ItermPitch = 0;
    ItermYaw = 0;
    ReceiverValue[0] = 1500;   // Level
    ReceiverValue[1] = 1500;  // Level
    ReceiverValue[3] = 1500;

    int Throttle = MIN_THROTTLE;
    updateMotors();

    int steps = TAKEOFF_RAMP_TIME / LOOP_DELAY_MS;
    int throttleIncrement = (TAKEOFF_THROTTLE - MIN_THROTTLE) / steps;

    for (int i = 0; i < steps; i++) {
        Throttle += throttleIncrement;

        if (Throttle > TAKEOFF_THROTTLE)
            Throttle = TAKEOFF_THROTTLE;

  
        updateMotors();

        vTaskDelay(pdMS_TO_TICKS(LOOP_DELAY_MS));
    }

    printf("Reached hover throttle at %d\n", Throttle);

    for (int i = 0; i < 100; i++) { 
        ReceiverValue[0] = 1500;
        ReceiverValue[1] = 1500;
        ReceiverValue[3] = 1500;

        updateMotors();
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    printf("Takeoff complete, returning control to main loop\n");
}


void app_main(void)
{
    ESP_LOGI("flight", "Connecting Wi-Fi");
    wifi_init_softap();
    ESP_LOGI("flight", "Starting web server");
    start_webserver();
    esp_log_level_set("flight", ESP_LOG_INFO);  
    ESP_LOGI("flight", "Starting flight controller");
    int64_t loop_timer = esp_timer_get_time();
    // Init I2C stack
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GPIO_NUM_33,
        .scl_io_num = GPIO_NUM_26,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000
    };
    i2c_param_config(I2C_NUM_0, &i2c_conf);
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
    // Set up ESCs
    // LEDC timer setup
    ledc_timer_config_t timer = {
        .speed_mode       = ESC_SPEEDMODE,
        .duty_resolution  = LEDC_TIMER_16_BIT,  
        .timer_num        = ESC_TIMER,
        .freq_hz          = ESC_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer);
    int pins[4] = {MOTOR1_PIN, MOTOR2_PIN, MOTOR3_PIN, MOTOR4_PIN}; /
    for (int i = 0; i < 4; i++) {
        ledc_channel_config_t ch = {
            .gpio_num   = pins[i],
            .speed_mode = ESC_SPEEDMODE,
            .channel    = channels[i],
            .intr_type  = LEDC_INTR_DISABLE,
            .timer_sel  = ESC_TIMER,
            .duty       = 0,
            .hpoint     = 0
        };
        ledc_channel_config(&ch);
    }

        ESP_LOGI("flight", "Arm ESC pulse");
    uint32_t duty = esc_duty_from_us(1000);
    for (int i = 0; i < 4; i++) {
        ledc_set_duty(ESC_SPEEDMODE, channels[i], duty);
        ledc_update_duty(ESC_SPEEDMODE, channels[i]);
    }
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Calibrate IMU while drone is on flat surface
    ESP_LOGI("flight", "Calibrating IMU");
    calibrate_IMU();
    ESP_LOGI("flight", "IMU calibration complete");
    xTaskCreate(
        report_speed, "ReportSpeed", 2048, NULL, 5, NULL
    );

    // Init roll, pitch, yaw
    ReceiverValue[0] = 1500;
    ReceiverValue[1] = 1500;
    ReceiverValue[3] = 1500;
    while(true){

        // Read from IMU
        MPU_6050();
        // PID 
        complementaryAngleRoll=0.991*(complementaryAngleRoll+RateRoll*t) + 0.009*AngleRoll;
        complementaryAnglePitch=0.991*(complementaryAnglePitch+RatePitch*t) + 0.009*AnglePitch;

        complementaryAngleRoll = (complementaryAngleRoll > 20) ? 20 : ((complementaryAngleRoll < -20) ? -20 : complementaryAngleRoll);
        complementaryAnglePitch = (complementaryAnglePitch > 20) ? 20 : ((complementaryAnglePitch < -20) ? -20 : complementaryAnglePitch);

        DesiredAngleRoll=0.1*(ReceiverValue[0]-1500);
        DesiredAnglePitch=0.1*(ReceiverValue[1]-1500);
        InputThrottle=ReceiverValue[2];
        DesiredRateYaw=0.15*(ReceiverValue[3]-1500);


        // Inlined PID equation for Roll
        ErrorAngleRoll = DesiredAngleRoll - complementaryAngleRoll;
        PtermRoll = PAngleRoll * ErrorAngleRoll;
        ItermRoll = PrevItermAngleRoll + (IAngleRoll * (ErrorAngleRoll + PrevErrorAngleRoll) * (t / 2));
        ItermRoll = (ItermRoll > 400) ? 400 : ((ItermRoll < -400) ? -400 : ItermRoll);
        DtermRoll = DAngleRoll * ((ErrorAngleRoll - PrevErrorAngleRoll) / t);
        PIDOutputRoll = PtermRoll + ItermRoll + DtermRoll;
        PIDOutputRoll = (PIDOutputRoll > 400) ? 400 : ((PIDOutputRoll < -400) ? -400 : PIDOutputRoll);
        DesiredRateRoll = PIDOutputRoll;
        PrevErrorAngleRoll = ErrorAngleRoll;
        PrevItermAngleRoll = ItermRoll;

        ErrorAnglePitch = DesiredAnglePitch - complementaryAnglePitch;
        PtermPitch = PAnglePitch * ErrorAnglePitch;
        ItermPitch = PrevItermAnglePitch + (IAnglePitch * (ErrorAnglePitch + PrevErrorAnglePitch) * (t / 2));
        ItermPitch = (ItermPitch > 400) ? 400 : ((ItermPitch < -400) ? -400 : ItermPitch);
        DtermPitch = DAnglePitch * ((ErrorAnglePitch - PrevErrorAnglePitch) / t);
        PIDOutputPitch = PtermPitch + ItermPitch + DtermPitch;
        PIDOutputPitch = (PIDOutputPitch > 400) ? 400 : ((PIDOutputPitch < -400) ? -400 : PIDOutputPitch);
        DesiredRatePitch = PIDOutputPitch;
        PrevErrorAnglePitch = ErrorAnglePitch;
        PrevItermAnglePitch = ItermPitch;

        // Compute errors
        ErrorRateRoll = DesiredRateRoll - RateRoll;
        ErrorRatePitch = DesiredRatePitch - RatePitch;
        ErrorRateYaw = DesiredRateYaw - RateYaw;

        // Roll Axis PID
        PtermRoll = PRateRoll * ErrorRateRoll;
        ItermRoll = PrevItermRateRoll + (IRateRoll * (ErrorRateRoll + PrevErrorRateRoll) * (t / 2));
        ItermRoll = (ItermRoll > 400) ? 400 : ((ItermRoll < -400) ? -400 : ItermRoll);
        DtermRoll = DRateRoll * ((ErrorRateRoll - PrevErrorRateRoll) / t);
        PIDOutputRoll = PtermRoll + ItermRoll + DtermRoll;
        PIDOutputRoll = (PIDOutputRoll > 400) ? 400 : ((PIDOutputRoll < -400) ? -400 : PIDOutputRoll);

        // Update output and previous values for Roll
        InputRoll = PIDOutputRoll;
        PrevErrorRateRoll = ErrorRateRoll;
        PrevItermRateRoll = ItermRoll;

        // Pitch Axis PID
        PtermPitch = PRatePitch * ErrorRatePitch;
        ItermPitch = PrevItermRatePitch + (IRatePitch * (ErrorRatePitch + PrevErrorRatePitch) * (t / 2));
        ItermPitch = (ItermPitch > 400) ? 400 : ((ItermPitch < -400) ? -400 : ItermPitch);
        DtermPitch = DRatePitch * ((ErrorRatePitch - PrevErrorRatePitch) / t);
        PIDOutputPitch = PtermPitch + ItermPitch + DtermPitch;
        PIDOutputPitch = (PIDOutputPitch > 400) ? 400 : ((PIDOutputPitch < -400) ? -400 : PIDOutputPitch);

        // Update output and previous values for Pitch
        InputPitch = PIDOutputPitch;
        PrevErrorRatePitch = ErrorRatePitch;
            PrevItermRatePitch = ItermPitch;

        // Yaw Axis PID
        PtermYaw = PRateYaw * ErrorRateYaw;
        ItermYaw = PrevItermRateYaw + (IRateYaw * (ErrorRateYaw + PrevErrorRateYaw) * (t / 2));
        ItermYaw = (ItermYaw > 400) ? 400 : ((ItermYaw < -400) ? -400 : ItermYaw);  
        DtermYaw = DRateYaw * ((ErrorRateYaw - PrevErrorRateYaw) / t);
        PIDOutputYaw = PtermYaw + ItermYaw + DtermYaw;
        PIDOutputYaw = (PIDOutputYaw > 400) ? 400 : ((PIDOutputYaw < -400) ? -400 : PIDOutputYaw); 


        // Update output and previous values for Yaw
        InputYaw = PIDOutputYaw;
        PrevErrorRateYaw = ErrorRateYaw;
        PrevItermRateYaw = ItermYaw;


        if (InputThrottle > 1800)
        {
            InputThrottle = 1800;
        }

        // End ------------------------------------------------------------------------------------------------------------------------------
        
        // Calculate motor speeds
        MotorInput1 =  (InputThrottle - InputRoll - InputPitch - InputYaw); // front right - counter clockwise
        MotorInput2 =  (InputThrottle - InputRoll + InputPitch + InputYaw); // rear right - clockwise
        MotorInput3 =  (InputThrottle + InputRoll + InputPitch - InputYaw); // rear left  - counter clockwise
        MotorInput4 =  (InputThrottle + InputRoll - InputPitch + InputYaw); //front left - clockwise

        // Clamp motor speeds
        if (MotorInput1 > 2000)
        {
            MotorInput1 = 1999;
        }

        if (MotorInput2 > 2000)
        {
            MotorInput2 = 1999;
        }

        if (MotorInput3 > 2000)
        {
            MotorInput3 = 1999;
        }

        if (MotorInput4 > 2000)
        {
            MotorInput4 = 1999;
        }

        if (MotorInput1 < ThrottleIdle)
        {
            MotorInput1 = ThrottleIdle;
        }
        if (MotorInput2 < ThrottleIdle)
        {
            MotorInput2 = ThrottleIdle;
        }
        if (MotorInput3 < ThrottleIdle)
        {
            MotorInput3 = ThrottleIdle;
        }
        if (MotorInput4 < ThrottleIdle)
        {
            MotorInput4 = ThrottleIdle;
        }

        if (ReceiverValue[2] < 1030 ) // dont Arm the motors
        {
        MotorInput1 = ThrottleCutOff;
        MotorInput2 = ThrottleCutOff;
        MotorInput3 = ThrottleCutOff;
        MotorInput4 = ThrottleCutOff;

        PrevErrorRateRoll=0; PrevErrorRatePitch=0; PrevErrorRateYaw=0;
        PrevItermRateRoll=0; PrevItermRatePitch=0; PrevItermRateYaw=0;
        PrevErrorAngleRoll=0; PrevErrorAnglePitch=0;    
        PrevItermAngleRoll=0; PrevItermAnglePitch=0;

        }

        updateMotors();
        vTaskDelay(1);
        while ((esp_timer_get_time() - loop_timer) < (t*1000000)) {
        ; // wait
        }
        loop_timer = esp_timer_get_time();
    }
}
