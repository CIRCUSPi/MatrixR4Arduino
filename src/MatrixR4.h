/**
 * @file MatrixR4.h
 * @author Zack Huang (zackhuang0513@gmail.com)
 * @brief MatrixR4 library for Arduino
 * @version 1.0.0
 * @date 2023-11-14
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef MATRIX_R4_H
#define MATRIX_R4_H

#include <Arduino.h>
#include <SoftwareSerial.h>

#define MatrixR4_COMM_LEAD     0x7B
#define MatrixR4_COMM_BAUDRATE 57600

#define MatrixR4_SERVO_NUM    4
#define MatrixR4_DC_MOTOR_NUM 4
#define MatrixR4_ENCODER_NUM  4
#define MatrixR4_BUTTON_NUM   2

class MatrixR4
{
public:
    MatrixR4(SoftwareSerial* uart);

    enum class COMM_STATE
    {
        WAIT_LEAD,
        WAIT_NOT_LEAD,
        WAIT_CMD,
        ERROR,
    };

    enum class COMM_CMD
    {
        NONE = 0x00,
        // Setting-Init
        SET_DC_MOTOR_DIR = 0x01,
        SET_ENCODER_DIR,
        SET_SERVO_DIR,
        SET_DC_MOTOR_SPEED_RANGE,
        SET_SERVO_PULSE_RANGE,
        SET_SERVO_ANGLE_RANGE,
        SET_BUTTON_INIT,
        SET_ENCODER_ECHO_MODE,
        SET_IMU_ECHO_MODE,
        SET_IMU_INIT,
        SET_POWER_PARAM,

        // Setting-Commonly used
        SET_DC_MOTOR_SPEED = 0x11,
        SET_ALL_DC_MOTOR_SPEED,
        SET_SERVO_ANGLE,
        SET_ALL_SERVO_ANGLE,
        SET_MOVE_DISTANCE,
        SET_ENCODER_RESET_COUNTER,

        // Getting
        GET_BUTTON_STATE = 0x21,
        GET_BUTTONS_STATE,
        GET_ENCODER_COUNTER,
        GET_ALL_ENCODER_COUNTER,
        GET_IMU_EULER,
        GET_IMU_GYRO,
        GET_IMU_ACC,
        GET_POWER_INFO,

        // Auto-Send
        AUTO_SEND_BUTTON_STATE = 0x31,
        AUTO_SEND_ENCODER_COUNTER,
        AUTO_SEND_IMU_EULER,
        AUTO_SEND_IMU_GYRO,
        AUTO_SEND_IMU_ACC,

        // Other-Info
        ECHO_TEST        = 0xFF,
        F_VERSION        = 0xFE,
        F_BUILD_DAY      = 0xFD,
        F_DESCRIPTOR     = 0xFC,
        READ_MODEL_INDEX = 0xFB,
        READ_ALL_INFO    = 0xFA,
        RUN_AUTO_QC      = 0xF9,
    };

    enum class BTN_STATE
    {
        NOKEY,
        F_EDGE,
        REPEAT,
        PRESSED,
        R_EDGE,
    };

    enum class DIR
    {
        REVERSE,
        FORWARD,
    };

    enum class BUTTON_ECHO_MODE
    {
        PASSIVE,
        ACTIVE,
        MAX,
    };

    enum class ENCODER_ECHO_MODE
    {
        PASSIVE,
        ACTIVE,
        MAX,
    };

    enum class IMU_ECHO_MODE
    {
        PASSIVE,
        TIMING,
        ACTIVE,
        MAX,
    };

    enum class IMU_ACC_FSR
    {
        _2G,
        _4G,
        _8G,
        _16G,
    };

    enum class IMU_GYRO_FSR
    {
        _250DPS,
        _500DPS,
        _1000DPS,
        _2000DPS,
    };

    enum class IMU_ODR
    {
        _10_SPS,
        _20_SPS,
        _25_SPS,
        _50_SPS,
        _100_SPS,
        _125_SPS,
        _250_SPS,
        _500_SPS,
        _1000_SPS,
        _2000_SPS,
        _4000_SPS,
        _8000_SPS,
    };

    enum class IMU_FIFO
    {
        ENABLE,
        DISABLE,
    };

    enum class MOVE_TYPE
    {
        // 兩輪差速
        DIFF,
        // 四輪全向
        OMNI,
    };

    enum class MOVE_ACTION
    {
        STOP,
        FORWARD,
        BACKWARD,
        LEFT,
        RIGHT,
    };

    enum class RESULT
    {
        OK,
        ERROR,
        ERROR_SOFTSERIAL_BEGIN,
        ERROR_INIT,
        ERROR_WAIT_TIMEOUT,
        ERROR_READ_TIMEOUT,

        // SERVO PULSE RANGE
        ERROR_SERVO_MIN_PULSE,
        ERROR_SERVO_MAX_PULSE,

        // SERVO ANGLE RANGE
        ERROR_SERVO_MIN_ANGLE,
        ERROR_SERVO_MAX_ANGLE,

        ERROR_MODE,
        ERROR_INTERVAL,

        // IMU Init
        ERROR_IMU_ACC_FSR,
        ERROR_IMU_GYRO_FSR,
        ERROR_IMU_ODR,

        // Motor Speed
        ERROR_MOTOR_SPEED,
        ERROR_MOTOR1_SPEED,
        ERROR_MOTOR2_SPEED,
        ERROR_MOTOR3_SPEED,
        ERROR_MOTOR4_SPEED,

        // Servo Angle
        ERROR_SERVO_ANGLE,
        ERROR_SERVO1_ANGLE,
        ERROR_SERVO2_ANGLE,
        ERROR_SERVO3_ANGLE,
        ERROR_SERVO4_ANGLE,

        // Move Distance
        ERROR_MOVE_ACTION,
        ERROR_MOVE_SPEED,
        ERROR_MOVE_ENCODER,

        // RunAutoQC
        ERROR_MOTOR,
        ERROR_OLED,
        ERROR_I2C_MUX,
        ERROR_IMU,

        ERROR_POWER_VOLT_RANGE,
    };

    typedef struct
    {
        DIR      m1_dir;
        DIR      m2_dir;
        DIR      m3_dir;
        DIR      m4_dir;
        uint16_t m1_speed;
        uint16_t m2_speed;
        uint16_t m3_speed;
        uint16_t m4_speed;
    } Motors_Param_t;

    typedef struct
    {
        String  fwVersion;
        String  fwBuildDay;
        uint8_t modelIndex;
    } AllInfo_t;

    typedef void (*BtnChgCallback)(uint8_t num, BTN_STATE newState);

    RESULT Init();
    // Application API
    // Setting-Init
    RESULT SetDCMotorDir(uint8_t num, DIR dir);
    RESULT SetEncoderDir(uint8_t num, DIR dir);
    RESULT SetServoDir(uint8_t num, DIR dir);
    RESULT SetDCMotorSpeedRange(uint8_t num, uint16_t min, uint16_t max);
    RESULT SetServoPulseRange(uint8_t num, uint16_t min, uint16_t max);
    RESULT SetServoAngleRange(uint8_t num, uint16_t min, uint16_t max);
    RESULT SetButtonInit(
        uint8_t num, BUTTON_ECHO_MODE mode, uint16_t repeatStartMs, uint16_t repeatWorkMs);
    RESULT SetEncoderEchoMode(ENCODER_ECHO_MODE mode, uint16_t echoIntervalMs);
    RESULT SetIMUEchoMode(IMU_ECHO_MODE mode, uint16_t echoIntervalMs);
    RESULT SetIMUInit(IMU_ACC_FSR accFSR, IMU_GYRO_FSR gyroFSR, IMU_ODR odr, IMU_FIFO fifo);
    RESULT SetPowerParam(float fullVolt, float cutOffVolt, float alarmVolt);
    // Setting-Commonly used
    RESULT SetDCMotorSpeed(uint8_t num, uint16_t speed, DIR dir);
    RESULT SetAllDCMotorSpeed(Motors_Param_t param);
    RESULT SetServoAngle(uint8_t num, uint16_t angle);
    RESULT SetAllServoAngle(uint16_t angle1, uint16_t angle2, uint16_t angle3, uint16_t angle4);
    RESULT SetMoveDistance(MOVE_TYPE type, MOVE_ACTION action, uint16_t speed, uint16_t enCounter);
    RESULT SetEncoderResetCounter(uint8_t num);
    // Getting
    RESULT GetButtonState(uint8_t num, bool& btnState);
    RESULT GetButtonsState(bool* btnsState);
    RESULT GetEncoderCounter(uint8_t num, int16_t& enCounter);
    RESULT GetAllEncoderCounter(int16_t* enCounter);
    RESULT GetIMUEuler(double& roll, double& pitch, double& yaw);
    RESULT GetIMUGyro(double& x, double& y, double& z);
    RESULT GetIMUAcc(double& x, double& y, double& z);
    RESULT GetPowerInfo(float& curVolt, float& curVoltPerc);
    // Other-Info
    RESULT EchoTest(void);
    RESULT GetFWVersion(String& version);
    RESULT GetFWBuildDay(String& date);
    RESULT GetFWDescriptor(String& descriptor);
    RESULT GetModelIndex(uint8_t& index);
    RESULT GetAllInfo(AllInfo_t& info);
    RESULT RunAutoQC(void);

    void loop(void);
    void onBtnChg(BtnChgCallback callback);

    // TODO: 外部存取?
    // Encoders
    int16_t enCounter[MatrixR4_ENCODER_NUM];
    // IMU
    double imuGyroX, imuGyroY, imuGyroZ;
    double imuAccX, imuAccY, imuAccZ;

private:
    SoftwareSerial* commSerial;
    BtnChgCallback  callbackFunc;

    void CommSendData(COMM_CMD cmd, uint8_t* data = NULL, uint16_t size = 0);
    void CommSendData(COMM_CMD cmd, uint8_t data);
    bool CommReadData(uint8_t* data, uint16_t size = 1, uint32_t timeout_ms = 10);
    bool WaitData(COMM_CMD cmd = COMM_CMD::NONE, uint32_t timeout_ms = 0);
    void HandleCommand(uint8_t cmd);
};

#endif   // MATRIX_R4_H
