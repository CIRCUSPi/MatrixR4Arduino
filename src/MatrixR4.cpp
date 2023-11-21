/**
 * @file MatrixR4.cpp
 * @author Zack Huang (zackhuang0513@gmail.com)
 * @brief Matrix Mini Example
 * @version 1.0.0
 * @date 2023-11-14
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "MatrixR4.h"
#include "BitConverter.h"

MatrixR4::MatrixR4(SoftwareSerial* commSerial)
    : commSerial(commSerial)
{}

MatrixR4::RESULT MatrixR4::Init()
{
    // TODO: 初始化所有變數

    if (!commSerial->begin(MatrixR4_COMM_BAUDRATE, SERIAL_8N1)) {
        return RESULT::ERROR_SOFTSERIAL_BEGIN;
    }

    RESULT result = EchoTest();
    if (result != RESULT::OK) {
        Serial.print("EchoTest Failed! Result: ");
        Serial.println((int)result);
        return RESULT::ERROR_INIT;
    }

    return RESULT::OK;
}

MatrixR4::RESULT MatrixR4::SetDCMotorDir(uint8_t num, DIR dir)
{
    uint8_t data[2] = {(1 << --num), (uint8_t)dir};
    CommSendData(COMM_CMD::SET_DC_MOTOR_DIR, data, 2);
    if (!WaitData(COMM_CMD::SET_DC_MOTOR_DIR, 10)) {
        return RESULT::ERROR_WAIT_TIMEOUT;
    }

    uint8_t b[1];
    if (!CommReadData(b, 1)) {
        return RESULT::ERROR_READ_TIMEOUT;
    }

    if (b[0] == 0x00) return RESULT::OK;

    return RESULT::ERROR;
}

MatrixR4::RESULT MatrixR4::SetEncoderDir(uint8_t num, DIR dir)
{
    uint8_t data[2] = {(1 << --num), (uint8_t)dir};
    CommSendData(COMM_CMD::SET_ENCODER_DIR, data, 2);
    if (!WaitData(COMM_CMD::SET_ENCODER_DIR, 10)) {
        return RESULT::ERROR_WAIT_TIMEOUT;
    }

    uint8_t b[1];
    if (!CommReadData(b, 1)) {
        return RESULT::ERROR_READ_TIMEOUT;
    }

    if (b[0] == 0x00) return RESULT::OK;

    return RESULT::ERROR;
}

MatrixR4::RESULT MatrixR4::SetServoDir(uint8_t num, DIR dir)
{
    uint8_t data[2] = {(1 << --num), (uint8_t)dir};
    CommSendData(COMM_CMD::SET_SERVO_DIR, data, 2);
    if (!WaitData(COMM_CMD::SET_SERVO_DIR, 10)) {
        return RESULT::ERROR_WAIT_TIMEOUT;
    }

    uint8_t b[1];
    if (!CommReadData(b, 1)) {
        return RESULT::ERROR_READ_TIMEOUT;
    }

    if (b[0] == 0x00) return RESULT::OK;

    return RESULT::ERROR;
}

MatrixR4::RESULT MatrixR4::SetDCMotorSpeedRange(uint8_t num, uint16_t min, uint16_t max)
{
    uint8_t data[5];
    data[0] = (1 << --num);
    BitConverter::GetBytes(data + 1, min);
    BitConverter::GetBytes(data + 3, max);
    CommSendData(COMM_CMD::SET_DC_MOTOR_SPEED_RANGE, data, 5);
    if (!WaitData(COMM_CMD::SET_DC_MOTOR_SPEED_RANGE, 10)) {
        return RESULT::ERROR_WAIT_TIMEOUT;
    }

    uint8_t b[1];
    if (!CommReadData(b, 1)) {
        return RESULT::ERROR_READ_TIMEOUT;
    }

    if (b[0] == 0x00) return RESULT::OK;

    return RESULT::ERROR;
}

MatrixR4::RESULT MatrixR4::SetServoPulseRange(uint8_t num, uint16_t min, uint16_t max)
{
    uint8_t data[5];
    data[0] = (1 << --num);
    BitConverter::GetBytes(data + 1, min);
    BitConverter::GetBytes(data + 3, max);
    CommSendData(COMM_CMD::SET_SERVO_PULSE_RANGE, data, 5);
    if (!WaitData(COMM_CMD::SET_SERVO_PULSE_RANGE, 10)) {
        return RESULT::ERROR_WAIT_TIMEOUT;
    }

    uint8_t b[1];
    if (!CommReadData(b, 1)) {
        return RESULT::ERROR_READ_TIMEOUT;
    }

    if (b[0] == 0x00) return RESULT::OK;
    if (b[0] == 0x02) return RESULT::ERROR_SERVO_MIN_PULSE;
    if (b[0] == 0x03) return RESULT::ERROR_SERVO_MAX_PULSE;

    return RESULT::ERROR;
}

MatrixR4::RESULT MatrixR4::SetServoAngleRange(uint8_t num, uint16_t min, uint16_t max)
{
    uint8_t data[5];
    data[0] = (1 << --num);
    BitConverter::GetBytes(data + 1, min);
    BitConverter::GetBytes(data + 3, max);
    CommSendData(COMM_CMD::SET_SERVO_ANGLE_RANGE, data, 5);
    if (!WaitData(COMM_CMD::SET_SERVO_ANGLE_RANGE, 10)) {
        return RESULT::ERROR_WAIT_TIMEOUT;
    }

    uint8_t b[1];
    if (!CommReadData(b, 1)) {
        return RESULT::ERROR_READ_TIMEOUT;
    }

    if (b[0] == 0x00) return RESULT::OK;
    if (b[0] == 0x02) return RESULT::ERROR_SERVO_MIN_ANGLE;
    if (b[0] == 0x03) return RESULT::ERROR_SERVO_MAX_ANGLE;

    return RESULT::ERROR;
}

MatrixR4::RESULT MatrixR4::SetButtonInit(
    uint8_t num, BUTTON_ECHO_MODE mode, uint16_t repeatStartMs, uint16_t repeatWorkMs)
{
    uint8_t data[6];
    data[0] = (1 << --num);
    data[1] = (uint8_t)mode;
    BitConverter::GetBytes(data + 2, repeatStartMs);
    BitConverter::GetBytes(data + 4, repeatWorkMs);
    CommSendData(COMM_CMD::SET_BUTTON_INIT, data, 6);
    if (!WaitData(COMM_CMD::SET_BUTTON_INIT, 10)) {
        return RESULT::ERROR_WAIT_TIMEOUT;
    }

    uint8_t b[1];
    if (!CommReadData(b, 1)) {
        return RESULT::ERROR_READ_TIMEOUT;
    }

    if (b[0] == 0x00) return RESULT::OK;

    return RESULT::ERROR;
}

MatrixR4::RESULT MatrixR4::SetEncoderEchoMode(ENCODER_ECHO_MODE mode, uint16_t echoIntervalMs)
{
    uint8_t data[3];
    data[0] = (uint8_t)mode;
    BitConverter::GetBytes(data + 1, echoIntervalMs);
    CommSendData(COMM_CMD::SET_ENCODER_ECHO_MODE, data, 3);
    if (!WaitData(COMM_CMD::SET_ENCODER_ECHO_MODE, 10)) {
        return RESULT::ERROR_WAIT_TIMEOUT;
    }

    uint8_t b[1];
    if (!CommReadData(b, 1)) {
        return RESULT::ERROR_READ_TIMEOUT;
    }

    if (b[0] == 0x00) return RESULT::OK;
    if (b[0] == 0x02) return RESULT::ERROR_MODE;
    if (b[0] == 0x03) return RESULT::ERROR_INTERVAL;

    return RESULT::ERROR;
}

MatrixR4::RESULT MatrixR4::SetIMUEchoMode(IMU_ECHO_MODE mode, uint16_t echoIntervalMs)
{
    uint8_t data[3];
    data[0] = (uint8_t)mode;
    BitConverter::GetBytes(data + 1, echoIntervalMs);
    CommSendData(COMM_CMD::SET_IMU_ECHO_MODE, data, 3);
    if (!WaitData(COMM_CMD::SET_IMU_ECHO_MODE, 10)) {
        return RESULT::ERROR_WAIT_TIMEOUT;
    }

    uint8_t b[1];
    if (!CommReadData(b, 1)) {
        return RESULT::ERROR_READ_TIMEOUT;
    }

    if (b[0] == 0x00) return RESULT::OK;
    if (b[0] == 0x02) return RESULT::ERROR_MODE;
    if (b[0] == 0x03) return RESULT::ERROR_INTERVAL;

    return RESULT::ERROR;
}

MatrixR4::RESULT MatrixR4::SetIMUInit(
    IMU_ACC_FSR accFSR, IMU_GYRO_FSR gyroFSR, IMU_ODR odr, IMU_FIFO fifo)
{
    uint8_t data[4];
    data[0] = (uint8_t)accFSR;
    data[1] = (uint8_t)gyroFSR;
    data[2] = (uint8_t)odr;
    data[3] = (uint8_t)fifo;
    CommSendData(COMM_CMD::SET_IMU_INIT, data, 4);
    if (!WaitData(COMM_CMD::SET_IMU_INIT, 10)) {
        return RESULT::ERROR_WAIT_TIMEOUT;
    }

    uint8_t b[1];
    if (!CommReadData(b, 1)) {
        return RESULT::ERROR_READ_TIMEOUT;
    }

    if (b[0] == 0x00) return RESULT::OK;
    if (b[0] == 0x02) return RESULT::ERROR_IMU_ACC_FSR;
    if (b[0] == 0x03) return RESULT::ERROR_IMU_GYRO_FSR;
    if (b[0] == 0x04) return RESULT::ERROR_IMU_ODR;

    return RESULT::ERROR;
}

MatrixR4::RESULT MatrixR4::SetPowerParam(float fullVolt, float cutOffVolt, float alarmVolt)
{
    uint8_t data[3];
    data[0] = (uint8_t)(fullVolt * 10.0f);
    data[1] = (uint8_t)(cutOffVolt * 10.0f);
    data[2] = (uint8_t)(alarmVolt * 10.0f);
    CommSendData(COMM_CMD::SET_POWER_PARAM, data, 3);
    if (!WaitData(COMM_CMD::SET_POWER_PARAM, 5)) {
        return RESULT::ERROR_WAIT_TIMEOUT;
    }

    uint8_t b[1];
    if (!CommReadData(b, 1, 5)) {
        return RESULT::ERROR_READ_TIMEOUT;
    }
    if (b[0] == 0x00) return RESULT::OK;
    if (b[0] == 0x02) return RESULT::ERROR_POWER_VOLT_RANGE;

    return RESULT::ERROR;
}
// Setting-Commonly used
MatrixR4::RESULT MatrixR4::SetDCMotorSpeed(uint8_t num, uint16_t speed, DIR dir)
{
    uint8_t data[4];
    data[0] = (1 << --num);
    data[1] = (uint8_t)dir;
    BitConverter::GetBytes(data + 2, speed);
    CommSendData(COMM_CMD::SET_DC_MOTOR_SPEED, data, 4);
    if (!WaitData(COMM_CMD::SET_DC_MOTOR_SPEED, 5)) {
        return RESULT::ERROR_WAIT_TIMEOUT;
    }

    uint8_t b[1];
    if (!CommReadData(b, 1)) {
        return RESULT::ERROR_READ_TIMEOUT;
    }

    if (b[0] == 0x00) return RESULT::OK;
    if (b[0] == 0x02) return RESULT::ERROR_MOTOR_SPEED;

    return RESULT::ERROR;
}

MatrixR4::RESULT MatrixR4::SetAllDCMotorSpeed(Motors_Param_t param)
{
    uint8_t data[9];
    data[0] = (uint8_t)param.m1_dir;
    data[0] |= ((uint8_t)param.m2_dir) << 1;
    data[0] |= ((uint8_t)param.m3_dir) << 2;
    data[0] |= ((uint8_t)param.m4_dir) << 3;
    BitConverter::GetBytes(data + 1, param.m1_speed);
    BitConverter::GetBytes(data + 3, param.m2_speed);
    BitConverter::GetBytes(data + 5, param.m3_speed);
    BitConverter::GetBytes(data + 7, param.m4_speed);
    CommSendData(COMM_CMD::SET_ALL_DC_MOTOR_SPEED, data, 10);
    if (!WaitData(COMM_CMD::SET_ALL_DC_MOTOR_SPEED, 10)) {
        return RESULT::ERROR_WAIT_TIMEOUT;
    }

    uint8_t b[1];
    if (!CommReadData(b, 1)) {
        return RESULT::ERROR_READ_TIMEOUT;
    }

    if (b[0] == 0x00) return RESULT::OK;
    if (b[0] == 0x02) return RESULT::ERROR_MOTOR1_SPEED;
    if (b[0] == 0x03) return RESULT::ERROR_MOTOR2_SPEED;
    if (b[0] == 0x04) return RESULT::ERROR_MOTOR3_SPEED;
    if (b[0] == 0x05) return RESULT::ERROR_MOTOR4_SPEED;

    return RESULT::ERROR;
}

MatrixR4::RESULT MatrixR4::SetServoAngle(uint8_t num, uint16_t angle)
{
    uint8_t data[3];
    data[0] = (1 << --num);
    BitConverter::GetBytes(data + 1, angle);
    CommSendData(COMM_CMD::SET_SERVO_ANGLE, data, 3);
    if (!WaitData(COMM_CMD::SET_SERVO_ANGLE, 10)) {
        return RESULT::ERROR_WAIT_TIMEOUT;
    }

    uint8_t b[1];
    if (!CommReadData(b, 1)) {
        return RESULT::ERROR_READ_TIMEOUT;
    }

    if (b[0] == 0x00) return RESULT::OK;
    if (b[0] == 0x02) return RESULT::ERROR_SERVO_ANGLE;

    return RESULT::ERROR;
}

MatrixR4::RESULT MatrixR4::SetAllServoAngle(
    uint16_t angle1, uint16_t angle2, uint16_t angle3, uint16_t angle4)
{
    uint8_t data[8];
    BitConverter::GetBytes(data + 0, angle1);
    BitConverter::GetBytes(data + 2, angle2);
    BitConverter::GetBytes(data + 4, angle3);
    BitConverter::GetBytes(data + 6, angle4);
    CommSendData(COMM_CMD::SET_ALL_SERVO_ANGLE, data, 8);
    if (!WaitData(COMM_CMD::SET_ALL_SERVO_ANGLE, 10)) {
        return RESULT::ERROR_WAIT_TIMEOUT;
    }

    uint8_t b[1];
    if (!CommReadData(b, 1)) {
        return RESULT::ERROR_READ_TIMEOUT;
    }

    if (b[0] == 0x00) return RESULT::OK;
    if (b[0] == 0x02) return RESULT::ERROR_SERVO1_ANGLE;
    if (b[0] == 0x03) return RESULT::ERROR_SERVO2_ANGLE;
    if (b[0] == 0x04) return RESULT::ERROR_SERVO3_ANGLE;
    if (b[0] == 0x05) return RESULT::ERROR_SERVO4_ANGLE;

    return RESULT::ERROR;
}

MatrixR4::RESULT MatrixR4::SetMoveDistance(
    MOVE_TYPE type, MOVE_ACTION action, uint16_t speed, uint16_t enCounter)
{
    uint8_t data[6];
    data[0] = (uint8_t)type;
    data[1] = (uint8_t)action;
    BitConverter::GetBytes(data + 2, speed);
    BitConverter::GetBytes(data + 4, enCounter);
    CommSendData(COMM_CMD::SET_MOVE_DISTANCE, data, 6);
    if (!WaitData(COMM_CMD::SET_MOVE_DISTANCE, 10)) {
        return RESULT::ERROR_WAIT_TIMEOUT;
    }

    uint8_t b[1];
    if (!CommReadData(b, 1)) {
        return RESULT::ERROR_READ_TIMEOUT;
    }

    if (b[0] == 0x00) return RESULT::OK;
    if (b[0] == 0x02) return RESULT::ERROR_MOVE_ACTION;
    if (b[0] == 0x03) return RESULT::ERROR_MOVE_SPEED;
    if (b[0] == 0x04) return RESULT::ERROR_MOVE_ENCODER;

    return RESULT::ERROR;
}

MatrixR4::RESULT MatrixR4::SetEncoderResetCounter(uint8_t num)
{
    uint8_t data[1] = {(1 << --num)};
    CommSendData(COMM_CMD::SET_ENCODER_RESET_COUNTER, data, 1);
    if (!WaitData(COMM_CMD::SET_ENCODER_RESET_COUNTER, 10)) {
        return RESULT::ERROR_WAIT_TIMEOUT;
    }

    uint8_t b[1];
    if (!CommReadData(b, 1)) {
        return RESULT::ERROR_READ_TIMEOUT;
    }
    if (b[0] == 0x00) return RESULT::OK;

    return RESULT::ERROR;
}

MatrixR4::RESULT MatrixR4::SetStateLED(uint8_t brightness, uint32_t colorRGB)
{
    uint8_t data[4];
    data[0] = brightness;
    data[1] = (uint8_t)(colorRGB >> 16);
    data[2] = (uint8_t)(colorRGB >> 8);
    data[3] = (uint8_t)(colorRGB);
    CommSendData(COMM_CMD::SET_STATE_LED, data, 4);
    if (!WaitData(COMM_CMD::SET_STATE_LED, 5)) {
        return RESULT::ERROR_WAIT_TIMEOUT;
    }

    uint8_t b[1];
    if (!CommReadData(b, 1, 10)) {
        return RESULT::ERROR_READ_TIMEOUT;
    }
    if (b[0] == 0x00) return RESULT::OK;

    return RESULT::ERROR;
}
// Getting
MatrixR4::RESULT MatrixR4::GetButtonState(uint8_t num, bool& btnState)
{
    uint8_t data[1] = {--num};
    CommSendData(COMM_CMD::GET_BUTTON_STATE, data, 1);
    if (!WaitData(COMM_CMD::GET_BUTTON_STATE, 10)) {
        return RESULT::ERROR_WAIT_TIMEOUT;
    }

    uint8_t b[1];
    if (!CommReadData(b, 1)) {
        return RESULT::ERROR_READ_TIMEOUT;
    }
    btnState = (bool)b[0];

    return RESULT::OK;
}

MatrixR4::RESULT MatrixR4::GetButtonsState(bool* btnsState)
{
    CommSendData(COMM_CMD::GET_BUTTONS_STATE);
    if (!WaitData(COMM_CMD::GET_BUTTONS_STATE, 10)) {
        return RESULT::ERROR_WAIT_TIMEOUT;
    }

    uint8_t b[2];
    if (!CommReadData(b, 2)) {
        return RESULT::ERROR_READ_TIMEOUT;
    }
    uint16_t flag = BitConverter::ToUInt16(b, 0);
    btnsState[0]  = (bool)(flag);
    btnsState[1]  = (bool)(flag >> 1);

    return RESULT::OK;
}

MatrixR4::RESULT MatrixR4::GetEncoderCounter(uint8_t num, int16_t& enCounter)
{
    uint8_t data[1] = {(1 << --num)};
    CommSendData(COMM_CMD::GET_ENCODER_COUNTER, data, 1);
    if (!WaitData(COMM_CMD::GET_ENCODER_COUNTER, 10)) {
        return RESULT::ERROR_WAIT_TIMEOUT;
    }

    uint8_t b[2];
    if (!CommReadData(b, 2)) {
        return RESULT::ERROR_READ_TIMEOUT;
    }
    enCounter = BitConverter::ToInt16(b, 0);

    return RESULT::OK;
}

MatrixR4::RESULT MatrixR4::GetAllEncoderCounter(int16_t* enCounter)
{
    CommSendData(COMM_CMD::GET_ALL_ENCODER_COUNTER);
    if (!WaitData(COMM_CMD::GET_ALL_ENCODER_COUNTER, 10)) {
        return RESULT::ERROR_WAIT_TIMEOUT;
    }

    uint8_t b[8];
    if (!CommReadData(b, 8)) {
        return RESULT::ERROR_READ_TIMEOUT;
    }
    enCounter[0] = BitConverter::ToInt16(b, 0);
    enCounter[1] = BitConverter::ToInt16(b, 2);
    enCounter[2] = BitConverter::ToInt16(b, 4);
    enCounter[3] = BitConverter::ToInt16(b, 6);

    return RESULT::OK;
}

MatrixR4::RESULT MatrixR4::GetIMUEuler(double& roll, double& pitch, double& yaw)
{
    return RESULT::ERROR;
}

MatrixR4::RESULT MatrixR4::GetIMUGyro(double& x, double& y, double& z)
{
    CommSendData(COMM_CMD::GET_IMU_GYRO);
    if (!WaitData(COMM_CMD::GET_IMU_GYRO, 10)) {
        return RESULT::ERROR_WAIT_TIMEOUT;
    }

    uint8_t b[6];
    if (!CommReadData(b, 6)) {
        return RESULT::ERROR_READ_TIMEOUT;
    }
    x = BitConverter::ToInt16(b, 0) / 100.0f;
    y = BitConverter::ToInt16(b, 2) / 100.0f;
    z = BitConverter::ToInt16(b, 4) / 100.0f;

    return RESULT::OK;
}

MatrixR4::RESULT MatrixR4::GetIMUAcc(double& x, double& y, double& z)
{
    CommSendData(COMM_CMD::GET_IMU_ACC);
    if (!WaitData(COMM_CMD::GET_IMU_ACC, 10)) {
        return RESULT::ERROR_WAIT_TIMEOUT;
    }

    uint8_t b[6];
    if (!CommReadData(b, 6)) {
        return RESULT::ERROR_READ_TIMEOUT;
    }
    x = BitConverter::ToInt16(b, 0) / 1000.0f;
    y = BitConverter::ToInt16(b, 2) / 1000.0f;
    z = BitConverter::ToInt16(b, 4) / 1000.0f;

    return RESULT::OK;
}

MatrixR4::RESULT MatrixR4::GetPowerInfo(float& curVolt, float& curVoltPerc)
{
    CommSendData(COMM_CMD::GET_POWER_INFO);
    if (!WaitData(COMM_CMD::GET_POWER_INFO, 5)) {
        return RESULT::ERROR_WAIT_TIMEOUT;
    }

    uint8_t b[3];
    if (!CommReadData(b, 3)) {
        return RESULT::ERROR_READ_TIMEOUT;
    }

    uint16_t voltRaw = BitConverter::ToUInt16(b, 0);
    curVolt          = (float)voltRaw / 1000.0f;
    curVoltPerc      = (float)b[1];

    return RESULT::OK;
}
// Other-Info
MatrixR4::RESULT MatrixR4::EchoTest(void)
{
    uint8_t data[1] = {0x55};
    CommSendData(COMM_CMD::ECHO_TEST, data, 1);
    if (!WaitData(COMM_CMD::ECHO_TEST, 10)) {
        return RESULT::ERROR_WAIT_TIMEOUT;
    }

    uint8_t b[1];
    if (!CommReadData(b, 1)) {
        return RESULT::ERROR_READ_TIMEOUT;
    }
    if (b[0] == data[0]) {
        return RESULT::OK;
    }
    return RESULT::ERROR;
}

MatrixR4::RESULT MatrixR4::GetFWVersion(String& version)
{
    CommSendData(COMM_CMD::F_VERSION);
    if (!WaitData(COMM_CMD::F_VERSION, 10)) {
        return RESULT::ERROR_WAIT_TIMEOUT;
    }

    uint8_t b[1];
    if (!CommReadData(b, 1)) {
        return RESULT::ERROR_READ_TIMEOUT;
    }

    version = String(b[0] / 10.0f);

    return RESULT::OK;
}

MatrixR4::RESULT MatrixR4::GetFWBuildDay(String& date)
{
    CommSendData(COMM_CMD::F_BUILD_DAY);
    if (!WaitData(COMM_CMD::F_BUILD_DAY, 10)) {
        return RESULT::ERROR_WAIT_TIMEOUT;
    }

    uint8_t b[4];
    if (!CommReadData(b, 4)) {
        return RESULT::ERROR_READ_TIMEOUT;
    }

    uint16_t year  = BitConverter::ToUInt16(b, 0);
    uint8_t  month = b[2];
    uint8_t  day   = b[3];

    char str[10];
    sprintf(str, "%04d-%02d-%02d", year, month, day);
    date = String(str);

    return RESULT::OK;
}

MatrixR4::RESULT MatrixR4::GetFWDescriptor(String& descriptor)
{
    CommSendData(COMM_CMD::F_DESCRIPTOR);
    if (!WaitData(COMM_CMD::F_DESCRIPTOR, 10)) {
        return RESULT::ERROR_WAIT_TIMEOUT;
    }

    uint8_t b[1];
    if (!CommReadData(b, 1)) {
        return RESULT::ERROR_READ_TIMEOUT;
    }

    uint8_t len = b[0];
    uint8_t str[len + 1];
    if (!CommReadData(str, len)) {
        return RESULT::ERROR_READ_TIMEOUT;
    }
    str[len]   = '\0';
    descriptor = String((char*)str);
    return RESULT::OK;
}

MatrixR4::RESULT MatrixR4::GetModelIndex(uint8_t& index)
{
    CommSendData(COMM_CMD::READ_MODEL_INDEX);
    if (!WaitData(COMM_CMD::READ_MODEL_INDEX, 10)) {
        return RESULT::ERROR_WAIT_TIMEOUT;
    }

    uint8_t b[1];
    if (!CommReadData(b, 1)) {
        return RESULT::ERROR_READ_TIMEOUT;
    }
    index = b[0];
    return RESULT::OK;
}

MatrixR4::RESULT MatrixR4::GetAllInfo(AllInfo_t& info)
{
    CommSendData(COMM_CMD::F_VERSION);
    if (!WaitData(COMM_CMD::F_VERSION, 10)) {
        return RESULT::ERROR_WAIT_TIMEOUT;
    }

    uint8_t b[6];
    if (!CommReadData(b, 6)) {
        return RESULT::ERROR_READ_TIMEOUT;
    }
    uint16_t year  = BitConverter::ToUInt16(b, 0);
    uint8_t  month = b[2];
    uint8_t  day   = b[3];

    char str[10];
    sprintf(str, "%04d-%02d-%02d", year, month, day);

    info.fwVersion  = String(b[0] / 10.0f);
    info.fwBuildDay = String(str);
    info.modelIndex = b[5];

    return RESULT::OK;
}

MatrixR4::RESULT MatrixR4::RunAutoQC(void)
{
    CommSendData(COMM_CMD::RUN_AUTO_QC);
    if (!WaitData(COMM_CMD::RUN_AUTO_QC, 10)) {
        return RESULT::ERROR_WAIT_TIMEOUT;
    }

    uint8_t b[1];
    if (!CommReadData(b, 1)) {
        return RESULT::ERROR_READ_TIMEOUT;
    }

    if ((b[0] & 0x01) == 0x00) return RESULT::ERROR_QC_OLED;
    if ((b[0] >> 1 & 0x01) == 0x00) return RESULT::ERROR_QC_I2C_MUX;
    if ((b[0] >> 2 & 0x01) == 0x00) return RESULT::ERROR_QC_IMU;

    return RESULT::OK;
}

void MatrixR4::loop(void)
{
    WaitData(COMM_CMD::NONE);
}

void MatrixR4::onBtnChg(BtnChgCallback callback)
{
    callbackFunc = callback;
}

void MatrixR4::CommSendData(COMM_CMD cmd, uint8_t* data, uint16_t size)
{
    uint8_t  arr[3 + size];
    uint8_t* ptr = arr;

    *ptr++ = MatrixR4_COMM_LEAD;
    *ptr++ = ((~MatrixR4_COMM_LEAD) & 0xFF);
    *ptr++ = (uint8_t)cmd;

    for (uint16_t i = 0; i < size; i++) {
        *ptr++ = data[i];
    }
    commSerial->write(arr, 3 + size);
    commSerial->flush();
}

void MatrixR4::CommSendData(COMM_CMD cmd, uint8_t data)
{
    uint8_t _data[1] = {data};
    CommSendData((COMM_CMD)cmd, _data, 1);
}

bool MatrixR4::CommReadData(uint8_t* data, uint16_t size, uint32_t timeout_ms)
{
    uint32_t timeout = millis() + timeout_ms;
    while (millis() <= timeout) {
        if (commSerial->available() >= size) {
            for (uint16_t i = 0; i < size; i++) {
                data[i] = commSerial->read();
            }
            return true;
        }
    }
    return false;
}

bool MatrixR4::WaitData(COMM_CMD cmd, uint32_t timeout_ms)
{
    static COMM_STATE state = COMM_STATE::WAIT_LEAD;

    uint32_t timeout = millis() + timeout_ms;
    while (millis() <= timeout) {
        if (commSerial->available() <= 0) {
            continue;
        }
        switch (state) {
        case COMM_STATE::WAIT_LEAD:
        {
            uint8_t b = commSerial->read();
            if (b == MatrixR4_COMM_LEAD) {
                state = COMM_STATE::WAIT_NOT_LEAD;
            }
        } break;

        case COMM_STATE::WAIT_NOT_LEAD:
        {
            uint8_t b = commSerial->read();
            if (b == ((~MatrixR4_COMM_LEAD) & 0xFF))
                state = COMM_STATE::WAIT_CMD;
            else
                state = COMM_STATE::WAIT_LEAD;
        } break;

        case COMM_STATE::WAIT_CMD:
        {
            uint8_t b = commSerial->read();
            if (b == (uint8_t)cmd) {
                state = COMM_STATE::WAIT_LEAD;
                return true;
            } else {
                state = COMM_STATE::WAIT_LEAD;
                HandleCommand(b);
            }
        } break;

        case COMM_STATE::ERROR:
        {
            // TODO: Handle Error
            Serial.println("COMM_STATE: ERROR");
            state = COMM_STATE::WAIT_LEAD;
        } break;

        default: state = COMM_STATE::WAIT_LEAD; break;
        }
    }
    return false;
}

void MatrixR4::HandleCommand(uint8_t cmd)
{
    switch (cmd) {
    case (uint8_t)COMM_CMD::AUTO_SEND_BUTTON_STATE:
    {
        uint8_t b[2];
        if (CommReadData(b, 2)) {
            if (b[0] < MatrixR4_BUTTON_NUM) {
                callbackFunc(b[0] + 1, (BTN_STATE)b[1]);
            }
        }
    } break;
    case (uint8_t)COMM_CMD::AUTO_SEND_ENCODER_COUNTER:
    {
        uint8_t b[8];
        if (CommReadData(b, 8)) {
            for (uint8_t i = 0; i < MatrixR4_ENCODER_NUM; i++) {
                enCounter[i] = BitConverter::ToInt16(b, i * 2);
            }
        }
    } break;
    case (uint8_t)COMM_CMD::AUTO_SEND_IMU_EULER:
    {
        uint8_t b[6];
        if (CommReadData(b, 6)) {}
    } break;
    case (uint8_t)COMM_CMD::AUTO_SEND_IMU_GYRO:
    {
        uint8_t b[6];
        if (CommReadData(b, 6)) {
            imuGyroX = BitConverter::ToInt16(b, 0) / 100.0f;
            imuGyroY = BitConverter::ToInt16(b, 2) / 100.0f;
            imuGyroZ = BitConverter::ToInt16(b, 4) / 100.0f;
        }
    } break;
    case (uint8_t)COMM_CMD::AUTO_SEND_IMU_ACC:
    {
        uint8_t b[6];
        if (CommReadData(b, 6)) {
            imuAccX = BitConverter::ToInt16(b, 0) / 1000.0f;
            imuAccY = BitConverter::ToInt16(b, 2) / 1000.0f;
            imuAccZ = BitConverter::ToInt16(b, 4) / 1000.0f;
        }
    } break;
    default: break;
    }
}
