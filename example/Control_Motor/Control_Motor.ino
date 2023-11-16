#include <MatrixR4.h>

SoftwareSerial softSerial(8, 9);   // rx, tx
MatrixR4       matrixR4(&softSerial);

void setup(void)
{
    Serial.begin(115200);

    MatrixR4::RESULT result;

    // 初始化
    result = matrixR4.Init();
    if (result != MatrixR4::RESULT::OK) {
        Serial.print("MatrixR4 Init Failed! Result: ");
        Serial.println((int)result);

        while (1) {
            delay(1000);
        }
    }
    Serial.println("MatrixR4 Init Success!");

    // 設定直流馬達方向
    result = matrixR4.SetDCMotorDir(1, MatrixR4::DIR::FORWARD);
    while (result != MatrixR4::RESULT::OK) {
        Serial.print("SetDCMotorDir Failed! Result: ");
        Serial.println((int)result);
        delay(1000);
        result = matrixR4.SetDCMotorDir(1, MatrixR4::DIR::FORWARD);
    }
    Serial.println("SetDCMotorDir Success!");

    // 設定編碼器方向
    result = matrixR4.SetEncoderDir(1, MatrixR4::DIR::FORWARD);
    while (result != MatrixR4::RESULT::OK) {
        Serial.print("SetEncoderDir Failed! Result: ");
        Serial.println((int)result);
        delay(1000);
        result = matrixR4.SetEncoderDir(1, MatrixR4::DIR::FORWARD);
    }
    Serial.println("SetEncoderDir Success!");

    // 設定直流馬達速度範圍
    result = matrixR4.SetDCMotorSpeedRange(1, 0, 100);
    while (result != MatrixR4::RESULT::OK) {
        Serial.print("SetDCMotorSpeedRange Failed! Result: ");
        Serial.println((int)result);
        delay(1000);
        result = matrixR4.SetDCMotorSpeedRange(1, 0, 100);
    }
    Serial.println("SetDCMotorSpeedRange Success!");

    // 設定編碼器回覆模式
    result = matrixR4.SetEncoderEchoMode(MatrixR4::ENCODER_ECHO_MODE::ACTIVE, 50);
    while (result != MatrixR4::RESULT::OK) {
        Serial.print("SetEncoderEchoMode Failed! Result: ");
        Serial.println((int)result);
        delay(1000);
        result = matrixR4.SetEncoderEchoMode(MatrixR4::ENCODER_ECHO_MODE::ACTIVE, 50);
    }
    Serial.println("SetEncoderEchoMode Success!");
}

void loop(void)
{
    MatrixR4::RESULT result;
    // 設定直流馬達速度，速度範圍為 0 ~ 100，由小到大，間隔 100ms
    for (int i = 0; i <= 100; i++) {
        result = matrixR4.SetDCMotorSpeed(1, i, MatrixR4::DIR::FORWARD);
        if (result != MatrixR4::RESULT::OK) {
            Serial.print("SetDCMotorSpeed Failed! Result: ");
            Serial.println((int)result);
        }
        // delay(25);
    }

    // 取得編碼器數值
    int16_t encoderValue = 0;
    result               = matrixR4.GetEncoderCounter(0, encoderValue);
    if (result != MatrixR4::RESULT::OK) {
        Serial.print("GetEncoderCounter Failed! Result: ");
        Serial.println((int)result);
    } else {
        // Serial.print("Encoder Value: ");
        // Serial.println(encoderValue);
    }

    Serial.print("Encoder1 Value: ");
    Serial.println(matrixR4.enCounter[0]);
    Serial.print("Encoder2 Value: ");
    Serial.println(matrixR4.enCounter[1]);
    Serial.print("Encoder3 Value: ");
    Serial.println(matrixR4.enCounter[2]);
    Serial.print("Encoder4 Value: ");
    Serial.println(matrixR4.enCounter[3]);
    Serial.println();
    matrixR4.loop();
}