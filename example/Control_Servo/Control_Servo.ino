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

    // 設定伺服馬達方向
    result = matrixR4.SetServoDir(1, MatrixR4::DIR::REVERSE);
    while (result != MatrixR4::RESULT::OK) {
        Serial.print("SetServoDir Failed! Result: ");
        Serial.println((int)result);
        delay(1000);
    }
    Serial.println("SetServoDir Success!");

    // 設定伺服馬達脈波寬度
    result = matrixR4.SetServoPulseRange(1, 600, 2500);
    while (result != MatrixR4::RESULT::OK) {
        Serial.print("SetServoPulseRange Failed! Result: ");
        Serial.println((int)result);
        delay(1000);
    }
    Serial.println("SetServoPulseRange Success!");

    // 設定伺服馬達角度範圍
    result = matrixR4.SetServoAngleRange(1, 0, 180);
    while (result != MatrixR4::RESULT::OK) {
        Serial.print("SetServoAngleRange Failed! Result: ");
        Serial.println((int)result);
        delay(1000);
    }
    Serial.println("SetServoAngleRange Success!");
}

void loop(void)
{
    MatrixR4::RESULT result;
    // 設定伺服馬達角度，0~180，間隔20ms，每次增加一度
    for (int i = 0; i <= 180; i++) {
        result = matrixR4.SetServoAngle(1, i);
        if (result != MatrixR4::RESULT::OK) {
            Serial.print("SetServoAngle Failed! Result: ");
            Serial.println((int)result);
        }
        delay(20);
    }

    matrixR4.loop();
}