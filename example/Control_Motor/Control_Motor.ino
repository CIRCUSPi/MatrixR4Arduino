#include <MatrixMiniR4.h>

MMLower matrixR4;

void setup(void)
{
    Serial.begin(115200);

    matrixR4.Init();

    for (int i = 1; i <= 4; i++) {
        // 設定直流馬達方向為正轉
        matrixR4.SetDCMotorDir(i, DIR_REVERSE);
        // 設定編碼器方向為正轉
        matrixR4.SetEncoderDir(i, DIR_REVERSE);
        // 設定直流馬達速度範圍為 0 ~ 100
        matrixR4.SetDCMotorSpeedRange(i, 0, 100);
    }
}

void loop(void)
{
    MMLower::RESULT result;
    // 設定直流馬達速度，速度範圍為 0 ~ 100，由小到大，間隔 50ms
    for (int i = 0; i <= 100; i++) {
        for (int j = 1; j <= 4; j++) {
            matrixR4.SetDCMotorSpeed(j, i, DIR_FORWARD);
        }
        delay(50);
    }

    // 取得編碼器數值
    for (int i = 1; i <= 4; i++) {
        int16_t encoderValue = 0;
        matrixR4.GetEncoderCounter(i, encoderValue);
        Serial.print("Encoder");
        Serial.print(i);
        Serial.print(" Value: ");
        Serial.println(encoderValue);
    }
    Serial.println();
}