#include <MatrixMiniR4.h>

MMLower matrixR4;

void setup(void)
{
    Serial.begin(115200);

    matrixR4.Init();
    // 設定伺服馬達方向為反轉
    for (int i = 1; i <= 4; i++) {
        matrixR4.SetServoDir(i, DIR_REVERSE);
        // 設定伺服馬達脈波寬度範圍 600~2500
        matrixR4.SetServoPulseRange(i, 600, 2500);
        // 設定伺服馬達角度範圍 0~180
        matrixR4.SetServoAngleRange(i, 0, 180);
    }
}

void loop(void)
{
    // 設定伺服馬達角度，0~180，間隔20ms，每次增加一度
    for (int i = 0; i <= 180; i++) {
        for (int j = 1; j <= 4; j++) {
            matrixR4.SetServoAngle(j, i);
        }
        delay(20);
    }
}