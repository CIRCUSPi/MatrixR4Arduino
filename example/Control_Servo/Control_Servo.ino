#include <MatrixR4.h>

MatrixR4 matrixR4;

void setup(void)
{
    Serial.begin(115200);

    matrixR4.Init();
    // 設定伺服馬達方向為反轉
    matrixR4.SetServoDir(1, DIR_REVERSE);
    // 設定伺服馬達脈波寬度範圍 600~2500
    matrixR4.SetServoPulseRange(1, 600, 2500);
    // 設定伺服馬達角度範圍 0~180
    matrixR4.SetServoAngleRange(1, 0, 180);
}

void loop(void)
{
    // 設定伺服馬達角度，0~180，間隔20ms，每次增加一度
    for (int i = 0; i <= 180; i++) {
        matrixR4.SetServoAngle(1, i);
        delay(20);
    }
}