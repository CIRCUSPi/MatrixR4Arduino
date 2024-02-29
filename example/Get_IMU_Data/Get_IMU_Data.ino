#include <MatrixMiniR4.h>

void setup(void)
{
    Serial.begin(115200);
    mmL.Init();
}

void loop(void)
{
    double X, Y, Z;

    mmL.GetIMUAcc(X, Y, Z);
    // mmL.GetIMUGyro(X, Y, Z);
    Serial.print("X=");
    Serial.print(X);
    Serial.print(",Y=");
    Serial.print(Y);
    Serial.print(",Z=");
    Serial.println(Z);
}