#include <MatrixR4.h>

MatrixR4 matrixR4;

void setup(void)
{
    Serial.begin(115200);
    matrixR4.Init();
}

void loop(void)
{
    double X, Y, Z;

    // result = matrixR4.GetIMUAcc(X, Y, Z);
    matrixR4.GetIMUGyro(X, Y, Z);
    Serial.print("X=");
    Serial.print(X);
    Serial.print(",Y=");
    Serial.print(Y);
    Serial.print(",Z=");
    Serial.println(Z);
}