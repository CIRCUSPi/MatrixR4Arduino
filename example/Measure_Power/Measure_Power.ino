#include <MatrixR4.h>

MatrixR4 matrixR4;

void setup(void)
{
    Serial.begin(115200);

    matrixR4.Init();
    matrixR4.SetPowerParam(12.6f, 10.5f, 11.1f);   // 3 Cell
}

void loop(void)
{
    float voltage, voltPercent;
    matrixR4.GetPowerInfo(voltage, voltPercent);

    Serial.print("Voltage: ");
    Serial.print(voltage, 3);
    Serial.print("V, Percent: ");
    Serial.print(voltPercent);
    Serial.println("%");

    delay(1000);
}