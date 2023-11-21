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

    result = matrixR4.SetPowerParam(8.4f, 7.2f, 7.4f);   // 2 Cell
    if (result != MatrixR4::RESULT::OK) {
        Serial.print("SetPowerParam Failed! Result: ");
        Serial.println((int)result);
    } else {
        Serial.println("SetPowerParam Success!");
    }
}

void loop(void)
{
    MatrixR4::RESULT result;
    float            voltage, voltPercent;

    result = matrixR4.GetPowerInfo(voltage, voltPercent);
    if (result != MatrixR4::RESULT::OK) {
        Serial.print("GetPowerVoltage Failed! Result: ");
        Serial.println((int)result);
    } else {
        Serial.print("Voltage: ");
        Serial.print(voltage, 3);
        Serial.print("V, Percent: ");
        Serial.print(voltPercent);
        Serial.println("%");
    }
    delay(1000);
}