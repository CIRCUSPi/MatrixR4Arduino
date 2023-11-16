#include <MatrixR4.h>

#define MODE 2

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

    String str;
    result = matrixR4.GetFWVersion(str);
    if (result != MatrixR4::RESULT::OK) {
        Serial.print("GetFWVersion Failed! Result: ");
        Serial.println((int)result);
    } else {
        Serial.print("FW Version: ");
        Serial.println(str);
    }

    result = matrixR4.GetFWBuildDay(str);
    if (result != MatrixR4::RESULT::OK) {
        Serial.print("GetFWBuildDay Failed! Result: ");
        Serial.println((int)result);
    } else {
        Serial.print("FW Build Day: ");
        Serial.println(str);
    }

    result = matrixR4.GetFWDescriptor(str);
    if (result != MatrixR4::RESULT::OK) {
        Serial.print("GetFWDescriptor Failed! Result: ");
        Serial.println((int)result);
    } else {
        Serial.print("FW Descriptor: ");
        Serial.println(str);
    }

    uint8_t modelIdx = 0;
    result           = matrixR4.GetModelIndex(modelIdx);
    if (result != MatrixR4::RESULT::OK) {
        Serial.print("GetModelIndex Failed! Result: ");
        Serial.println((int)result);
    } else {
        Serial.print("Model Index: ");
        Serial.println(modelIdx);
    }
}

void loop(void) {}