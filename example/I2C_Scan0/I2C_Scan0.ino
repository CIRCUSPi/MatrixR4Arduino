#include <Wire.h>

void setup()
{
    Serial.begin(115200);

    Wire.begin();
    Serial.println("Scanning I2C bus...");
    for (uint8_t address = 1; address < 128; address++) {
        Wire.beginTransmission(address);
        uint8_t error = Wire.endTransmission();
        if (error == 0) {
            Serial.print("I2C device found at address 0x");
            Serial.print(address, HEX);
            Serial.println("");
        }
    }
    Serial.println("Scan complete.");
}

void loop() {}
