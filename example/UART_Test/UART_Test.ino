#include <SoftwareSerial.h>

SoftwareSerial softSerial(8, 9);   // rx, tx

void setup()
{
    Serial.begin(115200);
    softSerial.begin(57600, SERIAL_8N1);
    Serial.println("Start!");
}

void loop()
{
    if (Serial.available() > 0) {
        softSerial.write(Serial.read());
    }

    if (softSerial.available() > 0) {
        Serial.write(softSerial.read());
    }
}
