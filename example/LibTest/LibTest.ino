#include <MatrixMiniR4.h>
#include <Wire.h>

void setup(void)
{
    Serial.begin(115200);
    Wire.begin();

    bool ret = MiniR4.begin();
    MiniR4.M1.setHWDir(true);
    MiniR4.M2.setHWDir(true);
    MiniR4.M3.setHWDir(true);
    MiniR4.M4.setHWDir(true);

    MiniR4.ENC1.setHWDir(true);
    MiniR4.ENC2.setHWDir(true);
    MiniR4.ENC3.setHWDir(true);
    MiniR4.ENC4.setHWDir(true);

    MiniR4.RC1.setHWDir(true);
    MiniR4.RC2.setHWDir(true);
    MiniR4.RC3.setHWDir(true);
    MiniR4.RC4.setHWDir(true);

    MiniR4.LED.setBrightness(1, 10);
    MiniR4.LED.setBrightness(2, 10);

    if (ret) {
        Serial.println("Matrix Mini R4 init success");
    } else {
        Serial.println("Matrix Mini R4 init failed");
    }
}

void loop(void)
{
    // TaskLED();     // pass
    // TaskButton();  // pass
    // TaskMotor();   // pass
    // TaskServo();   // pass
    // TaskMotion();  // pass
    // TaskBuzzer();  // pass

    // TODO: Test
    // TaskEncoder();
}

void TaskLED(void)
{
    static uint32_t timer = 0;
    static uint8_t  idx   = 0;

    uint32_t colors[] = {0xFF0000, 0x00FF00, 0x0000FF, 0xFFFFFF, 0x000000};
    if (millis() >= timer) {
        timer = millis() + 500;
        MiniR4.LED.setColor(1, colors[idx]);
        if (++idx >= 5) idx = 0;
    }
}

void TaskMotor(void)
{
    static uint32_t timer = 0;
    static bool     dir   = true;

    if (millis() >= timer) {
        timer = millis() + 5000;
        dir   = !dir;
        MiniR4.M1.setSpeed(100, dir);
        MiniR4.M2.setSpeed(50, !dir);
        MiniR4.M3.setSpeed(50, !dir);
        MiniR4.M4.setSpeed(100, dir);
    }
}

void TaskServo(void)
{
    static uint32_t timer = 0;
    static uint8_t  angle = 0;

    if (millis() >= timer) {
        timer = millis() + 50;
        angle += 5;
        MiniR4.RC1.setAngle(angle);
        MiniR4.RC2.setAngle(angle);
        MiniR4.RC3.setAngle(angle);
        MiniR4.RC4.setAngle(angle);
        if (angle > 180) angle = 0;
    }
}

void TaskButton(void)
{
    static uint32_t timer = 0;

    if (millis() >= timer) {
        timer     = millis() + 200;
        bool btnA = MiniR4.BTN1.getState();
        bool btnB = MiniR4.BTN2.getState();

        Serial.print("BTN1: ");
        Serial.print(btnA);
        Serial.print(" , BTN2: ");
        Serial.println(btnB);
    }
}

void TaskMotion(void)
{
    static uint32_t timer = 0;

    if (millis() >= timer) {
        timer    = millis() + 200;
        double x = MiniR4.Motion.getAccel(MiniR4Motion::AxisType::X);
        double y = MiniR4.Motion.getAccel(MiniR4Motion::AxisType::Y);
        double z = MiniR4.Motion.getAccel(MiniR4Motion::AxisType::Z);

        Serial.print("Accel: ");
        Serial.print(x);
        Serial.print(" , ");
        Serial.print(y);
        Serial.print(" , ");
        Serial.println(z);

        x = MiniR4.Motion.getGyro(MiniR4Motion::AxisType::X);
        y = MiniR4.Motion.getGyro(MiniR4Motion::AxisType::Y);
        z = MiniR4.Motion.getGyro(MiniR4Motion::AxisType::Z);

        Serial.print("Gyro: ");
        Serial.print(x);
        Serial.print(" , ");
        Serial.print(y);
        Serial.print(" , ");
        Serial.println(z);
    }
}

void TaskEncoder(void)
{
    static uint32_t timer = 0;

    if (millis() >= timer) {
        timer        = millis() + 200;
        int16_t enc1 = MiniR4.ENC1.getCounter();
        int16_t enc2 = MiniR4.ENC2.getCounter();
        int16_t enc3 = MiniR4.ENC3.getCounter();
        int16_t enc4 = MiniR4.ENC4.getCounter();

        Serial.print("Encoder: ");
        Serial.print(enc1);
        Serial.print(" , ");
        Serial.print(enc2);
        Serial.print(" , ");
        Serial.print(enc3);
        Serial.print(" , ");
        Serial.println(enc4);
    }
}

void TaskBuzzer(void)
{
    static uint32_t timer = 0;
    static uint8_t  idx   = 0;

    uint16_t notes[] = {262, 294, 330, 349, 392, 440, 494, 523};

    if (millis() >= timer) {
        timer = millis() + 150;
        MiniR4.Buzzer.Tone(notes[idx], 100);
        if (++idx >= 8) idx = 0;
    }
}
