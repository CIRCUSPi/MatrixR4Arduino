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
    // TaskEncoder(); // pass
    // TaskOLED();    // pass
    // TaskWiFi();    // pass
    // TaskI2CMotion();// pass
    // TaskI2CLaser(); // pass

    // TODO: Test
    // TaskI2CColor(); // fail Fix Bug
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
        MiniR4.M2.setSpeed(100, dir);
        MiniR4.M3.setSpeed(100, dir);
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

        char buff[100];
        sprintf(buff, "Encoder: %d, %d, %d, %d", enc1, enc2, enc3, enc4);

        Serial.println(buff);
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

void TaskOLED(void)
{
    static uint32_t timer = 0;

    if (millis() >= timer) {
        timer = millis() + 1000;

        MiniR4.OLED.clearDisplay();
        MiniR4.OLED.setTextSize(3);
        MiniR4.OLED.setTextColor(SSD1306_WHITE);
        MiniR4.OLED.setCursor(10, 10);
        MiniR4.OLED.print(String(millis() / 1000) + "s");
        MiniR4.OLED.display();
    }
}

void TaskWiFi(void)
{
    static uint32_t timer = 0;

    if (millis() >= timer) {
        timer = millis() + 5000;

        Serial.println("** Scan Networks **");
        int numSsid = MiniR4.WiFi.scanNetworks();
        if (numSsid == -1) {
            Serial.println("Couldn't get a WiFi connection");
            while (true)
                ;
        }

        Serial.print("number of available networks:");
        Serial.println(numSsid);

        for (int thisNet = 0; thisNet < numSsid; thisNet++) {
            Serial.print(thisNet);
            Serial.print(") ");
            Serial.print(MiniR4.WiFi.SSID(thisNet));
            Serial.print(" Signal: ");
            Serial.print(MiniR4.WiFi.RSSI(thisNet));
            Serial.println(" dBm");
        }
    }
}

void TaskI2CMotion(void)
{
    static bool initFlag = false;

    if (!initFlag) {
        bool ret = MiniR4.I2C4.MXMotion.begin();

        if (ret) {
            Serial.println("MXMotion init success");
        } else {
            Serial.println("MXMotion init failed");
        }
        initFlag = true;
    }

    static uint32_t timer = 0;
    if (millis() >= timer) {
        timer         = millis() + 200;
        int16_t roll  = MiniR4.I2C4.MXMotion.getRoll();
        int16_t pitch = MiniR4.I2C4.MXMotion.getPitch();
        int16_t yaw   = MiniR4.I2C4.MXMotion.getYaw();

        Serial.print("Roll: ");
        Serial.print(roll);
        Serial.print(" , Pitch: ");
        Serial.print(pitch);
        Serial.print(" , Yaw: ");
        Serial.println(yaw);
    }
}

void TaskI2CColor(void)
{
    static bool initFlag = false;

    if (!initFlag) {
        bool ret = MiniR4.I2C1.MXColor.begin();

        if (ret) {
            Serial.println("MXColor init success");
            MiniR4.I2C1.MXColor.setGamma(true);
            Serial.println("Set gamma correction as open");

            MiniR4.I2C1.MXColor.setLight(true, true, 0);
            Serial.println("Set fill-light open and in auto adjust mode");
        } else {
            Serial.println("MXColor init failed");
        }
        initFlag = true;
    }

    static uint32_t timer = 0;
    if (millis() >= timer) {
        timer = millis() + 500;
        Serial.println("===============================================================");
        Serial.print("R=");
        Serial.println(MiniR4.I2C1.MXColor.getColor(R));
        Serial.print("G=");
        Serial.println(MiniR4.I2C1.MXColor.getColor(G));
        Serial.print("B=");
        Serial.println(MiniR4.I2C1.MXColor.getColor(B));
        Serial.print("C=");
        Serial.println(MiniR4.I2C1.MXColor.getColor(C));
        Serial.print("M=");
        Serial.println(MiniR4.I2C1.MXColor.getColor(M));
        Serial.print("Y=");
        Serial.println(MiniR4.I2C1.MXColor.getColor(Y));
        Serial.print("K=");
        Serial.println(MiniR4.I2C1.MXColor.getColor(K));
    }
}

void TaskI2CLaser(void)
{
    static bool initFlag = false;

    if (!initFlag) {
        bool ret = MiniR4.I2C2.MXLaser.begin();

        if (ret) {
            Serial.println("MXLaser init success");
        } else {
            Serial.println("MXLaser init failed");
        }
        initFlag = true;
    }

    static uint32_t timer = 0;
    if (millis() >= timer) {
        timer   = millis() + 100;
        int dis = MiniR4.I2C2.MXLaser.getDistance();

        if (dis == 8191) {
            Serial.println("TIMEOUT");
        } else {
            Serial.print("Distance = ");
            Serial.print(dis);
            Serial.println("mm");
        }
    }
}
