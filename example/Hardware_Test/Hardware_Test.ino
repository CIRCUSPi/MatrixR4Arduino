#include "Wire.h"
#include <Adafruit_SSD1306.h>
#include <MatrixR4.h>


#define NOTE_C4 261.63
#define NOTE_D4 293.66
#define NOTE_E4 329.63
#define NOTE_F4 349.23
#define NOTE_G4 392.00
#define NOTE_A4 440.00
#define NOTE_B4 493.88
#define NOTE_C5 523.25

#define WS2812_PIN 7
#define BUZZER_PIN 6

#define pinWS2812B_Set1() (digitalWriteFast(WS2812_PIN, 1))
#define pinWS2812B_Clr0() (digitalWriteFast(WS2812_PIN, 0))

R_PORT0_Type* port_table[] = {
    R_PORT0, R_PORT1, R_PORT2, R_PORT3, R_PORT4, R_PORT5, R_PORT6, R_PORT7};
const uint16_t mask_table[] = {
    1 << 0,
    1 << 1,
    1 << 2,
    1 << 3,
    1 << 4,
    1 << 5,
    1 << 6,
    1 << 7,
    1 << 8,
    1 << 9,
    1 << 10,
    1 << 11,
    1 << 12,
    1 << 13,
    1 << 14,
    1 << 15};

inline void digitalWriteFast(uint8_t pin, uint8_t val) __attribute__((always_inline, unused));


SoftwareSerial   softSerial(8, 9);   // rx, tx
MatrixR4         matrixR4(&softSerial);
Adafruit_SSD1306 display(128, 32, &Wire1, -1);

double scale_arr[] = {NOTE_C4, NOTE_D4, NOTE_E4, NOTE_F4, NOTE_G4, NOTE_A4, NOTE_B4, NOTE_C5};

void setup(void)
{
    Serial.begin(115200);

    MatrixR4::RESULT result;

    // 初始化
    // 0. Communication
    result = matrixR4.Init();
    if (result != MatrixR4::RESULT::OK) {
        Serial.print("MatrixR4 Init Failed! Result: ");
        Serial.println((int)result);
        Serial.println("-----------------------------------------");
        while (1) delay(1000);
    }
    Serial.println("MatrixR4 Init Success!");
    Serial.println("-----------------------------------------");

    result = matrixR4.SetButtonInit(1, MatrixR4::BUTTON_ECHO_MODE::ACTIVE, 1000, 300);
    result = matrixR4.SetButtonInit(2, MatrixR4::BUTTON_ECHO_MODE::ACTIVE, 1000, 300);
    while (result != MatrixR4::RESULT::OK) {
        Serial.print("SetButtonInit Failed! Result: ");
        Serial.println((int)result);
        Serial.println("-----------------------------------------");
        while (1) delay(1000);
    }
    Serial.println("SetButtonInit Success!");
    Serial.println("-----------------------------------------");

    SetAllMotor(0, MatrixR4::DIR::FORWARD);
    SetAllServo(90);
    matrixR4.onBtnChg([](uint8_t num, MatrixR4::BTN_STATE newState) {
        if (num == 1) {
            if (newState == MatrixR4::BTN_STATE::F_EDGE) {
                SetAllMotor(1000, MatrixR4::DIR::FORWARD);
                SetAllServo(180);
            } else if (newState == MatrixR4::BTN_STATE::R_EDGE) {
                SetAllMotor(0, MatrixR4::DIR::FORWARD);
                SetAllServo(90);
                ShowEncoderValue();
                Serial.println("-----------------------------------------");
            }
        } else if (num == 2) {
            if (newState == MatrixR4::BTN_STATE::F_EDGE) {
                SetAllMotor(1000, MatrixR4::DIR::REVERSE);
                SetAllServo(0);
            } else if (newState == MatrixR4::BTN_STATE::R_EDGE) {
                SetAllMotor(0, MatrixR4::DIR::REVERSE);
                SetAllServo(90);
                ShowEncoderValue();
                Serial.println("-----------------------------------------");
            }
        }
    });

    result = matrixR4.RunAutoQC();
    if (result != MatrixR4::RESULT::OK) {
        Serial.print("RunAutoQC Failed! Result: ");
        Serial.println((int)result);
        switch (result) {
        case MatrixR4::RESULT::ERROR_QC_OLED:
        {
            Serial.println("ERROR OLED");
        } break;
        case MatrixR4::RESULT::ERROR_QC_I2C_MUX:
        {
            Serial.println("ERROR I2C MUX");
        } break;
        case MatrixR4::RESULT::ERROR_QC_IMU:
        {
            Serial.println("ERROR IMU");
        } break;
        default: break;
        }
        Serial.println("-----------------------------------------");
        while (1) delay(1000);
    }
    Serial.println("RunAutoQC Success!");
    Serial.println("-----------------------------------------");

    result = matrixR4.SetPowerParam(8.4f, 7.2f, 7.4f);   // 2 Cell
    if (result != MatrixR4::RESULT::OK) {
        Serial.print("SetPowerParam Failed! Result: ");
        Serial.println((int)result);
        Serial.println("-----------------------------------------");
        while (1) delay(1000);
    }
    Serial.println("SetPowerParam Success!");
    Serial.println("-----------------------------------------");

    float voltage = 0.0f, voltPercent = 0.0f;
    result = matrixR4.GetPowerInfo(voltage, voltPercent);
    if (result != MatrixR4::RESULT::OK) {
        Serial.print("GetPowerVoltage Failed! Result: ");
        Serial.println((int)result);
        Serial.println("-----------------------------------------");
        while (1) delay(1000);
    }
    Serial.print("Voltage: ");
    Serial.print(voltage);
    Serial.print("V, Percent: ");
    Serial.print(voltPercent);
    Serial.println("%");
    Serial.println("GetPowerInfo Success!");
    Serial.println("-----------------------------------------");

    result = matrixR4.SetStateLED(255, 0xFF0000);
    delay(500);
    result = matrixR4.SetStateLED(255, 0x00FF00);
    delay(500);
    result = matrixR4.SetStateLED(255, 0x0000FF);
    delay(500);
    result = matrixR4.SetStateLED(255, 0x000000);
    if (result != MatrixR4::RESULT::OK) {
        Serial.print("SetStateLED Failed! Result: ");
        Serial.println((int)result);
        Serial.println("-----------------------------------------");
        while (1) delay(1000);
    }
    Serial.println("SetStateLED Success!");
    Serial.println("-----------------------------------------");

    // Test R4 Buzzer
    for (uint8_t i = 0; i < 8; i++) {
        tone(BUZZER_PIN, scale_arr[i]);
        delay(200);
    }
    noTone(BUZZER_PIN);

    // Test WS2812
    pinMode(WS2812_PIN, OUTPUT);
    pinWS2812B_Clr0();
    SetWs2812(0xFF0000, 0xFF0000);
    delay(500);
    SetWs2812(0x00FF00, 0x00FF00);
    delay(500);
    SetWs2812(0x0000FF, 0x0000FF);
    delay(500);
    SetWs2812(0xFFFFFF, 0xFFFFFF);
    delay(500);
    SetWs2812(0x00, 0x00);
    delay(500);

    // Test SSD1306
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println(F("OLED allocation failed"));
        for (;;)
            ;
    }
    testdrawchar();

    /* 待測項目
    1. Button 1~2
    2. Motor 1~4
    3. Encoder 1~4
    4. Servo 1~4
    5. IMU
    6. I2C MUX
    7. OLED
    8. Button Reset
    9. Power
    10.STM WS2812
    11.R4 WS2812
    12.R4 Buzzer
    13.R4 OLED
    14.R4 I2C MUX
    */
}

void loop(void)
{
    matrixR4.loop();
}

void SetAllMotor(uint16_t speed, MatrixR4::DIR dir)
{
    for (uint8_t i = 0; i < 4; i++) {
        MatrixR4::RESULT result = matrixR4.SetDCMotorSpeed(i + 1, speed, dir);
        if (result != MatrixR4::RESULT::OK) {
            Serial.print("SetDCMotorSpeed Failed! Result: ");
            Serial.println((int)result);
        }
    }
}

void SetAllServo(uint16_t angle)
{
    for (uint8_t i = 0; i < 4; i++) {
        MatrixR4::RESULT result = matrixR4.SetServoAngle(i + 1, angle);
        if (result != MatrixR4::RESULT::OK) {
            Serial.print("SetServoAngle Failed! Result: ");
            Serial.println((int)result);
        }
    }
}

void ShowEncoderValue(void)
{
    int16_t          encoderValue[4];
    MatrixR4::RESULT result = matrixR4.GetAllEncoderCounter(encoderValue);
    if (result != MatrixR4::RESULT::OK) {
        Serial.print("GetEncoderCounter Failed! Result: ");
        Serial.println((int)result);
    }
    for (uint8_t i = 0; i < 4; i++) {
        Serial.print("Encoder Counter");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.println(encoderValue[i]);
    }
    Serial.println();
}

inline void digitalWriteFast(uint8_t pin, uint8_t val)
{
    if (val) {
        port_table[g_pin_cfg[pin].pin >> 8]->POSR = mask_table[g_pin_cfg[pin].pin & 0xff];
    } else {
        port_table[g_pin_cfg[pin].pin >> 8]->PORR = mask_table[g_pin_cfg[pin].pin & 0xff];
    }
}

void Process_WS2812B_Protocol(uint8_t r, uint8_t g, uint8_t b)
{
    volatile byte i, tempByte, m_Byte_R, m_Byte_G, m_Byte_B;

    m_Byte_R = r;
    m_Byte_G = g;
    m_Byte_B = b;

    noInterrupts();

    // Color: G
    for (i = 0; i < 8; i++) {
        pinWS2812B_Set1();

        if ((m_Byte_G & 0x80) == 0) {
            // asm("nop;");
            m_Byte_G <<= 1;
            tempByte <<= 1;
            pinWS2812B_Clr0();
            tempByte >>= 1;
            tempByte <<= 1;
        } else {
            for (uint32_t i = 0; i < 5; i++) {
                asm("nop;");
            }
            m_Byte_G <<= 1;
            pinWS2812B_Clr0();
        }
    }

    // Color: R
    for (i = 0; i < 8; i++) {
        pinWS2812B_Set1();

        if ((m_Byte_R & 0x80) == 0) {
            // asm("nop;");
            m_Byte_R <<= 1;
            tempByte <<= 1;
            pinWS2812B_Clr0();
            tempByte >>= 1;
            tempByte <<= 1;
        } else {
            for (uint32_t i = 0; i < 5; i++) {
                asm("nop;");
            }
            m_Byte_R <<= 1;
            pinWS2812B_Clr0();
        }
    }

    // Color: B
    for (i = 0; i < 8; i++) {
        pinWS2812B_Set1();

        if ((m_Byte_B & 0x80) == 0) {
            // asm("nop;");
            m_Byte_B <<= 1;
            tempByte <<= 1;
            pinWS2812B_Clr0();
            tempByte >>= 1;
            tempByte <<= 1;
        } else {
            for (uint32_t i = 0; i < 5; i++) {
                asm("nop;");
            }
            m_Byte_B <<= 1;
            pinWS2812B_Clr0();
        }
    }

    interrupts();
}

void SetWs2812(uint32_t led1, uint32_t led2)
{
    uint8_t r, g, b;
    r = ((led1 >> 16) & 0xFF);
    g = ((led1 >> 8) & 0xFF);
    b = (led1 & 0xFF);
    Process_WS2812B_Protocol(r, g, b);
    r = ((led2 >> 16) & 0xFF);
    g = ((led2 >> 8) & 0xFF);
    b = (led2 & 0xFF);
    Process_WS2812B_Protocol(r, g, b);
    delayMicroseconds(50);
}

void testdrawchar(void)
{
    display.clearDisplay();

    display.setTextSize(1);                // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE);   // Draw white text
    display.setCursor(0, 0);               // Start at top-left corner
    display.cp437(true);                   // Use full 256 char 'Code Page 437' font

    // Not all the characters will fit on the display. This is normal.
    // Library will draw what it can and the rest will be clipped.
    for (int16_t i = 0; i < 256; i++) {
        if (i == '\n')
            display.write(' ');
        else
            display.write(i);
    }

    display.display();
}
