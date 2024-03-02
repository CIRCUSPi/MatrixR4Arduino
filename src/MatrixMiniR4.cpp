/**
 * @file MatrixMiniR4.cpp
 * @author Zack Huang (zackhuang0513@gmail.com)
 * @brief Matrix Mini Arduino Library
 * @version 1.0.0
 * @date 2024-02-25
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "MatrixMiniR4.h"
#include "Modules/MMLower.h"

MatrixMiniR4::MatrixMiniR4() {}

bool MatrixMiniR4::begin()
{
    MMLower::RESULT result = mmL.Init();

    LED.begin(7);
    Buzzer.begin(6);

    M1.begin();
    M2.begin();
    M3.begin();
    M4.begin();

    RC1.begin();
    RC2.begin();
    RC3.begin();
    RC4.begin();

    OLED = Adafruit_SSD1306(128, 32, &Wire1, -1);
    OLED.begin(SSD1306_SWITCHCAPVCC, MATRIXMINIR4_OLED_ADDRESS);
    OLED.clearDisplay();
    OLED.display();

    return (result == MMLower::RESULT::OK);
}

MatrixMiniR4 MiniR4;
