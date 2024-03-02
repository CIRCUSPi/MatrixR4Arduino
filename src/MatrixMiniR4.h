/**
 * @file MatrixMiniR4.h
 * @author Zack Huang (zackhuang0513@gmail.com)
 * @brief Matrix Mini Arduino Library
 * @version 1.0.0
 * @date 2024-02-25
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef MATRIXMINIR4_H
#define MATRIXMINIR4_H

#include "Modules/MiniR4BTN.h"
#include "Modules/MiniR4Buzzer.h"
#include "Modules/MiniR4DC.h"
#include "Modules/MiniR4ENC.h"
#include "Modules/MiniR4LED.h"
#include "Modules/MiniR4Motion.h"
#include "Modules/MiniR4OLED.h"
#include "Modules/MiniR4RC.h"

#include <WiFiS3.h>
#include <Wire.h>

#define MATRIXMINIR4_OLED_ADDRESS 0x3D

class MatrixMiniR4
{
public:
    MatrixMiniR4();
    bool begin();

    // DC Motor
    MiniR4DC<1> M1;
    MiniR4DC<2> M2;
    MiniR4DC<3> M3;
    MiniR4DC<4> M4;

    // Encoder
    MiniR4ENC<1> ENC1;
    MiniR4ENC<2> ENC2;
    MiniR4ENC<3> ENC3;
    MiniR4ENC<4> ENC4;

    // Servo
    MiniR4RC<1> RC1;
    MiniR4RC<2> RC2;
    MiniR4RC<3> RC3;
    MiniR4RC<4> RC4;

    // Button
    MiniR4BTN<1> BTN1;
    MiniR4BTN<2> BTN2;

    // RGB LED (GPIO 7)
    MiniR4LED LED;

    // Motion
    MiniR4Motion Motion;

    // Buzzer
    MiniR4BUZZER Buzzer;

    // OLED
    Adafruit_SSD1306 OLED;

    CWifi WiFi;

private:
};

extern MatrixMiniR4 MiniR4;

#endif   // MATRIXMINIR4_H