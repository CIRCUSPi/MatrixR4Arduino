#ifndef MINIR4I2C_H
#define MINIR4I2C_H

#include "MiniR4LaserSensorExt.h"
#include "MiniR4MotionExt.h"
#include "MiniR4TCS34725.h"

template<uint8_t ID, TwoWire* WIRE> class MiniR4I2C
{
public:
    MiniR4I2C()
    {
        MXMotion._ch = ID;
        MXLaser._ch  = ID;

        MXMotion._pWire = WIRE;
        MXLaser._pWire  = WIRE;
    }

    MatrixMotion      MXMotion;
    MatrixLaser       MXLaser;
    Adafruit_TCS34725 MXColor = Adafruit_TCS34725(
        TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X, TCS34725_ADDRESS, WIRE, ID);

private:
};

#endif   // MINIR4I2C_H
