#ifndef MINIR4I2C_H
#define MINIR4I2C_H

#include "MMLower.h"
#include "Modules/MiniR4ColorSensorExt.h"
#include "Modules/MiniR4LaserSensorExt.h"
#include "Modules/MiniR4MotionExt.h"

template<uint8_t ID> class MiniR4I2C
{
public:
    MiniR4I2C()
    {
        MXMotion._ch = ID;
        MXColor._ch  = ID;
        MXLaser._ch  = ID;
    }

    MatrixMotion MXMotion;
    MatrixColor  MXColor;
    MatrixLaser  MXLaser;

private:
};

#endif   // MINIR4I2C_H
