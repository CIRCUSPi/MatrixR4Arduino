#ifndef MINIR4MOTION_H
#define MINIR4MOTION_H

#include "MMLower.h"

class MiniR4Motion
{
public:
    MiniR4Motion() {}

    enum class AxisType
    {
        X,
        Y,
        Z
    };

    double getGyro(AxisType axis)
    {
        double x = 0, y = 0, z = 0;
        mmL.GetIMUGyro(x, y, z);

        if (axis == AxisType::X)
            return x;
        else if (axis == AxisType::Y)
            return y;
        else if (axis == AxisType::Z)
            return z;
        else
            return 0;
    }

    double getAccel(AxisType axis)
    {
        double x = 0, y = 0, z = 0;
        mmL.GetIMUAcc(x, y, z);

        if (axis == AxisType::X)
            return x;
        else if (axis == AxisType::Y)
            return y;
        else if (axis == AxisType::Z)
            return z;
        else
            return 0;
    }

private:
};

#endif   // MINIR4MOTION_H
