#include <MatrixMiniR4.h>

MiniR4LED LED;

void setup(void)
{
    Serial.begin(115200);
    LED.begin(7);
}

void loop(void)
{
    TaskLED();
}

void TaskLED(void)
{
    static uint32_t timer = 0;
    static uint8_t  idx   = 0;

    uint32_t colors[] = {0xFF0000, 0x00FF00, 0x0000FF, 0xFFFFFF, 0x000000};
    if (millis() >= timer) {
        timer = millis() + 500;
        LED.setColor(1, colors[idx]);
        if (++idx >= 5) idx = 0;
    }
}
