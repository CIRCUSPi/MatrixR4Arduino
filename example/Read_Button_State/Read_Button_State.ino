#include <MatrixMiniR4.h>

void setup(void)
{
    Serial.begin(115200);
    mmL.Init();
}

void loop(void)
{
    bool state;
    mmL.GetButtonState(1, state);
    Serial.print("Btn1 State: ");
    Serial.println((int)state);

    mmL.GetButtonState(2, state);
    Serial.print("Btn2 State: ");
    Serial.println((int)state);

    Serial.println();
    delay(200);
}