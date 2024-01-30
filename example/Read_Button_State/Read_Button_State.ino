#include <MatrixR4.h>

MatrixR4 matrixR4;

void setup(void)
{
    Serial.begin(115200);
    matrixR4.Init();
}

void loop(void)
{
    bool state;
    matrixR4.GetButtonState(1, state);
    Serial.print("Btn1 State: ");
    Serial.println((int)state);

    matrixR4.GetButtonState(2, state);
    Serial.print("Btn2 State: ");
    Serial.println((int)state);

    Serial.println();
}