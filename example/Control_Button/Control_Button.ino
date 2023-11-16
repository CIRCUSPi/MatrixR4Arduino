#include <MatrixR4.h>

#define MODE 2

SoftwareSerial softSerial(8, 9);   // rx, tx
MatrixR4       matrixR4(&softSerial);

void setup(void)
{
    Serial.begin(115200);

    MatrixR4::RESULT result;

    // 初始化
    result = matrixR4.Init();
    if (result != MatrixR4::RESULT::OK) {
        Serial.print("MatrixR4 Init Failed! Result: ");
        Serial.println((int)result);

        while (1) {
            delay(1000);
        }
    }
    Serial.println("MatrixR4 Init Success!");

    // 按鈕初始化
    MatrixR4::BUTTON_ECHO_MODE buttonEchoMode;

#if MODE == 1
    buttonEchoMode = MatrixR4::BUTTON_ECHO_MODE::PASSIVE;
#else
    buttonEchoMode = MatrixR4::BUTTON_ECHO_MODE::ACTIVE;
#endif

    result = matrixR4.SetButtonInit(1, buttonEchoMode, 100, 25);
    result = matrixR4.SetButtonInit(2, buttonEchoMode, 2000, 100);
    while (result != MatrixR4::RESULT::OK) {
        Serial.print("SetButtonInit Failed! Result: ");
        Serial.println((int)result);

        while (1) {
            delay(1000);
        }
    }
    Serial.println("SetButtonInit Success!");

#if MODE == 1
#else
    buttonEchoMode = MatrixR4::BUTTON_ECHO_MODE::ACTIVE;
    matrixR4.onBtnChg([](uint8_t num, MatrixR4::BTN_STATE newState) {
        Serial.print("Btn");
        Serial.print(num);
        Serial.print(" State: ");
        Serial.println((int)newState);
    });
#endif
}

void loop(void)
{

#if MODE == 1
    // 取得按鈕狀態
    MatrixR4::RESULT result;
    bool             buttonState;
    result = matrixR4.GetButtonState(1, buttonState);
    if (result != MatrixR4::RESULT::OK) {
        Serial.print("GetButtonState Failed! Result: ");
        Serial.println((int)result);
    } else {
        Serial.print("Btn1 State: ");
        Serial.println((int)buttonState);
    }

    result = matrixR4.GetButtonState(2, buttonState);
    if (result != MatrixR4::RESULT::OK) {
        Serial.print("GetButtonState Failed! Result: ");
        Serial.println((int)result);
    } else {
        Serial.print("Btn2 State: ");
        Serial.println((int)buttonState);
    }
    Serial.println();
#endif
    matrixR4.loop();
}