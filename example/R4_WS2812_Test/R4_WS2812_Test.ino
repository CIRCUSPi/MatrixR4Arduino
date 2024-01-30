#define WS2812_PIN        7
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

void setup(void)
{
    Serial.begin(115200);
    pinMode(WS2812_PIN, OUTPUT);
    pinWS2812B_Clr0();
}

void loop(void)
{
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
