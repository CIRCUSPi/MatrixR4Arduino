#ifndef MINIR4OLED_H
#define MINIR4OLED_H

#include "ssd1306_fonts.h"
#include <Arduino.h>
#include <stdint.h>

class MiniR4OLED
{
public:
    MiniR4OLED();
    ~MiniR4OLED();

    typedef enum
    {
        Black = 0x00,
        White = 0x01
    } SSD1306_COLOR;

    typedef enum
    {
        SSD1306_OK  = 0x00,
        SSD1306_ERR = 0x01
    } SSD1306_Error_t;

    typedef struct
    {
        uint16_t CurrentX;
        uint16_t CurrentY;
        uint8_t  Initialized;
        uint8_t  DisplayOn;
    } SSD1306_t;

    typedef struct
    {
        uint8_t x;
        uint8_t y;
    } SSD1306_VERTEX;

    void begin(TwoWire* wire);
    void fill(SSD1306_COLOR color);
    void updateScreen(void);
    void drawPixel(uint8_t x, uint8_t y, SSD1306_COLOR color);
    char writeChar(char ch, FontDef Font, SSD1306_COLOR color);
    char writeString(char* str, FontDef Font, SSD1306_COLOR color);
    void setCursor(uint8_t x, uint8_t y);
    void line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SSD1306_COLOR color);
    void drawArc(
        uint8_t x, uint8_t y, uint8_t radius, uint16_t start_angle, uint16_t sweep,
        SSD1306_COLOR color);
    void drawArcWithRadiusLine(
        uint8_t x, uint8_t y, uint8_t radius, uint16_t start_angle, uint16_t sweep,
        SSD1306_COLOR color);
    void drawCircle(uint8_t par_x, uint8_t par_y, uint8_t par_r, SSD1306_COLOR color);
    void fillCircle(uint8_t par_x, uint8_t par_y, uint8_t par_r, SSD1306_COLOR par_color);
    void polyline(const SSD1306_VERTEX* par_vertex, uint16_t par_size, SSD1306_COLOR color);
    void drawRectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SSD1306_COLOR color);
    void fillRectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SSD1306_COLOR color);
    void drawBitmap(
        uint8_t x, uint8_t y, const unsigned char* bitmap, uint8_t w, uint8_t h,
        SSD1306_COLOR color);

    void    setContrast(const uint8_t value);
    void    setDisplayOn(const uint8_t on);
    uint8_t getDisplayOn();
    void    reset(void);

private:
    TwoWire* _wire;

    void            writeCommand(uint8_t byte);
    void            writeData(uint8_t* buffer, size_t buff_size);
    SSD1306_Error_t fillBuffer(uint8_t* buf, uint32_t len);
};

#endif   // MINIR4OLED_H
