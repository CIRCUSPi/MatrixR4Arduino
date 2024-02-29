#include "MiniR4OLED.h"
#include <Wire.h>

MiniR4OLED::MiniR4OLED() {}
MiniR4OLED::~MiniR4OLED() {}

void MiniR4OLED::begin(TwoWire* wire)
{
    _wire = wire;
    _wire->begin();
    _wire->setClock(100000);
}

void MiniR4OLED::fill(SSD1306_COLOR color) {}
void MiniR4OLED::updateScreen(void) {}
void MiniR4OLED::drawPixel(uint8_t x, uint8_t y, SSD1306_COLOR color) {}
char MiniR4OLED::writeChar(char ch, FontDef Font, SSD1306_COLOR color) {}
char MiniR4OLED::writeString(char* str, FontDef Font, SSD1306_COLOR color) {}
void MiniR4OLED::setCursor(uint8_t x, uint8_t y) {}
void MiniR4OLED::line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SSD1306_COLOR color) {}
void MiniR4OLED::drawArc(
    uint8_t x, uint8_t y, uint8_t radius, uint16_t start_angle, uint16_t sweep, SSD1306_COLOR color)
{}
void MiniR4OLED::drawArcWithRadiusLine(
    uint8_t x, uint8_t y, uint8_t radius, uint16_t start_angle, uint16_t sweep, SSD1306_COLOR color)
{}
void MiniR4OLED::drawCircle(uint8_t par_x, uint8_t par_y, uint8_t par_r, SSD1306_COLOR color) {}
void MiniR4OLED::fillCircle(uint8_t par_x, uint8_t par_y, uint8_t par_r, SSD1306_COLOR par_color) {}
void MiniR4OLED::polyline(const SSD1306_VERTEX* par_vertex, uint16_t par_size, SSD1306_COLOR color)
{}
void MiniR4OLED::drawRectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SSD1306_COLOR color)
{}
void MiniR4OLED::fillRectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SSD1306_COLOR color)
{}
void MiniR4OLED::drawBitmap(
    uint8_t x, uint8_t y, const unsigned char* bitmap, uint8_t w, uint8_t h, SSD1306_COLOR color)
{}

void    MiniR4OLED::setContrast(const uint8_t value) {}
void    MiniR4OLED::setDisplayOn(const uint8_t on) {}
uint8_t MiniR4OLED::getDisplayOn() {}
void    MiniR4OLED::reset(void) {}

void                        MiniR4OLED::writeCommand(uint8_t byte) {}
void                        MiniR4OLED::writeData(uint8_t* buffer, size_t buff_size) {}
MiniR4OLED::SSD1306_Error_t MiniR4OLED::fillBuffer(uint8_t* buf, uint32_t len) {}
