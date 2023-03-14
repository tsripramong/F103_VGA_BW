/*
 * vgamono.h
 *
 *  Created on: Feb 16, 2023
 *      Author: thanwa
 */
#include <stdint.h>
#include "vgafonts.h"
#ifndef INC_VGAMONO_H_
#define INC_VGAMONO_H_

//#define VGA_WIDTH  392
#define VGA_WIDTH  400
#define VGA_HEIGHT 298
#define VGA_VBUFFER 300

#define VGA_WHITE 1
#define VGA_BLACK 0
#define VGA_COLOR uint8_t

#define VGA_offsetX 0 //step by eight pixel (16 real pixel)
#define VGA_offsetY 6

extern uint8_t VGA_obuffer[4][64];
extern uint8_t VGA_buffer[VGA_VBUFFER][50];
typedef struct {
    uint16_t CurrentX;
    uint16_t CurrentY;
} VGA_t;

void ClearScreen(VGA_COLOR color);
void DrawPixel(int16_t x, int16_t y, VGA_COLOR color);
void DrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, VGA_COLOR c);
char WriteChar(char ch, FontDef Font, VGA_COLOR color);
char WriteString(char* str, FontDef Font, VGA_COLOR color);
void SetCursor(int16_t x, int16_t y);
void DrawArc(int16_t x, int16_t y, int16_t radius, int16_t start_angle, int16_t sweep, VGA_COLOR color);
void DrawCircle(int16_t par_x,int16_t par_y,int16_t par_r,VGA_COLOR par_color);
void DrawRectangle(int16_t x1, int16_t y1, int16_t x2, int16_t y2, VGA_COLOR color);
void FillRectangle(int16_t x1, int16_t y1, int16_t x2, int16_t y2, VGA_COLOR color);
void FillCircle(int16_t x0, int16_t y0, int16_t r, VGA_COLOR c);
#endif /* INC_VGAMONO_H_ */
