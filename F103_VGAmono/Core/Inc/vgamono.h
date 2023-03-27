/**
 * original author:  Tilen Majerle<tilen@majerle.eu>
 * modification for STM32f10x: Alexander Lutsai<s.lyra@ya.ru>
 * modification for STM32F103: thanwa (2023)
   ----------------------------------------------------------------------
   	Copyright (C) Alexander Lutsai, 2016
    Copyright (C) Tilen Majerle, 2015
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    any later version.
     
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
   ----------------------------------------------------------------------
 */

#include <stdint.h>
#include "vgafonts.h"
#ifndef INC_VGAMONO_H_
#define INC_VGAMONO_H_

//#define VGA_WIDTH  392
#define VGA_WIDTH  	398
#define VGA_HEIGHT 	298
#define VGA_VBUFFER 300  //Video Buffer height
#define VGA_LBUFFER 50   //Video Buffer width

#define VGA_LBUFFERSIZE 64  //Total display buffer width
#define VGA_FULL		256
#define VGA_HALF		128

#define VGA_WHITE 1
#define VGA_BLACK 0
#define VGA_COLOR uint8_t

extern uint8_t VGA_obuffer[VGA_FULL];
extern uint8_t VGA_buffer[VGA_VBUFFER][VGA_LBUFFER];
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
