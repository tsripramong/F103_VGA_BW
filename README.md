VGA monochrome display for STM32F103C8T6 (Blue Pill)
===================================================

A working example for 1-bit(monochrome) VGA display for Blue Pill board using (mainly)HAL API.

Video Mode and Timing
======================
SVGA 800x600 @56 Hz

Pixel frequency: 36MHz 
**(We actually use 18MHz for this project so 1 pixel to display equals to 2 pixels on screen. This results in output resolution at 400x600px.)

Horizontal timing:
==================
Visible Area 800px.
Front porch 24px.
Sync pulse 72px.
Back porch 128px.
Whole Line 1024px.

Vertical timing:
===============
Visible Area 600 lines
Front porch 1 line
Sync pulse 2 lines
Back porch 22 lines
Whole frame 625 lines

Upon actual implementation within the code, due to some inaccuracy in timings, we reduce displayable area to 398x298pixel (using line doubling, which explained later) 

All the settings:
==================
<img Pictures/1bit_VGA_F103_schem.png />
<br />


