# VGA monochrome display for STM32F103C8T6 (Blue Pill)

A working example for 1-bit(monochrome) VGA display for Blue Pill board using (mainly)HAL API.

[![VIDEO](https://img.youtube.com/vi/H0SbCmFXLQI/0.jpg)](https://www.youtube.com/watch?v=H0SbCmFXLQI)


## Video Mode and Timing
SVGA 800x600 @56 Hz

Pixel frequency: 36MHz

*(We actually use 18MHz for this project so 1 pixel to display equals to 2 pixels on screen. This results in output resolution at 400x600px.)*

### Horizontal timing:

Visible Area 800px. <br />
Front porch 24px. <br />
Sync pulse 72px. <br />
Back porch 128px. <br />
Whole Line 1024px.

### Vertical timing:

Visible Area 600 lines. <br />
Front porch 1 line. <br />
Sync pulse 2 lines. <br />
Back porch 22 lines. <br />
Whole frame 625 lines.

Upon actual implementation within the code, due to some inaccuracy in timings, we reduce displayable area to 398x298pixel (using line doubling, which explained later) 


## Circuit diagram:

We use STM32F103C8T6 (Blue pill) in this project, you might substitute with CBT6 but not C6T6 as its RAM is too small for this project.

TIM1 and TIM2 are used to generate H-sync and V-sync signal for VGA output. TIM4 is used to generate a delay time period before start picture signal generated using SPI1, which is connected to pin 2 of VGA port (GREEN signal). You may connect PA7 (SPI1_MOSI) to pin1,2, and 3 for displaying in black&white instead of green&black. All resistors shown in the diagram can be anything in the range of 100-270 ohms to give a bright image.

Two switches (MOVE RIGHT/MOVE LEFT) are not mandatory and can be omitted.

![](Pictures/1bit_VGA_F103_schem.png)
![](Pictures/F103C8T6_pinassignment.png)

## Device Configuration Tools settings:

This project uses STM32CubeIDE and internal Device Configuration Tools (STM32CubeMX) to generate start-up code.

Since Blue Pill board is not designed by ST, we have to set external clock and serial wire debug port manually. Here, we use only HSE that Blue pill also provide us with 8 MHz HSE.

![](Pictures/F103C8T6_rcc.png)
![](Pictures/F103C8T6_sys.png)

The VGA mode we use requires 36MHz pixel clock. (800x600px@56Hz) Thus we set the SYSCLK to 72MHz

![](Pictures/F103C8T6_clock.png)

TIM1 is used for generate H-sync signal. We set the Prescaler to 1. So it halves the input clock, which is 72MHz down to 36MHz for the pixel frequency.

One scan line requires 1024 pixel clocks, so the Counter Period is 1023. 

We use PWM channel 1 to generate H-sync signal, with pulse is 72 pixel clock wide.

![](Pictures/F103C8T6_tim1.png)

TIM2 is for V-sync signal generation. Here, we use overflow signal from TIM1 to trig TIM1 to count up. According to the manual for F103 series, this trigger source is routed via ITR0. 

The time perioud of V-sync is equal to 625 scan lines so we set Counter Period to 624. PWM channel1 of TIM2 is used to generate V-sync signal which the pulse width of 2.

![](Pictures/F103C8T6_timers.png)
![](Pictures/F103C8T6_tim2.png)

TIM2 is used for triggering the start of pixel output for each frame. We set the Prescaler to 1 so the time base of TIM2 is 36MHz, the same that we used for TIM1. However, we use TIM2 to trigger the counting reset for TIM4, so TIM4 will start counting   at the same time a new frame start.

Within the code, we discard the first interrupt triggered by TIM4, which will generated when TIM4 receives a reset signal. We then use the second interrupt to start transmit pixel data via SPI1.

Counter Period shown in picture below is not used in the program. We set the value using VGA_update() function (main.c) by setting value directly to TIM4->ARR.

![](Pictures/F103C8T6_tim4.png)

We use SPI1 to generate picture signal. We set SPI mode to Transmit Only Master and we only use SPI1_MOSI pin as picture signal. We set Prescaler to 4, which is the highest possible for STM32F103C8. This results in 18Mbits/s baud rate, which is half of actual 800x600px@56Hz pixel clock. So, one pixel generated here means two pixels displayed on screen. To keep the aspect ratio, we double the scan line by displaying eash scan line twice. So the actual output will be 400x300px.

We start transmit data to SPI1 using DMA once each frame starts and stop the DMA once the whole frame is sent. So, we enable DMA1 channel3 for SPI1_TX and set it to circular mode and data width is one byte. By doing so we can use HAL_SPI_TxHalfCpltCallback() and HAL_SPI_TxCpltCallback() to prepare the next section of screen image to be sent by SPI1.

![](Pictures/F103C8T6_spi1_parameters.png)
![](Pictures/F103C8T6_spi1_dma.png)

TIM2 interrupt is used in the program to reset the SPI1_DMA trigger TIM4 interrupt is for starting the DMA transfer for SPI1. 

![](Pictures/F103C8T6_nvic.png)

Finally, here we set PB8 and PB9 as External Interrupt Mode (rising edge trigger). Using switches connected to ground, we use them to change preVoffset value to adjust the picture location on screen. You may omitted this part and just change preVoffset value within the program code to make the horizontal adjustment yourself.

![](Pictures/F103C8T6_gpio.png)