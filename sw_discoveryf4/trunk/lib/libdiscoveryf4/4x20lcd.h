/******************************************************************************
* File Name          : 4x20lcd.h
* Date First Issued  : 2/13/2014
* Board              : Discovery F4 (F405 or F407)
* Description        : Helper functions for the 4x20 LCD screen
*						https://www.sparkfun.com/products/9568
*******************************************************************************/
#ifndef __4X20LCD
#define __4X20LCD

/*	LCD Line Size  */
#define LCDLINESIZE 20
#define LCDROWSIZE 4
#define LCD_BACKLIGHT_LEVEL 70

// initializes the lcd screen
void lcd_init(int uartnumber);

// clears the lcd screen
void lcd_clear(int uartnumber);

// turn the thing on/off
void lcd_on(int uartnumber);
void lcd_off(int uartnumber);

// write a message (limit one line) to the lcd board at the current position
void lcd_print(int uartnumber, char* p);

// prints a message to the specified line
void lcd_printToLine(int uartnumber, int line, char* p);

// sets the cursor to the row and column specified. Both are base 0
void lcd_moveCursor(int uartnumber, int row, int col);

#endif 