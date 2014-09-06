/******************************************************************************
* File Name          : 4x20lcd.c
* Date First Issued  : 2/13/2014
* Board              : Discovery F4 (F405 or F407)
* Description        : Helper functions for the 4x20 LCD screen
*						https://www.sparkfun.com/products/9568
*******************************************************************************/

#include "bsp_uart.h"
#include "4x20lcd.h"
#include <string.h>
#include "beep_n_lcd.h" // Time delay 

void lcd_init(int uartnumber) {
	// 9600 baud
	bsp_uart_int_init_number(uartnumber, 9600, 4, 128, 0xC0);

	delay_tenth(5);

	lcd_off(uartnumber);
	lcd_on(uartnumber);

	lcd_clear(uartnumber);
	lcd_moveCursor(uartnumber,0, 0);
}

void lcd_clear(int uartnumber) {
	bsp_uart_putc_uartnum(uartnumber, 254);
	bsp_uart_putc_uartnum(uartnumber, 0x01); // clear screen
}

void lcd_on(int uartnumber) {
	bsp_uart_putc_uartnum(uartnumber, 254);
	bsp_uart_putc_uartnum(uartnumber, 0x0C); // remove cursor
}

void lcd_off(int uartnumber) {
	bsp_uart_putc_uartnum(uartnumber, 254);
	bsp_uart_putc_uartnum(uartnumber, 0x08); // turn off screem
}

void lcd_print(int uartnumber, char* p) 
{
	int i;
	for (i = 0; i < LCDLINESIZE; i++)
	{
		if (*p == 0) break;
		bsp_uart_putc_uartnum(uartnumber, *p++);			
	}
}

void lcd_printToLine(int uartnumber, int line, char* p) {
	lcd_moveCursor(uartnumber, line, 0);
	lcd_print(uartnumber, p);
}

void lcd_moveCursor(int uartnumber, int row, int col) {
	bsp_uart_putc_uartnum(uartnumber, 254); // move cursor command

	// determine position
	if (row == 0) {
		bsp_uart_putc_uartnum(uartnumber, 128 + col);
	} else if (row == 1) {
		bsp_uart_putc_uartnum(uartnumber, 192 + col);
	} else if (row == 2) {
		bsp_uart_putc_uartnum(uartnumber, 148 + col);
	} else if (row == 3) {
		bsp_uart_putc_uartnum(uartnumber, 212 + col);
	}
}