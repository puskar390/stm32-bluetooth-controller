// FILE: lcd_lib.c
// DESCRIPTION: Provides a library to access a HD44780-based character LCD module.
// DATE: 12 Apr 2017
// REFERENCE: HD44780.C (Grant Phillips / Revised by Allan Smith)
// REVISED : Puskar Pandey


#include <stdio.h>
#include <string.h>
#include <math.h>
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_gpio.h"


#define RESET 0
#define SET 1

/*......................................constants.....................................*/

/* COMMANDS */
#define CMD_RESET            	0x30     	/*!< Resets display - used in init 3x */
#define CMD_CLEAR            	0x01     	/*!< Clears display */
#define CMD_RETURN_HOME      	0x02     	/*!< Sets DDRAM pointer to 0 */
#define CMD_ENTRY_MODE       	0x04     	/*!< Sets how the pointer is updated after a character write */
#define CMD_DISPLAY          	0x08     	/*!< Display settings */
#define CMD_SHIFT            	0x10     	/*!< Cursor and display movement */
#define CMD_FUNCTION         	0x20     	/*!< Screen type setup */

/* ENTRY_MODE Command parameters */
#define ENTRY_SHIFT_DISP 			0x01	 		/*!< Shift display */
#define ENTRY_SHIFT_CURS 			0x00	 		/*!< Shift cursor */
#define ENTRY_ADDR_INC   			0x02     		//!< Increments pointer 
#define ENTRY_ADDR_DEC   			0x00	 		/*!< Decrements pointer */

/* DISPLAY Command parameters */
#define DISP_ON       				0x04      /*!< Enables the display */
#define DISP_OFF      				0x00      /*!< Disables the display */
#define DISP_CURS_ON  				0x02      /*!< Enables cursor */
#define DISP_CURS_OFF 				0x00      /*!< Disables cursor */
#define DISP_BLINK_ON				0x01      /*!< Enables cursor blinking */
#define DISP_BLINK_OFF  			0x00      /*!< Disables cursor blinking */

/* SHIFT Command parameters */
#define SHIFT_DISPLAY    			0x08      /*!< Shifts the display or shifts the cursor if not set */
#define SHIFT_CURSOR    			0x00      /*!< Shifts the display or shifts the cursor if not set */
#define SHIFT_RIGHT      			0x04      /*!< Shift to the right */
#define SHIFT_LEFT      			0x00      /*!< Shift to the left  */

/* FUNCTION Command parameters */
//#define FUNC_BUS_8BIT  				0x10      /*!< 8 bit bus */
#define FUNC_BUS_4BIT  				0x00      /*!< 4 bit bus */
#define FUNC_LINES_2   				0x08      /*!< 2 lines */
#define FUNC_LINES_1   				0x00      /*!< 1 line */
#define FUNC_FONT_5x10 				0x04      /*!< 5x10 font */
#define FUNC_FONT_5x8  				0x00      /*!< 5x8 font */


/*.....................................................configuration..............................................*/

#define CONF_BUS					FUNC_BUS_4BIT
#define CONF_LINES					FUNC_LINES_2
#define CONF_FONT					FUNC_FONT_5x8

#define DISP_LENGTH						16
#define DISP_ROWS						2
#define CONF_SCROLL_MS				    20

/* HD44780 Data lines - use the same port for all the lines */
#define DATAPORT							GPIOD
#define DATABIT0							GPIO_PIN_0	//not used in 4-bit mode
#define DATABIT1							GPIO_PIN_1	//not used in 4-bit mode
#define DATABIT2							GPIO_PIN_2	//not used in 4-bit mode
#define DATABIT3							GPIO_PIN_3	//not used in 4-bit mode
#define DATABIT4							GPIO_PIN_4
#define DATABIT5							GPIO_PIN_5
#define DATABIT6							GPIO_PIN_6
#define DATABIT7							GPIO_PIN_7

/* HD44780 Control lines - use the same port for all the lines */
#define CONTROLPORT							GPIOD
#define RS_BIT								GPIO_PIN_10
#define RW_BIT								GPIO_PIN_8
#define EN_BIT								GPIO_PIN_9

/*................................................function declaration................................................*/

#define clear()                       	  wr_cmd( CMD_CLEAR )
#define entry( inc_dec, shift )           wr_cmd( ( CMD_ENTRY_MODE | inc_dec | shift ) & 0x07 )
#define display( on_off, cursor, blink )  wr_cmd( ( CMD_DISPLAY | on_off | cursor | blink ) & 0x0F )
#define shift( inc_dec, shift )           wr_cmd( ( CMD_SHIFT | inc_dec | shift ) & 0x1F )
#define function( bus, lines, font )      wr_cmd( ( CMD_FUNCTION | bus | lines | font ) & 0x3F )
#define write_char( c )                   wr_data( c & 0xff )


void wr_hi_nibble(unsigned char data);
void wr_lo_nibble(unsigned char data);
void write(unsigned char data);
void wr_cmd(unsigned char cmd);
void wr_data(unsigned char data);

void Init(void);
void PutChar(unsigned char c);
void GotoXY(unsigned char x, unsigned char y);
void PutStr(char *str);
void ClrScr(void);
void display_shift(void);
void display_cursor(int status);


void config_EN(int flag);
void config_RS(int flag);
void config_RW(int flag);


GPIO_InitTypeDef  GPIO_InitStruct;


void GPIO_SetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
}

void GPIO_ResetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
}


// Name: Init
// Description: Initializes the peripherals for LCD44780
// Parameters: None
// Return: None
void Init(){

	//Enableing periferals for HD44780
	__GPIOD_CLK_ENABLE();
	/* Configure the HD44780 Data lines (DB7 - DB4) as outputs*/
	GPIO_InitStruct.Pin = DATABIT7 | DATABIT6 | DATABIT5 | DATABIT4;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;

	HAL_GPIO_Init(DATAPORT, &GPIO_InitStruct);


	/* Configure the HD44780 Control lines (RS, RW, EN) as outputs*/
	GPIO_InitStruct.Pin = RS_BIT | RW_BIT | EN_BIT;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;

	HAL_GPIO_Init(CONTROLPORT, &GPIO_InitStruct);


	/*Enabling priferal for LCD backlit*/
	 __GPIOA_CLK_ENABLE();

    /*Configure the GPIOA PIN 10 for backlit*/
    GPIO_InitStruct.Pin = (GPIO_PIN_10);
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = 0;

    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* clear control bits */
	config_EN(0);
	config_RS(0);
	config_RW(0);
	
	/* wait initial delay for LCD to settle */
	/* reset procedure - 3 function calls resets the device */
	HAL_Delay(10);
	wr_hi_nibble( CMD_RESET);
	HAL_Delay(10);
	wr_hi_nibble( CMD_RESET);
	HAL_Delay(10);
	wr_hi_nibble( CMD_RESET);

	#if CONF_BUS == FUNC_BUS_4BIT
	/* 4bit interface */
	wr_hi_nibble( CMD_FUNCTION);
	#endif /* CONF_BUS == FUNC_BUS_4BIT */

	/* sets the configured values - can be set again only after reset */
	function(CONF_BUS, CONF_LINES, CONF_FONT);

	/* clear the display */
	clear();

	/* turn the display on with no cursor or blinking */

	display(DISP_ON, DISP_CURS_OFF, DISP_BLINK_OFF);

	/* addr increment, shift cursor */
	entry(ENTRY_ADDR_INC, ENTRY_SHIFT_CURS);
}

// Name: config_EN
// Description: Sets or Resets the EN pins
// Parameters: flag to set or reset EN pin
// Return: None
void config_EN(int flag){
	if(flag){
		HAL_GPIO_WritePin(CONTROLPORT, EN_BIT, SET);
	}
	else{
		HAL_GPIO_WritePin(CONTROLPORT, EN_BIT, RESET);
	}
}

// Name: config_RS
// Description: Sets or Resets the RS pins
// Parameters: flag to set or reset RS pin
// Return: None
void config_RS(int flag){
	if(flag){
		HAL_GPIO_WritePin(CONTROLPORT, RS_BIT, SET);
	}
	else{
		HAL_GPIO_WritePin(CONTROLPORT, RS_BIT, RESET);
	}
}

// Name: config_RW
// Description: Sets or Resets the RW pins
// Parameters: flag to set or reset RW pin
// Return: None
void config_RW(int flag){
	if(flag){
		HAL_GPIO_WritePin(CONTROLPORT, RW_BIT, SET);
	}
	else{
		HAL_GPIO_WritePin(CONTROLPORT, RW_BIT, RESET);
	}
}


/*................................. Function used from the CooCox HD44780 library ............................*/

void wr_cmd(unsigned char cmd) {
	config_RS(0);
	write(cmd);
}

void wr_data(unsigned char data) {
	config_RS(1);
	write(data);
}

#if HD44780_CONF_BUS == HD44780_FUNC_BUS_4BIT

void wr_lo_nibble(unsigned char data) {
	if (data & 0x01) {
		GPIO_SetBits( DATAPORT, DATABIT4);
	} else {
		GPIO_ResetBits( DATAPORT, DATABIT4);
	}
	if (data & 0x02) {
		GPIO_SetBits( DATAPORT, DATABIT5);
	} else {
		GPIO_ResetBits( DATAPORT, DATABIT5);
	}
	if (data & 0x04) {
		GPIO_SetBits( DATAPORT, DATABIT6);
	} else {
		GPIO_ResetBits( DATAPORT, DATABIT6);
	}
	if (data & 0x08) {
		GPIO_SetBits( DATAPORT, DATABIT7);
	} else {
		GPIO_ResetBits( DATAPORT, DATABIT7);
	}

	/* set the EN signal */
	config_EN(1);

	/* wait */
	HAL_Delay(10);

	/* reset the EN signal */
	config_EN(0);
}



/* 4bit bus version */
void write(unsigned char data) {
	/* send the data bits - high nibble first */
	wr_hi_nibble(data);
	wr_lo_nibble(data);
}
#endif /* CONF_BUS == FUNC_BUS_4BIT */


void wr_hi_nibble(unsigned char data) {
	if (data & 0x10) {
		GPIO_SetBits( DATAPORT, DATABIT4);
	} else {
		GPIO_ResetBits( DATAPORT, DATABIT4);
	}
	if (data & 0x20) {
		GPIO_SetBits( DATAPORT, DATABIT5);
	} else {
		GPIO_ResetBits( DATAPORT, DATABIT5);
	}
	if (data & 0x40) {
		GPIO_SetBits( DATAPORT, DATABIT6);
	} else {
		GPIO_ResetBits( DATAPORT, DATABIT6);
	}
	if (data & 0x80) {
		GPIO_SetBits( DATAPORT, DATABIT7);
	} else {
		GPIO_ResetBits( DATAPORT, DATABIT7);
	}

	/* set the EN signal */
	config_EN(1);
	/* wait */
	HAL_Delay(10);

	/* reset the EN signal */
	config_EN(0);
}

/*......................................................................................................*/


// Name: display_cursor
// Description: show and hide the cursor
// Parameters: cursor state
// Return: None
void display_cursor(int status){
	if (status == 1){
		display(DISP_ON, DISP_CURS_ON, DISP_BLINK_ON);
	}
	else{
		display(DISP_ON, DISP_CURS_OFF, DISP_BLINK_OFF);
	}
}

// Name: ClrScr 
// Description: clears screen
// Parameters: None
// Return: None
void ClrScr(void) {
	wr_cmd(CMD_CLEAR);
}

// Name: display_shift
// Description: scroll message on the screen
// Parameters: None
// Return: None
void display_shift(void) {
	wr_cmd(CMD_SHIFT   |SHIFT_DISPLAY   |SHIFT_LEFT);
}

// Name: PutChar
// Description: Prints a character at the current character cursor position
// Parameters: unsigned char to be displayed
// Return: None
void PutChar(unsigned char c) {
	wr_data(c & 0xff);
}

// Name: PutStr
// Description: Prints a string at the current character cursor position
// Parameters: string
// Return: None
void PutStr(char *str) {
	__IO unsigned int i = 0;

	do {
		PutChar(str[i]);
		i++;
	} while (str[i] != '\0');
}

// Name: GotoXY
// Description: sets the porition of the cursor
// Parameters: columns and rows 
// Return: None
void GotoXY(unsigned char x, unsigned char y) {
	unsigned char copy_y = 0;

	if (x > (DISP_LENGTH - 1))
		x = 0;

	if (y > (DISP_ROWS - 1))
		y = 0;

	switch (y) {
	case 0:
		copy_y = 0x80;
		break;
	case 1:
		copy_y = 0xc0;
		break;
	case 2:
		copy_y = 0x94;
		break;
	case 3:
		copy_y = 0xd4;
		break;
	}
	wr_cmd(x + copy_y);
}


