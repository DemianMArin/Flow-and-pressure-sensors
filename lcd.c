/*
 * lcd.c
 *
 *
 *      Author: rahu7p
 */

#include "main.h"
#include "lcd.h"

//Caracter definido por usuario para cargar en la memoria CGRAM del LCD
const char UserFont[8][8] =
{
		{ 0x11, 0x0A, 0x04, 0x1B, 0x11, 0x11, 0x11, 0x0E },
		{ 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10 },
		{ 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18 },
		{ 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C },
		{ 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E },
		{ 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F },
		{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
		{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }
};

//Funcion que inicializa el LCD a 4 bits
void LCD_Init(void){
	char const *p;

  /* ****************************************************** */
  /* Configurar los pines del Puerto B para las lineas:
   * RW, RS, EN, D4-D7 del LCD
   * como general purpose output push-pull and 50 MHz speed */

	//RS B9
	GPIOB->CRH &= ~GPIO_CRH_CNF9;
	GPIOB->CRH |= GPIO_CRH_MODE9;

	//RW B13
	GPIOB->CRH &= ~GPIO_CRH_CNF13;
	GPIOB->CRH |= GPIO_CRH_MODE13;

	//EN B2
	GPIOB->CRL &= ~GPIO_CRL_CNF2;
	GPIOB->CRL |= GPIO_CRL_MODE2;

	//D4 B6
	GPIOB->CRL &= ~GPIO_CRL_CNF6;
	GPIOB->CRL |= GPIO_CRL_MODE6;

	//D5 B11
	GPIOB->CRH &= ~GPIO_CRH_CNF11;
	GPIOB->CRH |= GPIO_CRH_MODE11;

	//D6 B12
	GPIOB->CRH &= ~GPIO_CRH_CNF12;
	GPIOB->CRH |= GPIO_CRH_MODE12;

	//D7 B8
	GPIOB->CRH &= ~GPIO_CRH_CNF8;
	GPIOB->CRH |= GPIO_CRH_MODE8;
	/* ****************************************************** */

	GPIOB->BSRR	 =	1U << LCD_D4_PIN_HIGH
			|	1U << LCD_D5_PIN_HIGH
			|	1U << LCD_D6_PIN_LOW
			|	1U << LCD_D7_PIN_LOW;
	HAL_Delay(15);

	GPIOB->BSRR	 =	1U << LCD_D4_PIN_HIGH
			|	1U << LCD_D5_PIN_HIGH
			|	1U << LCD_D6_PIN_LOW
			|	1U << LCD_D7_PIN_LOW;
	LCD_Pulse_EN( );
	HAL_Delay(5);//					deberia ser un delay de 4.1ms

	GPIOB->BSRR	 =	1U << LCD_D4_PIN_HIGH
			|	1U << LCD_D5_PIN_HIGH
			|	1U << LCD_D6_PIN_LOW
			|	1U << LCD_D7_PIN_LOW;
	LCD_Pulse_EN( );
	HAL_Delay(1);//					deberia ser un delay de 100us

	GPIOB->BSRR	 =	1U << LCD_D4_PIN_HIGH
			|	1U << LCD_D5_PIN_HIGH
			|	1U << LCD_D6_PIN_LOW
			|	1U << LCD_D7_PIN_LOW;
	LCD_Pulse_EN( );

	while( LCD_Busy( ) );//				espera a que el LCD este operativo
	GPIOB->BSRR	 =	1U << LCD_D4_PIN_LOW
			|	1U << LCD_D5_PIN_HIGH
			|	1U << LCD_D6_PIN_LOW
			|	1U << LCD_D7_PIN_LOW;
	LCD_Pulse_EN( );

	while( LCD_Busy( ) );//				espera a que se complete
	LCD_Write_Cmd( 0x28U );//			establecemos LCD como: datos 4-bit, #lineas=2, font=5x7 dots
	LCD_Write_Cmd( 0x0CU );//			enciende el LCD sin cursor
	LCD_Write_Cmd( 0x06U );//			inicializa cursor

	//Cargamos el caracter definido por el usuario en la CGRAM
	LCD_Write_Cmd( 0x40 );//			establece la direccion CGRAM desde 0
	p = &UserFont[0][0];

	for( int i = 0; i < sizeof( UserFont ); i++, p++ )
		LCD_Put_Char( *p );

	LCD_Write_Cmd( 0x80 );
}

//Funcion que genera un strobe en el LCD
void LCD_Out_Data4(unsigned char val){
	if( ( val & 0x01U ) == 0x01U )//			Bit[0]
		GPIOB->BSRR	=	1U << LCD_D4_PIN_HIGH;
	else
		GPIOB->BSRR	=	1U << LCD_D4_PIN_LOW;

	if( ( val & 0x02U ) == 0x02U )//			Bit[1]
		GPIOB->BSRR	=	1U << LCD_D5_PIN_HIGH;
	else
		GPIOB->BSRR	=	1U << LCD_D5_PIN_LOW;

	if( ( val & 0x04U ) == 0x04U )//			Bit[2]
		GPIOB->BSRR	=	1U << LCD_D6_PIN_HIGH;
	else
		GPIOB->BSRR	=	1U << LCD_D6_PIN_LOW;

	if( ( val & 0x08U ) == 0x08U )//			Bit[3]
		GPIOB->BSRR	=	1U << LCD_D7_PIN_HIGH;
	else
		GPIOB->BSRR	=	1U << LCD_D7_PIN_LOW;
}

//Funcion que escribe 1 byte de datos en el LCD
void LCD_Write_Byte(unsigned char val){
	LCD_Out_Data4( ( val >> 4 ) & 0x0FU );
	LCD_Pulse_EN( );

	LCD_Out_Data4( val & 0x0FU );
	LCD_Pulse_EN( );

	while( LCD_Busy( ) );
}

//Funcion que escribe un comando en el LCD
void LCD_Write_Cmd(unsigned char val){
	GPIOB->BSRR	=	1U << LCD_RS_PIN_LOW;//		RS=0 (seleccion de comando)
	LCD_Write_Byte( val );
}

//Escribe un caracter ASCII en el LCD
void LCD_Put_Char(unsigned char c){
	GPIOB->BSRR	=	1U << LCD_RS_PIN_HIGH;//	RS=1 (seleccion de caracteres)
	LCD_Write_Byte( c );
}

//Funcion que establece el cursor en una posicion de la pantalla del LCD
void LCD_Set_Cursor(unsigned char line, unsigned char column){
	unsigned char address;
	if( column != 0 )
		column--;
	if( line != 0 )
		line--;
	address = ( line * 40 ) + column;
	address = 0x80U + ( address & 0x7FU );
	LCD_Write_Cmd( address );
}

//Funcion que envia una cadena de caracteres ASCII al LCD
void LCD_Put_Str(char* str){
	for( int i = 0; i < 16 && str[ i ] != 0; i++ )
		LCD_Put_Char( str[ i ] );//			envia 1 byte al LCD
}

//Funcion que envia un caracter numerico al LCD
void LCD_Put_Num(int num){
	int p;
	int f = 0;
	char ch[ 5 ];

	for( int i = 0; i < 5; i++ ){
		p = 1;
		for( int j = 4 - i; j > 0; j-- )
			p = p * 10;
		ch[ i ] = ( num / p );
		if( num >= p && !f )
			f = 1;
		num = num - ch[ i ] * p;
		ch[ i ] = ch[ i ] + 48;
		if( f )
			LCD_Put_Char( ch[ i ] );
	}
}

//Funcion que provoca tiempos de espera en el LCD
char LCD_Busy(void){
  /* ***************************************************** */
  /* Configurar la linea D7 del LCD como:
   * floating input                                        */
	GPIOB->CRH &= ~GPIO_CRH_CNF8_1 & ~GPIO_CRH_MODE8;
	GPIOB->CRH |= GPIO_CRH_CNF8_0;

  /* ***************************************************** */

	GPIOB->BSRR	 =	1U << LCD_RS_PIN_LOW
			|	1U << LCD_RW_PIN_HIGH
			|	1U << LCD_EN_PIN_HIGH;
	HAL_Delay(1);//					          deberia de ser un delay of 100us

  /* ***************************************************** */
  if( GPIOB->BSRR & GPIO_BSRR_BS8 ){//					                  if D7 is set, then
  /* ***************************************************** */
		GPIOB->BSRR	 =	1U << LCD_RW_PIN_LOW
				|	1U << LCD_EN_PIN_LOW;

    /* ***************************************************** */
    /* Configurar la linea D7 del LCD como:
     * general purpose output and 50 MHz speed               */
		GPIOB->CRH &= ~GPIO_CRH_CNF8;
		GPIOB->CRH |= GPIO_CRH_MODE8;

    /* ***************************************************** */
		return 1;
	} else {
		GPIOB->BSRR	 =	1U << LCD_RW_PIN_LOW
				|	1U << LCD_EN_PIN_LOW;

    /* ***************************************************** */
    /* Configurar la linea D7 del LCD como:
     * general purpose output and 50 MHz speed               */
		GPIOB->CRH &= ~GPIO_CRH_CNF8;
		GPIOB->CRH |= GPIO_CRH_MODE8;

    /* ***************************************************** */
		return 0;
	}
}

//Funcion que genera un pulso en el pin EN del LCD
void LCD_Pulse_EN(void){
	GPIOB->BSRR	=	1U << LCD_EN_PIN_HIGH;//		habilita pin EN ON
	HAL_Delay(1);//							deberia de ser un delay de 50us
	GPIOB->BSRR	=	1U << LCD_EN_PIN_LOW;//			habilita pin EN OFF
}

/*
 * Funcion que muestra un caracter grafico en el LCD
 * en 'value' el valor de su posicion en CGRAM y
 * en 'size' especificamos su tamaño
 */
void LCD_BarGraphic(int value, int size){
	value = value * size / 20;//					matriz de 5x8 pixeles
	for( int i = 0; i < size; i++ ){
		if( value > 5 ){
			LCD_Put_Char( 0x05U );
			value -= 5;
		} else {
			LCD_Put_Char( value );
			break;
		}
	}
}

/*
 * Funcion que muestra un caracter grafico en el LCD
 * especificando la posicion pos_x horizontal de inicio y
 * la posicion pos_y vertical de la pantalla LCD
 */
void LCD_BarGraphicXY(int pos_x, int pos_y, int value){
	LCD_Set_Cursor( pos_x, pos_y );
	for( int i = 0; i < 16; i++ ){
		if( value > 5 ){
			LCD_Put_Char( 0x05U );
			value -= 5;
		} else {
			LCD_Put_Char( value );
			while( i++ < 16 )
				LCD_Put_Char( 0 );
		}
	}
}
