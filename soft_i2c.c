#include <xc.h>
#include <stdio.h>
#include <string.h>

#define FCY 69784687UL
#include <libpic30.h>
#include "soft_i2c.h"
#include "mcc_generated_files/mcc.h"


// ==================== I2C Start =============================
void I2C_start() {
	I2C_SDA_SetDigitalOutput();	// output
	I2C_SDA_SetHigh();
	I2C_SCL_SetDigitalOutput();	// output
	I2C_SCL_SetHigh();		// SCL high
    __delay_us(2);
	I2C_SDA_SetLow();		// SDA low : start
    __delay_us(2);
	I2C_SCL_SetLow();		// SCL low
}

// ==================== I2C Stop ==============================
void I2C_stop() {
	I2C_SDA_SetDigitalOutput();	// output
	I2C_SDA_SetLow();		// SDA low
	I2C_SCL_SetDigitalOutput();	// output
	I2C_SCL_SetHigh();		// SCL high
    __delay_us(2);
	I2C_SDA_SetDigitalInput();	// input(SDA high) : stop
	I2C_SCL_SetDigitalInput();	// input(SCL high) : stop
}

// ==================== I2C Send ==============================
void I2C_send(unsigned char send_data) {
	unsigned char i2c_data,i;

	I2C_SDA_SetDigitalOutput();	// SDA output
	I2C_SCL_SetDigitalOutput();	// SCL output

	i2c_data = send_data;
	for (i=8 ; i>0 ; i--) {
		if  (i2c_data & 0x80) {	// ビットセット
			I2C_SDA_SetHigh();
		} else {
			I2C_SDA_SetLow();
		}
		i2c_data = i2c_data << 1;	// next bit set

		I2C_SCL_SetHigh();	// 1ビット送信
        __delay_us(4);
		I2C_SCL_SetLow();
        __delay_us(3);
	}
	I2C_SDA_SetDigitalInput();	// SDA input
}

// ==================== I2C Recive ============================
unsigned char I2C_rcv() {
	unsigned char i2c_data,i;

	I2C_SDA_SetDigitalInput();	// SDA input
	I2C_SCL_SetDigitalOutput();	// SCL output

	for (i=8 ; i>0 ; i--) {
        __delay_us(1);
		I2C_SCL_SetHigh();	// １ビット受信
		i2c_data = i2c_data <<1;
		if (I2C_SDA_GetValue()) {
			i2c_data = i2c_data | 0x01;
		} else {
			i2c_data = i2c_data & 0xFE;
		}
		I2C_SCL_SetLow();
	}

	return(i2c_data);
}


// ==================== I2C ACK check =========================
unsigned char I2C_ackchk() {
	unsigned char i2c_data;
	I2C_SDA_SetDigitalInput();	// SDA input
	I2C_SCL_SetDigitalOutput();	// SCL output

	I2C_SCL_SetHigh();	// ACKビット受信
    __delay_us(1);
	i2c_data = I2C_SDA_GetValue();
	I2C_SCL_SetLow();

	return(i2c_data);
}

// ==================== I2C ACK send ==========================
void I2C_acksnd() {
	I2C_SDA_SetDigitalOutput();	// SDA output
	I2C_SDA_SetLow();		// ACK
	I2C_SCL_SetDigitalOutput();	// SCL output

    __delay_us(1);
	I2C_SCL_SetHigh();		// ACK送信
    __delay_us(2);
	I2C_SCL_SetLow();
	I2C_SDA_SetDigitalInput();	// SDA input
}

// ==================== I2C NACK send =========================
void I2C_nacksnd() {
	I2C_SDA_SetDigitalOutput();	// SDA output
	I2C_SDA_SetHigh();		// NACK
	I2C_SCL_SetDigitalOutput();	// SCL output
	I2C_SCL_SetHigh();		// NACK送信
    __delay_us(1);
	I2C_SCL_SetLow();
    __delay_us(1);
	I2C_SDA_SetDigitalInput();	// SDA input
}

