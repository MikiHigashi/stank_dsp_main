#include <xc.h>
#include <stdio.h>
#include <string.h>

#define FCY 69784687UL
#include <libpic30.h>
#include "soft_i2c.h"
#include "lcd_i2c.h"
#include "mcc_generated_files/mcc.h"


// 表示クリアしDDRAMアドレス設定
void LCD_clear_pos(unsigned char cmd) {
    LCD_i2C_cmd(0x01); // クリアディスプレイ
    __delay_ms(1);
    if (cmd == 0x80) return;
    LCD_i2C_cmd(cmd);
}


// 文字列 str を表示する
void LCD_i2C_data(char *str) {
	unsigned char c;
	char l;
	char i;

	I2C_start();
	I2C_send(LCD_dev_addr);	// スレーブアドレス
	if  (I2C_ackchk()) {
	}

	l = strlen(str);
	for (i=1; i<=l; i++) {
		c = 0xC0;
		if (i==l) {
			c = 0x40;
		}
		I2C_send(c);
		if  (I2C_ackchk()) {
		}

		c = (unsigned char)(*(str++));
		I2C_send(c);
		if  (I2C_ackchk()) {
		}
	}

	I2C_stop();
}


// ==================== I2C接続LCDにコマンド送信 ===========================
void LCD_i2C_cmd(unsigned char cmd) {
	I2C_start();
	I2C_send(LCD_dev_addr);	// スレーブアドレス
	if  (I2C_ackchk()) {
	}
	I2C_send(0);
	if  (I2C_ackchk()) {
	}
	I2C_send(cmd);
	if  (I2C_ackchk()) {
	}
	I2C_stop();
}


// ==================== I2C接続LCDの初期化 ===========================
// ctr=コントラスト 0 to 63
void LCD_i2c_init(unsigned char ctr) {
    __delay_ms(40);
    LCD_i2C_cmd(0x38);
    __delay_us(30);
    LCD_i2C_cmd(0x39);
    __delay_us(30);
    LCD_i2C_cmd(0x14);
    __delay_us(30);
    LCD_i2C_cmd(0x70 + (ctr & 0x0F));
    __delay_us(30);
    LCD_i2C_cmd(0x54 + (ctr >> 4));
    __delay_us(30);
    LCD_i2C_cmd(0x6C);
    __delay_ms(200);

    LCD_i2C_cmd(0x38);
    __delay_us(30);
    LCD_i2C_cmd(0x0C);
    __delay_us(30);
    LCD_i2C_cmd(0x01);
    __delay_ms(2);
}
