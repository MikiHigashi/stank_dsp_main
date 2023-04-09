#include <xc.h>
#include <stdio.h>
#include <string.h>

#define FCY 69784687UL
#include <libpic30.h>
#include "hard_i2c.h"
#include "lcd_i2c.h"
#include "mcc_generated_files/mcc.h"


// 表示クリアしDDRAMアドレス設定
// 1:タイムアウト 0:正常
int LCD_clear_pos(unsigned char cmd) {
    if (LCD_i2C_cmd(0x01)) return 1; // クリアディスプレイ
    __delay_ms(1);
    if (cmd == 0x80) return 1;
    return LCD_i2C_cmd(cmd);
}


// 文字列 str を表示する
// 1:タイムアウト 0:正常
int LCD_i2C_data(char *str) {
	unsigned char c;
	char l;
	char i;

	if (I2C_start()) return 1;
	if (I2C_send(LCD_dev_addr)) return 1;	// スレーブアドレス
	if (I2C_ackchk() == 2) return 1;

	l = strlen(str);
	for (i=1; i<=l; i++) {
		c = 0xC0;
		if (i==l) {
			c = 0x40;
		}
		if (I2C_send(c)) return 1;
    	if (I2C_ackchk() == 2) return 1;

		c = (unsigned char)(*(str++));
		if (I2C_send(c)) return 1;
    	if (I2C_ackchk() == 2) return 1;
	}

	return I2C_stop();
}


// ==================== I2C接続LCDにコマンド送信 ===========================
// 1:タイムアウト 0:正常
int LCD_i2C_cmd(unsigned char cmd) {
	if (I2C_start()) return 1;
	if (I2C_send(LCD_dev_addr)) return 1;	// スレーブアドレス
	if (I2C_ackchk() == 2) return 1;
	if (I2C_send(0)) return 1;
	if (I2C_ackchk() == 2) return 1;
	if (I2C_send(cmd)) return 1;
	if (I2C_ackchk() == 2) return 1;
	return I2C_stop();
}


// ==================== I2C接続LCDの初期化 ===========================
// ctr=コントラスト 0 to 63
// 1:タイムアウト 0:正常
int LCD_i2c_init(unsigned char ctr) {
    __delay_ms(40);
    if (LCD_i2C_cmd(0x38)) return 1;
    __delay_us(30);
    if (LCD_i2C_cmd(0x39)) return 1;
    __delay_us(30);
    if (LCD_i2C_cmd(0x14)) return 1;
    __delay_us(30);
    if (LCD_i2C_cmd(0x70 + (ctr & 0x0F))) return 1;
    __delay_us(30);
    if (LCD_i2C_cmd(0x54 + (ctr >> 4))) return 1;
    __delay_us(30);
    if (LCD_i2C_cmd(0x6C)) return 1;
    __delay_ms(200);

    if (LCD_i2C_cmd(0x38)) return 1;
    __delay_us(30);
    if (LCD_i2C_cmd(0x0C)) return 1;
    __delay_us(30);
    if (LCD_i2C_cmd(0x01)) return 1;
    __delay_ms(2);
    return 0;
}
