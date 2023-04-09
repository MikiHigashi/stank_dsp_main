#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define FCY 69784687UL
#include <libpic30.h>
#include "soft_i2c.h"
#include "adxl355.h"
#include "mcc_generated_files/mcc.h"


// レジスター regset に、regdata を書き込む
void ADXL355_write(uint8_t regset, uint8_t regdata) {
	I2C_start();
	I2C_send(ADXL355_dev_addr);
	if  (I2C_ackchk()) {
	}
	I2C_send(regset);
	if  (I2C_ackchk()) {
	}
    I2C_send(regdata);
	if  (I2C_ackchk()) {
	}
	I2C_stop();
}        


// 読み出し開始アドレスをセット
void ADXL355_setadr(uint8_t addr) {
	I2C_start();
	I2C_send(ADXL355_dev_addr);
	if  (I2C_ackchk()) {
	}
	I2C_send(addr);
	if  (I2C_ackchk()) {
	}
	I2C_stop();
}


void ADXL355_init(uint8_t sample) {
    ADXL355_write(0x28, sample); // ローパス&サンプリング
    ADXL355_write(0x2D, 0);
}


// 温度のナマ値を摂氏に変換
signed char ADXL355_calt(uint16_t t) {
    uint16_t a = t * 20; 
    if (a <= 41565) {
        a = 41655 - a;
        a /= 181;
        return (signed char)a;
    }
    a -= 41475;
    a /= 181;
    signed char r = (signed char)a;
    return (-r);
}


void ADXL355_read(ADXL355 *v) {
    ADXL355_setadr(8);
    
	I2C_start();
	I2C_send(ADXL355_dev_addr | 1);
	if  (I2C_ackchk()) {
	}

    // 温度取得
//    v->tH = I2C_rcv();
//    I2C_acksnd();
//    v->tL = I2C_rcv();
//    I2C_acksnd();
    
    // X加速度取得
    v->xH = I2C_rcv();
    I2C_acksnd();
    v->xM = I2C_rcv();
    I2C_acksnd();
    v->xL = I2C_rcv();
    I2C_acksnd();
    v->x >>= 12;
    
    // Y加速度取得
    v->yH = I2C_rcv();
    I2C_acksnd();
    v->yM = I2C_rcv();
    I2C_acksnd();
    v->yL = I2C_rcv();
    I2C_acksnd();
    v->y >>= 12;

    // Z加速度取得
    v->zH = I2C_rcv();
    I2C_acksnd();
    v->zM = I2C_rcv();
    I2C_acksnd();
    v->zL = I2C_rcv();
    I2C_nacksnd();
    I2C_stop();
    v->z >>= 12;
}


signed long ADXL355_readAcc(uint8_t addr) {
    union {
        signed long x;  // X加速
        struct {
            uint8_t xS;
            uint8_t xL;
            uint8_t xM;
            uint8_t xH;
        };
    } v;
    
    ADXL355_setadr(addr);

	I2C_start();
	I2C_send(ADXL355_dev_addr | 1);
	if  (I2C_ackchk()) {
	}

    // 加速度取得
    v.xH = I2C_rcv();
    I2C_acksnd();
    v.xM = I2C_rcv();
    I2C_acksnd();
    v.xL = I2C_rcv();
    I2C_nacksnd();
    I2C_stop();
    v.x >>= 12;
    
    return v.x;
}

