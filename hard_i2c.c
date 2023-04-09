#include <xc.h>
#include <stdio.h>
#include <string.h>

#define FCY 69784687UL
#include <libpic30.h>
#include "hard_i2c.h"
#include "mcc_generated_files/mcc.h"


#define TIMEOUT_CNT 300
#define TIMEOUT_CNT_L 5000


int IdleI2C1(void)
{
    uint16_t cnt = 0;
	//while(I2C1CON1bits.SEN || I2C1CON1bits.PEN || I2C1CON1bits.RCEN || I2C1CON1bits.RSEN || I2C1CON1bits.ACKEN || I2C1STATbits.TRSTAT) {
	//while(I2C1CON1bits.SEN || I2C1CON1bits.PEN || I2C1CON1bits.RCEN || I2C1CON1bits.RSEN || I2C1CON1bits.ACKEN) {
	while(I2C1CON1bits.SEN || I2C1CON1bits.PEN || I2C1CON1bits.RCEN || I2C1CON1bits.RSEN || I2C1CON1bits.ACKEN || I2C1STATbits.S) {
        cnt ++;
        if (cnt>=TIMEOUT_CNT_L) {
            i2c1_driver_close();            
            i2c1_driver_driver_open();
            i2c1_driver_initSlaveHardware();
            return 1; // タイムアウト
        }
        __delay_us(1);
    }
    return 0; // 正常
}


// ==================== I2C Start =============================
// 1: タイムアウト 0:正常
int I2C_start() {
    uint16_t cnt = 0;
    if (IdleI2C1()) {
        return 1;
    }
	i2c1_driver_start();
    for (cnt=0; cnt<TIMEOUT_CNT; cnt++) {
        if (I2C1CONLbits.SEN == 0) return 0;
        __delay_us(1);
    }
    return 1;
}


// ==================== I2C ReStart =============================
// 1: タイムアウト 0:正常
int I2C_restart() {
    uint16_t cnt = 0;
//    if (IdleI2C1()) {
//        return 1;
//    }
	i2c1_driver_restart();
    for (cnt=0; cnt<TIMEOUT_CNT; cnt++) {
        if (I2C1CONLbits.RSEN == 0) return 0;
        __delay_us(1);
    }
    return 1;
}


// ==================== I2C Stop ==============================
// 1:タイムアウト 0:正常
int I2C_stop() {
    uint16_t cnt = 0;
    i2c1_driver_stop();
    for (cnt=0; cnt<TIMEOUT_CNT; cnt++) {
        if (I2C1CONLbits.PEN == 0) return 0;
        __delay_us(1);
    }
    return 1;
}

// ==================== I2C Send ==============================
// 1:タイムアウト 0:正常
int I2C_send(unsigned char send_data) {
    uint16_t cnt = 0;
    
    for (cnt=0; cnt<TIMEOUT_CNT; cnt++) {
        I2C1STATbits.IWCOL = 0;
        i2c1_driver_TXData(send_data);
        if (I2C1STATbits.IWCOL == 0) break;
        __delay_us(1);
    }
    if (cnt>=TIMEOUT_CNT) {
    	i2c1_driver_stop();
        return 1;
    }

    for (cnt=0; cnt<TIMEOUT_CNT; cnt++) {
        if (I2C1STATbits.TBF) break;
        __delay_us(1);
    }
    if (cnt>=TIMEOUT_CNT) {
    	i2c1_driver_stop();
        return 1;
    }
    return 0;
}

// ==================== I2C Recive ============================
// 256:タイムアウト
uint16_t I2C_rcv() {
	uint8_t i2c_data;
    uint16_t cnt = 0;
//    if (IdleI2C1()) {
//    	i2c1_driver_stop();
//        return 0;
//    }
    i2c1_driver_startRX();

    for (cnt=0; cnt<TIMEOUT_CNT_L; cnt++) {
        if (I2C1STATbits.RBF) break;
        __delay_us(1);
    }

    if (cnt>=TIMEOUT_CNT_L) {
    	i2c1_driver_stop();
        return 256;
    }
    
    i2c_data = i2c1_driver_getRXData();
   
	return (uint16_t)(i2c_data);
}


// ==================== I2C ACK check =========================
// 2:タイムアウト
unsigned char I2C_ackchk() {
    uint16_t cnt = 0;

    for (cnt=0; cnt<TIMEOUT_CNT; cnt++) {
        if (I2C1STATbits.TRSTAT == 0) break;
        __delay_us(1);
    }
    if (cnt>=TIMEOUT_CNT) {
    	i2c1_driver_stop();
        return 2; // タイムアウト
    }

    if (i2c1_driver_isNACK()) {
        return 1;
    }
    return 0;
}

// ==================== I2C ACK send ==========================
// 1:タイムアウト 0:正常
int I2C_acksnd() {
    uint16_t cnt = 0;
//    if (IdleI2C1()) {
//    	i2c1_driver_stop();
//        return 1;
//    }

    //    IdleI2C1();
    i2c1_driver_sendACK();
    for (cnt=0; cnt<TIMEOUT_CNT; cnt++) {
        if (I2C1CONLbits.ACKEN == 0) return 0;
        __delay_us(1);
    }
    i2c1_driver_stop();
    return 1;
}

// ==================== I2C NACK send =========================
// 1:タイムアウト 0:正常
int I2C_nacksnd() {
    uint16_t cnt = 0;
//    if (IdleI2C1()) {
//    	i2c1_driver_stop();
//        return 1;
//    }

    i2c1_driver_sendNACK();
    for (cnt=0; cnt<TIMEOUT_CNT; cnt++) {
        if (I2C1CONLbits.ACKEN == 0) return 0;
        __delay_us(1);
    }
    i2c1_driver_stop();
    return 1;
}

