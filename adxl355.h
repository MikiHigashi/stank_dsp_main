#ifndef ADXL355_H
#define ADXL355_H

/**
  Section: Included Files
*/

#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif
        
        
// I2C アドレス
#define ADXL355_dev_addr 0x3A	// ADXL355アドレス

#define ADXL355_ADR_X 8 // X加速度アドレス        
#define ADXL355_ADR_Y 11 // Y加速度アドレス        
#define ADXL355_ADR_Z 14 // Z加速度アドレス        
        

// 測定結果        
typedef struct tagADXL355 {
//    union {
//        uint16_t t; // 温度
//        struct {
//            uint8_t tL;
//           uint8_t tH;
//        };
//    };
    union {
        signed long x;  // X加速
        unsigned long ux;
        struct {
            uint8_t xS;
            uint8_t xL;
            uint8_t xM;
            uint8_t xH;
        };
    };
    union {
        signed long y;  // Y加速
        unsigned long uy;
        struct {
            uint8_t yS;
            uint8_t yL;
            uint8_t yM;
            uint8_t yH;
        };
    };
    union {
        signed long z;  // Z加速
        unsigned long uz;
        struct {
            uint8_t zS;
            uint8_t zL;
            uint8_t zM;
            uint8_t zH;
        };
    };
} ADXL355;


void ADXL355_write(uint8_t regset, uint8_t regdata);
void ADXL355_setadr(uint8_t addr);
void ADXL355_init(uint8_t sample);
signed char ADXL355_calt(uint16_t t);
void ADXL355_read(ADXL355 *v);
signed long ADXL355_readAcc(uint8_t addr);


#endif	//ADXL355_H
/**
 End of File
*/
