#ifndef LCD_I2C_H
#define LCD_I2C_H

/**
  Section: Included Files
*/

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif

// I2C アドレス
#define LCD_dev_addr 0x7C	// LCDアドレス

int LCD_i2c_init(unsigned char ctr);
int LCD_i2C_cmd(unsigned char cmd);
int LCD_i2C_data(char *str);
int LCD_clear_pos(unsigned char cmd);


#endif	//LCD_I2C_H
/**
 End of File
*/

