#ifndef SOFT_I2C_H
#define SOFT_I2C_H

/**
  Section: Included Files
*/

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif

       
void I2C_start();
void I2C_send(unsigned char);
unsigned char I2C_ackchk();
void I2C_acksnd();
void I2C_nacksnd();
unsigned char I2C_rcv();
void I2C_stop();


#endif	//SOFT_I2C_H
/**
 End of File
*/