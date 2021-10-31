#ifndef TWE_LITE_H
#define TWE_LITE_H

/**
  Section: Included Files
*/

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif

void set_rsv_buf(uint8_t *buf, uint8_t size);
uint8_t get_rsv_size(void);
void clear_rsv_size(void);
void TWE_send(uint8_t bytes, uint8_t *buf);
//uint8_t TWE_rsv(uint8_t *buf, uint8_t buf_len);
void TWE_rsv_int(void);


#endif	//TWE_LITE_H
/**
 End of File
*/