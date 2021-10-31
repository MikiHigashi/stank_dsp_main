// 前照灯LED制御およびドーザーブレード制御 dsPIC 版
#define FCY 69784687UL
#include <libpic30.h>
#include "mcc_generated_files/mcc.h"
#include <stdio.h>
#include <string.h>
#include "lcd_i2c.h"


int main(void)
{
    unsigned long u;
    uint16_t pp;

    // initialize the device
    SYSTEM_Initialize();
    
    char buf[32];
    __delay_ms(100);    
    LCD_i2c_init(8);

    uint16_t cnt = 0;
    
    while (1)
    {
        cnt ++;
        LCD_i2C_cmd(0x80);
        sprintf(buf, "%5d", cnt);
        LCD_i2C_data(buf);
        __delay_ms(20);
 
        WATCHDOG_TimerClear();
    }
    return 1; 
}
/**
 End of File
*/

