// 受信機メインdsPIC
#define FCY 69784687UL
#include <libpic30.h>
#include "mcc_generated_files/mcc.h"
#include <stdio.h>
#include <string.h>
#include "soft_i2c.h"
#include "lcd_i2c.h"
#include "twe_lite.h"


typedef union tagHL16 {
    signed short SHL;
    uint16_t HL;
    struct {
        uint8_t L;
        uint8_t H;
    };
    struct {
        unsigned :8;
        unsigned :7;
        unsigned T:1;
    };
} HL16;


typedef union tagHL32 {
    unsigned long HL;
    struct {
        uint8_t L;
        uint16_t M;
        uint8_t H;
    };
} HL32;


// AD変換値 DMAセットされる
uint16_t battery = 0;
uint16_t temp1 = 0;

#define TIMEOUT 1000 /* 受信タイムアウト */
uint8_t sid[] = {0x82, 0x02, 0x2e, 0x90}; // 送信機の TWE LITE シリアル番号
#define RSV_BYTES 8 /* 電波受信すべきデーターのバイト数 送信機 send_main に応じた値でなければならない */ 
#define SPI_BYTES 8 /* SPI送受信するデーターのバイト数 */
uint8_t data[SPI_BYTES]; // SPI受信格納先
uint8_t send[SPI_BYTES]; // SPI送信格納先

#define CNT_LOWBATT 100 /* これだけ連続して電圧が低いとローバッテリー扱い */
#define VOLT_LOWBATT 6400 /* ローバッテリー電圧 mV単位 */
uint8_t cnt_lowbatt = 0;
uint8_t now_lowbatt = 0; // ローバッテリーなら１
uint8_t fired; // 射撃弾数

uint8_t rsvt[32]; // 受信バッファー
#define RSVA_BYTES 24 
uint8_t rsv[RSVA_BYTES]; // 正常に受信できたデーターの転送先
char buf[32];


// 受信データーをソフトウェアSPI送信
void spi_send(void) {
    uint8_t b, d, *dp = data;
    uint8_t idx, m, d2, *dp2 = send;
    // パケット先頭 STRB=1 で相手に伝える
    // クロックを1にするまで15μ秒以上空けるのを仕様とする
    SPI_STRB_SetHigh();
    SPI3_STRB_SetHigh();
    __delay_us(14);
    SPI_STRB_SetLow();
    SPI3_STRB_SetLow();
    for (idx=0; idx<SPI_BYTES; idx++) {
        d = 0;
        d2 = (*(dp2++));
        m = 0x80;
        for (b=0; b<8; b++) {
            SPI_CLOCK_SetHigh();
            SPI3_CLOCK_SetHigh();
            if (d2 & m) {
                SPI_DATA_SetHigh();
                SPI3_OUT_SetHigh();
            }
            else {
                SPI_DATA_SetLow();
                SPI3_OUT_SetLow();
            }
            __delay_us(4);
            m >>= 1;
            d <<= 1;
            SPI_CLOCK_SetLow();
            SPI3_CLOCK_SetLow();
            if (SPI3_IN_GetValue()) {
                d++;
            }
            __delay_us(4);
        }        
        (*(dp++)) = d;
    }
    SPI_DATA_SetLow();
    SPI3_OUT_SetLow();
}


// 受信データー確認
// 受信あれば1 なければ0 を返す
char check_rsv(void) {
    uint8_t i, n = get_rsv_size();
    if (n <= 15) {
        return 0; // 受信データーが少な過ぎる
    }
    // 送信機のシリアルIDを確認
    for (i=0; i<4; i++) {
        if (rsvt[i+3] != sid[i]) {
            return 0; // 送信機のシリアルIDと違う
        }
    }
    if (rsvt[13] != RSV_BYTES) {
        return 0; // データー長が想定と違う
    }

    for (i=0; i<RSVA_BYTES; i++) {
        rsv[i] = rsvt[i];
    }
    return 1;
}


void int_timer(void) {
    spi_send();
}


int main(void)
{
    // initialize the device
    SYSTEM_Initialize();
    UART1_SetRxInterruptHandler(TWE_rsv_int);
    set_rsv_buf(rsvt, 32);
    TMR2_SetInterruptHandler(int_timer);
    
    DMA_ChannelEnable(DMA_CHANNEL_1);
    DMA_PeripheralAddressSet(DMA_CHANNEL_1, (volatile unsigned int) &ADC1BUF0);
    DMA_StartAddressASet(DMA_CHANNEL_1, (uint16_t)(&battery));        
    //TMR2_SetInterruptHandler(int_timer);

    __delay_ms(100); // I2C バス安定化待ち    
//    LCD_i2c_init(8);


    cnt_lowbatt = 0;
    now_lowbatt = 0; // ローバッテリーなら１
    fired = 0;
            
    uint8_t id = 0; // 応答ID
    uint8_t WiFi = 0; // 受信感度
    uint8_t i;
    uint16_t t = 0;

    for (i=0; i<RSVA_BYTES; i++) {
        rsv[i] = 0;
    }
    send[0] = send[1] = send[2] = send[3] = 0;
    send[4] = send[5] = send[6] = send[7] = 0x80; // 停止

    //SPI2_CLOCK_SetLow();
    
    while (1)
    {
        WATCHDOG_TimerClear();

//        LCD_i2C_cmd(0x80);
//        sprintf(buf, "%6d", dc++);
//        LCD_i2C_data(buf);

        
        for (t=0; t<TIMEOUT; t++) {
            if (check_rsv()) {
                break;
            }
            __delay_ms(1);
        }
        clear_rsv_size();
        
        if (now_lowbatt) { // ローバッテリー
            for (i=0; i<RSVA_BYTES; i++) {
                rsv[i] = 0;
            }
            send[0] = send[1] = 0; // ノーコンを示す
            send[4] = send[5] = send[6] = send[7] = 0x80; // 停止
            // rsv[2] 応答ID
            id = rsv[2];
            // rsv[11] 受信強度
            WiFi = rsv[11];
        } else if (t >= TIMEOUT) { // 電波が届かない
            for (i=0; i<RSVA_BYTES; i++) {
                rsv[i] = 0;
            }
            send[0] = send[1] = 0; // ノーコンを示す
            send[4] = send[5] = send[6] = send[7] = 0x80; // 停止
            id = 0;
            WiFi = 1;
        }
        else {
            // rsv[2] 応答ID
            id = rsv[2];
            // rsv[11] 受信強度
            WiFi = rsv[11];
            
            // bit0  正常なら 1 ノーコンなら 0
            // bit1  TRR-D（BEEP切り替え）
            // bit2  TRR-L（車高ニュートラル）
            // bit3  TRR-R（車高ニュートラル）
            // bit4  RigtU B（ドーザーブレード展開）
            // bit5  RigtD A（ドーザーブレード収納）
            // bit6  LeftD ↑（前照灯明るく）
            // bit7  LeftU ↓（前照灯暗く）
            send[0] = (rsv[14] | 1);

            // bit0  TRL-U キーが押されたら1（レーザー切り替え）
            // bit1  TRL-D キーが押されたら1（レーザー切り替え）
            // bit2  TRL-L が押されたら1（カメラ左移動）
            // bit3  TRL-R が押されたら1（カメラ右移動）
            // bit4  カメラ切り替えスイッチが押されたら1
            // bit5  Rが押されたら1（セミオート射撃）
            // bit6  RLが押されたら1（フルオート射撃）
            // bit7  Lが押されたら1（３点バースト射撃）
            send[1] = rsv[15];

            // カメラ切り替え
            if (send[1] & 16) {
                TO_PIN19_SetHigh();
            }
            else {
                TO_PIN19_SetLow();
            }
            if (send[1] & 8) {
                TO_PIN21_SetHigh();
            }
            else {
                TO_PIN21_SetLow();
            }
            if (send[1] & 4) {
                TO_PIN23_SetHigh();
            }
            else {
                TO_PIN23_SetLow();
            }
            
            // 左キャタ
            send[4] = rsv[18];
            // 右キャタ
            send[5] = rsv[19];
            // 砲塔旋回
            send[6] = rsv[20];
            // 砲身上下
            send[7] = rsv[21];
        }
        
        // TWE LITE モジュールと通信
        uint8_t *cp = (uint8_t *)buf;
        cp[0] = 0x00; // 親機あて
        cp[1] = 0xA0; // 拡張形式
        cp[2] = id;   // 応答ID
        cp[3] = 0xFF; // オプション無し
        cp[4] = WiFi; // 受信感度

        HL16 v; // バッテリー電圧
        uint16_t vv = (uint16_t)(((long)10000 * battery) >> 12); // 1mV単位に変換
        if (vv < VOLT_LOWBATT) {
            cnt_lowbatt ++;
            if (cnt_lowbatt >= CNT_LOWBATT) {
                //now_lowbatt = 1; この段階でのローバッテリー判定しない
            }
        }
        else {
            cnt_lowbatt = 0;
        }
        v.HL = (vv / 10); // 0.01V単位に変換
        if ((vv % 10) >= 5) v.HL ++; // 四捨五入

        if (now_lowbatt) {
            cp[5] = 255;
            cp[6] = 255;
        }
        else {
            cp[5] = v.H;
            cp[6] = v.L;
        }

        vv = (uint16_t)(((long)5000 * temp1) >> 12); // 1mV単位に変換
        v.HL = (vv / 10); // 0.01V単位に変換
        if ((vv % 10) >= 5) v.HL ++; // 四捨五入
        
        if (data[1] & 8) {
            fired = data[0];
        }
        cp[7] = fired; // 射撃弾数
        cp[8] = v.L; // 温度
        TWE_send(9, cp);
        
//        LCD_i2C_cmd(0x80);
//        sprintf(buf, "%6d %3d %3d", dc++, send[3], send[2]);
//        LCD_i2C_data(buf);
    }
    return 1; 
}
/**
 End of File
*/

