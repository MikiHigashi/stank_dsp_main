#include <xc.h>

#define FCY 69784687UL
#include <libpic30.h>
#include "twe_lite.h"
#include "mcc_generated_files/mcc.h"


// ==================== TWE LITE への送信実行 =============================
#define TEMP_BUF_SIZE 48
static uint8_t temp_buf[TEMP_BUF_SIZE];

static uint8_t rsv_mode; // 受信モード 
// 0 to 79 次にデーター部の (1 to 80) バイト目を受信するはず
// 101 次にヘッダー0xA5を受信するはず
// 102 次にヘッダー0x5Aを受信するはず
// 103 次にヘッダー0x80を受信するはず
// 104 次にデーター長を受信するはず
// 105 次にチェックサムを受信するはず
// 106 次にフッタ0x04を受信するはず
static uint8_t rsv_n; // 受信バイト数 
static uint8_t rsv_size; // 受信バッファに格納されたバイト数 
static uint8_t rsv_csum; // チェックサム 
static uint8_t *rsv_buf; // 受信バッファアドレス
static uint8_t rsv_buf_size; // 受信バッファサイズ


// 受信バッファとそのサイズを設定する
void set_rsv_buf(uint8_t *buf, uint8_t size) {
    rsv_buf = buf;
    rsv_buf_size = size;
    rsv_size = 0;
    rsv_mode = 101; // ヘッダー待ち
}


// 受信バイト数を取得
// 0 でなければ、受信データーが存在すると分かる
uint8_t get_rsv_size(void) {
    return rsv_size;
}


// 受信バイト数をクリア
// 受信バッファを参照後にこれを呼ぶことで、受信バッファの更新を知ることができるようになる
void clear_rsv_size(void) {
    rsv_size = 0;
}

/*
void TWE_send1(uint8_t byte1) {
	while (!EUSART_is_tx_ready()) ;
	EUSART_Write(byte1);
}
*/

// バイナリモード送信
void TWE_send(uint8_t bytes, uint8_t *buf) {
    uint8_t i, c, csum = 0;
    uint8_t *cp = buf;
    UART1_Write(0xA5);
    UART1_Write(0x5A);
    UART1_Write(0x80);
    UART1_Write(bytes);
    for (i=0; i<bytes; i++) {
        c = (*(cp++));
        csum ^= c;
        UART1_Write(c);
    }
    UART1_Write(csum);
}


// バイナリモード受信　割り込みハンドラ
// 先に set_rsv_buf() で受信バッファを登録しておくこと
void TWE_rsv_int(void) {
    // ↓この部分は機種依存
    if (U1STAbits.OERR == 1) { // バッファーオーバーフロー
        U1STAbits.OERR = 0; // バッファクリア
        return;
    }
    uint8_t c = U1RXREG;
    // ↑この部分は機種依存

    switch (rsv_mode) {
        case 101: // 次にヘッダー0xA5を受信するはず
            if (c != 0xA5) return;
            rsv_mode = 102;
            break;
        case 102: // 次にヘッダー0x5Aを受信するはず
            if (c == 0x5A) {
                rsv_mode = 103;
            }
            else {
                rsv_mode = 101;
            }
            break;
        case 103: // 次にヘッダー0x80を受信するはず
            if (c == 0x80) {
                rsv_mode = 104;
            }
            else {
                rsv_mode = 101;
            }
            break;
        case 104: // 次にデーター長を受信するはず
            if ((c > 0) && (c < 81)) {
                rsv_n = c;
                rsv_csum = 0;
                rsv_mode = 0;
            }
            else {
                rsv_mode = 101;
            }
            break;
        case 105: // 次にチェックサムを受信するはず
            if (c == rsv_csum) {
                rsv_mode = 106;
            }
            else {
                rsv_mode = 101;
            }
            break;
        case 106: // 次にフッター0x04を受信するはず
            if (c == 0x04) {
                if (rsv_n > rsv_buf_size) {
                    rsv_n = rsv_buf_size; 
                }
                for (c=0; c<rsv_n; c++) {
                    rsv_buf[c] = temp_buf[c];
                }
                rsv_size = rsv_n;
            }
            rsv_mode = 101;
            break;
        default: // データー部受信
            rsv_csum ^= c;
            if (rsv_mode < TEMP_BUF_SIZE) {
                temp_buf[rsv_mode] = c;
            }
            rsv_mode ++;
            if (rsv_mode >= rsv_n) {
                rsv_mode = 105;
            }
            break;
    }
}
