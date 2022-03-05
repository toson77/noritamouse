#include "iodefine.h"
#include "e2_flash.h"
#include <stdint.h>
#define E2FLASH_BASE 0x100000UL
#define E2BLOCK_SIZE 0x80 /* 0x80 */
#define abs_addr(rel_addr) (E2FLASH_BASE + (uintptr_t)(rel_addr))
#define flash_access(addr, type) (*(volatile type *)abs_addr(addr))
#define block_addr(DBn) (E2FLASH_BASE + E2BLOCK_SIZE * (DBn))

typedef uint8_t BYTE;
typedef uint16_t WORD;

/* パラメーター保存用ROM領域「E2データフラッシュ」説明
 * RX220では容量は8kBytesであり、読み書きにはFCLKの4サイクル分時間が掛かる。
 * Flash用のプロセッサFPUが読み書きを担い、これにプログラム文を発行することで処理を行う。
 *
 * データ領域は128バイトごとに64個のブロック(DB00?DB63)に分かれている。
 * 消去はブロック単位で、書き込みは2バイトor8バイト単位で行う。
 */

// NOTE: このプログラム群では、引数内に記載するアドレスは0x0000から始まる相対アドレスとする。

// E2フラッシュ領域にWを許可する
static void e2_write_enable(void)
{
    FLASH.DFLWE0.WORD = 0x1E00 | 0x0F; // 全ブロックにプログラム/イレース許可
}

static void e2_read_enable(void)
{
    FLASH.DFLRE0.WORD = 0x2D00 | 0x0F; // 全ブロックに読み出し許可
}

// E2フラッシュ領域にWを禁止する
static void e2_write_disable(void)
{
    FLASH.DFLWE0.WORD = 0x1E00 | 0x00; // 全ブロックプログラム/イレース禁止
}

static void e2_read_disable(void)
{
    FLASH.DFLRE0.WORD = 0x2D00 | 0x00; // 全ブロック読み出し禁止
}

// E2データフラッシュモードへ
static void enter_pe_mode(void)
{
    FLASH.FENTRYR.WORD = 0xAA80; // E2フラッシュのP/Eモードに入る
    FLASH.FWEPROR.BIT.FLWE = 1;  // プログラム/イレース可能
}

// E2データフラッシュリードモードへ(E2データフラッシュもーどを抜ける)
static void exit_pe_mode(void)
{ // P/Eモードから抜ける
    while (!FLASH.FSTATR0.BIT.FRDY)
        ;                        // 実行されている処理が終了するまで待つ
    FLASH.FENTRYR.WORD = 0xAA00; // リードモードに
    while (FLASH.FENTRYR.WORD != 0x00)
        ;                       // リードモードに入るまで待つ
    FLASH.FWEPROR.BIT.FLWE = 2; // プログラム/イレース不可能に
}

static void e2_notify_peri(uint16_t *addr)
{
    FLASH.PCKAR.BIT.PCKA = 32; // 32MHz
    flash_access(addr, BYTE) = 0xE9;
    flash_access(addr, BYTE) = 0x03;
    flash_access(addr, WORD) = 0x0F0F;
    flash_access(addr, WORD) = 0x0F0F;
    flash_access(addr, WORD) = 0x0F0F;
    flash_access(addr, BYTE) = 0xD0;

    while (!FLASH.FSTATR0.BIT.FRDY)
        ; // 処理完了まで待つ
}

uint8_t e2_is_blank(uint16_t *addr)
{
    e2_write_enable();
    e2_read_enable();
    enter_pe_mode();
    FLASH.FMODR.BIT.FRDMD = 1;                 // ブランクチェックをするモードに入る
    FLASH.DFLBCCNT.BIT.BCSIZE = 0;             // 2バイト読み出し
    FLASH.DFLBCCNT.BIT.BCADR = abs_addr(addr); // チェック先のアドレス格納
    flash_access(addr, BYTE) = 0x71;           // この2つのコマンドを入力してFPUを動作させる
    flash_access(addr, BYTE) = 0xD0;
    while (!FLASH.FSTATR0.BIT.FRDY)
        ; // 処理完了まで待つ
    uint8_t ret = !FLASH.DFLBCSTAT.BIT.BCST;
    exit_pe_mode();
    e2_write_disable();
    return ret;
}

uint8_t e2_erase(uint16_t *addr)
{
    e2_write_enable();
    e2_read_enable();
    enter_pe_mode();
    e2_notify_peri((uint16_t *)((uintptr_t)addr & (E2FLASH_BASE | E2BLOCK_SIZE)));

    FLASH.DFLWE0.WORD = 0x1E0F; // 消去許可

    flash_access(addr, BYTE) = 0x20;
    flash_access(addr, BYTE) = 0xD0;
    while (!FLASH.FSTATR0.BIT.FRDY)
        ; // 処理完了まで待つ
    uint8_t ret = !(FLASH.FSTATR0.BIT.ILGLERR | FLASH.FSTATR0.BIT.ERSERR);
    exit_pe_mode();
    e2_write_disable();
    return ret;
}

uint8_t e2_write(uint16_t dat, uint16_t *addr)
{
    e2_write_enable();
    e2_read_enable();
    enter_pe_mode();
    e2_notify_peri(addr);

    flash_access(addr, BYTE) = 0xE8;
    flash_access(addr, BYTE) = 1;
    flash_access(addr, WORD) = dat;
    flash_access(addr, BYTE) = 0xD0;

    while (!FLASH.FSTATR0.BIT.FRDY)
        ; // 処理完了まで待つ
    uint8_t ret = !(FLASH.FSTATR0.BIT.ILGLERR | FLASH.FSTATR0.BIT.PRGERR);

    FLASH.DFLWE0.WORD = 0x1E00; // 消去許可を戻す
    exit_pe_mode();
    e2_write_disable();
    return ret;
}

uint16_t e2_read(uint16_t *addr)
{
    e2_read_enable();
    exit_pe_mode();
    uint16_t ret = flash_access(addr, uint16_t);
    e2_read_disable();
    return ret;
}

void e2_clear_status(uint16_t *addr)
{ // イリーガルビットが立った時にこれをクリアする
    enter_pe_mode();
    e2_notify_peri(addr);
    if (FLASH.FSTATR0.BIT.ILGLERR)
    {
        if (FLASH.FASTAT.BYTE != 0x10)
        {
            FLASH.FASTAT.BYTE = 0x10;
        }
    }
    flash_access(addr, BYTE) = 0x50;
    exit_pe_mode();
}
