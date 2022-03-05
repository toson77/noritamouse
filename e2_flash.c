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

/* �p�����[�^�[�ۑ��pROM�̈�uE2�f�[�^�t���b�V���v����
 * RX220�ł͗e�ʂ�8kBytes�ł���A�ǂݏ����ɂ�FCLK��4�T�C�N�������Ԃ��|����B
 * Flash�p�̃v���Z�b�TFPU���ǂݏ�����S���A����Ƀv���O�������𔭍s���邱�Ƃŏ������s���B
 *
 * �f�[�^�̈��128�o�C�g���Ƃ�64�̃u���b�N(DB00?DB63)�ɕ�����Ă���B
 * �����̓u���b�N�P�ʂŁA�������݂�2�o�C�gor8�o�C�g�P�ʂōs���B
 */

// NOTE: ���̃v���O�����Q�ł́A�������ɋL�ڂ���A�h���X��0x0000����n�܂鑊�΃A�h���X�Ƃ���B

// E2�t���b�V���̈��W��������
static void e2_write_enable(void)
{
    FLASH.DFLWE0.WORD = 0x1E00 | 0x0F; // �S�u���b�N�Ƀv���O����/�C���[�X����
}

static void e2_read_enable(void)
{
    FLASH.DFLRE0.WORD = 0x2D00 | 0x0F; // �S�u���b�N�ɓǂݏo������
}

// E2�t���b�V���̈��W���֎~����
static void e2_write_disable(void)
{
    FLASH.DFLWE0.WORD = 0x1E00 | 0x00; // �S�u���b�N�v���O����/�C���[�X�֎~
}

static void e2_read_disable(void)
{
    FLASH.DFLRE0.WORD = 0x2D00 | 0x00; // �S�u���b�N�ǂݏo���֎~
}

// E2�f�[�^�t���b�V�����[�h��
static void enter_pe_mode(void)
{
    FLASH.FENTRYR.WORD = 0xAA80; // E2�t���b�V����P/E���[�h�ɓ���
    FLASH.FWEPROR.BIT.FLWE = 1;  // �v���O����/�C���[�X�\
}

// E2�f�[�^�t���b�V�����[�h���[�h��(E2�f�[�^�t���b�V�����[�ǂ𔲂���)
static void exit_pe_mode(void)
{ // P/E���[�h���甲����
    while (!FLASH.FSTATR0.BIT.FRDY)
        ;                        // ���s����Ă��鏈�����I������܂ő҂�
    FLASH.FENTRYR.WORD = 0xAA00; // ���[�h���[�h��
    while (FLASH.FENTRYR.WORD != 0x00)
        ;                       // ���[�h���[�h�ɓ���܂ő҂�
    FLASH.FWEPROR.BIT.FLWE = 2; // �v���O����/�C���[�X�s�\��
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
        ; // ���������܂ő҂�
}

uint8_t e2_is_blank(uint16_t *addr)
{
    e2_write_enable();
    e2_read_enable();
    enter_pe_mode();
    FLASH.FMODR.BIT.FRDMD = 1;                 // �u�����N�`�F�b�N�����郂�[�h�ɓ���
    FLASH.DFLBCCNT.BIT.BCSIZE = 0;             // 2�o�C�g�ǂݏo��
    FLASH.DFLBCCNT.BIT.BCADR = abs_addr(addr); // �`�F�b�N��̃A�h���X�i�[
    flash_access(addr, BYTE) = 0x71;           // ����2�̃R�}���h����͂���FPU�𓮍삳����
    flash_access(addr, BYTE) = 0xD0;
    while (!FLASH.FSTATR0.BIT.FRDY)
        ; // ���������܂ő҂�
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

    FLASH.DFLWE0.WORD = 0x1E0F; // ��������

    flash_access(addr, BYTE) = 0x20;
    flash_access(addr, BYTE) = 0xD0;
    while (!FLASH.FSTATR0.BIT.FRDY)
        ; // ���������܂ő҂�
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
        ; // ���������܂ő҂�
    uint8_t ret = !(FLASH.FSTATR0.BIT.ILGLERR | FLASH.FSTATR0.BIT.PRGERR);

    FLASH.DFLWE0.WORD = 0x1E00; // ��������߂�
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
{ // �C���[�K���r�b�g�����������ɂ�����N���A����
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
