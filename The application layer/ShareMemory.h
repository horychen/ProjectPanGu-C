#ifndef SHARE_MOMORY_H
#define SHARE_MOMORY_H

//Share Memory Configuration
// �����ڴ�ṹ������
struct IPC_MEMORY_WRITE{
    /* read/write (RW) shared memory @ GS1 owned by CPU1 */
    REAL dac_buffer[8];
    REAL test;
};// ˫�����
extern struct IPC_MEMORY_WRITE Write;

struct IPC_MEMORY_READ{
    /* read only (RO) shared memory @ GS0 */
    REAL position_cmd_elec;
    REAL speed_cmd_elec;

    int16 SCI_char;
};// ˫�����
extern struct IPC_MEMORY_READ Read;

void write_DAC_buffer();
void single_core_dac();
#endif
