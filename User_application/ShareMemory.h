#ifndef SHARE_MOMORY_H
#define SHARE_MOMORY_H

//Share Memory Configuration
// �����ڴ�ṹ������
struct IPC_MEMORY_WRITE{
    /* read/write (RW) shared memory @ GS1 owned by CPU1 */
    REAL dac_buffer[8];
    REAL test;
    REAL Read_RPM;
};// ˫�����
extern struct IPC_MEMORY_WRITE Write;

struct IPC_MEMORY_READ{
    /* read only (RO) shared memory @ GS0 */
    Uint32 SCI_shank_position_count;
    Uint32 CAN_position_count_ID0x03;

    Uint32 SCI_hip_position_count;
    Uint32 CAN_position_count_ID0x01;

    REAL position_cmd_elec;
    REAL speed_cmd_elec;
    REAL current_cmd_from_PC;
    int16 SCI_char;
};// ˫�����
extern struct IPC_MEMORY_READ Read;

void write_DAC_buffer();
void single_core_dac();
#endif
