#include <All_Definition.h>

// Note: I2C Macros used in this example can be found in the F2837xD_I2C_defines.h file

// Prototype statements for functions found within this file.
void   I2CA_Init(void);
Uint32 I2CA_ReadData_Channel(Uint16 channel);
//Uint32 I2CA_ReadData_Channel0(void);
//Uint32 I2CA_ReadData_Channel1(void);
int I2cRead16bitData(Uint16 SlaveRegAddr);
int I2cWrite16bitData(Uint16 ConfigRegAddr, Uint16 value);
int Single_channel_config(Uint16 channel);
void Set_Rp(Uint16 channel, float n_kom);
void Set_L(Uint16 channel, float n_uh);
void Set_C(Uint16 channel, float n_pf);
void Set_Q_factor(Uint16 channel, float q);
Uint32 Set_FIN_FREF_DIV(Uint16 channel);
Uint32 Set_LC_stabilize_time(Uint16 channel);
Uint32 Set_conversion_time(Uint16 channel, Uint16 value);
Uint32 Set_driver_current(Uint16 channel, Uint16 value);
Uint32 Set_mux_config(Uint16 value);
Uint32 Set_sensor_config(Uint16 value);
void Select_channel_to_convert(Uint16 channel, Uint16* value);
Uint32 Reset_sensor();
int Parse_result_data(Uint16 channel, Uint32 raw_result, Uint32* result);
Uint32 Set_ERROR_CONFIG(Uint16 value);
void Read_sensor_infomation();
Uint32 reset_sensor();


__interrupt void i2c_int1a_isr(void);
void pass(void);
void fail(void);

#define I2C_SLAVE_ADDR        0x2B // I2C Address selection pin: when ADDR=L, I2C address = 0X2A,
                                                              // when ADDR=H, I2C address = 0x2B.
#define I2C_NUMBYTES         2     // 16 bits

// After transmitting the address of LDC1612(0X2B), it is needed to be transmitting the address of channel registers
// the channel registers is composed by 2 parts, that is, high address and low address.
#define I2C_EEPROM_HIGH_ADDR_CHANNEL0  0x00
#define I2C_EEPROM_LOW_ADDR_CHANNEL0   0x01
#define I2C_EEPROM_HIGH_ADDR_CHANNEL1  0x02
#define I2C_EEPROM_LOW_ADDR_CHANNEL1   0x03
#define I2C_EEPROM_HIGH_ADDR_CHANNEL2  0x04
#define I2C_EEPROM_LOW_ADDR_CHANNEL2   0x05
#define I2C_EEPROM_HIGH_ADDR_CHANNEL3  0x06
#define I2C_EEPROM_LOW_ADDR_CHANNEL3   0x07

// define the registers which are needed in LDC1612
#define SET_CONVERSION_TIME_REG_START           0X08
#define SET_CONVERSION_OFFSET_REG_START         0X0C
#define SET_LC_STABILIZE_REG_START              0X10
#define SET_FREQ_REG_START                      0X14
#define SENSOR_STATUS_REG                       0X18
#define ERROR_CONFIG_REG                        0X19
#define SENSOR_CONFIG_REG                       0X1A
#define MUL_CONFIG_REG                          0X1B
#define SENSOR_RESET_REG                        0X1C
#define SET_DRIVER_CURRENT_REG                  0X1E
#define READ_MANUFACTURER_ID                    0X7E
#define READ_DEVICE_ID                          0X7F


#define CHANNEL_NUM 2 // we use two channel
#define CHANNEL_0 0
#define CHANNEL_1 1
// Global variables Two bytes will be used for the outgoing address, thus only setup 14 bytes maximum
Uint16 ERROR;

Uint16 dataHigh;
Uint16 dataLow;
Uint16 dataHigh_one;
Uint16 dataLow_one;
Uint32 result_zero;
Uint32 result_one;
Uint32 raw_result;
Uint32 channel0DataResult;
Uint32 channel1DataResult;
uint32 raw_value_rdlu[4];
// Uint32 raw_value_zero;
// Uint32 raw_value_one;
// Uint32 raw_value_two;
// Uint32 raw_value_three;
Uint32 result;
int choose_channel;

/*定义第几个线圈*/
int channel_0_number, channel_1_number;
#define channel_0_number FALSE
#define channel_1_number FALSE

float resistance[CHANNEL_NUM];
float inductance[CHANNEL_NUM];
float capacitance[CHANNEL_NUM];
float Q_factor[CHANNEL_NUM];
float Fsensor[CHANNEL_NUM];
float Fref[CHANNEL_NUM];
float inductance[CHANNEL_NUM];
float capacitance[CHANNEL_NUM];
Uint16 value;
Uint16 FIN_DIV, FREF_DIV;
Uint16 config = 0x1601;



Uint16 DataBuffer;

Uint16 data[2];

/* Initialize I2C */ 
void I2CA_Init(void)
{
   I2caRegs.I2CSAR.all = I2C_SLAVE_ADDR;      // Slave address - EEPROM control code

   I2caRegs.I2CPSC.all = 8;            // Prescaler - need 7-12 MHz on module clk

   I2caRegs.I2CCLKL = 40;           // NOTE: must be non zero, sets the duration of the I2C clock low
   I2caRegs.I2CCLKH = 40;           // NOTE: must be non zero, sets the duration of the I2C clock high
   // Frequency = SYSCLKOUT / (I2CCLKL+I2CCLKH)
   // In this place, the target I2C frequency is 400kHz, the Prescaler is disposed to 12MHz, 12MHz / 400KHz = 30
   // To make the high and low levels 50/50, the I2caRegs.I2CCLKL = 15; I2caRegs.I2CCLKH = 15

   I2caRegs.I2CIER.all = 0x24;      // Enable SCD & ARDY __interrupts
   //  Bit 0: ARBL (Arbitration Lost)
   //  Bit 1: NACK (No Acknowledge)
   //  Bit 2: ARDY (Register Access Ready)
   //  Bit 3: RRDY (Receive Data Ready)
   //  Bit 4: XRDY (Transmit Data Ready)
   //  Bit 5: SCD (Stop Condition Detected)
   //  Bit 6: AAS (Addressed As Slave)

   I2caRegs.I2CMDR.bit.MST = 1; // Master mode bit. The I2C module is a master and generates the serial clock on the SCL pin.
   I2caRegs.I2CMDR.bit.TRX = 0; // Transmitter mode bit.The I2C module is a transmitter and transmits data on the SDA pin.
//   I2caRegs.I2CMDR.bit.DLB = 1;   // Digital Loopback Mode, if not using Loopback Mode, don't using this.
   I2caRegs.I2CMDR.bit.IRS = 1;  // Take I2C out of reset. In usual mode, just let it be 1.

   I2caRegs.I2CFFTX.all = 0x6000;   // Enable FIFO mode and TXFIFO
   // Reset Clears all data in the transmitted FIFO buffer and resets the FIFO pointer.
   // This operation is usually performed during initialization or when the FIFO needs to be reconfigured to ensure that there is no residual data in the FIFO buffer to avoid affecting subsequent transfer operations.

   I2caRegs.I2CFFRX.all = 0x2040;   // Enable RXFIFO, clear RXFFINT,
   // TX FIFO interrupt Enable Used to enable FIFO interrupt.
   // When the data in the TX FIFO reaches a predetermined threshold (such as when the FIFO becomes empty or the amount of data reaches a certain level), an interrupt is triggered,
   // notifying the CPU of the appropriate processing operation.
   // This facilitates efficient I2C communication, as the CPU can be notified of interruptions when needed, rather than constantly polling the FIFO state.
   return;
}

int I2cRead16bitData(Uint16 SlaveRegAddr){

    if (I2caRegs.I2CMDR.bit.STP == 1)
    {

       return I2C_STP_NOT_READY_ERROR;
    }

    // FRAME 1
    I2caRegs.I2CSAR.all = I2C_SLAVE_ADDR  //  I2cMsgIn1.SlaveAddress;


     // Check if bus busy
     if (I2caRegs.I2CSTR.bit.BB == 1)
     {

         return I2C_BUS_BUSY_ERROR;
     }

    // FRAME 2
    I2caRegs.I2CCNT = 0 + 1; // datLen = 0
    I2caRegs.I2CDXR.all = SlaveRegAddr; // Address of Channel 0 MSB Conversion Result of LDC1614

    I2caRegs.I2CMDR.all = 0x2620; // TRX=1, no stop
    /* I2C_MSGSTAT_SEND_NOSTOP */
    /* I2C_MSGSTAT_SEND_NOSTOP */
    /* I2C_MSGSTAT_SEND_NOSTOP */


    // SLAVE ACK
    // Wait Register-access-ready
    while(I2caRegs.I2CSTR.bit.ARDY == 0);

    // FRAME 3 is SLAVE ADDR
    I2caRegs.I2CSAR.all = I2C_SLAVE_ADDR  //  I2cMsgIn1.SlaveAddress;
    I2caRegs.I2CCNT = 2; // MSB + LSB of the MSB of the DATA
    I2caRegs.I2CMDR.all = 0x2C20; // TRX=0, with stop
    /* I2C_MSGSTAT_RECEIVE_WITHSTOP */
    /* I2C_MSGSTAT_RECEIVE_WITHSTOP */
    /* I2C_MSGSTAT_RECEIVE_WITHSTOP */


    // Check the status of the receive FIFO
    while(I2caRegs.I2CFFRX.bit.RXFFST == 0);
    DataBuffer = I2caRegs.I2CDRR.all; // MSB of the sensor data MSB/LSB
    DataBuffer = DataBuffer << 8;
    while(I2caRegs.I2CFFRX.bit.RXFFST == 0);
    DataBuffer |= I2caRegs.I2CDRR.all; // LSB of the sensor data MSB/LSB
    DELAY_US(50);
    return 1;
}


int I2cWrite16bitData(Uint16 ConfigRegAddr, Uint16 value){
    data[1] = value & 0x00ff;
    data[0] = value >> 8;
    while(I2caRegs.I2CMDR.bit.STP == 1);


    // FRAME 1
    I2caRegs.I2CSAR.all = I2C_SLAVE_ADDR  //  I2cMsgIn1.SlaveAddress;


     // Check if bus busy
     if (I2caRegs.I2CSTR.bit.BB == 1)
     {
         return I2C_BUS_BUSY_ERROR;
     }

     // FRAME 2
     I2caRegs.I2CCNT = 3; // Slave Register Address + Data MSB from Master + Data LSB from Master
     I2caRegs.I2CDXR.all = ConfigRegAddr; // The address of the registers which are needed in LDC1612.


     I2caRegs.I2CMDR.all = 0x6E20; // TRX=1, with stop
     /* I2C_MSGSTAT_RECEIVE_WITHSTOP */
     /* I2C_MSGSTAT_RECEIVE_WITHSTOP */
     /* I2C_MSGSTAT_RECEIVE_WITHSTOP */
     // BIT13, BIT11, BIT10, BIT9

     // SLAVE ACK
     // Wait Register-access-ready
     while(I2caRegs.I2CSTR.bit.XRDY == 0);
     I2caRegs.I2CDXR.all = data[0];
     while(I2caRegs.I2CSTR.bit.XRDY == 0);
     I2caRegs.I2CDXR.all = data[1];

     // FRAME 3 is SLAVE ADDR
     I2caRegs.I2CSAR.all = I2C_SLAVE_ADDR  //  I2cMsgIn1.SlaveAddress;


//     I2caRegs.I2CCNT = 2; // MSB + LSB of the MSB of the DATA

     // Check the status of the receive FIFO
     // According to Seeed, write MSB first.
//     while(I2caRegs.I2CFFTX.bit.TXFFST == 0);
//     data[0] = I2caRegs.I2CDXR.all; // MSB of the sensor data MSB/LSB
//     while(I2caRegs.I2CFFTX.bit.TXFFST == 0);
//     data[1] = I2caRegs.I2CDXR.all; // LSB of the sensor data MSB/LSB
//
     while(I2caRegs.I2CSTR.bit.XRDY == 0);
     DELAY_US(50);
     return 0;
}



Uint32 I2CA_ReadData_Channel(Uint16 channel){

    if(channel == 0){
        // CHANNEL 0
        // I2cGet16bitData(0x00, DataMSB);
        I2cRead16bitData(0x00);
        raw_value_rdlu[0] = (Uint32)DataBuffer << 16; //
        I2cRead16bitData(0x01);
        raw_value_rdlu[0] |= (Uint32)DataBuffer;
//        Parse_result_data(channel, raw_value, result);
    }else if(channel == 1){
        // CHANNEL 1
        I2cRead16bitData(0x02);
        raw_value_rdlu[1] = (Uint32)DataBuffer << 16;
        I2cRead16bitData(0x03);
        raw_value_rdlu[1] |= (Uint32)DataBuffer; //
//       Parse_result_data(channel, raw_value, result);
    }else if (channel == 2){
        // CHANNEL 2
        I2cRead16bitData(0x04);
        raw_value_rdlu[2] = (Uint32)DataBuffer << 16;
        I2cRead16bitData(0x05);
        raw_value_rdlu[2] |= (Uint32)DataBuffer; //
//       Parse_result_data(channel, raw_value, result);
    }else if (channel == 3){
        // CHANNEL 3
        I2cRead16bitData(0x06);
        raw_value_rdlu[3] = (Uint32)DataBuffer << 16;
        I2cRead16bitData(0x07);
        raw_value_rdlu[3] |= (Uint32)DataBuffer; //
//       Parse_result_data(channel, raw_value, result);
    }

   return 1;
}


// Configuration of single channel

int Single_channel_config(Uint16 channel){
    // The value of Rp, L, C, Q_factor can be set from TI calculator(https://webench.ti.com/wb5/LDC/#/spirals).
    switch (channel_0_number){
        case 1:
            Set_Rp(CHANNEL_0, 3.259);
            Set_L(CHANNEL_0, 51.35);
            Set_C(CHANNEL_0, 1000);
            Set_Q_factor(CHANNEL_0, 14.32);
            break;
        case 2:
            Set_Rp(CHANNEL_0, 1.462);
            Set_L(CHANNEL_0, 6.6);
            Set_C(CHANNEL_0, 1000);
            Set_Q_factor(CHANNEL_0, 17.96);
            break;
        case 3:
            Set_Rp(CHANNEL_0, 1.637);
            Set_L(CHANNEL_0, 5.65);
            Set_C(CHANNEL_0, 1000);
            Set_Q_factor(CHANNEL_0, 21.74);
            break;
        case 4:
            Set_Rp(CHANNEL_0, 1.462);
            Set_L(CHANNEL_0, 6.6);
            Set_C(CHANNEL_0, 1000);
            Set_Q_factor(CHANNEL_0, 17.96);
            break;
        case 5:
            Set_Rp(CHANNEL_0, 1.156);
            Set_L(CHANNEL_0, 4.15);
            Set_C(CHANNEL_0, 1000);
            Set_Q_factor(CHANNEL_0, 17.91);
            break;
        case 6:

            break;
        case 7:
            Set_Rp(CHANNEL_0, 1.637);
            Set_L(CHANNEL_0, 5.65);
            Set_C(CHANNEL_0, 1000);
            Set_Q_factor(CHANNEL_0, 21.74);
    //        Set_Rp(CHANNEL_0, 15.727);
    //        Set_L(CHANNEL_0, 18.147);
    //        Set_C(CHANNEL_0, 100);
    //        Set_Q_factor(CHANNEL_0, 35.97);
            break;
//        default:
    }

    switch (channel_1_number){
        case 1:
            Set_Rp(CHANNEL_1, 3.259);
            Set_L(CHANNEL_1, 51.35);
            Set_C(CHANNEL_1, 1000);
            Set_Q_factor(CHANNEL_1, 14.32);
            break;
        case 2:
            Set_Rp(CHANNEL_1, 1.462);
            Set_L(CHANNEL_1, 6.6);
            Set_C(CHANNEL_1, 1000);
            Set_Q_factor(CHANNEL_1, 17.96);
            break;
        case 3:
            Set_Rp(CHANNEL_1, 1.637);
            Set_L(CHANNEL_1, 5.65);
            Set_C(CHANNEL_1, 1000);
            Set_Q_factor(CHANNEL_1, 21.74);
            break;
        case 4:
            Set_Rp(CHANNEL_1, 1.462);
            Set_L(CHANNEL_1, 6.6);
            Set_C(CHANNEL_1, 1000);
            Set_Q_factor(CHANNEL_1, 17.96);
            break;
        case 5:
            Set_Rp(CHANNEL_1, 1.156);
            Set_L(CHANNEL_1, 4.15);
            Set_C(CHANNEL_1, 1000);
            Set_Q_factor(CHANNEL_1, 17.91);
            break;
        case 6:

            break;
        case 7:
            Set_Rp(CHANNEL_1, 1.637);
            Set_L(CHANNEL_1, 5.65);
            Set_C(CHANNEL_1, 1000);
            Set_Q_factor(CHANNEL_1, 21.74);
    //        Set_Rp(CHANNEL_1, 15.727);
    //        Set_L(CHANNEL_1, 18.147);
    //        Set_C(CHANNEL_1, 100);
    //        Set_Q_factor(CHANNEL_1, 35.97);
            break;
//        default:
//            return Please input number from 1 to 7
//            printf("Please input number from 1 to 7 \n");
    }

    if (Set_FIN_FREF_DIV(CHANNEL_0)) {
            return -1;
        }
    Set_FIN_FREF_DIV(CHANNEL_1);

    Set_LC_stabilize_time(CHANNEL_0);
    Set_LC_stabilize_time(CHANNEL_1);

        /*Set conversion interval time*/
    Set_conversion_time(CHANNEL_0, 0x0546);
    Set_conversion_time(CHANNEL_1, 0x0546);

        /*Set driver current!*/
    Set_driver_current(CHANNEL_0, 0xA000);
    Set_driver_current(CHANNEL_1, 0xA000);

    /*multiple conversion*/
    Set_mux_config(0x820C);
        /*single conversion*/
//    Set_mux_config(0x20C);
        /*start channel 0*/
    //Uint16 config = 0x1601; this line is on 98.
//    Select_channel_to_convert(CHANNEL_0, &config);
    Set_sensor_config(config);
    return 0;
}

/**Setting RCL and Q parameters**/
void Set_Rp(Uint16 channel, float n_kom) {
    resistance[channel] = n_kom;
}

void Set_L(Uint16 channel, float n_uh) {
    inductance[channel] = n_uh;
}

void Set_C(Uint16 channel, float n_pf) {
    capacitance[channel] = n_pf;
}

void Set_Q_factor(Uint16 channel, float q) {
    Q_factor[channel] = q;
}

Uint32 Set_FIN_FREF_DIV(Uint16 channel) {
    Fsensor[channel] = 1 / (2 * M_PI * sqrt(inductance[channel] * capacitance[channel] * pow(10, -18))) * pow(10, -6);

    FIN_DIV = (Uint16)(Fsensor[channel] / 8.75 + 1);


    if (Fsensor[channel] * 4 < 40) {
        FREF_DIV = 2;
        Fref[channel] = 40 / 2;
    } else {
        FREF_DIV = 4;
        Fref[channel] = 40 / 4;
    }

    value = FIN_DIV << 12;
    value |= FREF_DIV;
    return I2cWrite16bitData(SET_FREQ_REG_START + channel, value);
}


Uint32 Set_LC_stabilize_time(Uint16 channel){
    value = 30;
    return I2cWrite16bitData(SET_LC_STABILIZE_REG_START + channel, value);
}


Uint32 Set_conversion_time(Uint16 channel, Uint16 value) {
    return I2cWrite16bitData(SET_CONVERSION_TIME_REG_START + channel, value);
}


Uint32 Set_conversion_offset(Uint16 channel, Uint16 value) {
    return I2cWrite16bitData(SET_CONVERSION_OFFSET_REG_START + channel, value);
}


Uint32 Set_driver_current(Uint16 channel, Uint16 value) {
    return I2cWrite16bitData(SET_DRIVER_CURRENT_REG + channel, value);
}


Uint32 Set_ERROR_CONFIG(Uint16 value) {
    return I2cWrite16bitData(ERROR_CONFIG_REG, value);
}


Uint32 Set_mux_config(Uint16 value) {
    return I2cWrite16bitData(MUL_CONFIG_REG, value);
}


Uint32 Set_sensor_config(Uint16 value) {
    return I2cWrite16bitData(SENSOR_CONFIG_REG, value);
}


Uint32 Reset_sensor() {
    return I2cWrite16bitData(SENSOR_RESET_REG, 0x8000);
}



void Select_channel_to_convert(Uint16 channel, Uint16* value) {
    switch (channel) {
        case 0: *value &= 0x3fff;
            break;
        case 1: *value &= 0x7fff;
            *value |= 0x4000;
            break;
        case 2: *value &= 0xbfff;
            *value |= 0x8000;
            break;
        case 3: *value |= 0xc000;
            break;
    }
}


// not used
void Read_sensor_infomation() {
    I2cWrite16bitData(READ_MANUFACTURER_ID, value);
    return ;
}

// not used
int Parse_result_data(Uint16 channel, Uint32 raw_result, Uint32* result) {
    *result = raw_result & 0x0fffffff;
    if (0xfffffff == *result) {
        *result = 0;
        return -1;
    }
    // else if(0==*result)
    // {
    //     Serial.println("result is none!!!");
    // }
    value = raw_result >> 24;
    if (value & 0x80) {
        ERROR = 1;
    }
    if (value & 0x40) {
        ERROR = 2;
    }
    if (value & 0x20) {
        ERROR = 3;
    }
    if (value & 0x10) {
        ERROR = 4;
    }
    return 0;
}


//__interrupt void i2c_int1a_isr(void)     // I2C-A
//{
//   // Enable future I2C (PIE Group 8) __interrupts
//   PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;
//}

void pass()
{
   asm("   ESTOP0");
    for(;;);
}

void fail()
{
   asm("   ESTOP0");
    for(;;);
}




