extern int channel_0_number;
extern int channel_1_number;
extern int channel_2_number;
extern int channel_3_number;
extern uint16_t type_of_LDC;


__interrupt void i2c_int1a_isr(void);
void pass(void);
void fail(void);

void   I2CA_Init(void);
int Multiple_channel_config(uint16_t channel);
uint32_t I2CA_ReadData_Channel(uint16_t channel);

uint32_t I2CA_ReadData_Channel0(void);
uint32_t I2CA_ReadData_Channel1(void);
uint32_t I2CA_ReadData_Channel2(void);
uint32_t I2CA_ReadData_Channel3(void);
int I2cRead16bitData(uint16_t SlaveRegAddr);
int I2cWrite16bitData(uint16_t ConfigRegAddr, uint16_t value);

void Set_Rp(uint16_t channel, float n_kom);
void Set_L(uint16_t channel, float n_uh);
void Set_C(uint16_t channel, float n_pf);
void Set_Q_factor(uint16_t channel, float q);
uint32_t Set_FIN_FREF_DIV(uint16_t channel);
uint32_t Set_LC_stabilize_time(uint16_t channel);
uint32_t Set_conversion_time(uint16_t channel, uint16_t value);
uint32_t Set_driver_current(uint16_t channel, uint16_t value);
uint32_t Set_mux_config(uint16_t value);
uint32_t Set_sensor_config(uint16_t value);
void Select_channel_to_convert(uint16_t channel, uint16_t* value);
uint32_t Reset_sensor();
int Parse_result_data(uint16_t channel, uint32_t raw_result, uint32_t* result);
uint32_t Set_ERROR_CONFIG(uint16_t value);
void Read_sensor_infomation();
uint32_t reset_sensor();
//switch (channel_0_number){
//    case 1:
//        Set_Rp(CHANNEL_0, 3.259);
//        Set_L(CHANNEL_0, 51.35);
//        Set_C(CHANNEL_0, 1000);
//        Set_Q_factor(CHANNEL_0, 14.32);
//        break;
//
//    case 2:
//        Set_Rp(CHANNEL_0, 1.462);
//        Set_L(CHANNEL_0, 6.6);
//        Set_C(CHANNEL_0, 1000);
//        Set_Q_factor(CHANNEL_0, 17.96);
//        break;
//
//    case 3:
//        Set_Rp(CHANNEL_0, 1.637);
//        Set_L(CHANNEL_0, 5.65);
//        Set_C(CHANNEL_0, 1000);
//        Set_Q_factor(CHANNEL_0, 21.74);
//        break;
//
//    case 4:
//        Set_Rp(CHANNEL_0, 1.462);
//        Set_L(CHANNEL_0, 6.6);
//        Set_C(CHANNEL_0, 1000);
//        Set_Q_factor(CHANNEL_0, 17.96);
//        break;
//
//    case 5:
//        Set_Rp(CHANNEL_0, 1.156);
//        Set_L(CHANNEL_0, 4.15);
//        Set_C(CHANNEL_0, 1000);
//        Set_Q_factor(CHANNEL_0, 17.91);
//        break;
//    case 6:
//
//        break;
//    case 7:
//        Set_Rp(CHANNEL_0, 1.637);
//        Set_L(CHANNEL_0, 5.65);
//        Set_C(CHANNEL_0, 1000);
//        Set_Q_factor(CHANNEL_0, 21.74);
////        Set_Rp(CHANNEL_0, 15.727);
////        Set_L(CHANNEL_0, 18.147);
////        Set_C(CHANNEL_0, 100);
////        Set_Q_factor(CHANNEL_0, 35.97);
//        break;
//    default:
//        printf("Please input number from 1 to 7 \n");
//}

//switch (channel_1_number){
//    case 1:
//        Set_Rp(CHANNEL_1, 3.259);
//        Set_L(CHANNEL_1, 51.35);
//        Set_C(CHANNEL_1, 1000);
//        Set_Q_factor(CHANNEL_1, 14.32);
//        break;
//
//    case 2:
//        Set_Rp(CHANNEL_1, 1.462);
//        Set_L(CHANNEL_1, 6.6);
//        Set_C(CHANNEL_1, 1000);
//        Set_Q_factor(CHANNEL_1, 17.96);
//        break;
//
//    case 3:
//        Set_Rp(CHANNEL_1, 1.637);
//        Set_L(CHANNEL_1, 5.65);
//        Set_C(CHANNEL_1, 1000);
//        Set_Q_factor(CHANNEL_1, 21.74);
//        break;
//
//    case 4:
//        Set_Rp(CHANNEL_1, 1.462);
//        Set_L(CHANNEL_1, 6.6);
//        Set_C(CHANNEL_1, 1000);
//        Set_Q_factor(CHANNEL_1, 17.96);
//        break;
//
//    case 5:
//        Set_Rp(CHANNEL_1, 1.156);
//        Set_L(CHANNEL_1, 4.15);
//        Set_C(CHANNEL_1, 1000);
//        Set_Q_factor(CHANNEL_1, 17.91);
//        break;
//    case 6:
//
//        break;
//    case 7:
//        Set_Rp(CHANNEL_1, 1.637);
//        Set_L(CHANNEL_1, 5.65);
//        Set_C(CHANNEL_1, 1000);
//        Set_Q_factor(CHANNEL_1, 21.74);
////        Set_Rp(CHANNEL_1, 15.727);
////        Set_L(CHANNEL_1, 18.147);
////        Set_C(CHANNEL_1, 100);
////        Set_Q_factor(CHANNEL_1, 35.97);
//        break;
//    default:
//        printf("Please input number from 1 to 7 \n");
//}


