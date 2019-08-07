#ifndef MPU9250_H
#define MPU9250_H

#include<stdint.h>
//OK AND ERROR
#define MPU9250_OK                                         UINT8_C(0)
#define MPU9250_ERROR                                      UINT8_C(1)
//I2C ADDRESSES
#define MPU9250_I2C_PRIMARY_ADDR                           UINT8_C(0x68)
#define MPU9250_I2C_SECONDARY_ADDR                         UINT8_C(0x69)
//SPI ADDRESSES


//ENABLE AND DISABLE FLAGS
#define MPU9250_ENABLE_FLAG                                UINT8_C(0x01)
#define MPU9250_DISABLE_FLAG                               UINT8_C(0x00)

//SENSOR REGISTER ADDRESSES
#define MPU9250_RF_SELF_TEST_X_GYRO_RW                     UINT8_C(0x00)
#define MPU9250_RF_SELF_TEST_Y_GYRO_RW                     UINT8_C(0x01)
#define MPU9250_RF_SELF_TEST_Z_GYRO_RW                     UINT8_C(0x02)
/*
#define MPU9250_RF_X_FINE_GAIN                             UINT8_C(0x03) // [7:0] fine gain
#define MPU9250_RF_Y_FINE_GAIN                             UINT8_C(0x04)
#define MPU9250_RF_Z_FINE_GAIN                             UINT8_C(0x05)
#define MPU9250_RF_XA_OFFSET_H                             UINT8_C(0x06) // User-defined trim values for accelerometer
#define MPU9250_RF_XA_OFFSET_L_TC                          UINT8_C(0x07)
#define MPU9250_RF_YA_OFFSET_H                             UINT8_C(0x08)
#define MPU9250_RF_YA_OFFSET_L_TC                          UINT8_C(0x09)
#define MPU9250_RF_ZA_OFFSET_H                             UINT8_C(0x0A)
#define MPU9250_RF_ZA_OFFSET_L_TC                          UINT8_C(0x0B)
*/
#define MPU9250_RF_SELF_TEST_X_ACCEL_RW                    UINT8_C(0x0D)
#define MPU9250_RF_SELF_TEST_Y_ACCEL_RW                    UINT8_C(0x0E)
#define MPU9250_RF_SELF_TEST_Z_ACCEL_RW                    UINT8_C(0x0F)
/*
#define MPU9250_R_SELF_TEST_A                              UINT8_C(0x10)
*/
#define MPU9250_RF_XG_OFFSET_H_RW                          UINT8_C(0x13)  // User-defined trim values for gyroscope
#define MPU9250_RF_XG_OFFSET_L_RW                          UINT8_C(0x14)
#define MPU9250_RF_YG_OFFSET_H_RW                          UINT8_C(0x15)
#define MPU9250_RF_YG_OFFSET_L_RW                          UINT8_C(0x16)
#define MPU9250_RF_ZG_OFFSET_H_RW                          UINT8_C(0x17)
#define MPU9250_RF_ZG_OFFSET_L_RW                          UINT8_C(0x18)
#define MPU9250_RF_SMPLRT_DIV_RW                           UINT8_C(0x19)
#define MPU9250_R_CONFIG_RW                                UINT8_C(0x1A)
#define MPU9250_R_GYRO_CONFIG_RW                           UINT8_C(0x1B)
#define MPU9250_R_ACCEL_CONFIG_RW                          UINT8_C(0x1C)
#define MPU9250_R_ACCEL_CONFIG2_RW                         UINT8_C(0x1D)
#define MPU9250_R_LP_ACCEL_ODR_RW                          UINT8_C(0x1E)
#define MPU9250_RF_WOM_THR_RW                              UINT8_C(0x1F)
/*
#define MPU9250_R_MOT_DUR                                  UINT8_C(0x20)  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define MPU9250_R_ZMOT_THR                                 UINT8_C(0x21)  // Zero-motion detection threshold bits [7:0]
#define MPU9250_R_ZRMOT_DUR                                UINT8_C(0x22)  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms
*/
#define MPU9250_R_FIFO_EN_RW                               UINT8_C(0x23)
#define MPU9250_R_I2C_MST_CTRL_RW                          UINT8_C(0x24)
#define MPU9250_R_I2C_SLV0_ADDR_RW                         UINT8_C(0x25)
#define MPU9250_RF_I2C_SLV0_REG_RW                         UINT8_C(0x26)
#define MPU9250_R_I2C_SLV0_CTRL_RW                         UINT8_C(0x27)
#define MPU9250_R_I2C_SLV1_ADDR_RW                         UINT8_C(0x28)
#define MPU9250_RF_I2C_SLV1_REG_RW                         UINT8_C(0x29)
#define MPU9250_R_I2C_SLV1_CTRL_RW                         UINT8_C(0x2A)
#define MPU9250_R_I2C_SLV2_ADDR_RW                         UINT8_C(0x2B)
#define MPU9250_RF_I2C_SLV2_REG_RW                         UINT8_C(0x2C)
#define MPU9250_R_I2C_SLV2_CTRL_RW                         UINT8_C(0x2D)
#define MPU9250_R_I2C_SLV3_ADDR_RW                         UINT8_C(0x2E)
#define MPU9250_RF_I2C_SLV3_REG_RW                         UINT8_C(0x2F)
#define MPU9250_R_I2C_SLV3_CTRL_RW                         UINT8_C(0x30)
#define MPU9250_RF_I2C_SLV4_ADDR_RW                        UINT8_C(0x31)
#define MPU9250_RF_I2C_SLV4_REG_RW                         UINT8_C(0x32)
#define MPU9250_RF_I2C_SLV4_DO_RW                          UINT8_C(0x33)
#define MPU9250_R_I2C_SLV4_CTRL_RW                         UINT8_C(0x34)
#define MPU9250_RF_I2C_SLV4_DI_R                           UINT8_C(0x35)
#define MPU9250_R_I2C_MST_STATUS_R                         UINT8_C(0x36)
#define MPU9250_R_INT_PIN_CFG_RW                           UINT8_C(0x37)
#define MPU9250_R_INT_ENABLE_RW                            UINT8_C(0x38)
/*
#define MPU9250_R_DMP_INT_STATUS                           UINT8_C(0x39) // Check DMP interrupt
*/
#define MPU9250_R_INT_STATUS_R                             UINT8_C(0x3A)
#define MPU9250_RF_ACCEL_XOUT_H_R                          UINT8_C(0x3B)
#define MPU9250_RF_ACCEL_XOUT_L_R                          UINT8_C(0x3C)
#define MPU9250_RF_ACCEL_YOUT_H_R                          UINT8_C(0x3D)
#define MPU9250_RF_ACCEL_YOUT_L_R                          UINT8_C(0x3E)
#define MPU9250_RF_ACCEL_ZOUT_H_R                          UINT8_C(0x3F)
#define MPU9250_RF_ACCEL_ZOUT_L_R                          UINT8_C(0x40)
#define MPU9250_RF_TEMP_OUT_H_R                            UINT8_C(0x41)
#define MPU9250_RF_TEMP_OUT_L_R                            UINT8_C(0x42)
#define MPU9250_RF_GYRO_XOUT_H_R                           UINT8_C(0x43)
#define MPU9250_RF_GYRO_XOUT_L_R                           UINT8_C(0x44)
#define MPU9250_RF_GYRO_YOUT_H_R                           UINT8_C(0x45)
#define MPU9250_RF_GYRO_YOUT_L_R                           UINT8_C(0x46)
#define MPU9250_RF_GYRO_ZOUT_H_R                           UINT8_C(0x47)
#define MPU9250_RF_GYRO_ZOUT_L_R                           UINT8_C(0x48)
#define MPU9250_RF_EXT_SENS_DATA_00_R                      UINT8_C(0x49)
#define MPU9250_RF_EXT_SENS_DATA_01_R                      UINT8_C(0x4A)
#define MPU9250_RF_EXT_SENS_DATA_02_R                      UINT8_C(0x4B)
#define MPU9250_RF_EXT_SENS_DATA_03_R                      UINT8_C(0x4C)
#define MPU9250_RF_EXT_SENS_DATA_04_R                      UINT8_C(0x4D)
#define MPU9250_RF_EXT_SENS_DATA_05_R                      UINT8_C(0x4E)
#define MPU9250_RF_EXT_SENS_DATA_06_R                      UINT8_C(0x4F)
#define MPU9250_RF_EXT_SENS_DATA_07_R                      UINT8_C(0x50)
#define MPU9250_RF_EXT_SENS_DATA_08_R                      UINT8_C(0x51)
#define MPU9250_RF_EXT_SENS_DATA_09_R                      UINT8_C(0x52)
#define MPU9250_RF_EXT_SENS_DATA_10_R                      UINT8_C(0x53)
#define MPU9250_RF_EXT_SENS_DATA_11_R                      UINT8_C(0x54)
#define MPU9250_RF_EXT_SENS_DATA_12_R                      UINT8_C(0x55)
#define MPU9250_RF_EXT_SENS_DATA_13_R                      UINT8_C(0x56)
#define MPU9250_RF_EXT_SENS_DATA_14_R                      UINT8_C(0x57)
#define MPU9250_RF_EXT_SENS_DATA_15_R                      UINT8_C(0x58)
#define MPU9250_RF_EXT_SENS_DATA_16_R                      UINT8_C(0x59)
#define MPU9250_RF_EXT_SENS_DATA_17_R                      UINT8_C(0x5A)
#define MPU9250_RF_EXT_SENS_DATA_18_R                      UINT8_C(0x5B)
#define MPU9250_RF_EXT_SENS_DATA_19_R                      UINT8_C(0x5C)
#define MPU9250_RF_EXT_SENS_DATA_20_R                      UINT8_C(0x5D)
#define MPU9250_RF_EXT_SENS_DATA_21_R                      UINT8_C(0x5E)
#define MPU9250_RF_EXT_SENS_DATA_22_R                      UINT8_C(0x5F)
#define MPU9250_RF_EXT_SENS_DATA_23_R                      UINT8_C(0x60)
/*
#define MPU9250_R_MOT_DETECT_STATUS                        UINT8_C(0x61)
*/
#define MPU9250_RF_I2C_SLV0_DO_RW                          UINT8_C(0x63)
#define MPU9250_RF_I2C_SLV1_DO_RW                          UINT8_C(0x64)
#define MPU9250_RF_I2C_SLV2_DO_RW                          UINT8_C(0x65)
#define MPU9250_RF_I2C_SLV3_DO_RW                          UINT8_C(0x66)
#define MPU9250_R_I2C_MST_DELAY_CTRL_RW                    UINT8_C(0x67)
#define MPU9250_R_SIGNAL_PATH_RESET_RW                     UINT8_C(0x68)
#define MPU9250_R_MOT_DETECT_CTRL_RW                       UINT8_C(0x69)
#define MPU9250_R_USER_CTRL_RW                             UINT8_C(0x6A) // Bit 7 enable DMP, bit 3 reset DMP
#define MPU9250_R_PWR_MGMT_1_RW                            UINT8_C(0x6B) // Device defaults to the SLEEP mode
#define MPU9250_R_PWR_MGMT_2_RW                            UINT8_C(0x6C)
/*
#define MPU9250_R_DMP_BANK                                 UINT8_C(0x6D) // Activates a specific bank in the DMP
#define MPU9250_R_DMP_RW_PNT                               UINT8_C(0x6E)  // Set read/write pointer to a specific start address in specified DMP bank
#define MPU9250_R_DMP_REG                                  UINT8_C(0x6F)  // Register in DMP from which to read or to which to write
#define MPU9250_R_DMP_REG_1                                UINT8_C(0x70)
#define MPU9250_R_DMP_REG_2                                UINT8_C(0x71)
*/
#define MPU9250_RF_FIFO_COUNTH_RW                          UINT8_C(0x72)
#define MPU9250_RF_FIFO_COUNTL_RW                          UINT8_C(0x73)
#define MPU9250_RF_FIFO_R_W_RW                             UINT8_C(0x74)
#define MPU9250_RF_WHO_AM_I_R                              UINT8_C(0x75)// Should return                UINT8_C(0x71
#define MPU9250_RF_XA_OFFSET_H_RW                          UINT8_C(0x77)
#define MPU9250_RF_XA_OFFSET_L_RW                          UINT8_C(0x78)
#define MPU9250_RF_YA_OFFSET_H_RW                          UINT8_C(0x7A)
#define MPU9250_RF_YA_OFFSET_L_RW                          UINT8_C(0x7B)
#define MPU9250_RF_ZA_OFFSET_H_RW                          UINT8_C(0x7D)
#define MPU9250_RF_ZA_OFFSET_L_RW                          UINT8_C(0x7E)

//POS AND MASK
#define MPU9250_CONFIG_P_FIFO_MODE_0                       UINT8_C(6)
#define MPU9250_CONFIG_M_FIFO_MODE_0                       UINT8_C(0x40)
#define MPU9250_CONFIG_P_EXT_SYNC_SET_2_0                  UINT8_C(3)
#define MPU9250_CONFIG_M_EXT_SYNC_SET_2_0                  UINT8_C(0x38)
#define MPU9250_CONFIG_P_DLPF_CFG_2_0                      UINT8_C(0)
#define MPU9250_CONFIG_M_DLPF_CFG_2_0                      UINT8_C(0x07)

#define MPU9250_GYROCFG_P_XG_ST_EN_0                       UINT8_C(7)
#define MPU9250_GYROCFG_M_XG_ST_EN_0                       UINT8_C(0x80)
#define MPU9250_GYROCFG_P_YG_ST_EN_0                       UINT8_C(6)
#define MPU9250_GYROCFG_M_YG_ST_EN_0                       UINT8_C(0x40)
#define MPU9250_GYROCFG_P_ZG_ST_EN_0                       UINT8_C(5)
#define MPU9250_GYROCFG_M_ZG_ST_EN_0                       UINT8_C(0x20)
#define MPU9250_GYROCFG_P_GYRO_FS_SEL_1_0                  UINT8_C(3)
#define MPU9250_GYROCFG_M_GYRO_FS_SEL_1_0                  UINT8_C(0x18)
#define MPU9250_GYROCFG_P_FCHOICE_B_1_0                    UINT8_C(0)
#define MPU9250_GYROCFG_M_FCHOICE_B_1_0                    UINT8_C(0x03)

#define MPU9250_ACCELCFG_P_XA_ST_EN                        UINT8_C(7)
#define MPU9250_ACCELCFG_M_XA_ST_EN                        UINT8_C(0x80)
#define MPU9250_ACCELCFG_P_YA_ST_EN                        UINT8_C(6)
#define MPU9250_ACCELCFG_M_YA_ST_EN                        UINT8_C(0x40)
#define MPU9250_ACCELCFG_P_ZA_ST_EN                        UINT8_C(5)
#define MPU9250_ACCELCFG_M_ZA_ST_EN                        UINT8_C(0x20)
#define MPU9250_ACCELCFG_P_ACCEL_FS_SEL_1_0                UINT8_C(3)
#define MPU9250_ACCELCFG_M_ACCEL_FS_SEL_1_0                UINT8_C(0x18)

#define MPU9250_ACCELCFG1_P_FCHOICE_B_1_0                  UINT8_C(2)
#define MPU9250_ACCELCFG1_M_FCHOICE_B_1_0                  UINT8_C(0x0C)
#define MPU9250_ACCELCFG1_P_DLPF_B_1_0                     UINT8_C(0)
#define MPU9250_ACCELCFG1_M_DLPF_B_1_0                     UINT8_C(0x03)

#define MPU9250_ACCEL_ODR_P_CLKSEL_B_3_0                   UINT8_C(0)
#define MPU9250_ACCEL_ODR_M_CLKSEL_B_3_0                   UINT8_C(0x0F)

#define MPU9250_FIFOEN_P_TEMP_0                            UINT8_C(7)
#define MPU9250_FIFOEN_M_TEMP_0                            UINT8_C(0x80)
#define MPU9250_FIFOEN_P_GYRO_X_0                          UINT8_C(6)
#define MPU9250_FIFOEN_M_GYRO_X_0                          UINT8_C(0x40)
#define MPU9250_FIFOEN_P_GYRO_Y_0                          UINT8_C(5)
#define MPU9250_FIFOEN_M_GYRO_Y_0                          UINT8_C(0x20)
#define MPU9250_FIFOEN_P_GYRO_Z_0                          UINT8_C(4)
#define MPU9250_FIFOEN_M_GYRO_Z_0                          UINT8_C(0x10)
#define MPU9250_FIFOEN_P_ACCEL_0                           UINT8_C(3)
#define MPU9250_FIFOEN_M_ACCEL_0                           UINT8_C(0x08)
#define MPU9250_FIFOEN_P_SLV2_0                            UINT8_C(2)
#define MPU9250_FIFOEN_M_SLV2_0                            UINT8_C(0x04)
#define MPU9250_FIFOEN_P_SLV1_0                            UINT8_C(1)
#define MPU9250_FIFOEN_M_SLV1_0                            UINT8_C(0x02)
#define MPU9250_FIFOEN_P_SLV0_0                            UINT8_C(0)
#define MPU9250_FIFOEN_M_SLV0_0                            UINT8_C(0x01)

#define MPU9250_I2CMSTCTRL_P_I2C_MST_EN_0                  UINT8_C(7)
#define MPU9250_I2CMSTCTRL_M_I2C_MST_EN_0                  UINT8_C(0x80)
#define MPU9250_I2CMSTCTRL_P_WAIT_FOR_ES_0                 UINT8_C(6)
#define MPU9250_I2CMSTCTRL_M_WAIT_FOR_ES_0                 UINT8_C(0x40)
#define MPU9250_I2CMSTCTRL_P_SLV3_FIFO_EN_0                UINT8_C(5)
#define MPU9250_I2CMSTCTRL_M_SLV3_FIFO_EN_0                UINT8_C(0x20)
#define MPU9250_I2CMSTCTRL_P_I2C_MST_P_NSR_0               UINT8_C(4)
#define MPU9250_I2CMSTCTRL_M_I2C_MST_P_NSR_0               UINT8_C(0x10)
#define MPU9250_I2CMSTCTRL_P_I2C_MST_CLK_3_0               UINT8_C(0)
#define MPU9250_I2CMSTCTRL_M_I2C_MST_CLK_3_0               UINT8_C(0x0F)

#define MPU9250_I2CSLV0ADDR_P_RNW_0                        UINT8_C(7)
#define MPU9250_I2CSLV0ADDR_M_RNW_0                        UINT8_C(0x80)
#define MPU9250_I2CSLV0ADDR_P_ID_6_0                       UINT8_C(0)
#define MPU9250_I2CSLV0ADDR_M_ID_6_0                       UINT8_C(0x7F)

#define MPU9250_I2CSLV0CTRL_P_EN_0                         UINT8_C(7)
#define MPU9250_I2CSLV0CTRL_M_EN_0                         UINT8_C(0x80)
#define MPU9250_I2CSLV0CTRL_P_BYTE_SW_0                    UINT8_C(6)
#define MPU9250_I2CSLV0CTRL_M_BYTE_SW_0                    UINT8_C(0x40)
#define MPU9250_I2CSLV0CTRL_P_REG_DIS_0                    UINT8_C(5)
#define MPU9250_I2CSLV0CTRL_M_REG_DIS_0                    UINT8_C(0x20)
#define MPU9250_I2CSLV0CTRL_P_GRP_0                        UINT8_C(4)
#define MPU9250_I2CSLV0CTRL_M_GRP_0                        UINT8_C(0x10)
#define MPU9250_I2CSLV0CTRL_P_LENG_3_0                     UINT8_C(0)
#define MPU9250_I2CSLV0CTRL_M_LENG_3_0                     UINT8_C(0x0F)

#define MPU9250_I2CSLV1ADDR_P_RNW_0                        UINT8_C(7)
#define MPU9250_I2CSLV1ADDR_M_RNW_0                        UINT8_C(0x80)
#define MPU9250_I2CSLV1ADDR_P_ID_6_0                       UINT8_C(0)
#define MPU9250_I2CSLV1ADDR_M_ID_6_0                       UINT8_C(0x7F)

#define MPU9250_I2CSLV1CTRL_P_EN_0                         UINT8_C(7)
#define MPU9250_I2CSLV1CTRL_M_EN_0                         UINT8_C(0x80)
#define MPU9250_I2CSLV1CTRL_P_BYTE_SW_0                    UINT8_C(6)
#define MPU9250_I2CSLV1CTRL_M_BYTE_SW_0                    UINT8_C(0x40)
#define MPU9250_I2CSLV1CTRL_P_REG_DIS_0                    UINT8_C(5)
#define MPU9250_I2CSLV1CTRL_M_REG_DIS_0                    UINT8_C(0x20)
#define MPU9250_I2CSLV1CTRL_P_GRP_0                        UINT8_C(4)
#define MPU9250_I2CSLV1CTRL_M_GRP_0                        UINT8_C(0x10)
#define MPU9250_I2CSLV1CTRL_P_LENG_3_0                     UINT8_C(0)
#define MPU9250_I2CSLV1CTRL_M_LENG_3_0                     UINT8_C(0x0F)

#define MPU9250_I2CSLV2ADDR_P_RNW_0                        UINT8_C(7)
#define MPU9250_I2CSLV2ADDR_M_RNW_0                        UINT8_C(0x80)
#define MPU9250_I2CSLV2ADDR_P_ID_6_0                       UINT8_C(0)
#define MPU9250_I2CSLV2ADDR_M_ID_6_0                       UINT8_C(0x7F)

#define MPU9250_I2CSLV2CTRL_P_EN_0                         UINT8_C(7)
#define MPU9250_I2CSLV2CTRL_M_EN_0                         UINT8_C(0x80)
#define MPU9250_I2CSLV2CTRL_P_BYTE_SW_0                    UINT8_C(6)
#define MPU9250_I2CSLV2CTRL_M_BYTE_SW_0                    UINT8_C(0x40)
#define MPU9250_I2CSLV2CTRL_P_REG_DIS_0                    UINT8_C(5)
#define MPU9250_I2CSLV2CTRL_M_REG_DIS_0                    UINT8_C(0x20)
#define MPU9250_I2CSLV2CTRL_P_GRP_0                        UINT8_C(4)
#define MPU9250_I2CSLV2CTRL_M_GRP_0                        UINT8_C(0x10)
#define MPU9250_I2CSLV2CTRL_P_LENG_3_0                     UINT8_C(0)
#define MPU9250_I2CSLV2CTRL_M_LENG_3_0                     UINT8_C(0x0F)

#define MPU9250_I2CSLV3ADDR_P_RNW_0                        UINT8_C(7)
#define MPU9250_I2CSLV3ADDR_M_RNW_0                        UINT8_C(0x80)
#define MPU9250_I2CSLV3ADDR_P_ID_6_0                       UINT8_C(0)
#define MPU9250_I2CSLV3ADDR_M_ID_6_0                       UINT8_C(0x7F)

#define MPU9250_I2CSLV3CTRL_P_EN_0                         UINT8_C(7)
#define MPU9250_I2CSLV3CTRL_M_EN_0                         UINT8_C(0x80)
#define MPU9250_I2CSLV3CTRL_P_BYTE_SW_0                    UINT8_C(6)
#define MPU9250_I2CSLV3CTRL_M_BYTE_SW_0                    UINT8_C(0x40)
#define MPU9250_I2CSLV3CTRL_P_REG_DIS_0                    UINT8_C(5)
#define MPU9250_I2CSLV3CTRL_M_REG_DIS_0                    UINT8_C(0x20)
#define MPU9250_I2CSLV3CTRL_P_GRP_0                        UINT8_C(4)
#define MPU9250_I2CSLV3CTRL_M_GRP_0                        UINT8_C(0x10)
#define MPU9250_I2CSLV3CTRL_P_LENG_3_0                     UINT8_C(0)
#define MPU9250_I2CSLV3CTRL_M_LENG_3_0                     UINT8_C(0x0F)

#define MPU9250_I2CSLV4ADDR_P_RNW_0                        UINT8_C(7)
#define MPU9250_I2CSLV4ADDR_M_RNW_0                        UINT8_C(0x80)
#define MPU9250_I2CSLV4ADDR_P_ID_6_0                       UINT8_C(0)
#define MPU9250_I2CSLV4ADDR_M_ID_6_0                       UINT8_C(0x7F)

#define MPU9250_I2CSLV4CTRL_P_EN_0                         UINT8_C(7)
#define MPU9250_I2CSLV4CTRL_M_EN_0                         UINT8_C(0x80)
#define MPU9250_I2CSLV4CTRL_P_DONE_INT_EN_0                UINT8_C(6)
#define MPU9250_I2CSLV4CTRL_M_DONE_INT_EN_0                UINT8_C(0x40)
#define MPU9250_I2CSLV4CTRL_P_REG_DIS_0                    UINT8_C(5)
#define MPU9250_I2CSLV4CTRL_M_REG_DIS_0                    UINT8_C(0x20)
#define MPU9250_I2CSLV4CTRL_P_MST_DLY_4_0                  UINT8_C(0)
#define MPU9250_I2CSLV4CTRL_M_MST_DLY_4_0                  UINT8_C(0x1F)

#define MPU9250_I2CMSTSTATUS_P_PASS_THROUGH_0              UINT8_C(7)
#define MPU9250_I2CMSTSTATUS_M_PASS_THROUGH_0              UINT8_C(0x80)
#define MPU9250_I2CMSTSTATUS_P_DONE_0                      UINT8_C(6)
#define MPU9250_I2CMSTSTATUS_M_SLV4_DONE_0                 UINT8_C(0x40)
#define MPU9250_I2CMSTSTATUS_P_LOST_ARB_0                  UINT8_C(5)
#define MPU9250_I2CMSTSTATUS_M_LOST_ARB_0                  UINT8_C(0x20)
#define MPU9250_I2CMSTSTATUS_P_SLV4_NACK_0                 UINT8_C(4)
#define MPU9250_I2CMSTSTATUS_M_SLV4_NACK_0                 UINT8_C(0x10)
#define MPU9250_I2CMSTSTATUS_P_SLV3_NACK_0                 UINT8_C(3)
#define MPU9250_I2CMSTSTATUS_M_SLV3_NACK_0                 UINT8_C(0x08)
#define MPU9250_I2CMSTSTATUS_P_SLV2_NACK_0                 UINT8_C(2)
#define MPU9250_I2CMSTSTATUS_M_SLV2_NACK_0                 UINT8_C(0x04)
#define MPU9250_I2CMSTSTATUS_P_SLV1_NACK_0                 UINT8_C(1)
#define MPU9250_I2CMSTSTATUS_M_SLV1_NACK_0                 UINT8_C(0x02)
#define MPU9250_I2CMSTSTATUS_P_SLV0_NACK_0                 UINT8_C(0)
#define MPU9250_I2CMSTSTATUS_M_SLV0_NACK_0                 UINT8_C(0x01)

#define MPU9250_INTPINCFG_P_ACTL_0                         UINT8_C(7)
#define MPU9250_INTPINCFG_M_ACTL_0                         UINT8_C(0x80)
#define MPU9250_INTPINCFG_P_OPEN_0                         UINT8_C(6)
#define MPU9250_INTPINCFG_M_OPEN_0                         UINT8_C(0x40)
#define MPU9250_INTPINCFG_P_LATCH_INT_EN_0                 UINT8_C(5)
#define MPU9250_INTPINCFG_M_LATCH_INT_EN_0                 UINT8_C(0x20)
#define MPU9250_INTPINCFG_P_INT_ANYRD_2CLEAR_0             UINT8_C(4)
#define MPU9250_INTPINCFG_M_INT_ANYRD_2CLEAR_0             UINT8_C(0x10)
#define MPU9250_INTPINCFG_P_ACTL_FSYNC_0                   UINT8_C(3)
#define MPU9250_INTPINCFG_M_ACTL_FSYNC_0                   UINT8_C(0x08)
#define MPU9250_INTPINCFG_P_FSYNC_INT_MODE_EN_0            UINT8_C(2)
#define MPU9250_INTPINCFG_M_FSYNC_INT_MODE_EN_0            UINT8_C(0x04)
#define MPU9250_INTPINCFG_P_BYPASS_EN_0                    UINT8_C(1)
#define MPU9250_INTPINCFG_M_BYPASS_EN_0                    UINT8_C(0x02)

#define MPU9250_INTENABLE_P_WOM_EN_0                       UINT8_C(6)
#define MPU9250_INTENABLE_M_WOM_EN_0                       UINT8_C(0x40)
#define MPU9250_INTENABLE_P_FIFO_OFLOW_EN_0                UINT8_C(4)
#define MPU9250_INTENABLE_M_FIFO_OFLOW_EN_0                UINT8_C(0x10)
#define MPU9250_INTENABLE_P_FSYNC_INT_EN_0                 UINT8_C(3)
#define MPU9250_INTENABLE_M_FSYNC_INT_EN_0                 UINT8_C(0x08)
#define MPU9250_INTENABLE_P_RAW_RDY_EN_0                   UINT8_C(0)
#define MPU9250_INTENABLE_M_RAW_RDY_EN_0                   UINT8_C(0x01)

#define MPU9250_INTSTATUS_P_WOM_EN_0                       UINT8_C(6)
#define MPU9250_INTSTATUS_M_WOM_EN_0                       UINT8_C(0x40)
#define MPU9250_INTSTATUS_P_FIFO_OFLOW_EN_0                UINT8_C(4)
#define MPU9250_INTSTATUS_M_FIFO_OFLOW_EN_0                UINT8_C(0x10)
#define MPU9250_INTSTATUS_P_FSYNC_INT_EN_0                 UINT8_C(3)
#define MPU9250_INTSTATUS_M_FSYNC_INT_EN_0                 UINT8_C(0x08)
#define MPU9250_INTSTATUS_P_RAW_RDY_EN_0                   UINT8_C(0)
#define MPU9250_INTSTATUS_M_RAW_RDY_EN_0                   UINT8_C(0x01)

#define MPU9250_I2CMSTDELAYCTRL_P_DELAY_ES_SHADOW_0        UINT8_C(7)
#define MPU9250_I2CMSTDELAYCTRL_M_DELAY_ES_SHADOW_0        UINT8_C(0x80)
#define MPU9250_I2CMSTDELAYCTRL_P_SLV4_DLY_EN_0            UINT8_C(4)
#define MPU9250_I2CMSTDELAYCTRL_M_SLV4_DLY_EN_0            UINT8_C(0x10)
#define MPU9250_I2CMSTDELAYCTRL_P_SLV3_DLY_EN_0            UINT8_C(3)
#define MPU9250_I2CMSTDELAYCTRL_M_SLV3_DLY_EN_0            UINT8_C(0x08)
#define MPU9250_I2CMSTDELAYCTRL_P_SLV2_DLY_EN_0            UINT8_C(2)
#define MPU9250_I2CMSTDELAYCTRL_M_SLV2_DLY_EN_0            UINT8_C(0x04)
#define MPU9250_I2CMSTDELAYCTRL_P_SLV1_DLY_EN_0            UINT8_C(1)
#define MPU9250_I2CMSTDELAYCTRL_M_SLV1_DLY_EN_0            UINT8_C(0x02)
#define MPU9250_I2CMSTDELAYCTRL_P_SLV0_DLY_EN_0            UINT8_C(0)
#define MPU9250_I2CMSTDELAYCTRL_M_SLV0_DLY_EN_0            UINT8_C(0x01)

#define MPU9250_SIGNALPATHRESET_P_GYRO_RST_0               UINT8_C(2)
#define MPU9250_SIGNALPATHRESET_M_GYRO_RST_0               UINT8_C(0x04)
#define MPU9250_SIGNALPATHRESET_P_ACCEL_RST_0              UINT8_C(1)
#define MPU9250_SIGNALPATHRESET_M_ACCEL_RST_0              UINT8_C(0x02)
#define MPU9250_SIGNALPATHRESET_P_TEMP_RST_0               UINT8_C(0)
#define MPU9250_SIGNALPATHRESET_M_TEMP_RST_0               UINT8_C(0x01)

#define MPU9250_MOTDETECTCTRL_P_ACCEL_INTEL_EN_0           UINT8_C(7)
#define MPU9250_MOTDETECTCTRL_M_ACCEL_INTEL_EN_0           UINT8_C(0x80)
#define MPU9250_MOTDETECTCTRL_P_ACCEL_INTEL_MODE_0         UINT8_C(6)
#define MPU9250_MOTDETECTCTRL_M_ACCEL_INTEL_MODE_0         UINT8_C(0x40)

#define MPU9250_USERCTRL_P_FIFO_EN_0                       UINT8_C(6)
#define MPU9250_USERCTRL_M_FIFO_EN_0                       UINT8_C(0x40)
#define MPU9250_USERCTRL_P_I2C_MST_EN_0                    UINT8_C(5)
#define MPU9250_USERCTRL_M_I2C_MST_EN_0                    UINT8_C(0x20)
#define MPU9250_USERCTRL_P_I2C_IF_DIS_0                    UINT8_C(4)
#define MPU9250_USERCTRL_M_I2C_IF_DIS_0                    UINT8_C(0x10)
#define MPU9250_USERCTRL_P_FIFO_RST_0                      UINT8_C(2)
#define MPU9250_USERCTRL_M_FIFO_RST_0                      UINT8_C(0x04)
#define MPU9250_USERCTRL_P_I2C_MST_RST_0                   UINT8_C(1)
#define MPU9250_USERCTRL_M_I2C_MST_RST_0                   UINT8_C(0x02)
#define MPU9250_USERCTRL_P_SIG_COND_RST_0                  UINT8_C(0)
#define MPU9250_USERCTRL_M_SIG_COND_RST_0                  UINT8_C(0x01)

#define MPU9250_PWRMGMT1_P_H_RST_0                         UINT8_C(7)
#define MPU9250_PWRMGMT1_M_H_RST_0                         UINT8_C(0x80)
#define MPU9250_PWRMGMT1_P_SLEEP_0                         UINT8_C(6)
#define MPU9250_PWRMGMT1_M_SLEEP_0                         UINT8_C(0x40)
#define MPU9250_PWRMGMT1_P_CYCLE_0                         UINT8_C(5)
#define MPU9250_PWRMGMT1_M_CYCLE_0                         UINT8_C(0x20)
#define MPU9250_PWRMGMT1_P_GYRO_STANDBY_0                  UINT8_C(4)
#define MPU9250_PWRMGMT1_M_GYRO_STANDBY_0                  UINT8_C(0x10)
#define MPU9250_PWRMGMT1_P_PD_PTAT_0                       UINT8_C(3)
#define MPU9250_PWRMGMT1_M_PD_PTAT_0                       UINT8_C(0x08)
#define MPU9250_PWRMGMT1_P_CLKSEL_2_0                      UINT8_C(0)
#define MPU9250_PWRMGMT1_M_CLKSEL_2_0                      UINT8_C(0x07)

#define MPU9250_PWRMGMT2_P_DIS_XA_0                        UINT8_C(5)
#define MPU9250_PWRMGMT2_M_DIS_XA_0                        UINT8_C(0x20)
#define MPU9250_PWRMGMT2_P_DIS_YA_0                        UINT8_C(4)
#define MPU9250_PWRMGMT2_M_DIS_YA_0                        UINT8_C(0x10)
#define MPU9250_PWRMGMT2_P_DIS_ZA_0                        UINT8_C(3)
#define MPU9250_PWRMGMT2_M_DIS_ZA_0                        UINT8_C(0x08)
#define MPU9250_PWRMGMT2_P_DIS_XG_0                        UINT8_C(2)
#define MPU9250_PWRMGMT2_M_DIS_XG_0                        UINT8_C(0x04)
#define MPU9250_PWRMGMT2_P_DIS_YG_0                        UINT8_C(1)
#define MPU9250_PWRMGMT2_M_DIS_YG_0                        UINT8_C(0x02)
#define MPU9250_PWRMGMT2_P_DIS_ZG_0                        UINT8_C(0)
#define MPU9250_PWRMGMT2_M_DIS_ZG_0                        UINT8_C(0x01)

#define MPU9250_FIFOCNTH_P_12_8                            UINT8_C(0)
#define MPU9250_FIFOCNTH_M_12_8                            UINT8_C(0x1F)

#define MPU9250_XAOFFSL_P_6_0                              UINT8_C(1)
#define MPU9250_XAOFFSL_M_6_0                              UINT8_C(0xFE)
#define MPU9250_YAOFFSL_P_6_0                              UINT8_C(1)
#define MPU9250_YAOFFSL_M_6_0                              UINT8_C(0xFE)
#define MPU9250_ZAOFFSL_P_6_0                              UINT8_C(1)
#define MPU9250_ZAOFFSL_M_6_0                              UINT8_C(0xFE)

//OPTIONS
#define MPU9250_O_EXT_SYNC_SET_DISABLE                     UINT8_C(0x00)
#define MPU9250_O_EXT_SYNC_SET_TEMP_OUT_L                  UINT8_C(0x01)
#define MPU9250_O_EXT_SYNC_SET_GYRO_XOUT_L                 UINT8_C(0x02)
#define MPU9250_O_EXT_SYNC_SET_GYRO_YOUT_L                 UINT8_C(0x03)
#define MPU9250_O_EXT_SYNC_SET_GYRO_ZOUT_L                 UINT8_C(0x04)
#define MPU9250_O_EXT_SYNC_SET_ACCEL_XOUT_L                UINT8_C(0x05)
#define MPU9250_O_EXT_SYNC_SET_ACCEL_YOUT_L                UINT8_C(0x06)
#define MPU9250_O_EXT_SYNC_SET_ACCEL_ZOUT_L                UINT8_C(0x07)

#define MPU9250_O_DLPF_CFG_0                               UINT8_C(0x00)
#define MPU9250_O_DLPF_CFG_1                               UINT8_C(0x01)
#define MPU9250_O_DLPF_CFG_2                               UINT8_C(0x02)
#define MPU9250_O_DLPF_CFG_3                               UINT8_C(0x03)
#define MPU9250_O_DLPF_CFG_4                               UINT8_C(0x04)
#define MPU9250_O_DLPF_CFG_5                               UINT8_C(0x05)
#define MPU9250_O_DLPF_CFG_6                               UINT8_C(0x06)
#define MPU9250_O_DLPF_CFG_7                               UINT8_C(0x07)

#define MPU9250_O_GYR0_FS_SEL_250_DPS                      UINT8_C(0x00)
#define MPU9250_O_GYRO_FS_SEL_500_DPS                      UINT8_C(0x01)
#define MPU9250_O_GYRO_FS_SEL_1000_DPS                     UINT8_C(0x02)
#define MPU9250_O_GYRO_FS_SEL_2000_DPS                     UINT8_C(0x03)

#define MPU9250_O_G_RES_250DPS                             0.0076293945312
#define MPU9250_O_G_RES_500DPS                             0.0152587890625
#define MPU9250_O_G_RES_1000DPS                            0.030517578125
#define MPU9250_O_G_RES_2000DPS                            0.06103515625

#define MPU9250_O_FCHOICE_B_0                              UINT8_C(0x00)
#define MPU9250_O_FCHOICE_B_1                              UINT8_C(0x01)
#define MPU9250_O_FCHOICE_B_2                              UINT8_C(0x02)
#define MPU9250_O_FCHOICE_B_3                              UINT8_C(0x03)

#define MPU9250_O_ACCEL_FS_SEL_2_G                         UINT8_C(0x00)
#define MPU9250_O_ACCEL_FS_SEL_4_G                         UINT8_C(0x01)
#define MPU9250_O_ACCEL_FS_SEL_8_G                         UINT8_C(0x02)
#define MPU9250_O_ACCEL_FS_SEL_16_G                        UINT8_C(0x03)

#define MPU9250_O_A_RES_2G                                 0.00006103515625
#define MPU9250_O_A_RES_4G                                 0.0001220703125
#define MPU9250_O_A_RES_8G                                 0.000244140625
#define MPU9250_O_A_RES_16G                                0.00048828125

#define MPU9250_O_ODR_0                                    UINT8_C(0x00)
#define MPU9250_O_ODR_1                                    UINT8_C(0x01)
#define MPU9250_O_ODR_2                                    UINT8_C(0x02)
#define MPU9250_O_ODR_3                                    UINT8_C(0x03)
#define MPU9250_O_ODR_4                                    UINT8_C(0x04)
#define MPU9250_O_ODR_5                                    UINT8_C(0x05)
#define MPU9250_O_ODR_6                                    UINT8_C(0x06)
#define MPU9250_O_ODR_7                                    UINT8_C(0x07)
#define MPU9250_O_ODR_8                                    UINT8_C(0x08)
#define MPU9250_O_ODR_9                                    UINT8_C(0x09)
#define MPU9250_O_ODR_10                                   UINT8_C(0x0A)
#define MPU9250_O_ODR_11                                   UINT8_C(0x0B)

#define MPU9250_O_I2C_MST_CLK_0                            UINT8_C(0x00)
#define MPU9250_O_I2C_MST_CLK_1                            UINT8_C(0x01)
#define MPU9250_O_I2C_MST_CLK_2                            UINT8_C(0x02)
#define MPU9250_O_I2C_MST_CLK_3                            UINT8_C(0x03)
#define MPU9250_O_I2C_MST_CLK_4                            UINT8_C(0x04)
#define MPU9250_O_I2C_MST_CLK_5                            UINT8_C(0x05)
#define MPU9250_O_I2C_MST_CLK_6                            UINT8_C(0x06)
#define MPU9250_O_I2C_MST_CLK_7                            UINT8_C(0x07)
#define MPU9250_O_I2C_MST_CLK_8                            UINT8_C(0x08)
#define MPU9250_O_I2C_MST_CLK_9                            UINT8_C(0x09)
#define MPU9250_O_I2C_MST_CLK_10                           UINT8_C(0x0A)
#define MPU9250_O_I2C_MST_CLK_11                           UINT8_C(0x0B)
#define MPU9250_O_I2C_MST_CLK_12                           UINT8_C(0x0C)
#define MPU9250_O_I2C_MST_CLK_13                           UINT8_C(0x0D)
#define MPU9250_O_I2C_MST_CLK_14                           UINT8_C(0x0E)
#define MPU9250_O_I2C_MST_CLK_15                           UINT8_C(0x0F)

#define MPU9250_O_CLKSEL_0                                 UINT8_C(0x00)
#define MPU9250_O_CLKSEL_1                                 UINT8_C(0x01)
#define MPU9250_O_CLKSEL_2                                 UINT8_C(0x02)
#define MPU9250_O_CLKSEL_3                                 UINT8_C(0x03)
#define MPU9250_O_CLKSEL_4                                 UINT8_C(0x04)
#define MPU9250_O_CLKSEL_5                                 UINT8_C(0x05)
#define MPU9250_O_CLKSEL_6                                 UINT8_C(0x06)
#define MPU9250_O_CLKSEL_7                                 UINT8_C(0x07)

#define MPU9250_O_WHO_AM_I_ID                              UINT8_C(0x71)



//I2C ADDRESS
#define AK8963_I2C_PRIMARY_ADDR                                 UINT8_C(0x0C)

//REGISTERS
#define AK8963_RF_WHO_AM_I_R                                    UINT8_C(0x00) // should return 0x48
#define AK8963_R_INFO_R                                         UINT8_C(0x01)
#define AK8963_R_ST1_R                                          UINT8_C(0x02)  // data ready status bit 0
#define AK8963_RF_XOUT_L_R                                      UINT8_C(0x03)  // data
#define AK8963_RF_XOUT_H_R                                      UINT8_C(0x04)
#define AK8963_RF_YOUT_L_R                                      UINT8_C(0x05)
#define AK8963_RF_YOUT_H_R                                      UINT8_C(0x06)
#define AK8963_RF_ZOUT_L_R                                      UINT8_C(0x07)
#define AK8963_RF_ZOUT_H_R                                      UINT8_C(0x08)
#define AK8963_RF_ST2_R                                         UINT8_C(0x09)  // Data overflow bit 3 and data read error status bit 2
#define AK8963_R_CNTL_RW                                        UINT8_C(0x0A)  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_RF_CNTL2_RW                                      UINT8_C(0x0B)  // Reset
#define AK8963_RF_ASTC_RW                                       UINT8_C(0x0C)  // Self test control
#define AK8963_RF_I2CDIS_RW                                     UINT8_C(0x0F)  // I2C disable
#define AK8963_RF_ASAX_R                                        UINT8_C(0x10)  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_RF_ASAY_R                                        UINT8_C(0x11)  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_RF_ASAZ_R                                        UINT8_C(0x12)  // Fuse ROM z-axis sensitivity adjustment value

//POS AND MASK
#define AK8963_ST1_P_DOR_0                                      UINT8_C(1)
#define AK8963_ST1_M_DOR_0                                      UINT8_C(0x02)
#define AK8963_ST1_P_DRDY_0                                     UINT8_C(0)
#define AK8963_ST1_M_DRDY_0                                     UINT8_C(0x01)

#define AK8963_ST2_P_HOFL_0                                     UINT8_C(3)
#define AK8963_ST2_M_HOFL_0                                     UINT8_C(0x08)
#define AK8963_ST2_P_BITM_0                                     UINT8_C(4)
#define AK8963_ST2_M_BITM_0                                     UINT8_C(0x10)

#define AK8963_CNTL1_P_BIT_0                                    UINT8_C(4)
#define AK8963_CNTL1_M_BIT_0                                    UINT8_C(0x10)
#define AK8963_CNTL1_P_MODE_3_0                                 UINT8_C(0)
#define AK8963_CNTL1_M_MODE_3_0                                 UINT8_C(0x0F)

#define AK8963_CNTL2_P_SRST_0                                   UINT8_C(0)
#define AK8963_CNTL2_M_SRST_0                                   UINT8_C(0x01)

#define AK8963_ASTC_P_SELF_0                                    UINT8_C(6)
#define AK8963_ASTC_M_SELF_0                                    UINT8_C(0x40)

//OPTIONS
#define AK8963_O_CNTL1MODE_POWERDWN                             UINT8_C(0x00)
#define AK8963_O_CNTL1MODE_SNGL_MSRMT                           UINT8_C(0x01)
#define AK8963_O_CNTL1MODE_CONT_MSRMT1                          UINT8_C(0x02)
#define AK8963_O_CNTL1MODE_CONT_MSRMT2                          UINT8_C(0x06)
#define AK8963_O_CNTL1MODE_EXT_TRIG_MSRMT                       UINT8_C(0x04)
#define AK8963_O_CNTL1MODE_SELF_TEST                            UINT8_C(0x08)
#define AK8963_O_CNTL1MODE_FUSE_ACCESS                          UINT8_C(0x0F)

#define AK8963_O_WHO_AM_I_ID                                    UINT8_C(0x48)
#define AK8963_O_M_RES_14                                       5.9975579975579
#define AK8963_O_M_RES_16                                       1.4993894993894

#define AK8963_O_M_8Hz                                          UINT8_C(0x02)
#define AK8963_O_M_100Hz                                        UINT8_C(0x06)


class MPU9250
{

public:
/*
    struct config
    {
        int8_t SMPLRDIV;
        int8_t EXT_SYNC_SET;
        int8_t G_Fchoice;
        int8_t A_Fchoice;
        int8_t A_DLPF;
        int8_t G_DLPF;
        int8_t G_FS;
        int8_t A_FS;
        int8_t lposc_clksel;

    };
*/

    int8_t piHandle;
    uint8_t i2cHandle;
    //PUT CALIBRATION VALUES HERE

    MPU9250(int8_t address);
    ~MPU9250();
    int8_t checkMPU9250Comms();
    int8_t resetMPU9250();
    int8_t checkAK8963Comms();
    int8_t resetAK8963();
    int8_t setMPU9250Config(uint8_t sampleRate, uint8_t Ascale, uint8_t Gscale);
    int8_t setAK8963Config(const uint8_t& scale, const uint8_t& mode, float* magCal);
    int8_t getST();
    int8_t calibrateMag(uint8_t mode, float mRes, float* magCal, int32_t* magBiasADC,float* magBiasDisp, float* magScaleDisp);
    int8_t getMagUncalADC(int16_t* mdata);
    int8_t calibrateAccelGyro(float* gyro_offs, float* accel_offs, int16_t* gyro_temp, int16_t* accel_temp);
    int8_t getAccelGyroTempCalADC(int16_t* adata , int16_t* gdata, int16_t &tdata);
    bool checkNewAccelGyroData();
    bool checkNewMagData();
    float getGres(uint8_t Gscale);
    float getAres(uint8_t Ascale);
};


#endif // MPU9250_H
