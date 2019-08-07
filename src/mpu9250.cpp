#include "mpu9250.h"
#include <math.h>
#include <unistd.h>
#include <stdint.h>
#include <iostream>
#include <pigpiod_if2.h>

MPU9250::MPU9250(int8_t address)
{
    std::cout<< "[INFO]: mpu is constructed!"<<std::endl;
    piHandle = pigpio_start(0, 0);
    if(piHandle < 0) std::cout<<"[ERROR]: Cannot find pigpio daemon! Please start daemon and run start sequence again!"<<std::endl;
    i2cHandle =i2c_open(piHandle, 1, address, 0);
}

MPU9250::~MPU9250()
{
    i2c_close(piHandle, i2cHandle);
    pigpio_stop(piHandle);
}

int8_t MPU9250::checkMPU9250Comms()
{
    int8_t rslt = 0;
    uint8_t op = 0;
    uint8_t tryCount = 5;
    while(tryCount)
    {
        std::cout<<"[INFO]: communication attempt remaining: "<<(uint16_t)tryCount<<std::endl;
        op = i2c_read_byte_data(piHandle, i2cHandle, MPU9250_RF_WHO_AM_I_R);
        if(op<0)
            rslt = 1;
        if(rslt == 0)
        {
            if(op == MPU9250_O_WHO_AM_I_ID)
            {
                std::cout<<"[INFO]: communications working cool !"<<std::endl;
                break;

            }

            usleep(10000);
            --tryCount;
        }
        else
        {
            std::cout<<"[ERROR]: unable to read who am i register"<<std::endl;

        }
    }
    if(op!= MPU9250_O_WHO_AM_I_ID)
    {
        std::cout<<"[ERROR]: unable to communicate with MPU9250"<<std::endl;

    }
    return rslt;
}

int8_t MPU9250::checkAK8963Comms()
{
    int8_t rslt = 0;
    uint8_t op = 0;
    uint8_t tryCount = 5;
    while(tryCount)
    {   std::cout<<"[INFO]: communication attempt remaining: "<<(uint16_t)tryCount<<std::endl;
        rslt = i2c_write_byte_data(piHandle, i2cHandle, MPU9250_R_I2C_SLV0_ADDR_RW, AK8963_I2C_PRIMARY_ADDR|0x80);
        if(rslt == 0)
            rslt = i2c_write_byte_data(piHandle, i2cHandle, MPU9250_RF_I2C_SLV0_REG_RW, AK8963_RF_WHO_AM_I_R);
        if(rslt == 0)
            rslt = i2c_write_byte_data(piHandle, i2cHandle, MPU9250_R_I2C_SLV0_CTRL_RW,
                                       0x01<<MPU9250_I2CSLV0CTRL_P_EN_0|0x01<<MPU9250_I2CSLV0CTRL_P_LENG_3_0);
        usleep(10000);//10ms
        op = i2c_read_byte_data(piHandle, i2cHandle, MPU9250_RF_EXT_SENS_DATA_00_R);
        if(rslt == 0)
        {
            if(op == AK8963_O_WHO_AM_I_ID)
            {
                std::cout<<"[INFO]: communications working cool !"<<std::endl;
                break;
            }
            usleep(10000);
            --tryCount;
        }
        else
        {
            std::cout<<"[ERROR]: unable to read who am i register"<<std::endl;
        }
    }
    if(op!= AK8963_O_WHO_AM_I_ID)
    {
        std::cout<<"[ERROR]: unable to communicate with MPU9250"<<std::endl;
    }
    return rslt;
}


int8_t MPU9250::resetMPU9250()
{
    int8_t rslt = 0;
    rslt = i2c_write_byte_data(piHandle, i2cHandle, MPU9250_R_PWR_MGMT_1_RW, MPU9250_PWRMGMT1_M_H_RST_0);
    usleep(100000);//100ms
    return rslt;
}

int8_t MPU9250::resetAK8963()
{
    int8_t rslt = 0;
    rslt = i2c_write_byte_data(piHandle, i2cHandle,MPU9250_R_I2C_SLV0_ADDR_RW, AK8963_I2C_PRIMARY_ADDR);
    if(rslt == 0)
        rslt = i2c_write_byte_data(piHandle, i2cHandle,MPU9250_RF_I2C_SLV0_REG_RW, AK8963_RF_CNTL2_RW);
    if(rslt == 0)
        rslt = i2c_write_byte_data(piHandle, i2cHandle,MPU9250_RF_I2C_SLV0_DO_RW, 0x01);
    if(rslt == 0)
        rslt = i2c_write_byte_data(piHandle, i2cHandle,MPU9250_R_I2C_SLV0_CTRL_RW,
                                0x01<<MPU9250_I2CSLV0CTRL_P_EN_0|0X01<<MPU9250_I2CSLV0CTRL_P_LENG_3_0);
    usleep(100000);//100ms
    return rslt;

}


bool MPU9250::checkNewMagData()
{
    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_R_I2C_SLV0_ADDR_RW, AK8963_I2C_PRIMARY_ADDR|0x80);
    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_RF_I2C_SLV0_REG_RW, AK8963_R_ST1_R);
    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_R_I2C_SLV0_CTRL_RW,
                     0x01<<MPU9250_I2CSLV0CTRL_P_EN_0|0x01<<MPU9250_I2CSLV0CTRL_P_LENG_3_0);
    return ((bool)(i2c_read_byte_data(piHandle, i2cHandle, MPU9250_RF_EXT_SENS_DATA_00_R)&0x01));
}

bool MPU9250::checkNewAccelGyroData()
{
    return ((bool) (i2c_read_byte_data(piHandle, i2cHandle, MPU9250_R_INT_STATUS_R) & 0x01));
}



float MPU9250::getAres(uint8_t Ascale)
{
    float val = 0.0f;

    switch(Ascale)
    {
    case MPU9250_O_ACCEL_FS_SEL_2_G:
        val = (2.0f/32768.0f);

    case MPU9250_O_ACCEL_FS_SEL_4_G:
        val = (4.0f/32768.0f);

    case MPU9250_O_ACCEL_FS_SEL_8_G:
        val = (8.0f/32768.0f);

    case MPU9250_O_ACCEL_FS_SEL_16_G:
        val = (16.0f/32768.0f);
    }

    return val;
}

float MPU9250::getGres(uint8_t Gscale)
{
    float val = 0.0f;

    switch(Gscale)
    {
    case MPU9250_O_GYR0_FS_SEL_250_DPS:
        val = (250.0f/32768.0f);

    case MPU9250_O_GYRO_FS_SEL_500_DPS:
        val = (500.0f/32768.0f);

    case MPU9250_O_GYRO_FS_SEL_1000_DPS:
        val = (1000.0f/32768.0f);

    case MPU9250_O_GYRO_FS_SEL_2000_DPS:
        val = (2000.0f/32768.0f);

    }

    return val;
}

int8_t MPU9250::getST()
{
    char raw[] = {0, 0, 0, 0, 0, 0};
    uint8_t selfTest[] = {0, 0, 0, 0, 0, 0};
    int32_t aAvg[] = {0, 0, 0};
    int32_t gAvg[] = {0, 0, 0};
    int32_t aSTAvg[] = {0, 0, 0};
    int32_t gSTAvg[] = {0, 0, 0};
    float FactoryTrim[] = {0, 0, 0, 0, 0, 0};
    float op[] = {0, 0, 0, 0, 0, 0};

    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_RF_SMPLRT_DIV_RW, 0x00);
    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_R_CONFIG_RW, MPU9250_O_DLPF_CFG_2<<MPU9250_CONFIG_P_DLPF_CFG_2_0);
    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_R_GYRO_CONFIG_RW, MPU9250_O_GYR0_FS_SEL_250_DPS << MPU9250_GYROCFG_P_GYRO_FS_SEL_1_0);
    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_R_ACCEL_CONFIG_RW, MPU9250_O_ACCEL_FS_SEL_2_G << MPU9250_ACCELCFG_P_ACCEL_FS_SEL_1_0);
    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_R_ACCEL_CONFIG2_RW, MPU9250_O_DLPF_CFG_2<<MPU9250_ACCELCFG1_P_DLPF_B_1_0);

    for (int ii = 0; ii < 200; ii++)
    {
        i2c_read_i2c_block_data(piHandle, i2cHandle, MPU9250_RF_ACCEL_XOUT_H_R, raw, 6);
        aAvg[0] += (int16_t)((((int16_t)raw[0]<<8))|((int8_t)raw[1]));
        aAvg[1] += (int16_t)((((int16_t)raw[2]<<8))|((int8_t)raw[3]));
        aAvg[2] += (int16_t)((((int16_t)raw[4]<<8))|((int8_t)raw[5]));

        i2c_read_i2c_block_data(piHandle, i2cHandle, MPU9250_RF_GYRO_XOUT_H_R, raw, 6);
        gAvg[0] += (int16_t)((((int16_t)raw[0]<<8))|((int8_t)raw[1]));
        gAvg[1] += (int16_t)((((int16_t)raw[2]<<8))|((int8_t)raw[3]));
        gAvg[2] += (int16_t)((((int16_t)raw[4]<<8))|((int8_t)raw[5]));

    }

    for (int ii = 0; ii < 3; ii++)
    {
        aAvg[ii] /=200;
        gAvg[ii] /=200;
    }

    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_R_ACCEL_CONFIG_RW, 0xE0);
    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_R_GYRO_CONFIG_RW, 0xE0);
    usleep(25000);

    for (int ii = 0; ii < 200; ii++)
    {
        i2c_read_i2c_block_data(piHandle, i2cHandle, MPU9250_RF_ACCEL_XOUT_H_R, raw, 6);
        aSTAvg[0] += (int16_t)((((int16_t)raw[0])<<8)|((int8_t)raw[1]));
        aSTAvg[1] += (int16_t)((((int16_t)raw[2])<<8)|((int8_t)raw[3]));
        aSTAvg[2] += (int16_t)((((int16_t)raw[4])<<8)|((int8_t)raw[5]));

        i2c_read_i2c_block_data(piHandle, i2cHandle, MPU9250_RF_GYRO_XOUT_H_R, raw, 6);
        gSTAvg[0] += (int16_t)((((int16_t)raw[0])<<8)|((int8_t)raw[1]));
        gSTAvg[1] += (int16_t)((((int16_t)raw[2])<<8)|((int8_t)raw[3]));
        gSTAvg[2] += (int16_t)((((int16_t)raw[4])<<8)|((int8_t)raw[5]));

    }

    for (int ii = 0; ii < 3; ii++)
    {
        aSTAvg[ii] /=200;
        gSTAvg[ii] /=200;
    }

    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_R_ACCEL_CONFIG_RW, 0X00);
    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_R_GYRO_CONFIG_RW, 0x00);
    usleep(25000);
    /*
    i2cReadI2CBlockData(piHandle, i2cHandle, MPU9250_RF_SELF_TEST_X_ACCEL_RW, raw, 3);

    for (int ii = 0; ii < 3; ii++)
    {
        selfTest[ii] = (int8_t)raw[ii];
    }

    i2cReadI2CBlockData(piHandle, i2cHandle, MPU9250_RF_SELF_TEST_X_GYRO_RW, raw, 3);

    for (int ii = 3; ii < 6; ii++)
    {
        selfTest[ii] = (int8_t)raw[ii];
    }
    */

    selfTest[0] = i2c_read_byte_data(piHandle, i2cHandle, MPU9250_RF_SELF_TEST_X_ACCEL_RW);
    selfTest[1] = i2c_read_byte_data(piHandle, i2cHandle, MPU9250_RF_SELF_TEST_Y_ACCEL_RW);
    selfTest[2] = i2c_read_byte_data(piHandle, i2cHandle, MPU9250_RF_SELF_TEST_Z_ACCEL_RW);
    selfTest[3] = i2c_read_byte_data(piHandle, i2cHandle, MPU9250_RF_SELF_TEST_X_GYRO_RW);
    selfTest[4] = i2c_read_byte_data(piHandle, i2cHandle, MPU9250_RF_SELF_TEST_Y_GYRO_RW);
    selfTest[5] = i2c_read_byte_data(piHandle, i2cHandle, MPU9250_RF_SELF_TEST_Z_GYRO_RW);

    for (int ii = 0; ii < 6; ii++)
    {
        FactoryTrim[ii] = (float)(2620 * (pow(1.01, ((float)selfTest[ii] - 1.0))));
    }

    for(int ii = 0; ii < 3; ii++)
    {
        op[ii] = 100.0f * ((float)(aSTAvg[ii] - aAvg[ii]))/FactoryTrim[ii];// - 100.0f;
        op[ii + 3] = 100.0f * ((float)(gSTAvg[ii] - gAvg[ii]))/FactoryTrim[ii + 3];// - 100.0f;

    }
    std::cout<<"[INFO]: Self Test Percentage Results for X, Accel :"<<op[0]<<" Gyro :"<<op[3]<<std::endl;
    std::cout<<"[INFO]: Self Test Percentage Results for Y, Accel :"<<op[1]<<" Gyro :"<<op[4]<<std::endl;
    std::cout<<"[INFO]: Self Test Percentage Results for Z, Accel :"<<op[2]<<" Gyro :"<<op[5]<<std::endl;

    if((0 <= op[5]) && (op[5] <= 100) && (0 <= op[4]) && (op[4] <= 100) && (0 <= op[3]) && (op[3] <= 100) &&
            (0 <= op[2]) && (op[2] <= 100) && (0 <= op[1]) && (op[1] <= 100) && (0 <= op[0]) && (op[0] <= 100))
    {
        std::cout<<"[INFO] :Self test is successful!"<<std::endl;
        return 0;
    }
    else
    {
        std::cout<<"[ERROR] :Self test has failed!"<<std::endl;
        return 1;
    }

}


int8_t MPU9250::calibrateAccelGyro(float* gyro_d_offs, float* accel_d_offs, int16_t* gyro_bits, int16_t* accel_bits)
{
    char raw[12];
    uint16_t packet_count, fifo_count;
    int32_t gyro_bias[] = {0, 0, 0};
    int32_t accel_bias[] = {0, 0, 0};

    uint16_t gyro_sensitivity = 131;
    uint16_t accel_sensitivity = 16384;

    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_R_PWR_MGMT_1_RW,
                     MPU9250_ENABLE_FLAG<<MPU9250_PWRMGMT1_P_H_RST_0);
    usleep(100000);
    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_R_PWR_MGMT_1_RW, 0x01);
    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_R_PWR_MGMT_2_RW, 0x00);
    usleep(200000);
    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_R_INT_ENABLE_RW, 0x00);
    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_R_FIFO_EN_RW, 0x00);
    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_R_PWR_MGMT_1_RW, 0x00);
    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_R_I2C_MST_CTRL_RW, 0x00);
    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_R_USER_CTRL_RW, 0x00);
    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_R_USER_CTRL_RW, 0x0C);
    usleep(15000);

    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_R_CONFIG_RW, 0x01);
    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_RF_SMPLRT_DIV_RW, 0x00);
    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_R_GYRO_CONFIG_RW, 0x00);
    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_R_ACCEL_CONFIG_RW, 0x00);

    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_R_USER_CTRL_RW, 0x40);
    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_R_FIFO_EN_RW, 0x78);
    usleep(40000);
    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_R_FIFO_EN_RW, 0x00);
    i2c_read_i2c_block_data(piHandle, i2cHandle, MPU9250_RF_FIFO_COUNTH_RW, raw, 2);
    fifo_count = ((uint16_t)raw[0]<<8|(uint8_t)raw[1]);
    packet_count = fifo_count/12;

    int16_t accel_temp[3], gyro_temp[3];
    for (int ii = 0;ii < packet_count;ii++)
    {
        i2c_read_i2c_block_data(piHandle, i2cHandle, MPU9250_RF_FIFO_R_W_RW, raw, 12);
        accel_temp[0] = (int16_t)(((int16_t)raw[0]<<8)|(int8_t)raw[1]);
        accel_temp[1] = (int16_t)(((int16_t)raw[2]<<8)|(int8_t)raw[3]);
        accel_temp[2] = (int16_t)(((int16_t)raw[4]<<8)|(int8_t)raw[5]);
        gyro_temp[0]  = (int16_t)(((int16_t)raw[6]<<8)|(int8_t)raw[7]);
        gyro_temp[1]  = (int16_t)(((int16_t)raw[8]<<8)|(int8_t)raw[9]);
        gyro_temp[2]  = (int16_t)(((int16_t)raw[10]<<8)|(int8_t)raw[11]);

        accel_bias[0] += (int32_t) accel_temp[0];
        accel_bias[1] += (int32_t) accel_temp[1];
        accel_bias[2] += (int32_t) accel_temp[2];
        gyro_bias[0]  += (int32_t) gyro_temp[0];
        gyro_bias[1]  += (int32_t) gyro_temp[1];
        gyro_bias[2]  += (int32_t) gyro_temp[2];
    }

    accel_bias[0] /= (int32_t)packet_count;
    accel_bias[1] /= (int32_t)packet_count;
    accel_bias[2] /= (int32_t)packet_count;
    gyro_bias[0]  /= (int32_t)packet_count;
    gyro_bias[1]  /= (int32_t)packet_count;
    gyro_bias[2]  /= (int32_t)packet_count;


    gyro_bits[0] = (-gyro_bias[0]/4 >> 8) & 0xFF;
    gyro_bits[1] = (-gyro_bias[0]/4)      & 0xFF;
    gyro_bits[2] = (-gyro_bias[1]/4 >> 8) & 0xFF;
    gyro_bits[3] = (-gyro_bias[1]/4)      & 0xFF;
    gyro_bits[4] = (-gyro_bias[2]/4 >> 8) & 0xFF;
    gyro_bits[5] = (-gyro_bias[2]/4)      & 0xFF;

    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_RF_XG_OFFSET_H_RW,gyro_bits[0]);
    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_RF_XG_OFFSET_L_RW,gyro_bits[1]);
    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_RF_YG_OFFSET_H_RW,gyro_bits[2]);
    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_RF_YG_OFFSET_L_RW,gyro_bits[3]);
    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_RF_ZG_OFFSET_H_RW,gyro_bits[4]);
    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_RF_ZG_OFFSET_L_RW,gyro_bits[5]);

    //char buf[] = {(char)gyro_temp[0], (char)gyro_temp[1], (char)gyro_temp[2], (char)gyro_temp[3], (char)gyro_temp[4], (char)gyro_temp[5]};
    //i2cWriteI2CBlockData(piHandle, i2cHandle, MPU9250_RF_XG_OFFSET_H_RW, buf, 6);

    gyro_d_offs[0] = (float) gyro_bias[0]/(float) gyro_sensitivity;
    gyro_d_offs[1] = (float) gyro_bias[1]/(float) gyro_sensitivity;
    gyro_d_offs[2] = (float) gyro_bias[2]/(float) gyro_sensitivity;

    if(accel_bias[2] > 0) accel_bias[2] -= (int32_t)accel_sensitivity;
    else accel_bias[2] += (int32_t)accel_sensitivity;

    int16_t accel_bias_reg[] = {0, 0, 0};
    int16_t mask_bit[] = {1, 1, 1};
    accel_bias_reg[0] = (i2c_read_byte_data(piHandle, i2cHandle, MPU9250_RF_XA_OFFSET_H_RW)<<8 | i2c_read_byte_data(piHandle, i2cHandle, MPU9250_RF_XA_OFFSET_L_RW));
    accel_bias_reg[1] = (i2c_read_byte_data(piHandle, i2cHandle, MPU9250_RF_YA_OFFSET_H_RW)<<8 | i2c_read_byte_data(piHandle, i2cHandle, MPU9250_RF_YA_OFFSET_L_RW));
    accel_bias_reg[2] = (i2c_read_byte_data(piHandle, i2cHandle, MPU9250_RF_ZA_OFFSET_H_RW)<<8 | i2c_read_byte_data(piHandle, i2cHandle, MPU9250_RF_ZA_OFFSET_L_RW));

    /*
    i2cReadI2CBlockData(piHandle, i2cHandle, MPU9250_RF_XA_OFFSET_H_RW, raw, 6);
    accel_bias_reg[0] = (int16_t)((int16_t) raw[0]<<8 | (int8_t) raw[1]);
    accel_bias_reg[0] = (int16_t)((int16_t) raw[2]<<8 | (int8_t) raw[3]);
    accel_bias_reg[0] = (int16_t)((int16_t) raw[4]<<8 | (int8_t) raw[5]);
   */
    for (int ii = 0; ii < 3; ii++)
    {
        if(accel_bias_reg[ii] % 2)
        {
            mask_bit[ii] = 0;
        }

        accel_bias_reg[ii] -=accel_bias[ii] >> 3;

        if(mask_bit[ii])
        {
            accel_bias_reg[ii] = accel_bias_reg[ii] & ~mask_bit[ii];
        }
        else
        {
            accel_bias_reg[ii] = accel_bias_reg[ii] | 0x0001;
        }
    }


    accel_bits[0] = (accel_bias_reg[0]/4 >> 8) & 0xFF;
    accel_bits[1] = (accel_bias_reg[0]/4)      & 0xFF;
    accel_bits[2] = (accel_bias_reg[1]/4 >> 8) & 0xFF;
    accel_bits[3] = (accel_bias_reg[1]/4)      & 0xFF;
    accel_bits[4] = (accel_bias_reg[2]/4 >> 8) & 0xFF;
    accel_bits[5] = (accel_bias_reg[2]/4)      & 0xFF;

    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_RF_XA_OFFSET_H_RW,accel_bits[0]);
    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_RF_XA_OFFSET_L_RW,accel_bits[1]);
    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_RF_YA_OFFSET_H_RW,accel_bits[2]);
    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_RF_YA_OFFSET_L_RW,accel_bits[3]);
    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_RF_ZA_OFFSET_H_RW,accel_bits[4]);
    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_RF_ZA_OFFSET_L_RW,accel_bits[5]);


    //char buf1[] = {(char)accel_temp[0], (char)accel_temp[1], (char)accel_temp[2], (char)accel_temp[3], (char)accel_temp[4], (char)accel_temp[5]};
    //i2cWriteI2CBlockData(piHandle, i2cHandle, MPU9250_RF_XG_OFFSET_H_RW, buf1, 6);

    accel_d_offs[0] = -(float)accel_bias_reg[0]/(float)accel_sensitivity;
    accel_d_offs[1] = -(float)accel_bias_reg[1]/(float)accel_sensitivity;
    accel_d_offs[2] = -(float)accel_bias_reg[2]/(float)accel_sensitivity;

    return 0;
}

int8_t MPU9250::calibrateMag(uint8_t mode ,float mRes, float* magCal, int32_t* magBiasADC,float* magBiasDisp, float* magScaleDisp)
{
    uint16_t sampleCount = 0;
    //int32_t magBias[] = {0, 0, 0};
    int32_t magScale[] = {0, 0, 0};
    int32_t magMax[] = {32767, 32767, 32767};
    int32_t magMin[] = {-32767, -32767, -32767};
    int16_t magTemp[] = {0, 0, 0};

    std::cout<<"[INFO]: Wave the device in an 8 figure until done!"<<std::endl;
    usleep(4000000);
    if(mode == 0x02) sampleCount = 128;
    if(mode == 0x06) sampleCount = 1500;
    for (int ii = 0; ii < sampleCount; ii++)
    {
        getMagUncalADC(magTemp);
        for (int jj = 0;jj < 3; jj++)
        {
            if(magTemp[jj] > magMax[jj]) magMax[jj] = magTemp[jj];
            if(magTemp[jj] < magMin[jj]) magMin[jj] = magTemp[jj];
        }
        if(mode == 0x02) usleep(135000);
        if(mode == 0x06) usleep(12000);
    }

    for (int ii = 0; ii < 3; ii++)
    {
        magBiasADC[ii] = (magMax[ii] + magMin[ii])/2;
    }

    for (int ii = 0; ii < 3; ii++)
    {
        magBiasDisp[ii] = (float) magBiasADC[ii] * mRes * magCal[ii];
    }

    for (int ii = 0; ii < 3; ii++)
    {
        magScale[ii] = (magMax[ii] - magMin[ii])/2;
    }

    float avgRad = magScale[0] + magScale[1] + magScale[2];
    avgRad /= 3;

    for (int ii = 0; ii < 3; ii++)
    {
        magScaleDisp[ii] = avgRad/((float) magScale[ii]);
    }

    std::cout<<"[INFO]: Magnetometer calibration is done !"<<std::endl;

    return 0;
}


int8_t MPU9250::setMPU9250Config(uint8_t sampleRate, uint8_t Ascale, uint8_t Gscale)
{

    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_R_PWR_MGMT_1_RW,
                     0X00);
    usleep(100000);
    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_R_PWR_MGMT_1_RW,
                     0X01);
    usleep(200000);

    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_R_CONFIG_RW,
                     0x03);
    //data.G_DLPF << MPU9250_CONFIG_P_DLPF_CFG_2_0);
    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_RF_SMPLRT_DIV_RW,
                     sampleRate);
    //data.SMPLRDIV);


    uint8_t c = i2c_read_byte_data(piHandle, i2cHandle, MPU9250_R_GYRO_CONFIG_RW);
    c = c & ~0x02;
    c = c & ~0x18;
    c = c | (Gscale << 3);

    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_R_GYRO_CONFIG_RW,
                     c);
    //data.G_FS<<MPU9250_GYROCFG_P_GYRO_FS_SEL_1_0 | data.G_Fchoice<<MPU9250_GYROCFG_P_FCHOICE_B_1_0);
    c = i2c_read_byte_data(piHandle, i2cHandle, MPU9250_R_ACCEL_CONFIG_RW);
    c = c & ~0x18;
    c = c | (Ascale << 3);
    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_R_ACCEL_CONFIG_RW,
                     c);
    //data.A_FS<<MPU9250_ACCELCFG_P_ACCEL_FS_SEL_1_0);
    c = i2c_read_byte_data(piHandle, i2cHandle, MPU9250_R_ACCEL_CONFIG2_RW);
    c = c & ~0x0F;
    c = c | 0x03;

    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_R_ACCEL_CONFIG2_RW,
                     c);
    //data.A_Fchoice<<MPU9250_ACCELCFG1_P_FCHOICE_B_1_0 | data.A_DLPF<<MPU9250_ACCELCFG1_P_DLPF_B_1_0);

    //i2cWriteByteData(piHandle, i2cHandle, MPU9250_R_LP_ACCEL_ODR_RW,
    //                 data.lposc_clksel<<MPU9250_R_LP_ACCEL_ODR_RW);

    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_R_INT_PIN_CFG_RW, 0x10);
    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_R_INT_ENABLE_RW, 0X01);


    usleep(100000);
    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_R_USER_CTRL_RW,
                     0x20);
    //MPU9250_ENABLE_FLAG << MPU9250_USERCTRL_P_I2C_MST_EN_0);
    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_R_I2C_MST_CTRL_RW,
                     0x1D);
    //MPU9250_ENABLE_FLAG<<MPU9250_I2CMSTCTRL_P_I2C_MST_P_NSR_0 | MPU9250_O_I2C_MST_CLK_13 << MPU9250_I2CMSTCTRL_P_I2C_MST_CLK_3_0);
    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_R_I2C_MST_DELAY_CTRL_RW,
                     0x81);
    //MPU9250_ENABLE_FLAG << MPU9250_I2CMSTDELAYCTRL_P_DELAY_ES_SHADOW_0 | MPU9250_ENABLE_FLAG << MPU9250_I2CMSTDELAYCTRL_M_SLV0_DLY_EN_0);
    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_R_I2C_SLV4_CTRL_RW,
                     0x01);

    return 0;
}

int8_t MPU9250::setAK8963Config(const uint8_t& scale, const uint8_t& mode, float* magCal)
{
    uint8_t en_1 = ((MPU9250_ENABLE_FLAG << MPU9250_I2CSLV0CTRL_P_EN_0) | (MPU9250_ENABLE_FLAG << MPU9250_I2CSLV0CTRL_P_LENG_3_0));
    uint8_t en_3 = ((MPU9250_ENABLE_FLAG << MPU9250_I2CSLV0CTRL_P_EN_0) | (0x03 << MPU9250_I2CSLV0CTRL_P_LENG_3_0));
    char raw[] = {0, 0, 0};
    int8_t cntl = scale << AK8963_CNTL1_P_BIT_0 | mode << AK8963_CNTL1_P_MODE_3_0;

    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_R_I2C_SLV0_ADDR_RW, AK8963_I2C_PRIMARY_ADDR);
    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_RF_I2C_SLV0_REG_RW, AK8963_RF_CNTL2_RW);
    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_RF_I2C_SLV0_DO_RW, 0x01);
    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_R_I2C_SLV0_CTRL_RW, en_1);
    usleep(50000);

    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_R_I2C_SLV0_ADDR_RW, AK8963_I2C_PRIMARY_ADDR);
    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_RF_I2C_SLV0_REG_RW, AK8963_R_CNTL_RW);
    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_RF_I2C_SLV0_DO_RW, AK8963_O_CNTL1MODE_POWERDWN);
    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_R_I2C_SLV0_CTRL_RW, en_1);
    usleep(50000);

    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_R_I2C_SLV0_ADDR_RW, AK8963_I2C_PRIMARY_ADDR);
    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_RF_I2C_SLV0_REG_RW, AK8963_R_CNTL_RW);
    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_RF_I2C_SLV0_DO_RW, AK8963_O_CNTL1MODE_FUSE_ACCESS);
    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_R_I2C_SLV0_CTRL_RW, en_1);
    usleep(50000);

    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_R_I2C_SLV0_ADDR_RW, AK8963_I2C_PRIMARY_ADDR);
    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_RF_I2C_SLV0_REG_RW, AK8963_RF_ASAX_R);
    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_R_I2C_SLV0_CTRL_RW, en_3);
    usleep(50000);

    i2c_read_i2c_block_data(piHandle, i2cHandle, MPU9250_RF_EXT_SENS_DATA_00_R, raw, 3);
    magCal[0] = (float)((int16_t)raw[0] - 128)/ 256.0f + 1.0f;
    magCal[1] = (float)((int16_t)raw[1] - 128)/ 256.0f + 1.0f;
    magCal[2] = (float)((int16_t)raw[2] - 128)/ 256.0f + 1.0f;

    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_R_I2C_SLV0_ADDR_RW, AK8963_I2C_PRIMARY_ADDR);
    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_RF_I2C_SLV0_REG_RW, AK8963_R_CNTL_RW);
    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_RF_I2C_SLV0_DO_RW, AK8963_O_CNTL1MODE_POWERDWN);
    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_R_I2C_SLV0_CTRL_RW, en_1);
    usleep(50000);

    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_R_I2C_SLV0_ADDR_RW, AK8963_I2C_PRIMARY_ADDR);
    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_RF_I2C_SLV0_REG_RW, AK8963_R_CNTL_RW);
    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_RF_I2C_SLV0_DO_RW, cntl);
    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_R_I2C_SLV0_CTRL_RW, en_1);
    usleep(50000);

    return 0;
}




int8_t MPU9250::getMagUncalADC(int16_t* mdata)
{
    char raw[] = {0, 0, 0, 0, 0, 0, 0};
    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_R_I2C_SLV0_ADDR_RW, AK8963_I2C_PRIMARY_ADDR|0x80);
    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_RF_I2C_SLV0_REG_RW, AK8963_RF_XOUT_H_R);
    i2c_write_byte_data(piHandle, i2cHandle, MPU9250_R_I2C_SLV0_CTRL_RW,
                     MPU9250_ENABLE_FLAG<<MPU9250_I2CSLV0CTRL_P_EN_0|0X07<<MPU9250_I2CSLV0CTRL_P_LENG_3_0);
    usleep(2000);
    i2c_read_i2c_block_data(piHandle, i2cHandle, MPU9250_RF_EXT_SENS_DATA_00_R, raw, 7);
    if(!(raw[6]&0x08))
    {
        mdata[0] = ((int16_t)raw[1] << 8) | (int8_t)raw[0] ;
        mdata[1] = ((int16_t)raw[3] << 8) | (int8_t)raw[2] ;
        mdata[2] = ((int16_t)raw[5] << 8) | (int8_t)raw[4] ;
    }

    return 0;
}

int8_t MPU9250::getAccelGyroTempCalADC(int16_t* adata , int16_t* gdata, int16_t &tdata)
{
    char raw[14];
    i2c_read_i2c_block_data(piHandle, i2cHandle, MPU9250_RF_ACCEL_XOUT_H_R, raw, 14);
    adata[0] = (int16_t)((int16_t)raw[0]<<8) |(int8_t)raw[1];
    adata[1] = (int16_t)((int16_t)raw[2]<<8) |(int8_t)raw[3];
    adata[2] = (int16_t)((int16_t)raw[4]<<8) |(int8_t)raw[5];
    tdata    = (int16_t)((int16_t)raw[6]<<8) |(int8_t)raw[7];
    gdata[0] = (int16_t)((int16_t)raw[8]<<8) |(int8_t)raw[9];
    gdata[1] = (int16_t)((int16_t)raw[10]<<8)|(int8_t)raw[11];
    gdata[2] = (int16_t)((int16_t)raw[12]<<8)|(int8_t)raw[13];
//    i2cReadI2CBlockData(piHandle, i2cHandle, MPU9250_RF_TEMP_OUT_H_R, raw, 2);
//    raw[0] = i2c_read_byte_data(piHandle, i2cHandle, MPU9250_RF_TEMP_OUT_H_R);
//    raw[1] = i2c_read_byte_data(piHandle, i2cHandle, MPU9250_RF_TEMP_OUT_L_R);

//    tdata = (int16_t)((int16_t)raw[0]<<8) |(int8_t)raw[1];

    return 0;
}
