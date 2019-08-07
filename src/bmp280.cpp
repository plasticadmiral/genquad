#include "bmp280.h"
#include <unistd.h>
#include <stdint.h>
#include <iostream>
#include <pigpiod_if2.h>


BMP280::BMP280(uint8_t address)
{
    std::cout<< "[INFO]: bmp is constructed!"<<std::endl;
    rslt = 0;
    piHandle = pigpio_start(0, 0);
    if(piHandle < 0) std::cout<<"[ERROR]: Cannot find pigpio daemon! Please start daemon and run start sequence again!"<<std::endl;
    i2cHandle =i2c_open(piHandle, 1, address, 0);

}

BMP280::~BMP280()
{
    i2c_close(piHandle, i2cHandle);
    pigpio_stop(piHandle);

}

int8_t BMP280::st_check_cal_params(const calibParams& calib_param)
{
    if(rslt == 1)
    {
        return rslt;
    }

    if ((calib_param.dig_t1 < BMP280_ST_DIG_T1_MIN) || (calib_param.dig_t1 > BMP280_ST_DIG_T1_MAX))
    {
        rslt = 1;
    }
    if ((calib_param.dig_t2 < BMP280_ST_DIG_T2_MIN) || (calib_param.dig_t2 > BMP280_ST_DIG_T2_MAX))
    {
        rslt = 1;

    }
    if ((calib_param.dig_t3 < BMP280_ST_DIG_T3_MIN) || (calib_param.dig_t3 > BMP280_ST_DIG_T3_MAX))
    {
        rslt = 1;
    }
    if ((calib_param.dig_p1 < BMP280_ST_DIG_P1_MIN) || (calib_param.dig_p1 > BMP280_ST_DIG_P1_MAX))
    {
        rslt = 1;
    }
    if ((calib_param.dig_p2 < BMP280_ST_DIG_P2_MIN) || (calib_param.dig_p2 > BMP280_ST_DIG_P2_MAX))
    {
        rslt = 1;
    }
    if ((calib_param.dig_p3 < BMP280_ST_DIG_P3_MIN) || (calib_param.dig_p3 > BMP280_ST_DIG_P3_MAX))
    {
        rslt = 1;
    }
    if ((calib_param.dig_p4 < BMP280_ST_DIG_P4_MIN) || (calib_param.dig_p4 > BMP280_ST_DIG_P4_MAX))
    {
        rslt = 1;
    }
    if ((calib_param.dig_p5 < BMP280_ST_DIG_P5_MIN) || (calib_param.dig_p5 > BMP280_ST_DIG_P5_MAX))
    {
        rslt = 1;
    }
    if ((calib_param.dig_p6 < BMP280_ST_DIG_P6_MIN) || (calib_param.dig_p6 > BMP280_ST_DIG_P6_MAX))
    {
        rslt = 1;
    }
    if ((calib_param.dig_p8 < BMP280_ST_DIG_P8_MIN) || (calib_param.dig_p8 > BMP280_ST_DIG_P8_MAX))
    {
        rslt = 1;
    }
    if ((calib_param.dig_p9 < BMP280_ST_DIG_P9_MIN) || (calib_param.dig_p9 > BMP280_ST_DIG_P9_MAX))
    {
        rslt = 1;
    }
    if(rslt == 1)
    {
        std::cout<<"[WARNING]: calib params are not in range"<<std::endl;

    }
    else
        std::cout<<"[INFO]: calib params are in range"<<std::endl;
    return rslt;
}

int8_t BMP280::st_check_boundaries(const int32_t& utemperature, const int32_t& upressure)
{
    if(rslt == 1)
    {
        return rslt;
    }

    if ((utemperature <= BMP280_ST_ADC_T_MIN || utemperature >= BMP280_ST_ADC_T_MAX) &&
            (upressure <= BMP280_ST_ADC_P_MIN || upressure >= BMP280_ST_ADC_P_MAX))
    {
        std::cout<<"[WARNING]:ADC values of pressure and temperature are out of boundaries"<<std::endl;
        rslt = 1;
    }
    else if (utemperature <= BMP280_ST_ADC_T_MIN || utemperature >= BMP280_ST_ADC_T_MAX)
    {
        std::cout<<"[WARNING]: ADC values of temperature are out of boundaries"<<std::endl;
        rslt = 1;
    }
    else if (upressure <= BMP280_ST_ADC_P_MIN || upressure >= BMP280_ST_ADC_P_MAX)
    {
        std::cout<<"[WARNING]: ADC values of pressure are out of boundaries"<<std::endl;
        rslt = 1;
    }

    return rslt;
}


int8_t BMP280::st_check_sensor_range(const int32_t& temperature, const int32_t& pressure)
{
    if(rslt == 1)
    {
        return rslt;
    }

    if ((temperature < BMP280_ST_PLAUSIBLE_TEMP_MIN) ||
            (temperature > (BMP280_ST_PLAUSIBLE_TEMP_MAX * BMP280_ST_TEMPERATURE_RESOLUTION_INT32)))
    {
        std::cout<<"[WARNING]: temperature is not within plausable range"<<std::endl;
        rslt = 1;
    }
    if (rslt != 1)
    {
        if ((pressure < (BMP280_ST_PLAUSIBLE_PRESS_MIN * BMP280_ST_PRESSURE_RESOLUTION_INT32)) ||
                (pressure > (BMP280_ST_PLAUSIBLE_PRESS_MAX * BMP280_ST_PRESSURE_RESOLUTION_INT32)))
        {

            std::cout<<"[WARNING]: pressure is not within plausable range"<<std::endl;
            rslt = 1;
        }

    }

    return rslt;
}

int8_t BMP280::getST()
{
    struct config st_config_data;
    struct unCalibOp uncali_data;
    double temperature = 0;
    double pressure = 0;

    if(rslt == 1)
    {
        return rslt;
    }

    st_config_data.os_temp = BMP280_OS_1X;
    st_config_data.os_pres = BMP280_OS_1X;
    st_config_data.filter = BMP280_FILTER_OFF;
    st_config_data.odr = BMP280_ODR_0_5_MS;
    st_config_data.spi3w_en = BMP280_SPI3_WIRE_DISABLE;

    rslt = rst();
    if(rslt == 0)
    {
        rslt = st_check_cal_params(calib_params_data);
        if(rslt == 0)
        {
            rslt = setConfig(st_config_data);
            if(rslt == 0)
            {
                rslt = setPowerMode(BMP280_FORCED_MODE);
                usleep(10000);
                if(rslt == 0)
                {

                    rslt = getUnCalibData(uncali_data);
                    if(rslt == 0)
                    {

                        rslt = getCompTemp(uncali_data.unCalibTemp, calib_params_data, temperature);
                        if(rslt == 0)
                        {
                            rslt = getCompPres(uncali_data.unCalibPres, calib_params_data, pressure);
                            if(rslt == 0)
                            {

                                rslt = st_check_sensor_range((int32_t)temperature, (int32_t)pressure);
                                if(rslt == 0)
                                    std::cout<< "[INFO]: self test has completed successfully with no errors"<< std::endl;

                            }
                        }
                    }
                }
            }
        }
    }
    if(rslt == 1)
        std::cout<< "[ERROR]: self test has FAILED!"<< std::endl;
    return rslt;
}


int8_t BMP280::setCalibParams()
{
    if(rslt == 1)
    {
        return rslt;
    }

    rslt = getCalibParams(calib_params_data);

    if(rslt == 0)
    {
        std::cout<< "[INFO]: calib params have been set!"<< std::endl;
        st_check_cal_params(calib_params_data);

    }
    else
        std::cout<< "[ERROR]: calib params not set!"<< std::endl;

    return rslt;
}
int8_t BMP280::checkComms()
{
    if(rslt == 1)
    {
        return rslt;
    }

    uint8_t op = 0;
    uint16_t tryCount =5;
    while(tryCount)
    {
        std::cout<<"[INFO]: communication attempt remaining: "<<tryCount<<std::endl;
        op = i2c_read_byte_data(piHandle, i2cHandle, BMP280_CHIP_ID_ADDR);
        if(op<0)
            rslt = 1;
        if(rslt == 0)
        {
            if(op == BMP280_CHIP_ID)
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
    if(op!= BMP280_CHIP_ID)
    {
        std::cout<<"[ERROR]: unable to communicate with BMP280"<<std::endl;

    }
    return rslt;
}

int8_t BMP280::setConfig(const config& data)
{
    if(rslt == 1)
    {
        return rslt;
    }

    uint8_t config = 0;
    uint8_t ctrl_meas = 0;

    config = (((data.odr << BMP280_STANDBY_DURN_POS) | data.filter << BMP280_FILTER_POS) | data.spi3w_en << BMP280_SPI3_ENABLE_POS);
    ctrl_meas = (((data.os_temp << BMP280_OS_TEMP_POS) | data.os_pres << BMP280_OS_PRES_POS) | BMP280_NORMAL_MODE << BMP280_POWER_MODE_POS);

    rslt = i2c_write_byte_data(piHandle, i2cHandle, BMP280_CONFIG_ADDR,config);
    if(rslt == 0)
    {
        std::cout<< "[INFO]: config data has been written to register"<<std::endl;
        rslt = i2c_write_byte_data(piHandle, i2cHandle, BMP280_CTRL_MEAS_ADDR, ctrl_meas);
        if(rslt == 0)
            std::cout<< "[INFO]: ctrl_meas data has been written to registers"<<std::endl;
        else
            std::cout<< "[ERROR]: unable to write to ctrl_meas register"<<std::endl;

    }
    else
        std::cout<< "[ERROR]: unable to write to config register "<<std::endl;

    return rslt;
}

int8_t BMP280::setPowerMode(const int8_t& mode)
{

    if(rslt == 1)
    {
        return rslt;
    }
    int8_t temporary = 0;
    temporary = i2c_read_byte_data(piHandle, i2cHandle, BMP280_CTRL_MEAS_ADDR);
    if(temporary<0)
        rslt = 1;
    if(rslt == 0)
    {
        std::cout<< "[INFO]: ctrl_meas register has been read"<<std::endl;
        temporary &= ~BMP280_POWER_MODE_MASK;
        temporary |= mode;
        rslt = i2c_write_byte_data(piHandle, i2cHandle, BMP280_CTRL_MEAS_ADDR, temporary);
        if(rslt == 0)
            std::cout<<"[INFO]: power mode has been set "<<std::endl;
        else
            std::cout<< "[ERROR]: unable to write to ctrl_config reg"<<std::endl;
    }
    else
    {
        std::cout<< "[ERROR]: unable to read ctrl_meas register"<<std::endl;

    }
    return rslt;
}

int8_t BMP280::rst()
{
    if(rslt == 1)
    {
        return rslt;
    }

    rslt = i2c_write_byte_data(piHandle, i2cHandle, BMP280_SOFT_RESET_ADDR, BMP280_SOFT_RST_CMD);
    if(rslt == 0)
        std::cout<<"[INFO]: reset successfully performed..."<<std::endl;
    else
        std::cout<<"[ERROR]: reset failed!!"<<std::endl;
    usleep(2000);
    return rslt;
}


int8_t BMP280::getStatus(status &data)
{
    if(rslt == 1)
    {
        return rslt;
    }

    int8_t temp = 0;
    temp = i2c_read_byte_data(piHandle, i2cHandle, BMP280_STATUS_ADDR);
    if(temp <0)
        rslt = 1;
    if(rslt == 0)
    {
        data.measuring = bool(temp >> BMP280_STATUS_MEAS_POS);
        data.im_update = bool(temp >> BMP280_STATUS_IM_UPDATE_POS);
    }
    else
        std::cout<<"[WARNING]: unable to read status register"<<std::endl;

    return rslt;
}

int8_t BMP280::getCalibParams(calibParams& data)
{
    if(rslt == 1)
    {
        return rslt;
    }

    uint8_t temp[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};


    for (uint8_t i=0;i<BMP280_CALIB_DATA_SIZE;i++)
    {
        temp[i] = i2c_read_byte_data(piHandle, i2cHandle, (BMP280_DIG_T1_LSB_ADDR+i));
    }

    if (rslt == 0)
    {
        data.dig_t1 =
                (uint16_t) (((uint16_t) temp[BMP280_DIG_T1_MSB_POS] << 8) | ((uint16_t) temp[BMP280_DIG_T1_LSB_POS]));
        data.dig_t2 =
                (int16_t) (((int16_t) temp[BMP280_DIG_T2_MSB_POS] << 8) | ((int16_t) temp[BMP280_DIG_T2_LSB_POS]));
        data.dig_t3 =
                (int16_t) (((int16_t) temp[BMP280_DIG_T3_MSB_POS] << 8) | ((int16_t) temp[BMP280_DIG_T3_LSB_POS]));
        data.dig_p1 =
                (uint16_t) (((uint16_t) temp[BMP280_DIG_P1_MSB_POS] << 8) | ((uint16_t) temp[BMP280_DIG_P1_LSB_POS]));
        data.dig_p2 =
                (int16_t) (((int16_t) temp[BMP280_DIG_P2_MSB_POS] << 8) | ((int16_t) temp[BMP280_DIG_P2_LSB_POS]));
        data.dig_p3 =
                (int16_t) (((int16_t) temp[BMP280_DIG_P3_MSB_POS] << 8) | ((int16_t) temp[BMP280_DIG_P3_LSB_POS]));
        data.dig_p4 =
                (int16_t) (((int16_t) temp[BMP280_DIG_P4_MSB_POS] << 8) | ((int16_t) temp[BMP280_DIG_P4_LSB_POS]));
        data.dig_p5 =
                (int16_t) (((int16_t) temp[BMP280_DIG_P5_MSB_POS] << 8) | ((int16_t) temp[BMP280_DIG_P5_LSB_POS]));
        data.dig_p6 =
                (int16_t) (((int16_t) temp[BMP280_DIG_P6_MSB_POS] << 8) | ((int16_t) temp[BMP280_DIG_P6_LSB_POS]));
        data.dig_p7 =
                (int16_t) (((int16_t) temp[BMP280_DIG_P7_MSB_POS] << 8) | ((int16_t) temp[BMP280_DIG_P7_LSB_POS]));
        data.dig_p8 =
                (int16_t) (((int16_t) temp[BMP280_DIG_P8_MSB_POS] << 8) | ((int16_t) temp[BMP280_DIG_P8_LSB_POS]));
        data.dig_p9 =
                (int16_t) (((int16_t) temp[BMP280_DIG_P9_MSB_POS] << 8) | ((int16_t) temp[BMP280_DIG_P9_LSB_POS]));
    }

    return rslt;
}

int8_t BMP280::getUnCalibData(unCalibOp& data)
{
    if(rslt == 1)
    {
        return rslt;
    }

    char temp[] = {0, 0, 0, 0, 0, 0};

    rslt = i2c_read_i2c_block_data(piHandle, i2cHandle, BMP280_PRES_MSB_ADDR,temp, 6);

    if (rslt > 0)
    {

        data.unCalibPres =
                (int32_t) ((((uint32_t) (temp[0])) << 12) | (((uint32_t) (temp[1])) << 4) | (((uint32_t) (temp[2])) >> 4));
        data.unCalibTemp =
                (int32_t) ((((int32_t) (temp[3])) << 12) | (((int32_t) (temp[4])) << 4) | (((int32_t) (temp[5])) >> 4));
        rslt = st_check_boundaries((int32_t)data.unCalibTemp, (int32_t)data.unCalibPres);
    }
    else
    {
        std::cout<<"[ERROR]: Unable to get adc data"<<std::endl;
        rslt = 1;
    }

    return 0;

}

int8_t BMP280::getCompTemp(const int32_t& uncomp_temp, calibParams& calib_params, double& temperature)
{

    if(rslt == 1)
    {
        return rslt;
    }


    double var1, var2;

    if (rslt == 0)
    {
        var1 = (((double) uncomp_temp) / 16384.0 - ((double) calib_params.dig_t1) / 1024.0) *
                ((double) calib_params.dig_t2);
        var2 =
                ((((double) uncomp_temp) / 131072.0 - ((double) calib_params.dig_t1) / 8192.0) *
                 (((double) uncomp_temp) / 131072.0 - ((double) calib_params.dig_t1) / 8192.0)) *
                ((double) calib_params.dig_t3);
        calib_params.t_fine = (int32_t) (var1 + var2);
        temperature = ((var1 + var2) / 5120.0);
    }
    else
    {
        temperature = 0;
        rslt = 1;
        std::cout<<"[ERROR]: unable to compensate temperature data" <<std::endl;
    }

    return rslt;
}

int8_t BMP280::getCompPres(const int32_t& uncomp_pres, const calibParams& calib_params, double& pressure)
{
    if(rslt == 1)
    {
        return rslt;
    }

    double var1, var2;
    int32_t temp = 0;

    if (rslt == 0)
    {
        var1 = ((double) calib_params.t_fine / 2.0) - 64000.0;
        var2 = var1 * var1 * ((double) calib_params.dig_p6) / 32768.0;
        var2 = var2 + var1 * ((double) calib_params.dig_p5) * 2.0;
        var2 = (var2 / 4.0) + (((double) calib_params.dig_p4) * 65536.0);
        var1 =
                (((double) calib_params.dig_p3) * var1 * var1 / 524288.0 + ((double) calib_params.dig_p2) * var1) /
                524288.0;
        var1 = (1.0 + var1 / 32768.0) * ((double) calib_params.dig_p1);
        temp = (uint32_t)(1048576.0 - (double) uncomp_pres);
        if (var1 < 0 || var1 > 0)
        {
            temp = (uint32_t)((temp - (var2 / 4096.0)) * 6250.0 / var1);
            var1 = ((double) calib_params.dig_p9) * temp * temp / 2147483648.0;
            var2 = temp * ((double) calib_params.dig_p8) / 32768.0;
            pressure = (temp + (var1 + var2 + ((double) calib_params.dig_p7)) / 16.0);
        }
        else
        {
            pressure = 0;
            rslt = 1;
            std::cout<<"[ERROR]: unable to compensate pressure data" <<std::endl;
        }
    }

    return rslt;

}

int8_t BMP280::getReadings(calibOp& data)
{
    if(rslt == 1)
    {
        return rslt;
    }

    unCalibOp uncali;
    rslt = getUnCalibData(uncali);
    if (rslt == 1)
    {
        std::cout<<"i failed "<<std::endl;
    }
    getCompTemp( uncali.unCalibTemp, calib_params_data, data.CalibTemp);
    getCompPres( uncali.unCalibPres, calib_params_data, data.CalibPres);

    return rslt;
}



