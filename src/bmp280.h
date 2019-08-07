#ifndef BMP280_H
#define BMP280_H
#include<stdint.h>

#define BMP280_OK                            INT8_C(0)
//i2c addresses
#define BMP280_I2C_ADDR_PRIM                 INT8_C(0x76)
#define BMP280_I2C_ADDR_SEC                  INT8_C(0x77)
//register addresses
#define BMP280_CHIP_ID_ADDR                  UINT8_C(0xD0)
#define BMP280_SOFT_RESET_ADDR               UINT8_C(0xE0)
#define BMP280_STATUS_ADDR                   UINT8_C(0xF3)
#define BMP280_CTRL_MEAS_ADDR                UINT8_C(0xF4)
#define BMP280_CONFIG_ADDR                   UINT8_C(0xF5)
#define BMP280_PRES_MSB_ADDR                 UINT8_C(0xF7)
#define BMP280_PRES_LSB_ADDR                 UINT8_C(0xF8)
#define BMP280_PRES_XLSB_ADDR                UINT8_C(0xF9)
#define BMP280_TEMP_MSB_ADDR                 UINT8_C(0xFA)
#define BMP280_TEMP_LSB_ADDR                 UINT8_C(0xFB)
#define BMP280_TEMP_XLSB_ADDR                UINT8_C(0xFC)


#define BMP280_DIG_T1_LSB_ADDR               UINT8_C(0x88)
#define BMP280_DIG_T1_MSB_ADDR               UINT8_C(0x89)
#define BMP280_DIG_T2_LSB_ADDR               UINT8_C(0x8A)
#define BMP280_DIG_T2_MSB_ADDR               UINT8_C(0x8B)
#define BMP280_DIG_T3_LSB_ADDR               UINT8_C(0x8C)
#define BMP280_DIG_T3_MSB_ADDR               UINT8_C(0x8D)
#define BMP280_DIG_P1_LSB_ADDR               UINT8_C(0x8E)
#define BMP280_DIG_P1_MSB_ADDR               UINT8_C(0x8F)
#define BMP280_DIG_P2_LSB_ADDR               UINT8_C(0x90)
#define BMP280_DIG_P2_MSB_ADDR               UINT8_C(0x91)
#define BMP280_DIG_P3_LSB_ADDR               UINT8_C(0x92)
#define BMP280_DIG_P3_MSB_ADDR               UINT8_C(0x93)
#define BMP280_DIG_P4_LSB_ADDR               UINT8_C(0x94)
#define BMP280_DIG_P4_MSB_ADDR               UINT8_C(0x95)
#define BMP280_DIG_P5_LSB_ADDR               UINT8_C(0x96)
#define BMP280_DIG_P5_MSB_ADDR               UINT8_C(0x97)
#define BMP280_DIG_P6_LSB_ADDR               UINT8_C(0x98)
#define BMP280_DIG_P6_MSB_ADDR               UINT8_C(0x99)
#define BMP280_DIG_P7_LSB_ADDR               UINT8_C(0x9A)
#define BMP280_DIG_P7_MSB_ADDR               UINT8_C(0x9B)
#define BMP280_DIG_P8_LSB_ADDR               UINT8_C(0x9C)
#define BMP280_DIG_P8_MSB_ADDR               UINT8_C(0x9D)
#define BMP280_DIG_P9_LSB_ADDR               UINT8_C(0x9E)
#define BMP280_DIG_P9_MSB_ADDR               UINT8_C(0x9F)

//commands
#define BMP280_CHIP_ID                       INT8_C(0x58)

#define BMP280_SOFT_RST_CMD                  INT8_C(0xB6)

#define BMP280_MEAS_DONE                     UINT8_C(0)
#define BMP280_MEAS_ONGOING                  UINT8_C(1)

#define BMP280_IM_UPDATE_DONE                UINT8_C(0)
#define BMP280_IM_UPDATE_ONGOING             UINT8_C(1)

#define BMP280_OS_NONE                       UINT8_C(0x00)
#define BMP280_OS_1X                         UINT8_C(0x01)
#define BMP280_OS_2X                         UINT8_C(0x02)
#define BMP280_OS_4X                         UINT8_C(0x03)
#define BMP280_OS_8X                         UINT8_C(0x04)
#define BMP280_OS_16X                        UINT8_C(0x05)

#define BMP280_SLEEP_MODE                    UINT8_C(0x00)
#define BMP280_FORCED_MODE                   UINT8_C(0x01)
#define BMP280_NORMAL_MODE                   UINT8_C(0x03)

#define BMP280_ODR_0_5_MS                    UINT8_C(0x00)
#define BMP280_ODR_62_5_MS                   UINT8_C(0x01)
#define BMP280_ODR_125_MS                    UINT8_C(0x02)
#define BMP280_ODR_250_MS                    UINT8_C(0x03)
#define BMP280_ODR_500_MS                    UINT8_C(0x04)
#define BMP280_ODR_1000_MS                   UINT8_C(0x05)
#define BMP280_ODR_2000_MS                   UINT8_C(0x06)
#define BMP280_ODR_4000_MS                   UINT8_C(0x07)

#define BMP280_FILTER_OFF                    UINT8_C(0x00)
#define BMP280_FILTER_COEFF_2                UINT8_C(0x01)
#define BMP280_FILTER_COEFF_4                UINT8_C(0x02)
#define BMP280_FILTER_COEFF_8                UINT8_C(0x03)
#define BMP280_FILTER_COEFF_16               UINT8_C(0x04)

#define BMP280_SPI3_WIRE_ENABLE              UINT8_C(1)
#define BMP280_SPI3_WIRE_DISABLE             UINT8_C(0)

//positions and masks
#define BMP280_STATUS_IM_UPDATE_POS          UINT8_C(0)
#define BMP280_STATUS_IM_UPDATE_MASK         UINT8_C(0x01)
#define BMP280_STATUS_MEAS_POS               UINT8_C(3)
#define BMP280_STATUS_MEAS_MASK              UINT8_C(0x08)
#define BMP280_OS_TEMP_POS                   UINT8_C(5)
#define BMP280_OS_TEMP_MASK                  UINT8_C(0xE0)
#define BMP280_OS_PRES_POS                   UINT8_C(2)
#define BMP280_OS_PRES_MASK                  UINT8_C(0x1C)
#define BMP280_POWER_MODE_POS                UINT8_C(0)
#define BMP280_POWER_MODE_MASK               UINT8_C(0x03)
#define BMP280_STANDBY_DURN_POS              UINT8_C(5)
#define BMP280_STANDBY_DURN_MASK             UINT8_C(0xE0)
#define BMP280_FILTER_POS                    UINT8_C(2)
#define BMP280_FILTER_MASK                   UINT8_C(0x1C)
#define BMP280_SPI3_ENABLE_POS               UINT8_C(0)
#define BMP280_SPI3_ENABLE_MASK              UINT8_C(0x01)

#define BMP280_DIG_T1_LSB_POS                UINT8_C(0)
#define BMP280_DIG_T1_MSB_POS                UINT8_C(1)
#define BMP280_DIG_T2_LSB_POS                UINT8_C(2)
#define BMP280_DIG_T2_MSB_POS                UINT8_C(3)
#define BMP280_DIG_T3_LSB_POS                UINT8_C(4)
#define BMP280_DIG_T3_MSB_POS                UINT8_C(5)
#define BMP280_DIG_P1_LSB_POS                UINT8_C(6)
#define BMP280_DIG_P1_MSB_POS                UINT8_C(7)
#define BMP280_DIG_P2_LSB_POS                UINT8_C(8)
#define BMP280_DIG_P2_MSB_POS                UINT8_C(9)
#define BMP280_DIG_P3_LSB_POS                UINT8_C(10)
#define BMP280_DIG_P3_MSB_POS                UINT8_C(11)
#define BMP280_DIG_P4_LSB_POS                UINT8_C(12)
#define BMP280_DIG_P4_MSB_POS                UINT8_C(13)
#define BMP280_DIG_P5_LSB_POS                UINT8_C(14)
#define BMP280_DIG_P5_MSB_POS                UINT8_C(15)
#define BMP280_DIG_P6_LSB_POS                UINT8_C(16)
#define BMP280_DIG_P6_MSB_POS                UINT8_C(17)
#define BMP280_DIG_P7_LSB_POS                UINT8_C(18)
#define BMP280_DIG_P7_MSB_POS                UINT8_C(19)
#define BMP280_DIG_P8_LSB_POS                UINT8_C(20)
#define BMP280_DIG_P8_MSB_POS                UINT8_C(21)
#define BMP280_DIG_P9_LSB_POS                UINT8_C(22)
#define BMP280_DIG_P9_MSB_POS                UINT8_C(23)
#define BMP280_CALIB_DATA_SIZE               UINT8_C(24)

//max and min sizes
#define BMP280_ST_DIG_T1_MIN UINT16_C(19000)
#define BMP280_ST_DIG_T1_MAX UINT16_C(35000)
#define BMP280_ST_DIG_T2_MIN UINT16_C(22000)
#define BMP280_ST_DIG_T2_MAX UINT16_C(30000)
#define BMP280_ST_DIG_T3_MIN INT16_C(-3000)
#define BMP280_ST_DIG_T3_MAX INT16_C(-1000)
#define BMP280_ST_DIG_P1_MIN UINT16_C(30000)
#define BMP280_ST_DIG_P1_MAX UINT16_C(42000)
#define BMP280_ST_DIG_P2_MIN INT16_C(-12970)
#define BMP280_ST_DIG_P2_MAX INT16_C(-8000)
#define BMP280_ST_DIG_P3_MIN INT16_C(-5000)
#define BMP280_ST_DIG_P3_MAX UINT16_C(8000)
#define BMP280_ST_DIG_P4_MIN INT16_C(-10000)
#define BMP280_ST_DIG_P4_MAX UINT16_C(18000)
#define BMP280_ST_DIG_P5_MIN INT16_C(-500)
#define BMP280_ST_DIG_P5_MAX UINT16_C(1100)
#define BMP280_ST_DIG_P6_MIN INT16_C(-1000)
#define BMP280_ST_DIG_P6_MAX UINT16_C(1000)
#define BMP280_ST_DIG_P7_MIN INT16_C(-32768)
#define BMP280_ST_DIG_P7_MAX UINT16_C(32767)
#define BMP280_ST_DIG_P8_MIN INT16_C(-30000)
#define BMP280_ST_DIG_P8_MAX UINT16_C(10000)
#define BMP280_ST_DIG_P9_MIN INT16_C(-10000)
#define BMP280_ST_DIG_P9_MAX UINT16_C(30000)


#define BMP280_ST_TRIMCUSTOM_REG               UINT8_C(0x87)
#define BMP280_ST_TRIMCUSTOM_REG_APIREV__POS   UINT8_C(1)
#define BMP280_ST_TRIMCUSTOM_REG_APIREV__MSK   UINT8_C(0x06)
#define BMP280_ST_TRIMCUSTOM_REG_APIREV__LEN   UINT8_C(2)
#define BMP280_ST_TRIMCUSTOM_REG_APIREV__REG   BMP280_ST_TRIMCUSTOM_REG

#define BMP280_ST_MAX_APIREVISION              UINT8_C(0x00)

#define BMP280_ST_ADC_T_MIN                    INT32_C(0x00000)

#define BMP280_ST_ADC_T_MAX                    INT32_C(0xFFFF0)

#define BMP280_ST_ADC_P_MIN                    INT32_C(0x00000)

#define BMP280_ST_ADC_P_MAX                    INT32_C(0xFFFF0)


#define BMP280_ST_PLAUSIBLE_TEMP_MIN           UINT8_C(0)
#define BMP280_ST_PLAUSIBLE_TEMP_MAX           UINT8_C(40)


#define BMP280_ST_PLAUSIBLE_PRESS_MIN          UINT8_C(900)
#define BMP280_ST_PLAUSIBLE_PRESS_MAX          UINT8_C(1100)

#define BMP280_ST_TEMPERATURE_RESOLUTION_INT32 UINT8_C(100)
#define BMP280_ST_PRESSURE_RESOLUTION_INT32    UINT8_C(100)


class BMP280
{
private:

    int8_t rslt;

    struct calibParams
    {
        uint16_t dig_t1;
        int16_t dig_t2;
        int16_t dig_t3;
        uint16_t dig_p1;
        int16_t dig_p2;
        int16_t dig_p3;
        int16_t dig_p4;
        int16_t dig_p5;
        int16_t dig_p6;
        int16_t dig_p7;
        int16_t dig_p8;
        int16_t dig_p9;
        int32_t t_fine;

    }calib_params_data;

    struct unCalibOp
    {
        int32_t unCalibTemp;
        int32_t unCalibPres;

    };

    int8_t getCalibParams(calibParams& data);
    int8_t st_check_cal_params(const calibParams& calib_param);
    int8_t st_check_boundaries(const int32_t& utemperature, const int32_t& upressure);
    int8_t st_check_sensor_range(const int32_t& temperature, const int32_t& pressure);
public:

    int8_t piHandle;
    uint8_t i2cHandle;

    struct config
    {


        uint8_t os_temp;
        uint8_t os_pres;
        uint8_t odr;
        uint8_t filter;
        uint8_t spi3w_en;

    }config_data;

    struct calibOp
    {
        double CalibTemp;
        double CalibPres;

    };

    struct status
    {
        bool measuring;
        bool im_update;
    };

    BMP280(uint8_t address);
    ~BMP280();
    int8_t setCalibParams();
    int8_t checkComms();
    int8_t setConfig(const config& data);
    int8_t getST();
    int8_t setPowerMode(const int8_t& mode);
    int8_t rst();
    int8_t getStatus(status& data);
    int8_t getUnCalibData(unCalibOp& data);
    int8_t getReadings(calibOp& data);
    int8_t getCompPres(const int32_t& uncomp_pres, const calibParams& calib_params, double& pressure);
    int8_t getCompTemp(const int32_t& uncomp_temp, calibParams& calib_params, double& temperature);

};

#endif // BMP280_H
