#include <iostream>
#include <ros/ros.h>
#include "../src/bmp280.h"
#include "genquad/bmp280_dat.h"
/*
void setBMPConfig(BMP280::config& config)
{

    config.os_temp = BMP280_OS_16X;
    config.os_pres = BMP280_OS_16X;
    config.odr = BMP280_ODR_1000_MS;
    config.filter = BMP280_FILTER_OFF;
    config.spi3w_en = BMP280_SPI3_WIRE_DISABLE;


}
*/

int main(int argc, char* argv[])
{
    BMP280::config config;

    BMP280::status status;
    BMP280::calibOp output;
    genquad::bmp280_dat packet;

    //setBMPConfig(config);
    BMP280 instance(0x76);
    instance.setCalibParams();
    instance.getST();
    instance.setPowerMode(BMP280_NORMAL_MODE);

    /*
    if(argc != 5)
    {
        std::cout<<"[ERROR]: Please enter all the arguements in the order,"<<std::endl;
        std::cout<<"         os_temp os_pres odr filter"<<std::endl;
        std::cout<<"         (1, 2, 4, 8, 16) (5, 63, 125, 250, 500, 1000, 2000, 4000) (keep filter off for now '0')"<<std::endl;
        return 1;
    }
    */
    switch (atoi(argv[1]))
    {
    case 1:
        config.os_temp = BMP280_OS_1X;
        break;
    case 2:
        config.os_temp = BMP280_OS_2X;
        break;
    case 4:
        config.os_temp = BMP280_OS_4X;
        break;
    case 8:
        config.os_temp = BMP280_OS_8X;
        break;
    case 16:
        config.os_temp = BMP280_OS_16X;
        break;
    default:
        std::cout<<"[ERROR]: Invalid os_temp value given please restart sequence with right args!"<<std::endl;
        std::cout <<"[ERROR]: the invalid value given is: "<<argv[1];
        return 1;

    }

    switch (atoi(argv[2]))
    {
    case 1:
        config.os_pres = BMP280_OS_1X;
        break;
    case 2:
        config.os_pres = BMP280_OS_2X;
        break;
    case 4:
        config.os_pres = BMP280_OS_4X;
        break;
    case 8:
        config.os_pres = BMP280_OS_8X;
        break;
    case 16:
        config.os_pres = BMP280_OS_16X;
        break;
    default:
        std::cout<<"[ERROR]: Invalid os_pres value given please restart sequence with right args!"<<std::endl;
        std::cout <<"[ERROR]: the invalid value given is: "<<argv[2];
        return 1;

    }

    switch (atoi(argv[3]))
    {
    case 5:
        config.odr = BMP280_ODR_0_5_MS;
        break;
    case 63:
        config.odr = BMP280_ODR_62_5_MS;
        break;
    case 125:
        config.odr = BMP280_ODR_125_MS;
        break;
    case 250:
        config.odr = BMP280_ODR_250_MS;
        break;
    case 500:
        config.odr = BMP280_ODR_500_MS;
        break;
    case 1000:
        config.odr = BMP280_ODR_1000_MS;
        break;
    case 2000:
        config.odr = BMP280_ODR_2000_MS;
        break;
    case 4000:
        config.odr = BMP280_ODR_4000_MS;
        break;
    default:
        std::cout<<"[ERROR]: Invalid ODR value given please restart sequence with right args!"<<std::endl;
        std::cout <<"[ERROR]: the invalid value given is: "<<argv[3];
        return 1;

    }

    switch (atoi(argv[4]))
    {
    case 0:
        config.filter = BMP280_FILTER_OFF;
        break;
    case 2:
        config.filter = BMP280_FILTER_COEFF_2;
        break;
    case 4:
        config.filter = BMP280_FILTER_COEFF_4;
        break;
    case 8:
        config.filter = BMP280_FILTER_COEFF_8;
        break;
    case 16:
        config.filter = BMP280_FILTER_COEFF_16;
        break;
    default:
        std::cout<<"[ERROR]: Invalid filter value given please restart sequence with right args!"<<std::endl;
        std::cout <<"[ERROR]: the invalid value given is: "<<argv[4];
        return 1;

    }

    config.spi3w_en = BMP280_SPI3_WIRE_DISABLE;

    instance.setConfig(config);

    ros::init(argc, argv, "BMP280");
    ros::NodeHandle n;
    ros::Publisher publisher = n.advertise<genquad::bmp280_dat>("Press_and_Temp", 10);

    packet.header.frame_id = "~";
    while (ros::ok())
    {
        while(status.measuring == 1)
        {
            instance.getStatus(status);
        }

        instance.getReadings(output);

        packet.header.stamp = ros::Time::now();
        packet.temp.temperature = output.CalibTemp;
        packet.press.fluid_pressure = output.CalibPres;

        packet.temp.variance = 0;
        packet.press.variance = 0;

        publisher.publish(packet);

        while(status.measuring == 0)
        {
            instance.getStatus(status);
        }
    }
/////////////////////////////////////////////////
/*
    BMP280::status status;
    int16_t count;
    do
    {
        while(status.measuring == 1)
        {
            instance.getStatus(status);
        }
        instance.getReadings(output);
        std::cout<<"calibrated temperature is "<< output.CalibTemp<<" !" <<std::endl;
        std::cout<<"calibrated pressure is "<< output.CalibPres<<" !" <<std::endl;
        std::cout<<count++<<std::endl;
        while(status.measuring == 0)
        {
            instance.getStatus(status);
        }

    }while (true);
*/
/////////////////////////////////////////////////

    return 0;
}
