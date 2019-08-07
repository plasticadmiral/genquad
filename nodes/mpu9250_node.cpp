#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <ros/ros.h>
#include "../src/mpu9250.h"
#include "genquad/mpu9250_dat.h"
#include <random>

int randVal(int min,int max)
{
  std::random_device dev;
  std::mt19937 rng(dev());
  std::uniform_int_distribution<std::mt19937::result_type> dist6(min, max); // distribution in range [1, 6]

  return dist6(rng);
}


int main(int argc, char* argv[])
{
    genquad::mpu9250_dat packet;
    int8_t rslt = 0;
    float gyrodisp[] = {0, 0, 0};
    float acceldisp[] = {0, 0, 0};
    int16_t gyroip[] = {0, 0, 0, 0, 0, 0};
    int16_t accelip[] = {0, 0, 0, 0, 0, 0};

    int16_t adata[] = {0, 0, 0};
    int16_t gdata[] = {0, 0, 0};
    int16_t tdata = 0;

    uint8_t gFS = MPU9250_O_GYR0_FS_SEL_250_DPS;
    uint8_t aFS = MPU9250_O_ACCEL_FS_SEL_2_G;
    float gmul, amul;

//    MPU9250 obj(0x68);
//    obj.getST();
//    obj.calibrateAccelGyro(gyrodisp, acceldisp, gyroip, accelip);
//    std::cout<<"1 "<< gyrodisp[0]<< " "<<gyrodisp[1]<<" "<<gyrodisp[2] <<std::endl;
//    std::cout<<"2 "<< acceldisp[0]<< " "<<acceldisp[1]<<" "<<acceldisp[2] <<std::endl;
//    std::cout<<"3 "<< gyroip[0]<< " "<<gyroip[1]<<" "<<gyroip[2]<<" "<<gyroip[3]<< " "<<gyroip[4]<<" "<<gyroip[5] <<std::endl;
//    std::cout<<"4 "<< accelip[0]<< " "<<accelip[1]<<" "<<accelip[2] <<" "<<accelip[3]<< " "<< accelip[4]<< " "<< accelip[5] <<std::endl;

//    obj.setMPU9250Config(0x04, aFS, gFS);
//    amul = obj.getAres(aFS);
//    gmul = obj.getGres(gFS);


    ros::init(argc, argv, "MPU9250_and_AK8963");
    ros::NodeHandle n;
    ros::Publisher publisher = n.advertise<genquad::mpu9250_dat>("Accel_Gyro_Mag_Temp", 10);

    ros::Rate loop_rate(1);

    packet.header.frame_id = "~";
    while(ros::ok())
    {
        packet.header.stamp = ros::Time::now();

        packet.temp.temperature = randVal(20, 30);

        packet.mag.magnetic_field.x = randVal(20, 30);
        packet.mag.magnetic_field.y = randVal(20, 30);
        packet.mag.magnetic_field.z = randVal(20, 30);

        packet.imu.linear_acceleration.x = randVal(10, 20);
        packet.imu.linear_acceleration.y = randVal(10, 20);
        packet.imu.linear_acceleration.z = randVal(10, 20);

        packet.imu.angular_velocity.x = randVal(40, 50);
        packet.imu.angular_velocity.y = randVal(40, 50);
        packet.imu.angular_velocity.z = randVal(40, 50);



        publisher.publish(packet);

        loop_rate.sleep();

    }
    //        obj.getAccelGyroTempCalADC(adata, gdata, tdata);
    //        //std::cout<<"accel is "<<(adata[0] * amul)<<", "<<(adata[1] * amul)<<", "<<(adata[2] * amul)<<std::endl;
    //        //std::cout<<"gyro is "<<(gdata[0] * gmul)<<", "<<(gdata[1] * gmul)<<", "<<(gdata[2] * gmul)<<std::endl;
    //        std::cout<<tdata<<std::endl;
    //        std::cout<<"temperature is "<<(tdata/ 333.87f + 21.0f)<<std::endl;
    //    }


    return rslt;
}
