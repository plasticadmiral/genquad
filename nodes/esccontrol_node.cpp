#include <iostream>
#include <ros/ros.h>
#include "../src/esccontrol.h"
#include "genquad/ESCControl.h"
#include <boost/bind.hpp>

bool motionManipulate(ESCControl* ptr, genquad::ESCControl::Request& request, genquad::ESCControl::Response& response)
{
    std::cout<<"yo here !"<<request.thrustPercentage<<std::endl;
    uint16_t* opPtr = ptr->motionHandler(request.thrustPercentage, request.rollPercentage,
                       request.pitchPercentage, request.yawPercentage);

    std::cout<<opPtr[0]<<" "<<opPtr[1]<<" "<<opPtr[2]<<" "<<opPtr[3]<<std::endl;
    response.frontLeftPW  = opPtr[0];
    response.frontRightPW = opPtr[1];
    response.backLeftPW   = opPtr[2];
    response.backRightPW  = opPtr[3];

    return true;
}

bool throttleLimits()
{



    return true;
}


bool agressionLimits()
{



    return true;
}

int main(int argc, char* argv[])
{

    int FL, FR, BL, BR, TMa, TMi, ARP, AY;
    if(argc != 9)
    {
        std::cout<<"[ERROR]: Please enter all the arguements in the order,"<<std::endl;
        std::cout<<"         FLGPIO FRGPIO BLGPIO BRGPIO ThrottleMin ThrottleMax AggrRP AggrY"<<std::endl;
        std::cout<<"         AggrRP > AggrY, ThrottleMin & ThrottleMax in between 700 & 2000"<<std::endl;
        return 1;
    }


    FL  = atoi(argv[1]);
    FR  = atoi(argv[2]);
    BL  = atoi(argv[3]);
    BR  = atoi(argv[4]);
    TMa = atoi(argv[5]);
    TMi = atoi(argv[6]);
    ARP = atoi(argv[7]);
    AY  = atoi(argv[8]);

    ESCControl instance(FL, FR, BL, BR);
    instance.setThrottleMinMax(TMi, TMa);
    instance.setAgressionLimits(ARP, AY);

    ros::init(argc, argv, "ESCControl");
    ros::NodeHandle n;
    ros::ServiceServer manipulateService =
            n.advertiseService<genquad::ESCControl::Request, genquad::ESCControl::Response>
            ("manipulateESC", boost::bind(&motionManipulate, &instance,
            _1, _2));


    ros::spin();
    return 0;
}
