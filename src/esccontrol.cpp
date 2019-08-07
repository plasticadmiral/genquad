#include "esccontrol.h"
#include <iostream>
#include <stdint.h>
#include <pigpiod_if2.h>

ESCControl::ESCControl(int FL, int FR, int BL, int BR)
    :frontLeftGPIO(FL), frontRightGPIO(FR), backLeftGPIO(BL), backRightGPIO(BR)
{
    std::cout<<"ESC has been constructed!"<<std::endl;
    piHandle = pigpio_start(0, 0);
    if(piHandle < 0) std::cout<<"[ERROR]: Cannot find pigpio daemon! Please start daemon and run start sequence again!"<<std::endl;

}

ESCControl::~ESCControl()
{
    pigpio_stop(piHandle);
}

void ESCControl::setThrottleMinMax(int min, int max)
{
    if(max > maxBoundary) std::cout<<"[WARNING]: maximum pulse width set above recommended threshold!"<<std::endl;
    if(min < minBoundary) std::cout<<"[WARNING]: minimum pulse width set below recommended threshold!"<<std::endl;
    if((min <= minBoundary) && (max >= maxBoundary)) std::cout<<"[INFO]: Throttle max and min limits have been set appropriately!"<<std::endl;
    maxThrottle = max;
    minThrottle = min;
}

void ESCControl::setAgressionLimits(int RP, int Y)
{
    if((maxThrottle+RP) > maxBoundary) std::cout<<"[WARNING]: RPAgression and maxThrottle net is beyond recommended threshold! Please modify AgressionLimits or ThrottleMinMax values!"<<std::endl;
    if((minThrottle-RP) < minBoundary) std::cout<<"[WARNING]: RPAgression and minThrottle net is beyond recommended threshold! Please modify AgressionLimits or ThrottleMinMax values!"<<std::endl;
    if((maxThrottle+Y) > maxBoundary) std::cout<<"[WARNING]: YAgression and maxThrottle net is beyond recommended threshold! Please modify AgressionLimits or ThrottleMinMax values!"<<std::endl;
    if((minThrottle-Y) < minBoundary) std::cout<<"[WARNING]: YAgression and minThrottle net is beyond recommended threshold! Please modify AgressionLimits or ThrottleMinMax values!"<<std::endl;
    if(Y>=RP) std::cout<<"[WARNING]: Y agressionlimit is more than or equal to RP agressionlimit! It will lead to errors! Please correct!"<<std::endl;
    RPAgression = RP;
    YAgression = Y;
}

void ESCControl::manualControl()
{

}


uint16_t* ESCControl::motionHandler(float throttlePercent, float rollPercent, float pitchPercent, float yawPercent)
{
    std::cout<<"class received input: "<<throttlePercent<<" "<<rollPercent<<" "<<pitchPercent<<" "<<yawPercent<<std::endl;

    if ((throttlePercent > 100.0f) || (rollPercent > 100.0f) || (pitchPercent > 100.0f) || (yawPercent > 100.0f))
    {
        std::cout<<"[INFO]: Please provide a value in between 0 and 100 to manipulate!"<<std::endl;
        std::cout<<"        Returned value is from last manipulation!"<<std::endl;


        returnarr[0] = frontLeftThrottle;
        returnarr[1] = frontRightThrottle;
        returnarr[2] = backLeftThrottle;
        returnarr[3] = backRightThrottle;
        return returnarr;
    }
    else if ((throttlePercent == 0.0f) && (rollPercent == 0.0f) && (pitchPercent == 0.0f) && (yawPercent == 0.0f))
    {
        std::cout<<"[INFO]: Cutting Throttle!"<<std::endl;

        cutThrottle();

        returnarr[0] = frontLeftThrottle;
        returnarr[1] = frontRightThrottle;
        returnarr[2] = backLeftThrottle;
        returnarr[3] = backRightThrottle;
        return returnarr;

    }
    else if ((throttlePercent > 0.0f) && (rollPercent == 0.0f) && (pitchPercent == 0.0f) && (yawPercent == 0.0f))
    {
        std::cout<<"[INFO]: Throttle input given!"<<std::endl;

        scaledThrottle = static_cast<int>((throttlePercent * (maxThrottle - minThrottle) / 100) + minThrottle);
        std::cout<<scaledThrottle<<std::endl;

        setThrottle(scaledThrottle);

        returnarr[0] = frontLeftThrottle;
        returnarr[1] = frontRightThrottle;
        returnarr[2] = backLeftThrottle;
        returnarr[3] = backRightThrottle;
        return returnarr;
    }
    else if ((throttlePercent == 0.0f) && (rollPercent > 0.0f) && (pitchPercent == 0.0f) && (yawPercent == 0.0f))
    {
        std::cout<<"[INFO]: roll input given!"<<std::endl;

        scaledRoll = static_cast<int>(rollPercent * RPAgression/100);
        std::cout<<scaledRoll<<std::endl;

        roll(scaledRoll);

        returnarr[0] = frontLeftThrottle;
        returnarr[1] = frontRightThrottle;
        returnarr[2] = backLeftThrottle;
        returnarr[3] = backRightThrottle;
        return returnarr;

    }
    else if ((throttlePercent == 0.0f) && (rollPercent == 0.0f) && (pitchPercent > 0.0f) && (yawPercent == 0.0f))
    {
        std::cout<<"[INFO]: pitch input given!"<<std::endl;

        scaledPitch = static_cast<int>(pitchPercent * RPAgression/100);
        std::cout<<scaledPitch<<std::endl;

        pitch(scaledPitch);

        returnarr[0] = frontLeftThrottle;
        returnarr[1] = frontRightThrottle;
        returnarr[2] = backLeftThrottle;
        returnarr[3] = backRightThrottle;
        return returnarr;
    }
    else if ((throttlePercent == 0.0f) && (rollPercent == 0.0f) && (pitchPercent == 0.0f) && (yawPercent > 0.0f))
    {
        std::cout<<"[INFO]: Yaw input given!"<<std::endl;

        scaledYaw = static_cast<int>(yawPercent * YAgression/100);
        std::cout<<scaledYaw<<std::endl;

        yaw(scaledYaw);

        returnarr[0] = frontLeftThrottle;
        returnarr[1] = frontRightThrottle;
        returnarr[2] = backLeftThrottle;
        returnarr[3] = backRightThrottle;
        return returnarr;
    }
    else if ((throttlePercent > 0.0f) && (rollPercent > 0.0f) && (pitchPercent == 0.0f) && (yawPercent == 0.0f))
    {
        std::cout<<"[INFO]: Throttle and roll input given!"<<std::endl;

        scaledRoll = static_cast<int>(rollPercent * RPAgression/100);
        std::cout<<scaledRoll<<std::endl;
        scaledThrottle = static_cast<int>((throttlePercent * (maxThrottle - minThrottle) / 100) + minThrottle);
        std::cout<<scaledThrottle<<std::endl;

        rollThrottle(scaledThrottle, scaledRoll);

        returnarr[0] = frontLeftThrottle;
        returnarr[1] = frontRightThrottle;
        returnarr[2] = backLeftThrottle;
        returnarr[3] = backRightThrottle;
        return returnarr;
    }
    else if ((throttlePercent > 0.0f) && (rollPercent == 0.0f) && (pitchPercent > 0.0f) && (yawPercent == 0.0f))
    {
        std::cout<<"[INFO]: Throttle and pitch input given!"<<std::endl;

        scaledPitch = static_cast<int>(pitchPercent * RPAgression/100);
        std::cout<<scaledPitch<<std::endl;
        scaledThrottle = static_cast<int>((throttlePercent * (maxThrottle - minThrottle) / 100) + minThrottle);
        std::cout<<scaledThrottle<<std::endl;

        pitchThrottle(scaledThrottle, scaledPitch);

        returnarr[0] = frontLeftThrottle;
        returnarr[1] = frontRightThrottle;
        returnarr[2] = backLeftThrottle;
        returnarr[3] = backRightThrottle;
        return returnarr;
    }
    else if ((throttlePercent > 0.0f) && (rollPercent == 0.0f) && (pitchPercent == 0.0f) && (yawPercent > 0.0f))
    {
        std::cout<<"[INFO]: Throttle and yaw input given!"<<std::endl;

        scaledYaw = static_cast<int>(yawPercent * YAgression/100);
        std::cout<<scaledYaw<<std::endl;
        scaledThrottle = static_cast<int>((throttlePercent * (maxThrottle - minThrottle) / 100) + minThrottle);
        std::cout<<scaledThrottle<<std::endl;

        yawThrottle(scaledThrottle, scaledYaw);

        returnarr[0] = frontLeftThrottle;
        returnarr[1] = frontRightThrottle;
        returnarr[2] = backLeftThrottle;
        returnarr[3] = backRightThrottle;
        return returnarr;
    }
    else if ((throttlePercent == 0.0f) && (rollPercent > 0.0f) && (pitchPercent > 0.0f) && (yawPercent == 0.0f))
    {
        std::cout<<"[INFO]: Roll and Pitch input given!"<<std::endl;

        scaledRoll = static_cast<int>(rollPercent * RPAgression/100);
        std::cout<<scaledRoll<<std::endl;
        scaledPitch = static_cast<int>(pitchPercent * RPAgression/100);
        std::cout<<scaledPitch<<std::endl;

        rollpitch(scaledRoll, scaledPitch);

        returnarr[0] = frontLeftThrottle;
        returnarr[1] = frontRightThrottle;
        returnarr[2] = backLeftThrottle;
        returnarr[3] = backRightThrottle;
        return returnarr;
    }
    else if ((throttlePercent == 0.0f) && (rollPercent == 0.0f) && (pitchPercent > 0.0f) && (yawPercent > 0.0f))
    {
        std::cout<<"[INFO]: Pitch and yaw input given!"<<std::endl;

        scaledPitch = static_cast<int>(pitchPercent * RPAgression/100);
        std::cout<<scaledPitch<<std::endl;
        scaledYaw = static_cast<int>(yawPercent * YAgression/100);
        std::cout<<scaledYaw<<std::endl;

        pitchyaw(scaledPitch, scaledYaw);

        returnarr[0] = frontLeftThrottle;
        returnarr[1] = frontRightThrottle;
        returnarr[2] = backLeftThrottle;
        returnarr[3] = backRightThrottle;
        return returnarr;
    }
    else if ((throttlePercent == 0.0f) && (rollPercent > 0.0f) && (pitchPercent == 0.0f) && (yawPercent > 0.0f))
    {
        std::cout<<"[INFO]: yaw and roll input given!"<<std::endl;

        scaledRoll = static_cast<int>(rollPercent * RPAgression/100);
        std::cout<<scaledRoll<<std::endl;
        scaledYaw = static_cast<int>(yawPercent * YAgression/100);
        std::cout<<scaledYaw<<std::endl;

        rollyaw(scaledRoll, scaledYaw);

        returnarr[0] = frontLeftThrottle;
        returnarr[1] = frontRightThrottle;
        returnarr[2] = backLeftThrottle;
        returnarr[3] = backRightThrottle;
        return returnarr;
    }
    else if ((throttlePercent > 0.0f) && (rollPercent > 0.0f) && (pitchPercent > 0.0f) && (yawPercent == 0.0f))
    {
        std::cout<<"[INFO]: Throttle, Roll and pitch input given!"<<std::endl;

        scaledThrottle = static_cast<int>((throttlePercent * (maxThrottle - minThrottle) / 100) + minThrottle);
        std::cout<<scaledThrottle<<std::endl;
        scaledRoll = static_cast<int>(rollPercent * RPAgression/100);
        std::cout<<scaledRoll<<std::endl;
        scaledPitch = static_cast<int>(pitchPercent * RPAgression/100);
        std::cout<<scaledPitch<<std::endl;

        rollPitchThrottle(scaledThrottle, scaledRoll, scaledPitch);

        returnarr[0] = frontLeftThrottle;
        returnarr[1] = frontRightThrottle;
        returnarr[2] = backLeftThrottle;
        returnarr[3] = backRightThrottle;
        return returnarr;
    }
    else if ((throttlePercent > 0.0f) && (rollPercent == 0.0f) && (pitchPercent > 0.0f) && (yawPercent > 0.0f))
    {
        std::cout<<"[INFO]: Throttle and pitch and yaw input given!"<<std::endl;

        scaledThrottle = static_cast<int>((throttlePercent * (maxThrottle - minThrottle) / 100) + minThrottle);
        std::cout<<scaledThrottle<<std::endl;
        scaledPitch = static_cast<int>(pitchPercent * RPAgression/100);
        std::cout<<scaledPitch<<std::endl;
        scaledYaw = static_cast<int>(yawPercent * YAgression/100);
        std::cout<<scaledYaw<<std::endl;

        pitchYawThrottle(scaledThrottle, scaledPitch, scaledYaw);

        returnarr[0] = frontLeftThrottle;
        returnarr[1] = frontRightThrottle;
        returnarr[2] = backLeftThrottle;
        returnarr[3] = backRightThrottle;
        return returnarr;
    }
    else if ((throttlePercent > 0.0f) && (rollPercent > 0.0f) && (pitchPercent == 0.0f) && (yawPercent > 0.0f))
    {
        std::cout<<"[INFO]: Throttle and roll and yaw input given!"<<std::endl;

        scaledThrottle = static_cast<int>((throttlePercent * (maxThrottle - minThrottle) / 100) + minThrottle);
        std::cout<<scaledThrottle<<std::endl;
        scaledRoll = static_cast<int>(rollPercent * RPAgression/100);
        std::cout<<scaledRoll<<std::endl;
        scaledYaw = static_cast<int>(yawPercent * YAgression/100);
        std::cout<<scaledYaw<<std::endl;

        rollYawThrottle(scaledThrottle, scaledRoll, scaledYaw);

        returnarr[0] = frontLeftThrottle;
        returnarr[1] = frontRightThrottle;
        returnarr[2] = backLeftThrottle;
        returnarr[3] = backRightThrottle;
        return returnarr;
    }
    else if ((throttlePercent > 0.0f) && (rollPercent > 0.0f) && (pitchPercent > 0.0f) && (yawPercent > 0.0f))
    {
        std::cout<<"[INFO]: Throttle roll pitch and yaw input given!"<<std::endl;

        scaledThrottle = static_cast<int>((throttlePercent * (maxThrottle - minThrottle) / 100) + minThrottle);
        std::cout<<scaledThrottle<<std::endl;
        scaledRoll = static_cast<int>(rollPercent * RPAgression/100);
        std::cout<<scaledRoll<<std::endl;
        scaledPitch = static_cast<int>(pitchPercent * RPAgression/100);
        std::cout<<scaledPitch<<std::endl;
        scaledYaw = static_cast<int>(yawPercent * YAgression/100);
        std::cout<<scaledYaw<<std::endl;

        rollPitchYawThrottle(scaledThrottle, scaledRoll, scaledPitch, scaledYaw);

        returnarr[0] = frontLeftThrottle;
        returnarr[1] = frontRightThrottle;
        returnarr[2] = backLeftThrottle;
        returnarr[3] = backRightThrottle;
        return returnarr;
    }
}

void ESCControl::setThrottle(int thrust)
{
    if(thrust)
    {
        commonThrottle = thrust;
    }

    frontLeftThrottle  = commonThrottle;
    frontRightThrottle = commonThrottle;
    backLeftThrottle   = commonThrottle;
    backRightThrottle  = commonThrottle;

    set_servo_pulsewidth(piHandle, frontLeftGPIO,  frontLeftThrottle);
    set_servo_pulsewidth(piHandle, frontRightGPIO, frontRightThrottle);
    set_servo_pulsewidth(piHandle, backLeftGPIO,   backLeftThrottle);
    set_servo_pulsewidth(piHandle, backRightGPIO,  backRightThrottle);

}


void ESCControl::cutThrottle()
{
    frontLeftThrottle  = 0;
    frontRightThrottle = 0;
    backLeftThrottle   = 0;
    backRightThrottle  = 0;

    set_servo_pulsewidth(piHandle, frontLeftGPIO,  frontLeftThrottle);
    set_servo_pulsewidth(piHandle, frontRightGPIO, frontRightThrottle);
    set_servo_pulsewidth(piHandle, backLeftGPIO,   backLeftThrottle);
    set_servo_pulsewidth(piHandle, backRightGPIO,  backRightThrottle);

}


void ESCControl::roll(int agression)
{

    frontLeftThrottle  = commonThrottle + agression;
    frontRightThrottle = commonThrottle - agression;
    backLeftThrottle   = commonThrottle + agression;
    backRightThrottle  = commonThrottle - agression;

    set_servo_pulsewidth(piHandle, frontLeftGPIO,  frontLeftThrottle);
    set_servo_pulsewidth(piHandle, frontRightGPIO, frontRightThrottle);
    set_servo_pulsewidth(piHandle, backLeftGPIO,   backLeftThrottle);
    set_servo_pulsewidth(piHandle, backRightGPIO,  backRightThrottle);

}


void ESCControl::pitch(int agression)
{

    frontLeftThrottle  = commonThrottle - agression;
    frontRightThrottle = commonThrottle - agression;
    backLeftThrottle   = commonThrottle + agression;
    backRightThrottle  = commonThrottle + agression;

    set_servo_pulsewidth(piHandle, frontLeftGPIO,  frontLeftThrottle);
    set_servo_pulsewidth(piHandle, frontRightGPIO, frontRightThrottle);
    set_servo_pulsewidth(piHandle, backLeftGPIO,   backLeftThrottle);
    set_servo_pulsewidth(piHandle, backRightGPIO,  backRightThrottle);

}


void ESCControl::yaw(int agression)
{
    //change signs based on testing results for clockwise or anticlockwise

    frontLeftThrottle  = commonThrottle + agression;
    frontRightThrottle = commonThrottle - agression;
    backLeftThrottle   = commonThrottle - agression;
    backRightThrottle  = commonThrottle + agression;

    set_servo_pulsewidth(piHandle, frontLeftGPIO,  frontLeftThrottle);
    set_servo_pulsewidth(piHandle, frontRightGPIO, frontRightThrottle);
    set_servo_pulsewidth(piHandle, backLeftGPIO,   backLeftThrottle);
    set_servo_pulsewidth(piHandle, backRightGPIO,  backRightThrottle);

}

void ESCControl::rollpitch(int agressionR, int agressionP)
{

    frontLeftThrottle  = commonThrottle + (( + agressionR - agressionP) / 2);
    frontRightThrottle = commonThrottle + (( - agressionR - agressionP) / 2);
    backLeftThrottle   = commonThrottle + (( + agressionR + agressionP) / 2);
    backRightThrottle  = commonThrottle + (( - agressionR + agressionP) / 2);

    set_servo_pulsewidth(piHandle, frontLeftGPIO,  frontLeftThrottle);
    set_servo_pulsewidth(piHandle, frontRightGPIO, frontRightThrottle);
    set_servo_pulsewidth(piHandle, backLeftGPIO,   backLeftThrottle);
    set_servo_pulsewidth(piHandle, backRightGPIO,  backRightThrottle);

}

void ESCControl::pitchyaw(int agressionP, int agressionY)
{

    frontLeftThrottle  = commonThrottle + (( - agressionP + agressionY) / 2);
    frontRightThrottle = commonThrottle + (( - agressionP - agressionY) / 2);
    backLeftThrottle   = commonThrottle + (( + agressionP - agressionY) / 2);
    backRightThrottle  = commonThrottle + (( + agressionP + agressionY) / 2);

    set_servo_pulsewidth(piHandle, frontLeftGPIO , frontLeftThrottle);
    set_servo_pulsewidth(piHandle, frontRightGPIO, frontRightThrottle);
    set_servo_pulsewidth(piHandle, backLeftGPIO  , backLeftThrottle);
    set_servo_pulsewidth(piHandle, backRightGPIO , backRightThrottle);

}

void ESCControl::rollyaw(int agressionR, int agressionY)
{
    frontLeftThrottle  = commonThrottle + (( + agressionR + agressionY) / 2);
    frontRightThrottle = commonThrottle + (( - agressionR - agressionY) / 2);
    backLeftThrottle   = commonThrottle + (( + agressionR - agressionY) / 2);
    backRightThrottle  = commonThrottle + (( - agressionR + agressionY) / 2);

    set_servo_pulsewidth(piHandle, frontLeftGPIO , frontLeftThrottle);
    set_servo_pulsewidth(piHandle, frontRightGPIO, frontRightThrottle);
    set_servo_pulsewidth(piHandle, backLeftGPIO  , backLeftThrottle);
    set_servo_pulsewidth(piHandle, backRightGPIO , backRightThrottle);
}

void ESCControl::rollThrottle(int thrust, int agressionR)
{

    commonThrottle = thrust;

    frontLeftThrottle  = commonThrottle + agressionR;
    frontRightThrottle = commonThrottle - agressionR;
    backLeftThrottle   = commonThrottle + agressionR;
    backRightThrottle  = commonThrottle - agressionR;

    set_servo_pulsewidth(piHandle, frontLeftGPIO , frontLeftThrottle);
    set_servo_pulsewidth(piHandle, frontRightGPIO, frontRightThrottle);
    set_servo_pulsewidth(piHandle, backLeftGPIO  , backLeftThrottle);
    set_servo_pulsewidth(piHandle, backRightGPIO , backRightThrottle);

}

void ESCControl::pitchThrottle(int thrust, int agressionP)
{

    commonThrottle = thrust;

    frontLeftThrottle  = commonThrottle - agressionP;
    frontRightThrottle = commonThrottle - agressionP;
    backLeftThrottle   = commonThrottle + agressionP;
    backRightThrottle  = commonThrottle + agressionP;

    set_servo_pulsewidth(piHandle, frontLeftGPIO , frontLeftThrottle);
    set_servo_pulsewidth(piHandle, frontRightGPIO, frontRightThrottle);
    set_servo_pulsewidth(piHandle, backLeftGPIO  , backLeftThrottle);
    set_servo_pulsewidth(piHandle, backRightGPIO , backRightThrottle);
}

void ESCControl::yawThrottle(int thrust, int agressionY)
{

    commonThrottle = thrust;

    frontLeftThrottle  = commonThrottle + agressionY;
    frontRightThrottle = commonThrottle - agressionY;
    backLeftThrottle   = commonThrottle - agressionY;
    backRightThrottle  = commonThrottle + agressionY;

    set_servo_pulsewidth(piHandle, frontLeftGPIO , frontLeftThrottle);
    set_servo_pulsewidth(piHandle, frontRightGPIO, frontRightThrottle);
    set_servo_pulsewidth(piHandle, backLeftGPIO  , backLeftThrottle);
    set_servo_pulsewidth(piHandle, backRightGPIO , backRightThrottle);

}

void ESCControl::rollPitchThrottle(int thrust, int agressionR, int agressionP)
{
    commonThrottle = thrust;

    frontLeftThrottle  = commonThrottle + (( + agressionR - agressionP) / 2);
    frontRightThrottle = commonThrottle + (( - agressionR - agressionP) / 2);
    backLeftThrottle   = commonThrottle + (( + agressionR + agressionP) / 2);
    backRightThrottle  = commonThrottle + (( - agressionR + agressionP) / 2);

    set_servo_pulsewidth(piHandle, frontLeftGPIO , frontLeftThrottle);
    set_servo_pulsewidth(piHandle, frontRightGPIO, frontRightThrottle);
    set_servo_pulsewidth(piHandle, backLeftGPIO  , backLeftThrottle);
    set_servo_pulsewidth(piHandle, backRightGPIO , backRightThrottle);


}

void ESCControl::pitchYawThrottle(int thrust, int agressionP, int agressionY)
{
    commonThrottle = thrust;

    frontLeftThrottle  = commonThrottle + (( - agressionP + agressionY) / 2);
    frontRightThrottle = commonThrottle + (( - agressionP - agressionY) / 2);
    backLeftThrottle   = commonThrottle + (( + agressionP - agressionY) / 2);
    backRightThrottle  = commonThrottle + (( + agressionP + agressionY) / 2);

    set_servo_pulsewidth(piHandle, frontLeftGPIO , frontLeftThrottle);
    set_servo_pulsewidth(piHandle, frontRightGPIO, frontRightThrottle);
    set_servo_pulsewidth(piHandle, backLeftGPIO  , backLeftThrottle);
    set_servo_pulsewidth(piHandle, backRightGPIO , backRightThrottle);

}

void ESCControl::rollYawThrottle(int thrust, int agressionR, int agressionY)
{
    commonThrottle = thrust;

    frontLeftThrottle  = commonThrottle + (( + agressionR + agressionY) / 2);
    frontRightThrottle = commonThrottle + (( - agressionR - agressionY) / 2);
    backLeftThrottle   = commonThrottle + (( + agressionR - agressionY) / 2);
    backRightThrottle  = commonThrottle + (( - agressionR + agressionY) / 2);

    set_servo_pulsewidth(piHandle, frontLeftGPIO , frontLeftThrottle);
    set_servo_pulsewidth(piHandle, frontRightGPIO, frontRightThrottle);
    set_servo_pulsewidth(piHandle, backLeftGPIO  , backLeftThrottle);
    set_servo_pulsewidth(piHandle, backRightGPIO , backRightThrottle);

}

void ESCControl::rollPitchYawThrottle(int thrust, int agressionR, int agressionP, int agressionY)
{
    commonThrottle = thrust;

    frontLeftThrottle  = commonThrottle + (( + agressionR - agressionP + agressionY) / 3);
    frontRightThrottle = commonThrottle + (( - agressionR - agressionP - agressionY) / 3);
    backLeftThrottle   = commonThrottle + (( + agressionR - agressionP - agressionY) / 3);
    backRightThrottle  = commonThrottle + (( - agressionR - agressionP + agressionY) / 3);

    set_servo_pulsewidth(piHandle, frontLeftGPIO , frontLeftThrottle);
    set_servo_pulsewidth(piHandle, frontRightGPIO, frontRightThrottle);
    set_servo_pulsewidth(piHandle, backLeftGPIO  , backLeftThrottle);
    set_servo_pulsewidth(piHandle, backRightGPIO , backRightThrottle);

}
