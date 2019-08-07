#ifndef ESCCONTROL_H
#define ESCCONTROL_H
#include <stdint.h>

class ESCControl
{
private:
    int piHandle;
    int minBoundary = 700, maxBoundary = 2000;
    int frontLeftGPIO, frontRightGPIO, backLeftGPIO, backRightGPIO;
    uint16_t frontLeftThrottle, frontRightThrottle, backLeftThrottle, backRightThrottle;
    //maintain net sum of agression and height value to remain between
    //700 and 2000
    int RPAgression, YAgression;
    int commonThrottle = 0;
    int maxThrottle = 700, minThrottle = 2000;
    int scaledThrottle, scaledRoll, scaledPitch, scaledYaw = 0;
    uint16_t returnarr[4];

    inline void roll(int agression);
    inline void pitch(int agression);
    inline void yaw(int agression);

    inline void setThrottle(int thrust = 0);
    inline void cutThrottle();
    inline void getThrottle();

    inline void rollpitch(int agressionR, int agressionP);
    inline void pitchyaw(int agressionP, int agressionY);
    inline void rollyaw(int agressionR, int agressionY);

    inline void rollThrottle(int thrust, int agressionR);
    inline void pitchThrottle(int thrust, int agressionP);
    inline void yawThrottle(int thrust, int agressionY);

    inline void rollPitchThrottle(int thrust, int agressionR, int agresssionP);
    inline void pitchYawThrottle(int thrust, int agressionP, int agressionY);
    inline void rollYawThrottle(int thrust, int agressionR, int agressionY);

    inline void rollPitchYawThrottle(int thrust, int agressionR, int agressionP, int agressionY);


public:

    ESCControl(int FL, int FR, int BL, int BR);
    ~ESCControl();
    void setThrottleMinMax(int min, int Max);
    void setAgressionLimits(int RP, int Y);
    void manualControl();
    uint16_t* motionHandler(float throttle, float roll, float pitch, float yaw);
};

#endif // ESCCONTROL_H
