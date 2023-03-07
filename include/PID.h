#ifndef _PID_H_
#define _PID_H_
#include "definitions_and_declarations.h"

class PID
{
private:
    float output, target;
    float kp, ki, kd;
    float timeTolerance, timerOffset;
    float delayTime;
    float errorTolerance, dTolerance;
    float i, Istart, Imin;
    bool arrived, firstOver;
    float lim, speedCoefficient;
    float error, lasterror;
    V2 vectorTarget;

public:
    void setCoefficient(float p, float i, float d)
    {
        kp = p;
        ki = i;
        kd = d;
    }                                         // must
    void setTarget(float in) { target = in; } // must
    void setTarget(V2 in) { vectorTarget = in; }
    void setIMin(float in) { Imin = in / ki; }
    void setIstart(float in) { Istart = in; }
    void setLim(float in) { lim = in; }
    void setSpeedCoefficeint(float in) { speedCoefficient = in; }
    void setErrorTolerance(float in) { errorTolerance = in; } // must
    void setdTolerance(float in) { dTolerance = in; }
    void setDelayTime(float in) { delayTime = in; }
    bool targetArrived() { return arrived; }
    float getOutput() { return output; }
    float getDelay() { return delayTime * 1000; }
    float getVelocity() { return (error - lasterror) / (delayTime * 1000); }
    PID() : firstOver(true), arrived(false), Istart(10), lim(100), speedCoefficient(1), delayTime(0.01), dTolerance(0.5)
    {
        timerOffset = TIMER;
        timeTolerance = 1e9;
    }
    void update(float input)
    {
        float timeused = TIMER - timerOffset;
        error = target - input;
        float v = (error - lasterror);
        // v = Gyro.gyroRate(zaxis, dps) * dt;
        if ((abbs(error) < errorTolerance && abbs(v) <= dTolerance) || timeused > timeTolerance)
        {
            arrived = true;
        }
        else
        {
            arrived = false;
        }
        if (abbs(error) < Istart)
            i += (error)*delayTime;
        if (error * lasterror <= 0)
        {
            if (firstOver)
            {
                i = sign(error) * Imin;
                firstOver = false;
            }
            else
            {
                i = 0;
            }
        }
        float pow = kp * error + kd * v + ki * i;
        pow = abbs(pow) > lim ? sign(pow) * lim : pow;
        // cout << "PID: " << pow << "   " << error << "    " << v << "   " << endl;
        // printScreen(10,100,"Iner %f",Iner.rotation());
        output = pow;
        lasterror = error;
        // }
    }
    void update(float input, float velocity)
    {
        float timeused = TIMER - timerOffset;
        error = target - input;
        // float v = (error - lasterror) / delayTime;
        // v = Gyro.gyroRate(zaxis, dps) * dt;
        float v = velocity;
        if ((abbs(error) < errorTolerance && abbs(v) <= dTolerance) || timeused > timeTolerance)
        {
            arrived = true;
        }
        else
        {
            arrived = false;
        }

        if (abbs(error) < Istart)
            i += (error)*delayTime;
        if (error * lasterror <= 0)
        {
            if (firstOver)
            {
                i = sign(error) * Imin;
                firstOver = false;
            }
            else
            {
                i = 0;
            }
        }
        float pow = kp * error + kd * v + ki * i;
        pow = abbs(pow) > lim ? sign(pow) * lim : pow;
        // cout << "PID: " << pow << "   " << error << "    " << v << "   " << endl;
        // printScreen(10,100,"Iner %f",Iner.rotation());
        output = pow;
        lasterror = error;
        // }
    }
    void update(V2 input)
    {
        float timeused = TIMER - timerOffset;
        error = (vectorTarget - input).norm();
        float v = (error - lasterror);
        // v = Gyro.gyroRate(zaxis, dps) * dt;
        if ((abbs(error) < errorTolerance && abbs(v) <= dTolerance) || timeused > timeTolerance)
        {
            arrived = true;
        }
        else
        {
            arrived = false;
        }
        if (abbs(error) < Istart)
            i += (error)*delayTime;
        if (error * lasterror <= 0)
        {
            if (firstOver)
            {
                i = sign(error) * Imin;
                firstOver = false;
            }
            else
            {
                i = 0;
            }
        }
        float pow = kp * error + kd * v + ki * i;
        pow = abbs(pow) > lim ? sign(pow) * lim : pow;
        // cout<<vectorTarget.transpose()<<endl;
        // cout<<input.transpose()<<endl;
        // cout << "PID: " << pow << "   " << error << "    " << v << "   " << endl;
        // printScreen(10,100,"Iner %f",Iner.rotation());
        output = pow;
        lasterror = error;
        // }
    }
};

#endif