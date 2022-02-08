#ifndef Odom_h
#define Odom_h

#include <Arduino.h>
#include "FunctionType.h"

class Odom {
    public:
        Odom(volatile long *_encL, volatile long *_encR, double *_outL, double *_outR, functiontype _resetEncoders);
        void configureWheelbase(double _radius, double _length, unsigned long _tpr);
        void configureTranslationPID(double kP, double kI, double kD, double _kPs);
        void configureRotationPID(double kP, double kI, double kD);

        void startTranslate(double dist);
        void startRotate(double alpha, bool _useLeft);

        void loop();

        bool isTranslating();
        void setTranslating(bool _translating);
        bool isRotating();
        void setRotating(bool _rotating);


    private:
        void continueTranslate();
        void continueRotate();
        double distTraveled();
        double diffTraveled();
        double alphaTraveled();
        double limit(double input);

        unsigned long sampleTime = 20; //ms
        double alphaTol = 0.05;
        double distTol = 1.0;
        unsigned long countTol = 20;
        double radius; //mm
        double length; //mm
        unsigned long tpr; //ticks per rev

        bool translating = false;
        bool rotating = false;

        bool useLeft;
        double targetAlpha; //rad
        double prevAlpha; //rad
        double sumAlpha;
        unsigned long alphaStopCount;
        double targetDist; //mm
        double prevDist; //mm
        double sumDist;
        unsigned long distStopCount;
        unsigned long prevT;

        double kPt;
        double kIt;
        double kDt;
        double kPs;
        double kPr;
        double kIr;
        double kDr;

        volatile long *encL;
        volatile long *encR;
        double *outL;
        double *outR;
        functiontype resetEncoders;

};

#endif