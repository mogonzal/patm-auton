#include "Odom.h"

Odom::Odom(volatile long *_encL, volatile long *_encR, double *_outL, double *_outR, functiontype _resetEncoders) {
    encL = _encL;
    encR = _encR;
    outL = _outL;
    outR = _outR;
    resetEncoders = _resetEncoders;
    
    configureWheelbase(0, 0, 0);
    configureTranslationPID(0, 0, 0, 0);
    configureRotationPID(0, 0, 0);
}

void Odom::configureWheelbase(double _radius, double _length, unsigned long _tpr) {
    radius = _radius;
    length = _length;
    tpr = _tpr;
}

void Odom::configureTranslationPID(double kP, double kI, double kD, double _kPs) {
    kPt = kP;
    kIt = kI;
    kDt = kD;
    kPs = _kPs;
}

void Odom::configureRotationPID(double kP, double kI, double kD) {
    kPr = kP;
    kIr = kI;
    kDr = kD;
}

void Odom::startTranslate(double dist) {
    if (!rotating) {
        targetDist = dist;
        prevDist = 0;
        sumDist = 0;
        distStopCount = 0;
        resetEncoders();
        translating = true;
        Serial.println("STARTED TRANSLATE");
    }
}

void Odom::continueTranslate() {
    if (millis() - prevT >= sampleTime) {
        //Serial.println("CONTINUING TRANSLATE");
        // main calculations
        double dist = distTraveled();
        Serial.print("dist traveled: "); Serial.println(dist);
        double error = targetDist - distTraveled();
        double res = kPt * error + kIt * sumDist - kDt * (dist - prevDist);

        // account for non-straightness
        double diff = diffTraveled();
        *outL = limit(res + kPs * diff);
        *outR = limit(res - kPs * diff);

        // check for stopping
        if (fabs(error) <= distTol) {
            distStopCount++;
            if (distStopCount > countTol) {
                Serial.println("DONE TRANSLATING");
                *outL = 0.0;
                *outR = 0.0;
                translating = false;
            }
        }

        prevDist = error;
        sumDist += error;
        prevT = millis();
    }
}

void Odom::startRotate(double alpha) {
    if (!translating) {
        targetAlpha = alpha;
        prevAlpha = 0;
        sumAlpha = 0;
        alphaStopCount = 0;
        resetEncoders();
        rotating = true;
        Serial.println("STARTED ROTATE");
    }
}

void Odom::continueRotate() {
    if (millis() - prevT >= sampleTime) {
        // main calculations
        double alpha = alphaTraveled();
        double error = targetAlpha - alphaTraveled();
        Serial.print("alpha: "); Serial.println(alpha);
        double res = kPr * error + kIr * sumAlpha - kDr * (alpha - prevAlpha);

        //Serial.print("res: "); Serial.println(res);

        *outL = limit(-res);
        *outR = limit(res);

        // check for stopping
        if (fabs(error) <= alphaTol) {
            alphaStopCount++;
            if (alphaStopCount > countTol) {
                *outL = 0.0;
                *outR = 0.0;
                rotating = false;
            }
        }

        prevAlpha = alpha;
        sumAlpha += error;
        prevT = millis();
    }
}

void Odom::loop() {
    if (translating) {
        continueTranslate();
    }
    else if (rotating) {
        continueRotate();
    }
}

double Odom::distTraveled() {
    double distL = radius * TWO_PI * (*encL) / tpr;
    double distR = radius * TWO_PI * (*encR) / tpr;
    return (distL + distR) / 2;
}

double Odom::diffTraveled() {
    double distL = radius * TWO_PI * (*encL) / tpr;
    double distR = radius * TWO_PI * (*encR) / tpr;
    return distR - distL; //pos = right has traveled further, neg = left has traveled further
}

double Odom::alphaTraveled() {
    return 2 * diffTraveled() / length;
}

double Odom::limit(double input) {
    if (input > 100.0) {
        return 100.0;
    }
    else if (input < -100.0) {
        return -100.0;
    }
    return input;
}

bool Odom::isTranslating() {
    return translating;
}

void Odom::setTranslating(bool _translating) {
    translating = _translating;
}

bool Odom::isRotating() {
    return rotating;
}

void Odom::setRotating(bool _rotating) {
    rotating = _rotating;
}
